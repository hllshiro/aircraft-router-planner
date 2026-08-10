//! 端到端 solver（Phase 4 M1）：把 Phase 0-3 库层组件串成主流程。
//!
//! parse → validate（main）→ TerrainSource（ARPK1/无地形）→ 语义代价场
//! （Land/Water/NoData/NODATA 5x + Zone 硬墙 INF）→ FMM → 回溯 → 平滑链
//! （Theta* 去锯齿 + 抽稀 + Dubins 拟合 + 复验门）→ VehicleOutput 契约。
//!
//! M1 范围（主管拍板 2026-08-05）：单机/多机（共享代价场，每机独立 FMM）；
//! Zone 水平几何禁入（高度层 M2）；SmoothOptions 默认值（VehicleProfile 派生 M4）；
//! 无雷达代价（M3）；target_ref 暂按 mission.target（M5 每机 waypoints）。

use std::path::PathBuf;
use std::time::Instant;

use crate::config::{
    zone_contains, zone_contains_at, Input, Output, PathPoint, Stats, TerrainSourceType,
    VehicleOutput, Zone, ZoneShape,
};
use crate::coord::Geo;
use crate::costfield::{backtrack_path, build_semantic_cost_field, build_semantic_cost_field_par_local, fmm_propagate};
use crate::error::{AppError, InputInvalidReason};
use crate::path::{Path, PathPoint as RouterPoint};
use crate::smooth::{default_chain, segment_circle_intersect_t, smooth_path_chain, VerifyContext};
use crate::spatial::{CircleEntry, CircleIndex};
use crate::terrain::builtin::BuiltinSource;
use crate::terrain::mask::{GeoMask, MaskedSource};
use crate::terrain::{BulkPrefetch, Sample, TerrainSource};
use crate::threat::{SphericalRadarThreat, ThreatModel, ThreatParams};

/// 地形高度过滤的松弛量（2026-08-10）：只挡「明显高于巡航高度（+300m）」的山。
/// 3000m 任务经过 3058m 五台山（净空不足但历史行为可过）不得被区域级过滤困住而
/// 水平绕行破坏 restricted 剖面语义；1000m 低空场景的 1200m+ 高原仍会被挡 → 无解 →
/// probe → 路径级抬升。撞山抬升判定同口径（避免与 verify 采样密度差异导致误抬）。
const TERRAIN_MASK_SLACK_M: f64 = 300.0;

/// 解算参数（M1：地形路径 CLI/输入指定；grid 粗网格分辨率）。
#[derive(Debug, Clone)]
pub struct SolveParams {
    pub terrain_path: Option<PathBuf>,
    /// 海岸掩膜文件（GSHHG 3 态；None 时自动探测默认掩膜 mask_7p5as.mask 全球 7.5as）
    pub mask_path: Option<PathBuf>,
    pub grid: usize,
}

impl Default for SolveParams {
    fn default() -> Self {
        Self {
            terrain_path: None,
            mask_path: None,
            grid: 256,
        }
    }
}

/// 地形句柄：无地形 / 纯 ARPK1 / 掩膜包装。
/// 掩膜包装（Phase 2 水体判定）：海洋 → Sample::Water（0 高程）、内陆湖 → Sample::Lake(DEM)、
/// 陆地 → 委托内层；平滑链/代价场统一走 TerrainSource/BulkPrefetch 抽象。
enum TerrainHandle {
    None,
    Plain(BuiltinSource),
    Masked(MaskedSource<BuiltinSource>),
}

impl TerrainHandle {
    fn as_source(&self) -> Option<&dyn TerrainSource> {
        match self {
            Self::None => None,
            Self::Plain(t) => Some(t),
            Self::Masked(t) => Some(t),
        }
    }
    fn as_bulk(&self) -> Option<&(dyn BulkPrefetch + Sync)> {
        match self {
            Self::None => None,
            Self::Plain(t) => Some(t),
            Self::Masked(t) => Some(t),
        }
    }
}

/// 解算任务区域（方形经纬度包围盒 + 缓冲）。
#[derive(Debug, Clone, Copy)]
struct Region {
    min_lon: f64,
    min_lat: f64,
    span_deg: f64,
}

/// 待解算的车辆规格（多机或默认单机）。
struct VehicleSpec {
    id: String,
    start: Geo,
    /// 每机目标（target_ref 解析；缺省 = mission.target）。
    target: Geo,
    alt_m: f64,
    /// 机型配置（Phase 4 M4：平滑参数派生输入）。
    profile: crate::config::VehicleProfile,
    /// 中途必经点（Phase 4 M5：start → mid[0..] → target 分段拼接）。
    mid_waypoints: Vec<Geo>,
}

/// 端到端解算。elapsed_ms 为端到端耗时（main 计时传入）。
pub fn solve(input: &Input, params: &SolveParams, elapsed_ms: u64) -> Result<Output, AppError> {
    // 1. 地形源（none = 无地形平面；path/builtin = ARPK1 文件）。
    //    具体类型 BuiltinSource（solver 仅用 ARPK1）：field build 可走 BulkPrefetch
    //    并行无锁预取（候选③，3.71×，对比测试验证）。
    let terrain: TerrainHandle = match input.mission.terrain.source {
        TerrainSourceType::None => TerrainHandle::None,
        _ => {
            let p = params
                .terrain_path
                .clone()
                .or_else(|| input.mission.terrain.path.clone().map(PathBuf::from));
            let inner = match p {
                Some(p) => BuiltinSource::open(&p)?,
                None => {
                    // 主管决策 2026-08-08：默认地形 = 7.5as 东亚压缩版（east_asia_7p5as.arpack）。
                    // 候选：exe 同目录 / 工作目录 data/（2026-08-08 数据迁移到项目根 data/）。
                    let mut candidates = vec![
                        PathBuf::from("east_asia_7p5as.arpack"),
                        PathBuf::from("data/east_asia_7p5as.arpack"),
                    ];
                    if let Ok(exe) = std::env::current_exe() {
                        if let Some(dir) = exe.parent() {
                            candidates.insert(0, dir.join("east_asia_7p5as.arpack"));
                            // 上溯 3 层（target/release → target → workspace 根），逐层试 data/
                            for anc in dir.ancestors().skip(1).take(3) {
                                candidates.push(anc.join("data/east_asia_7p5as.arpack"));
                            }
                        }
                    }
                    if let Some(c) = candidates.iter().find(|c| c.exists()) {
                        BuiltinSource::open(c)?
                    } else {
                        return Err(AppError::Data(
                            "terrain.source=path/builtin 但未提供地形文件，且默认地形 \
                             (east_asia_7p5as.arpack) 未找到（--terrain / terrain.path / exe 同目录 / data/）"
                                .into(),
                        ));
                    }
                }
            };
            // 海岸掩膜（主管 2026-08-08 默认提供）：
            // - 显式 --mask / terrain.mask_path：必须存在，始终套用；
            // - 未显式指定掩膜时：仅**默认地形**（未显式 --terrain/path）自动探测默认掩膜，
            //   显式地形不自动套（用户明确选数据 → 掩膜也应显式，避免语义意外变化）。
            let mask = params
                .mask_path
                .clone()
                .or_else(|| input.mission.terrain.mask_path.clone().map(PathBuf::from));
            let explicit_terrain = params.terrain_path.is_some()
                || input.mission.terrain.path.is_some();
            match mask {
                Some(mp) => {
                    if !mp.exists() {
                        return Err(AppError::Data(format!(
                            "mask file not found: {}（--mask / terrain.mask_path）",
                            mp.display()
                        )));
                    }
                    let gm = GeoMask::open(&mp)?;
                    TerrainHandle::Masked(MaskedSource::new(inner, gm))
                }
                None if !explicit_terrain => match default_mask_candidates() {
                    Some(mp) => {
                        let gm = GeoMask::open(&mp)?;
                        TerrainHandle::Masked(MaskedSource::new(inner, gm))
                    }
                    None => TerrainHandle::Plain(inner),
                },
                None => TerrainHandle::Plain(inner),
            }
        }
    };

    // 2. 车辆规格（vehicles 空 → 默认单机：mission.start → mission.target）
    let specs: Vec<VehicleSpec> = if input.mission.vehicles.is_empty() {
        let t = input.mission.target.to_geo()?;
        vec![VehicleSpec {
            id: "v1".into(),
            start: input.mission.start.to_geo()?,
            target: t,
            alt_m: input.mission.start.alt_m,
            profile: crate::config::VehicleProfile::default(),
            mid_waypoints: Vec::new(),
        }]
    } else {
        let mission_target = input.mission.target.to_geo()?;
        input
            .mission
            .vehicles
            .iter()
            .map(|v| {
                let mid = v
                    .mid_waypoints
                    .iter()
                    .map(|w| {
                        Geo::new(w.lon, w.lat)
                            .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))
                    })
                    .collect::<Result<Vec<_>, AppError>>()?;
                Ok(VehicleSpec {
                    id: v.id.clone(),
                    start: Geo::new(v.start_pose.lon, v.start_pose.lat)
                        .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))?,
                    target: resolve_target_ref(v.target_ref.as_deref(), &mission_target)?,
                    alt_m: v.start_pose.alt_m,
                    profile: v.profile.clone(),
                    mid_waypoints: mid,
                })
            })
            .collect::<Result<Vec<_>, AppError>>()?
    };

    // 3. 任务区域（所有起点 + target 包围盒 + 缓冲）
    let target = input.mission.target.to_geo()?;
    let region = region_of(&specs, &target);
    // 3b. 网格自适应（主管 2026-08-06 双大雷达/多边形场景）：**仅大区域**（span > 2.5°）时
    // 固定 256 格 → 格距粗 → FMM 绕行弧锯齿曲率 < 物理转弯半径 → 平滑链转弯半径
    // verify 拒 → 回退锯齿。
    // 自适应格距：**含多边形墙（NoFly/Obstacle 多边形）→ ≤600m**（绕多边形尖角曲率
    // 敏感：顶点处绕弧曲率 ≈ 膨胀距离，锯齿误差 2% 即不足，实测 zigzag7 grid512
    // cell 0.72km 失败 / grid600 cell 0.64km 成功）；纯圆墙 → ≤1100m（绕圆弧曲率 =
    // 墙半径，不敏感，zigzag5 grid300 即通过）。
    // 小区域（≤2.5°）保持默认 grid——细网格会让 5c2 软罚带物理宽度变窄 → FMM 贴墙
    // 更近 → 绕行 clearance 余量不足（双禁飞区 real_bad 256 成功、301 失败）。
    // 上限 1024 防 OOM。2026-08-07 主管 2000km 场景（span 17.4° 含多边形，auto_grid
    // 3228 被 clamp → cell 1.89km）锯齿根因不是网格粗，而是膨胀墙未补偿 8 邻域楼梯
    // 切角 → FMM 路径离原始墙 < verify 要求的 inflation → 平滑链全失败回退锯齿；
    // 已由 inflation_cells +1.0×cell 补偿修复（1024/2048 均通过，保持 1024 上限）。
    let span_km = region.span_deg * 111.32;
    let has_poly_wall = input
        .mission
        .no_fly_zones
        .iter()
        .chain(input.mission.restricted_zones.iter())
        .chain(input.mission.obstacles.iter())
        .any(|z| z.is_wall() && matches!(z.shape, ZoneShape::Polygon { .. }));
    let target_cell = if has_poly_wall { 600.0 } else { 1100.0 };
    let auto_grid = if region.span_deg > 2.5 {
        ((span_km * 1000.0) / target_cell).ceil() as usize
    } else {
        0
    };
    let grid = params.grid.max(8).max(auto_grid).min(1024);
    eprintln!("[debug] region span={:.2}deg grid={} cell_m={:.0}", region.span_deg, grid, region.span_deg * 111_320.0 / grid as f64);

    // 4. Zone 集合（no_fly + restricted + obstacles）
    //    代价场墙策略（M2 高度层）：
    //    - NoFly/Obstacle → 全高度水平墙（代价场 INF）——保守禁入；
    //    - Restricted → 不画墙（高度区间外可穿越），由 Theta* check + verify 高度判定。
    let all_zones: Vec<Zone> = input
        .mission
        .no_fly_zones
        .iter()
        .chain(input.mission.restricted_zones.iter())
        .chain(input.mission.obstacles.iter())
        .cloned()
        .collect();
    let nofly = circle_index(&all_zones.iter().collect::<Vec<_>>());

    // 4b. 参数合并 + 禁飞区膨胀距离（主管 2026-08-06：绕飞太贴边→考虑飞机机动）。
    //     规划转弯半径 r = turn_radius（信任输入/默认表，2026-08-07 起不再钳巡航物理
    //     下限；小半径经转弯段降速实现）；绕行弧需要 ≥r 的转弯空间，把 NoFly/Obstacle
    //     硬墙向外膨胀 max(0.5×r)（clamp [2km, 10km]）——FMM 绕行自然远离边界，
    //     Dubins 转弯弧留足空间（不再因贴边急弯被物理复验拒绝）。
    let params_merged = crate::config::DefaultParams::default().merge(&input.mission.parameters);
    let mut degradations = Vec::new();
    radar_param_degradations(input, &mut degradations);
    let inflation_m = specs
        .iter()
        .map(|v| {
            let (opts, _phys) = crate::smooth::smooth_options_for(&v.profile, &params_merged);
            (opts.turn_radius_m * 0.5).clamp(2_000.0, 10_000.0)
        })
        .fold(0.0f64, f64::max);
    let inflation_km = inflation_m / 1000.0;

    // 5. 语义代价场（Land=1 / Water=1 / Lake=1 / NoData=5 / OOB=INF；NoFly/Obstacle 墙）
    //    硬墙判定闭包（每格：墙内 → OutOfBounds 禁行）——par_local 与串行回退共用。
    let walled = |lon: f64, lat: f64| -> bool {
        if let Ok(g) = Geo::new(lon, lat) {
            all_zones
                .iter()
                .any(|z| z.is_wall() && zone_contains(z, &g))
        } else {
            false
        }
    };
    let mut field = match &terrain {
        // 候选③：并行 + 无锁批量预取（3.71× vs 串行，对比测试 9504381 之后验证）
        TerrainHandle::Plain(t) => build_semantic_cost_field_par_local(
            t,
            region.min_lon,
            region.min_lat,
            region.span_deg,
            grid,
            5.0,
            &walled,
        ),
        TerrainHandle::Masked(t) => build_semantic_cost_field_par_local(
            t,
            region.min_lon,
            region.min_lat,
            region.span_deg,
            grid,
            5.0,
            &walled,
        ),
        TerrainHandle::None => build_semantic_cost_field(grid, grid, |r, c| {
            let (lon, lat) = cell_lonlat(r, c, &region, grid);
            if walled(lon, lat) {
                return Sample::OutOfBounds;
            }
            Sample::Land(0.0)
        }, 5.0),
    };

    // 5c. 禁飞区墙向外膨胀 + 过渡带软罚（见 apply_inflation_and_band）
    let cell_m = region.span_deg * 111_320.0 / grid as f64;
    // 2026-08-07 主管 2000km 场景根因：FMM 8 邻域楼梯沿膨胀墙走时对角线切角
    // ~0.71×cell，路径离原始墙 = inflation_cells×cell − 0.71×cell < verify 要求的
    // inflation_m → 平滑链全失败回退锯齿。切角危害随 cell 增大：小区域（span≤2.5°，
    // 默认 grid 256，cell≤0.9km）原膨胀 ceil 后余量刚好盖过切角（双禁飞区 7 格
    // 实测切角 0.70×cell 边界通过，加补偿会变 8 格挤窄缝隙）；大跨度场景 cell 1.9km
    // 时 ceil 余量不足（3 格 5.67km − 切角 1.34km = 4.33km < 5.52km）。因此仅
    // span>2.5° 补理论切角 0.71×cell（2000km 场景 3→4 格触发修复）。
    // 2026-08-07 zigzag17：多边形**尖角顶点**（poly3 西南角 (116.198,37.111)）处
    // 格点墙角是钝的（墙格在顶点东北），FMM 路径从顶点西侧绕过时离**几何边**
    // 1.33km < inflation 2km——0.71×cell 切角补偿不够（ceil((2000+0.71×1953)/1953)=2
    // 格，路径离几何边 = 2×1953−偏差 ≈1.4km）。大区域再 +1 格兜底尖角偏差。
    let inflation_cells = if region.span_deg > 2.5 {
        ((inflation_m + 0.71 * cell_m) / cell_m.max(1.0)).ceil() as usize + 1
    } else {
        (inflation_m / cell_m.max(1.0)).ceil() as usize
    };
    apply_inflation_and_band(&mut field, inflation_cells, cell_m);

    // 5b. 雷达静态代价（Phase 4 M3）：膨胀半径内 cost ×(1+coef·(几何并集概率 + 深穿惩罚))
    //     ——FMM 倾向绕行；LOS 动态遮挡由 verify 威胁评估判定（不进静态代价场）。
    //     coef = radar_cost_coef（默认 200）。几何深穿惩罚（u<1 时 ×(1+coef·(1-u))，
    //     探测区外 u≥1 无几何项）：确保穿探测区明确绕行——主管 2026-08-06：
    //     并排双雷达不得直穿探测区（即使 P_cross 调高，几何绕行与验收阈值解耦）。
    let threat_params = radar_threat_params(&params_merged);
    let threat = SphericalRadarThreat::new(&input.mission.red_forces.radars, threat_params.clone());
    if !input.mission.red_forces.radars.is_empty() {
        for r in 0..grid {
            for c in 0..grid {
                let (lon, lat) = cell_lonlat(r, c, &region, grid);
                let p = threat.static_union_probability(lon, lat);
                if p > 0.0 {
                    let idx = r * grid + c;
                    if field.cost[idx].is_finite() {
                        let u = threat.static_penetration(lon, lat, 0.0);
                        let geom = if u < 1.0 { 1.0 - u } else { 0.0 };
                        field.cost[idx] *= (1.0 + params_merged.radar_cost_coef * (p + geom)) as f32;
                    }
                }
            }
        }
    }

    // 6. 每机：分段 FMM（start → mid[0..] → target，共享代价场）→ 拼接 → 平滑 → 输出
    let mut out_vehicles = Vec::new();
    let mut fmm_ms = 0.0f64;
    'veh: for v in &specs {
        // 每机目标（shadow：闭包与剖面切分统一使用 v.target，mission.target 仅作缺省）
        let target = v.target;
        // 段序列：起点 + 必经点 + 目标
        let mut seg_ends: Vec<Geo> = Vec::with_capacity(v.mid_waypoints.len() + 2);
        seg_ends.push(v.start);
        seg_ends.extend(v.mid_waypoints.iter().copied());
        seg_ends.push(target);
        // 机型平滑参数提前（受限区剖面需要 max_climb：决定下降/爬升距离）
        let (opts, phys_min_radius_m) = crate::smooth::smooth_options_for(&v.profile, &params_merged);
        // 受限区墙（剖面直穿语义，主管 2026-08-06 二轮+三轮）：飞行高度落在 restricted
        // 高度区间内 → 比较底部穿行 / 顶部绕飞（底部可行恒更优，否则顶部）→ 可行则
        // 不画墙，FMM 直穿后由 build_restricted_profiles 沿 raw 路径生成剖面；两者都
        // 不可行（地形过高且超升限 / 太贴边 / 多边形）→ 画墙水平绕行（fallback 保底）；
        // 高度在区间外（如低于 alt_min_m 的"底部通道"）→ 不画墙直穿（可通行）。
        // 三轮架构增强（主管 2026-08-06 指定实现）：不再因"start→target 直线穿硬墙"
        // 而画 restricted 墙——FMM 只绕硬墙（no_fly/obstacle），restricted 直穿后在
        // build 内沿 raw 轨迹剖面（绕硬墙弧 + 圆内弦，即"先绕 no_fly 再剖面直穿"的
        // 组合机动）；若剖面段（desc→in→out→climb）仍穿硬墙 → need_wall → 第二轮
        // FMM 画 restricted 墙水平绕行兜底。
        let mut smooth_src: Vec<Path> = Vec::new();
        let mut profile_mask: Vec<bool> = Vec::new();
        let mut raw_joined: Path = Path::new(Vec::new());
        let mut force_restricted_wall = false;
        // 地形高度过滤 + 抬升机制（2026-08-10 主管 e2e_zz 低空撞山）：
        // FMM 2D 不感知飞行高度 vs 地形——1000m 巡航可穿过 1447m 山 → raw 穿山 →
        // Theta*/verify 全拒 → 回退密集网格楼梯。方案：代价场按本机飞行高度把
        // 「Land 高度 + 净空 ≥ alt」格点置 INF（FMM 水平绕山）；
        //   · 过滤后 FMM 无解 → 先用**无过滤场**探测路径（区分「真无通道」与「区域级
        //     过度过滤」——大区域如青藏高原边缘，3000m 任务原本可走，不得抬升）；
        //   · 探测/过滤路径**本身撞山**（路径地形 + 净空 ≥ 巡航高度）→ 把巡航高度抬到
        //     「路径地形最高 + 净空 + 100m」重跑（**路径级**抬升，非区域级——避免把
        //     区域最大地形当目标导致过度抬升破坏 restricted 底部/顶部剖面语义）；
        //   · 抬升后仍无解 → 无过滤场保底（宁丑勿违）。
        let mut terrain_probe_done = false;   // 已用无过滤场探测路径
        let mut terrain_alt_raised = false;   // 已抬升巡航高度
        let mut terrain_fallback_done = false; // 已回退无过滤场（保底）
        // 有效巡航高度（可被抬升逻辑更新；初始 = 起点高度）
        let mut alt_eff = v.alt_m;
        'fmm_attempt: for _attempt in 0..4 {
            let use_terrain_mask = if terrain_fallback_done {
                false
            } else if terrain_alt_raised {
                true
            } else {
                !terrain_probe_done
            };
            let restricted_wall_for = |z: &Zone| {
                force_restricted_wall
                    || restricted_detour_required(
                        z,
                        alt_eff,
                        v.profile.ceiling_m,
                        terrain.as_source(),
                        &v.start,
                        &target,
                        opts.max_climb_deg,
                    )
            };
            let has_restricted_wall = all_zones.iter().any(|z| restricted_wall_for(z));
            let has_terrain_mask = use_terrain_mask && terrain.as_source().is_some();
            let veh_field: Option<crate::costfield::CostField> =
                if has_restricted_wall || has_terrain_mask {
                    let mut f = field.clone();
                    let g = f.rows;
                    if has_restricted_wall {
                        for r in 0..g {
                            for c in 0..g {
                                let (lon, lat) = cell_lonlat(r, c, &region, g);
                                if let Ok(gg) = Geo::new(lon, lat) {
                                    if all_zones
                                        .iter()
                                        .any(|z| restricted_wall_for(z) && zone_contains(z, &gg))
                                    {
                                        f.cost[r * g + c] = f32::INFINITY;
                                    }
                                }
                            }
                        }
                        // 膨胀/软罚带只对 restricted 墙做；**不得**因 terrain 存在而调用——
                        // apply_inflation_and_band 会膨胀**所有** INF 格点（含基础场 NoData/
                        // OOB 墙）+ 加 1.5km 软罚带，terrain 存在时误调会改变全场代价导致
                        // FMM 路径被 NoData/OOB 墙膨胀挤开（zigzag21 实测：路径绕行 rz1，
                        // 6500m 剖面丢失）。
                        apply_inflation_and_band(&mut f, inflation_cells, cell_m);
                    }
                    // 地形高度过滤：Land 格点高度 + 净空 ≥ 本机飞行高度 + slack → 禁行（INF）。
                    // 注意：Water/Lake 水面净空从 0 起算，飞行高度 > 0 即可通行；
                    // NoData/OOB 已由基础代价/墙处理，此处不覆盖。地形墙是连续格点场，
                    // FMM 格点步进天然绕开，不需要膨胀（膨胀会过度阻塞窄通道）。
                    if has_terrain_mask {
                        let tsrc = terrain.as_source().unwrap();
                        let alt = alt_eff;
                        let clearance = opts.clearance_m.max(1.0);
                        for r in 0..g {
                            for c in 0..g {
                                let (lon, lat) = cell_lonlat(r, c, &region, g);
                                if let Sample::Land(h) = tsrc.sample_at(lon, lat) {
                                    if h + clearance >= alt + TERRAIN_MASK_SLACK_M {
                                        f.cost[r * g + c] = f32::INFINITY;
                                    }
                                }
                            }
                        }
                    }
                    Some(f)
                } else {
                    None
                };
            let field_ref = veh_field.as_ref().unwrap_or(&field);
            eprintln!("[debug] fmm attempt {} field ready (veh={})", _attempt, veh_field.is_some());
            // 逐段 FMM → 回溯 → 拼接（去重段端点）
            let mut raw_segs: Vec<Path> = Vec::new();
            let mut no_solution = false;
            for seg in seg_ends.windows(2) {
                let (s, e) = (seg[0], seg[1]);
                let (sr, sc) = lonlat_cell(s.lon, s.lat, &region, grid);
                let (dr, dc) = lonlat_cell(e.lon, e.lat, &region, grid);
                let t0 = Instant::now();
                let res = fmm_propagate(field_ref, sr, sc);
                fmm_ms += t0.elapsed().as_secs_f64() * 1000.0;
                let Some(mut cells) = backtrack_path(field_ref, &res, dr, dc, sr, sc) else {
                    no_solution = true;
                    break;
                };
                // backtrack 返回 dst→src 顺序 → 反转为 src→dst（路径语义）
                cells.reverse();
                raw_segs.push(Path::new(
                    cells
                        .iter()
                        .map(|&(r, c)| {
                            let (lon, lat) = cell_lonlat(r, c, &region, grid);
                            RouterPoint::new(lon, lat, alt_eff)
                        })
                        .collect(),
                ));
            }
            if no_solution || raw_segs.is_empty() {
                if use_terrain_mask && !terrain_probe_done {
                    // 过滤无解 → 无过滤场探测路径（区分「真无通道」与「区域级过度过滤」）
                    terrain_probe_done = true;
                    eprintln!(
                        "[debug] terrain-masked FMM no path -> probe unmasked (v={})",
                        v.id
                    );
                    continue 'fmm_attempt;
                }
                if use_terrain_mask && !terrain_fallback_done {
                    // 抬升后仍无解 → 无过滤场保底（保可用性，宁丑勿违）
                    terrain_fallback_done = true;
                    eprintln!(
                        "[debug] raised FMM no path -> fallback unmasked (v={})",
                        v.id
                    );
                    continue 'fmm_attempt;
                }
                out_vehicles.push(VehicleOutput {
                    id: v.id.clone(),
                    status: "no_solution".into(),
                    path: Vec::new(),
                    distance_m: 0.0,
                    warnings: vec!["coarse FMM no path".into()],
                });
                continue 'veh;
            }
            // 段端点（必经点/目标）是硬约束：任何平滑不得移除
            raw_joined = join_paths(&raw_segs);

            // 路径撞山检查 + 抬升决策（未抬升时）：沿 **分段直线**（起点→必经点→目标）
            // 采样地形，若直线地形 + 净空 ≥ 巡航高度（撞山）→ 抬到「直线地形最高 + 净空
            // + 100m」重跑。**必须采样直线而非 FMM 路径点**：FMM 网格路径（楼梯）点
            // 间隔 ~1km 且可能恰好错过尖峰（2026-08-10 主管输入：直线经 2137m 峰，
            // 网格点只采到 1692m → 抬升不足 → FMM 绕山楼梯 + Theta* 拉直穿山 →
            // 全链失败回退锯齿）。直线是平滑链/直线替代交付路径的上界；密度
            // ~1km（同 verify 口径），覆盖尖峰。**路径级**抬升：只抬到直线地形
            // 最高 + 净空 + 100m，避免区域级过度抬升破坏 restricted 剖面语义。
            if !terrain_alt_raised && terrain.as_source().is_some() {
                let t = terrain.as_source().unwrap();
                let mut path_max_terr: f64 = 0.0;
                for ends in seg_ends.windows(2) {
                    let (a, b) = (ends[0], ends[1]);
                    let seg_len_m = crate::path::haversine_m(a.lon, a.lat, b.lon, b.lat);
                    let n = ((seg_len_m / 1_000.0).ceil() as usize).max(2);
                    for i in 0..=n {
                        let tt = i as f64 / n as f64;
                        let lon = a.lon + (b.lon - a.lon) * tt;
                        let lat = a.lat + (b.lat - a.lat) * tt;
                        if let Sample::Land(h) = t.sample_at(lon, lat) {
                            path_max_terr = path_max_terr.max(h);
                        }
                    }
                }
                let clearance = opts.clearance_m.max(1.0);
                if path_max_terr > 0.0 && path_max_terr + clearance >= alt_eff + TERRAIN_MASK_SLACK_M {
                    let new_alt = (path_max_terr + clearance + 100.0).max(v.alt_m);
                    let ceiling_ok = v
                        .profile
                        .ceiling_m
                        .is_none_or(|c| new_alt <= c);
                    if new_alt > alt_eff + 0.5 && ceiling_ok {
                        terrain_alt_raised = true;
                        alt_eff = new_alt;
                        eprintln!(
                            "[debug] terrain path collision -> raise cruise alt {:.0}->{:.0}m (path terrain {:.0}m, v={})",
                            v.alt_m, alt_eff, path_max_terr, v.id
                        );
                        continue 'fmm_attempt;
                    }
                    // 超升限 → 不抬升，用当前路径（verify 会记穿山，保可用性）
                }
            }

            // 受限区底部/顶部剖面切分（沿 raw 路径；剖面段跳过平滑链）
            smooth_src.clear();
            profile_mask.clear();
            let mut need_wall = false;
            for seg in &raw_segs {
                let (sub, mask, nw) = build_restricted_profiles(
                    seg,
                    &all_zones,
                    alt_eff,
                    opts.max_climb_deg,
                    v.profile.ceiling_m,
                    terrain.as_source(),
                    &v.start,
                    &target,
                    inflation_m / 1000.0,
                );
                need_wall |= nw;
                smooth_src.extend(sub);
                profile_mask.extend(mask);
            }
            if need_wall {
                // 剖面段穿硬墙（如 no_fly 圆）→ 该 restricted 必须画墙水平绕行 → 第二轮重算
                force_restricted_wall = true;
                continue 'fmm_attempt;
            }
            break;
        }
        let mut warnings = Vec::new();
        // 抬升提示（2026-08-10）：巡航高度被抬升必须显式告知（低空任务被地形抬升）。
        if terrain_alt_raised && alt_eff > v.alt_m + 0.5 {
            let terr_max = (alt_eff - opts.clearance_m.max(1.0) - 100.0).max(0.0);
            let msg = format!(
                "terrain clearance: cruise altitude raised {:.0}->{:.0}m (terrain up to {:.0}m)",
                v.alt_m, alt_eff, terr_max
            );
            warnings.push(msg.clone());
            degradations.push(msg);
        }
        // 降速提示（主管 2026-08-07：速度非锁定，转弯段可降速实现小半径）：
        // turn_radius < 巡航物理下限 → 转弯段需降到 v_turn = sqrt(r·g·tanφ)。
        if opts.turn_radius_m > 0.0 {
            let bank = v
                .profile
                .max_bank_deg
                .unwrap_or(params_merged.default_max_bank_deg);
            let v_turn = (opts.turn_radius_m * 9.81 * bank.to_radians().tan()).sqrt();
            let cruise_v = v
                .profile
                .cruise_speed_mps
                .or_else(|| v.profile.speed_range_mps.map(|[a, b]| (a + b) / 2.0))
                .unwrap_or(match v.profile.aircraft_type {
                    crate::config::AircraftType::FixedWing => {
                        params_merged.default_fixed_wing_speed_mps
                    }
                    crate::config::AircraftType::Rotorcraft => {
                        params_merged.default_rotorcraft_speed_mps
                    }
                });
            if v_turn < cruise_v - 1e-9 {
                warnings.push(format!(
                    "turn radius {:.0}m: turn segments require speed reduction {:.0}->{:.0} m/s",
                    opts.turn_radius_m, cruise_v, v_turn
                ));
            }
        }
        let mut pts = raw_joined.points.clone();
        if pts.len() >= 2 {
            let check = make_segment_check(
                &all_zones,
                Some(&threat as &dyn crate::threat::ThreatModel),
                inflation_km,
                terrain.as_source(),
                opts.clearance_m,
            );
            let ctx = VerifyContext {
                terrain: terrain.as_source(),
                nofly: Some(&nofly),
                zones: Some(&all_zones),
                threat: Some(&threat),
                zone_inflation_m: inflation_m,
            };
            // 风险1修复（2026-08-07）：平滑链 verify + 威胁 LOS 采样直接打地形源
            // （height_at 走 LRU），采样点可能越出代价场预取矩形——region 仅起点/target
            // 包围盒 + 0.15° 缓冲，而绕行弧（NoFly/雷达/restricted）可偏出该矩形 → 冷块
            // mmap 切片 + zstd 解压延迟。平滑前按 smooth_src 联合包围盒 + 机动 slack
            // （转弯半径 + 5km，Dubins 弧偏出 raw 的量级）补一次批量预取：块进全局 LRU，
            // 之后 height_at 全部命中缓存。region 本身不动——扩大会粗化 FMM 网格 cell
            // （小区域固定 256 格），有锯齿风险。
            if let Some(t) = terrain.as_bulk() {
                let slack_deg = (phys_min_radius_m + 5_000.0) / 111_320.0;
                let mut min_lon = f64::INFINITY;
                let mut min_lat = f64::INFINITY;
                let mut max_lon = f64::NEG_INFINITY;
                let mut max_lat = f64::NEG_INFINITY;
                for seg in &smooth_src {
                    for p in &seg.points {
                        min_lon = min_lon.min(p.lon);
                        min_lat = min_lat.min(p.lat);
                        max_lon = max_lon.max(p.lon);
                        max_lat = max_lat.max(p.lat);
                    }
                }
                if min_lon.is_finite() {
                    t.prefetch_lonlat(
                        min_lon - slack_deg,
                        min_lat - slack_deg,
                        max_lon + slack_deg,
                        max_lat + slack_deg,
                    );
                }
            }
            // 每段独立平滑（首尾段端点保留——Theta* 截直不得移除必经点）。
            // 入口航向：前一段输出方向，约束当前段首跳（段边界转角，否则拼接后
            // 终检暴露——2026-08-07 主管 1755 点场景 seg3 out→climb 与 seg4
            // climb→A 夹角 61.94° > 60°，climb 是段首点单段 verify 无法发现）。
            let mut smooth_segs: Vec<crate::path::Path> = Vec::new();
            let mut seg_warnings = Vec::new();
            let mut entry_heading: Option<f64> = None;
            // 段边界硬约束点（起点/必经点/目标）：arc 修复会弹出边界点 b，必经点不得
            // 被替代（user 硬约束），否则违反"任何平滑不得移除必经点"。
            let hard_boundary: Vec<(f64, f64)> = seg_ends.iter().map(|g| (g.lon, g.lat)).collect();
            for (idx, seg) in smooth_src.iter().enumerate() {
                if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                    eprintln!(
                        "[smooth-dbg] SEG{idx} mask={} len={} first=({:.4},{:.4})@{} last=({:.4},{:.4})@{}",
                        profile_mask[idx] as u8,
                        seg.points.len(),
                        seg.points.first().map_or(0.0, |p| p.lon),
                        seg.points.first().map_or(0.0, |p| p.lat),
                        seg.points.first().map_or(0.0, |p| p.alt_m),
                        seg.points.last().map_or(0.0, |p| p.lon),
                        seg.points.last().map_or(0.0, |p| p.lat),
                        seg.points.last().map_or(0.0, |p| p.alt_m),
                    );
                }
                let mut out_seg = if profile_mask[idx] {
                    // 受限区剖面段：已按 max_climb 生成下降/平飞/爬升，直接采用
                    seg.clone()
                } else {
                    // 首跳 entry 放宽上限：当前段起点是硬边界点（起点/必经点/目标）
                    // → 必经点处大转向合法（zigzag27：wp3 160° 掉头），放宽到 175°
                    // 让 theta_star 直接拉直，段边界由 arc_transition 切弧；
                    // 非硬点段（如受限区剖面锚点间过渡段）保持 95°（zigzag11 保护）。
                    // 容差与下方硬点识别一致（max(0.75×cell, 250m)）。
                    let hard_tol_m = (cell_m * 0.75).max(250.0);
                    let seg_start_is_hard = seg.points.first().map_or(false, |p0| {
                        hard_boundary.iter().any(|(lo, la)| {
                            dist_km(*lo, *la, p0.lon, p0.lat) * 1000.0 < hard_tol_m
                        })
                    });
                    let entry_max_deg = if seg_start_is_hard { 175.0 } else { 95.0 };
                    let chain = default_chain(&opts, &check, entry_heading, entry_max_deg);
                    let result = smooth_path_chain(seg, &chain, &opts, &ctx, Some(phys_min_radius_m));
                    if let Some(w) = &result.warning {
                        seg_warnings.push(w.clone());
                    }
                    seg_warnings.extend(result.verify.warnings.iter().cloned());
                    result.path
                };
                // 段边界转角修复（2026-08-08 主管真实地形场景 zigzag19）：
                // desc_in/out_climb（mask=true 固定直线）方向不受 entry_heading 约束
                // （entry 只约束 default_chain 段首跳），且 build 的 climb 出口约束用
                // tail 终点方向近似、与 theta 拉直后实际首段方向偏差大 → 拼接后段边界
                // 转角可超 max_turn（pt3 65.9° / pt4 70.5°）→ final verify 拒 → 全链
                // 回退 raw 密集锯齿。每段（含 mask=true）push 前检查与前一段输出在
                // 边界点 b 的转角，超限 → arc_transition 插入过渡弧（弹出 b，弧点高度
                // = b.alt_m 平飞，逐段 check 不穿墙；E→c 仍沿出段方向，爬升角由
                // climb_base 保证）。arc 失败（穿墙等）保持原样，宁丑勿违；必经点
                // （keep_b）处大转角同样插弧——物理上必经点平滑转弯必须切弧（偏差
                // ≤ r·tan(θ/2) ≈ 0.6km，2026-08-10 zigzag25 主管输入实测）。
                if let Some(prev) = smooth_segs.last_mut() {
                    let n = prev.points.len();
                    if n >= 2 && out_seg.points.len() >= 2 {
                        let a = prev.points[n - 2];
                        let b = prev.points[n - 1];
                        let c = out_seg.points[1];
                        let h0 = crate::path::bearing_deg(a.lon, a.lat, b.lon, b.lat);
                        let h1 = crate::path::bearing_deg(b.lon, b.lat, c.lon, c.lat);
                        let d = crate::path::angle_diff_deg(h0, h1).abs();
                        // 段端点网格离散：FMM 终点 snap 到最近网格节点，段端点（起点/
                        // 必经点/目标）可偏离输入坐标 ~0.5 cell（cell 818m → ~400m）。
                        // 1e-9 精确匹配会漏判（2026-08-10 zigzag25：b 距必经点 242m
                        // → 必经点未受保护 → 大半径弧弹出 b 且 E 越过出段节点 → 折返
                        // 178° → final verify 拒 → 回退 471 点锯齿）。容差 = max(0.75
                        // ×cell, 250m) 覆盖网格离散；keep_b=true → 弧用物理转弯半径
                        // （紧贴 b，切点偏差 ≤ r·tan(θ/2) ≈ 0.6km，满足必经点容差
                        // 0.05°≈5.5km 测试断言——物理上必经点处平滑转弯必须切弧）。
                        let hard_tol_m = (cell_m * 0.75).max(250.0);
                        let is_hard = hard_boundary.iter().any(|(lo, la)| {
                            dist_km(*lo, *la, b.lon, b.lat) * 1000.0 < hard_tol_m
                        });
                        if d > opts.max_turn_deg {
                            if let Some((arc_pts, _k)) = crate::smooth::arc_transition(
                                &a,
                                &b,
                                &c,
                                opts.max_turn_deg,
                                opts.turn_radius_m,
                                &check,
                                &prev.points,
                                n - 1,
                                false,
                                is_hard,
                            ) {
                                let arc_len = arc_pts.len();
                                let e = *arc_pts.last().unwrap();
                                prev.points.truncate(n - 1);
                                prev.points.extend(arc_pts);
                                // 后续段起点若为被弹出的 b（剖面锚点/平滑段端点，非硬约束）
                                // → 同步到弧末点 E，否则 joined 出现 E→b 回头路
                                // （2026-08-08 实测 E→原 pt3 转角 179.99°）。
                                if !out_seg.points.is_empty() {
                                    let p0 = &out_seg.points[0];
                                    if (p0.lon - b.lon).abs() < 1e-9
                                        && (p0.lat - b.lat).abs() < 1e-9
                                    {
                                        out_seg.points[0] = e;
                                    }
                                }
                                if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                    eprintln!(
                                        "[smooth-dbg] boundary arc at ({:.4},{:.4}) turn {:.1}->{} pts",
                                        b.lon, b.lat, d, arc_len
                                    );
                                }
                            } else if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                eprintln!(
                                    "[smooth-dbg] boundary arc FAIL at ({:.4},{:.4}) turn {:.1} hard={is_hard}",
                                    b.lon, b.lat, d
                                );
                            }
                        }
                    }
                }
                let entry_next = out_seg.last_segment_heading();
                smooth_segs.push(out_seg);
                entry_heading = entry_next;
            }
            // 拼接 + 全路径终检（段间转角/整路径威胁在拼接后才可见）
            let joined = join_paths(&smooth_segs);
            // 段端点 = 起点 + 必经点 + 目标（直线替代用；必经点硬约束，任何平滑不得移除）
            let mut straight_pts: Vec<crate::path::PathPoint> = Vec::new();
            for g in &seg_ends {
                let p = crate::path::PathPoint::new(g.lon, g.lat, alt_eff);
                let dup = straight_pts.last().map_or(false, |q| {
                    (q.lon - p.lon).abs() < 1e-12 && (q.lat - p.lat).abs() < 1e-12
                });
                if !dup {
                    straight_pts.push(p);
                }
            }
            let straight = Path::new(straight_pts);
            let final_rep = crate::smooth::verify_path(
                &joined,
                None,
                &opts,
                &ctx,
                Some(phys_min_radius_m),
            );
            if final_rep.ok {
                pts = joined.points;
                // extend 而非覆盖：保留 profile 级降速提示（turn_radius 信任输入）
                warnings.extend(seg_warnings.iter().cloned());
            } else {
                if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                    eprintln!(
                        "[smooth-dbg] FINAL VERIFY FAIL points={} issues={} warnings={}",
                        joined.points.len(),
                        final_rep.issues.len(),
                        final_rep.warnings.len()
                    );
                    for (pi, pp) in joined.points.iter().enumerate() {
                        eprintln!(
                            "[smooth-dbg]   pt{pi}: lon={:.6} lat={:.6} alt={:.0}",
                            pp.lon, pp.lat, pp.alt_m
                        );
                    }
                    for iss in final_rep.issues.iter().take(10) {
                        eprintln!("[smooth-dbg]   final issue: {iss}");
                    }
                    for (si, seg) in smooth_segs.iter().enumerate() {
                        eprintln!("[smooth-dbg]   seg{si}: {} pts", seg.points.len());
                    }
                }
                // 终检失败 → 回退未平滑拼接（必经点保留，宁丑勿违）
                pts = raw_joined.points;
                let msg = "smoothing_failed: no smoothed stage passed full verification";
                warnings.push(msg.into());
                degradations.push(msg.into());
                warnings.extend(final_rep.warnings.iter().cloned());
                // 空洞/代价场网格伪影兜底（空洞策略 2026-08-04：可用结果 + 降级警告进
                // stats.degradations）：FMM 对 NoData 5x 代价区域绕行 → raw 是密集网格
                // 楼梯（本场景绕渤海 NoData 44km 侧偏，1138km vs 直线 784km）→ Theta*
                // 拉直被弦高门（相对 raw 100m）拒绝 → 全链失败回退楼梯。若 raw 显著长于
                // 分段直线（网格伪影而非真实绕障）且直线通过完整几何复验（不穿硬墙/不超
                // 机动/净空满足，NoData 已降级为警告）→ 交付直线（必经点保留）+ 降级警告。
                let cur_dist = Path::new(pts.clone()).length_m();
                // 不穿雷达深区（≥0.7×有效半径）才走通用兜底——穿雷达由下方雷达专用
                // 直线直穿替代处理（主管 2026-08-05 拍板语义，保持雷达行为不变）。
                let threat_ok = straight.points.iter().all(|p| {
                    threat.static_penetration(p.lon, p.lat, p.alt_m) >= 0.7
                });
                if straight.points.len() >= 2
                    && cur_dist > straight.length_m() * 1.05 + 1_000.0
                    && threat_ok
                {
                    let rep_s = crate::smooth::verify_path(
                        &straight,
                        None,
                        &opts,
                        &ctx,
                        Some(phys_min_radius_m),
                    );
                    if rep_s.ok {
                        pts = straight.points.clone();
                        // 直线替代成功 → 最终交付已平滑，撤销 smoothing_failed 误报
                        warnings.retain(|w| !w.starts_with("smoothing_failed"));
                        degradations.retain(|d| !d.starts_with("smoothing_failed"));
                        warnings.extend(rep_s.warnings.iter().cloned());
                        let msg2 = format!(
                            "raw FMM grid artifact: straight-line transit adopted (terrain NoData degraded)"
                        );
                        if !degradations.contains(&msg2) {
                            degradations.push(msg2.clone());
                        }
                        warnings.push(msg2);
                    }
                }
            }
            // 雷达 degradation：从终检 issues + 终检 warnings + 段警告提取（雷达软约束，去重）
            for i in final_rep
                .issues
                .iter()
                .chain(final_rep.warnings.iter())
                .chain(seg_warnings.iter())
            {
                if i.contains("radar") && !degradations.contains(i) {
                    degradations.push(i.clone());
                }
            }
            // 雷达避不开 → 直线直穿替代（主管 2026-08-05 锯齿问题修复）：
            // FMM 直穿雷达区时 Theta* 拒绝拉直（check 穿雷达=false）→ 交付网格锯齿；
            // 若整路径探测概率仍超阈值 且 距离显著大于直线（锯齿是网格伪影而非真实绕行）
            // → 用分段直线直穿（必经点保留，最短暴露时长）。直线需过几何复验（防穿山/超机动）。
            if let Some(tm) = ctx.threat {
                let rep_now = tm.evaluate(&Path::new(pts.clone()), ctx.terrain);
                // 直穿判定：路径某点深入任一雷达有效半径 70% 以内（与 Theta* 深探测
                // DEEP_RATIO 一致）才视为"避不开的直穿"；完全绕出（最近点 ≥ 0.7×半径）
                // 保持绕行，不替代。（P_cross 是验收阈值，不参与直穿判定——主管
                // 2026-08-06：航路必须绕开雷达探测区域，不得因调高 P_cross 而直穿。）
                let mut penetrates = false;
                for p in &pts {
                    if threat.static_penetration(p.lon, p.lat, p.alt_m) < 0.7 {
                        penetrates = true;
                        break;
                    }
                }
                if rep_now.over_threshold && penetrates {
                    let cur_dist = Path::new(pts.clone()).length_m();
                    if straight.points.len() >= 2
                        && cur_dist > straight.length_m() * 1.05 + 1_000.0
                    {
                        let rep_s = crate::smooth::verify_path(
                            &straight,
                            None,
                            &opts,
                            &ctx,
                            Some(phys_min_radius_m),
                        );
                        if rep_s.ok {
                            pts = straight.points.clone();
                            // 直线替代成功 → 最终交付已平滑，撤销 smoothing_failed 误报
                            warnings.retain(|w| !w.starts_with("smoothing_failed"));
                            degradations.retain(|d| !d.starts_with("smoothing_failed"));
                            let msg = format!(
                                "radar: unavoidable crossing -> straight-line transit (p {:.4})",
                                rep_now.cumulative_p
                            );
                            if !degradations.contains(&msg) {
                                degradations.push(msg.clone());
                            }
                            warnings.push(msg);
                        }
                    }
                }
            }
        }
        // NoData 退化汇总（空洞策略 2026-08-04：最坏降级警告进 stats.degradations）：
        // verify 对空洞只降级警告（不阻断），此处把沿途 NoData 采样汇总为一条 degradation。
        let nodata_n = warnings
            .iter()
            .filter(|w| w.contains("NoData terrain"))
            .count();
        if nodata_n > 0 {
            let msg = format!(
                "terrain: {nodata_n} NoData sample(s) along route, clearance unknown (degraded)"
            );
            if !degradations.contains(&msg) {
                degradations.push(msg);
            }
        }
        let dist = Path::new(pts.clone()).length_m();
        out_vehicles.push(VehicleOutput {
            id: v.id.clone(),
            status: "planned".into(),
            path: pts
                .iter()
                .map(|p| PathPoint {
                    x: p.lon,
                    y: p.lat,
                    alt_m: p.alt_m,
                })
                .collect(),
            distance_m: dist,
            warnings,
        });
    }

    Ok(Output {
        schema_version: crate::config::SCHEMA_VERSION.into(),
        status: "success".into(),
        error: None,
        elapsed_ms: Some(elapsed_ms),
        vehicles: out_vehicles,
        stats: Stats {
            fmm_ms,
            los_checks: 0,
            degradations,
        },
    })
}

// ==================== 辅助 ====================

/// 默认海岸掩膜候选（主管 2026-08-08：默认提供和使用的掩膜为全球版本，
/// **7.5 弧秒（mask_7p5as.mask，与默认地形 east_asia_7p5as.arpack 同分辨率）**；
/// GSHHG 全球 V2 3 态，覆盖 360°×180°，86400×172800，30.8MB）。
/// 候选：exe 同目录 / 工作目录 data/（2026-08-08 数据迁移到项目根 data/）；
/// 未找到 → None（纯地形，无掩膜分层）。
/// 区域窗口掩膜（east_asia_7p5as.mask 等）不自动探测——用户可显式 terrain.mask_path 指定。
fn default_mask_candidates() -> Option<PathBuf> {
    let mut candidates = vec![
        PathBuf::from("mask_7p5as.mask"),
        PathBuf::from("data/mask_7p5as.mask"),
    ];
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            candidates.insert(0, dir.join("mask_7p5as.mask"));
            for anc in dir.ancestors().skip(1).take(3) {
                candidates.push(anc.join("data/mask_7p5as.mask"));
            }
        }
    }
    candidates.into_iter().find(|c| c.exists())
}

/// 解析每机目标引用（Demo 每机独立终点，主管 2026-08-10）：
/// 缺省 / "mission.target" → mission.target；"lon,lat[,alt]" → 自定义坐标
/// （alt 解析但当前仅水平语义，与 M5 mid_waypoints 高度一致）；其他 → 未识别
/// 引用硬拒（InputInvalid）。
fn resolve_target_ref(r: Option<&str>, mission_target: &Geo) -> Result<Geo, AppError> {
    let Some(s) = r.map(str::trim).filter(|s| !s.is_empty()) else {
        return Ok(*mission_target);
    };
    if s == "mission.target" {
        return Ok(*mission_target);
    }
    let parts: Vec<&str> = s.split(',').map(str::trim).collect();
    if parts.len() == 2 || parts.len() == 3 {
        if let (Ok(lon), Ok(lat)) = (parts[0].parse::<f64>(), parts[1].parse::<f64>()) {
            return Geo::new(lon, lat)
                .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate));
        }
    }
    Err(AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))
}

/// 任务区域：所有起点 + 每机目标 + mission.target 的方形包围盒 + 0.15° 缓冲
/// （保证源/目标不贴边）。
fn region_of(specs: &[VehicleSpec], target: &Geo) -> Region {
    let mut min_lon = f64::INFINITY;
    let mut max_lon = f64::NEG_INFINITY;
    let mut min_lat = f64::INFINITY;
    let mut max_lat = f64::NEG_INFINITY;
    for s in specs {
        min_lon = min_lon.min(s.start.lon);
        max_lon = max_lon.max(s.start.lon);
        min_lat = min_lat.min(s.start.lat);
        max_lat = max_lat.max(s.start.lat);
        min_lon = min_lon.min(s.target.lon);
        max_lon = max_lon.max(s.target.lon);
        min_lat = min_lat.min(s.target.lat);
        max_lat = max_lat.max(s.target.lat);
    }
    min_lon = min_lon.min(target.lon);
    max_lon = max_lon.max(target.lon);
    min_lat = min_lat.min(target.lat);
    max_lat = max_lat.max(target.lat);
    let pad = 0.15;
    min_lon -= pad;
    min_lat -= pad;
    let span = (max_lon - min_lon).max(max_lat - min_lat) + 2.0 * pad;
    Region {
        min_lon,
        min_lat,
        span_deg: span,
    }
}

fn cell_lonlat(r: usize, c: usize, region: &Region, grid: usize) -> (f64, f64) {
    let u = (c as f64 + 0.5) / grid as f64;
    let v = (r as f64 + 0.5) / grid as f64;
    (
        region.min_lon + u * region.span_deg,
        region.min_lat + v * region.span_deg,
    )
}

fn lonlat_cell(lon: f64, lat: f64, region: &Region, grid: usize) -> (usize, usize) {
    let c = (((lon - region.min_lon) / region.span_deg) * grid as f64) as usize;
    let r = (((lat - region.min_lat) / region.span_deg) * grid as f64) as usize;
    (r.min(grid - 1), c.min(grid - 1))
}

/// 圆形 zone → CircleIndex（smooth 复验禁飞包含用；zones 提供时 verify 不再用它）。
fn circle_index(zones: &[&Zone]) -> CircleIndex {
    let entries: Vec<CircleEntry> = zones
        .iter()
        .filter_map(|z| match &z.shape {
            ZoneShape::Circle { center, radius_km } => Some(CircleEntry {
                id: z.id.clone(),
                lon: center[0],
                lat: center[1],
                radius_m: radius_km * 1000.0,
            }),
            ZoneShape::Polygon { .. } => None,
        })
        .collect();
    CircleIndex::build(entries)
}

/// 雷达威胁参数（默认参数表落默认值；输入覆盖合并）。
/// base_p（中心探测概率）固定 0.1 占位，**与 p_cross 解耦**（主管 2026-08-05 反馈：
/// P_cross 是验收阈值——调高只放宽"容忍探测"的评估/拉直判定，不应把物理探测概率
/// 一起抬高导致代价爆炸 + 强绕行 + 锯齿；真实雷达参数标定（A2）后接入独立 base_p）。
fn radar_threat_params(d: &crate::config::DefaultParams) -> ThreatParams {
    ThreatParams {
        radar_inflation: d.radar_inflation,
        detection_curve: d.detection_curve,
        p_cross: d.p_cross,
        suppression_delta: d.suppression_delta,
        base_p: 0.1, // 解耦：固定占位（A2 标定后接入独立 base_p）
    }
}

/// 无效参数回落默认的降级报告（主管决策 2026-08-05：无外部参数或参数无效使用默认值）。
/// merge 已回落；此处把"输入无效"事实记入 stats.degradations 供验收可见。
fn radar_param_degradations(input: &Input, out: &mut Vec<String>) {
    let p = &input.mission.parameters;
    if let Some(v) = p.radar_inflation
        && !(v.is_finite() && v > 1.0)
    {
        out.push(format!("parameter radar_inflation={v} invalid -> default 1.2"));
    }
    if let Some(v) = p.p_cross
        && !(v.is_finite() && v >= 0.0 && v <= 1.0)
    {
        out.push(format!("parameter p_cross={v} invalid -> default 0.1"));
    }
    if let Some(v) = p.suppression_delta
        && !(v.is_finite() && v >= 0.0 && v < 1.0)
    {
        out.push(format!("parameter suppression_delta={v} invalid -> default 0.5"));
    }
    if let Some(v) = p.radar_cost_coef
        && !(v.is_finite() && v > 0.0)
    {
        out.push(format!("parameter radar_cost_coef={v} invalid -> default 200"));
    }
    if let Some(v) = p.los_mask_coef
        && !(v.is_finite() && v >= 0.0 && v <= 1.0)
    {
        out.push(format!("parameter los_mask_coef={v} invalid -> default 0.08"));
    }
    if let Some(s) = &p.detection_curve
        && !matches!(s.to_ascii_lowercase().as_str(), "exponential" | "linear")
    {
        out.push(format!("parameter detection_curve={s} invalid -> default exponential"));
    }
}

/// 拼接多段路径（去重相邻重复点；段端点保留——必经点/目标硬约束）。
fn join_paths(segs: &[Path]) -> Path {
    let mut pts: Vec<RouterPoint> = Vec::new();
    for seg in segs {
        for p in &seg.points {
            let dup = pts
                .last()
                .map(|q: &RouterPoint| (q.lon - p.lon).abs() < 1e-12 && (q.lat - p.lat).abs() < 1e-12)
                .unwrap_or(false);
            if !dup {
                pts.push(*p);
            }
        }
    }
    Path::new(pts)
}

/// Theta* 去锯齿段检查：直连 (a)→(b) 不穿任何 Zone（几何精确判定——
/// 多边形：线段与任一边相交或端点在内；圆形：段到圆心最近距离 ≤ 半径。
/// 含边界接触（保守拒绝）。此前为等距 16 点采样，斜切多边形的线段采样点
/// 可能恰好全部落在外部 → 拉直穿过禁飞区内部（主管 2026-08-06 梯形禁飞区
/// 航路从边缘穿过）；几何判定无采样漏判。
/// 高度层（M2）：NoFly/Obstacle 全高度水平墙（相交即拒）；Restricted 相交后
/// 按段高度采样判定（高度沿线段线性插值，区间外可穿越）。
/// 雷达威胁：直连"深穿"任一雷达（归一化深度 < 0.7，即深入有效半径 70% 以内）
/// → 拒绝拉直（保住 FMM 绕行决策——P_cross 只是验收阈值，不得因调高 P_cross
/// 而把绕行弧拉直成穿雷达区的直线；主管 2026-08-06：航路必须绕开雷达探测区域）。
/// **例外**：段两端点任一已在深区（目标/必经点本身在雷达探测区内，绕不开）→
/// 允许拉直（雷达软约束由 verify 记录；无条件拒绝会让最后接近段无法拉直 →
/// 交付 FMM 网格伪影，主管 2026-08-06 37 点场景）。
/// 低概率边缘（≥0.7，即有效半径外）允许拉直 → 绕行路径可平滑。
/// 线段合法性检查（Theta* 去锯齿拉直用）。
/// Zone 水平判定：NoFly/Obstacle 全高度墙——段到 Zone 水平净距 < inflation_km 即拒绝
/// （主管 2026-08-06：绕飞太贴边→考虑飞机机动；膨胀距离按物理转弯半径 v²/(g·tanφ)
/// 的 0.5 倍（clamp [2,10]km），拉直不得贴进膨胀带，FMM 绕行留转弯空间）；
/// Restricted 保持"水平相交 + 段高度采样"（M2 高度层语义，不膨胀）。
/// 雷达威胁：直连"深穿"任一雷达（归一化深度 < 0.7，即深入有效半径 70% 以内）
/// → 拒绝拉直（保住 FMM 绕行决策——P_cross 只是验收阈值，不得因调高 P_cross
/// 而把绕行弧拉直成穿雷达区的直线；主管 2026-08-06：航路必须绕开雷达探测区域）。
/// **例外**：段两端点任一已在深区（目标/必经点本身在雷达探测区内，绕不开）→
/// 允许拉直（雷达软约束由 verify 记录；无条件拒绝会让最后接近段无法拉直 →
/// 交付 FMM 网格伪影，主管 2026-08-06 37 点场景）。
/// 低概率边缘（≥0.7，即有效半径外）允许拉直 → 绕行路径可平滑。
/// 地形净空（2026-08-10 主管输入撞山修复）：直连段沿途采样地形，
/// 任何采样点 `Land 高度 + 净空 ≥ 段高度` → 拒绝拉直（穿山拉直会交付
/// 撞山路径——FMM 楼梯绕山但 Theta* 把楼梯拉直成直线穿 2137m 峰，
/// 而 verify 固定 9 点/段采样间隔 ~30km 漏峰 → 撞山路径通过复验交付）。
/// 采样密度同 verify 口径（按段长自适应，间隔 ~1km，上限 256 点防性能退化；
/// Water/NoData/OOB 语义同 verify：水面净空从 0 起算、NoData 不硬拒、
/// OOB 拒绝）。Theta* 是 O(n²) 贪心跳点，每个候选段都查地形 → 上限
/// 256 点控制 worst case（段长 256km 已超 demo 场景量级）。
fn make_segment_check<'a>(
    zones: &'a [Zone],
    threat: Option<&'a dyn crate::threat::ThreatModel>,
    inflation_km: f64,
    terrain: Option<&'a dyn TerrainSource>,
    clearance_m: f64,
) -> impl Fn(f64, f64, f64, f64, f64, f64) -> bool + 'a {
    move |lon1, lat1, alt1, lon2, lat2, alt2| {
        const N: usize = 16;
        const DEEP_RATIO: f64 = 0.7;
        // 地形净空：段沿程采样（段高度线性插值；与 verify 同口径的 Land 判定）。
        if let Some(t) = terrain {
            let seg_len_m = crate::path::haversine_m(lon1, lat1, lon2, lat2);
            let n_t = ((seg_len_m / 1_000.0).ceil() as usize).max(2).min(256);
            for i in 0..=n_t {
                let tt = i as f64 / n_t as f64;
                let lon = lon1 + (lon2 - lon1) * tt;
                let lat = lat1 + (lat2 - lat1) * tt;
                let alt = alt1 + (alt2 - alt1) * tt;
                match t.sample_at(lon, lat) {
                    Sample::Land(h) => {
                        if alt < h + clearance_m {
                            return false;
                        }
                    }
                    Sample::Water | Sample::Lake(_) => {
                        if alt < clearance_m {
                            return false;
                        }
                    }
                    Sample::NoData => { /* 空洞不硬拒（降级警告由 verify 汇总） */ }
                    Sample::OutOfBounds => return false,
                }
            }
        }
        for z in zones {
            let clr = crate::config::zone_segment_clearance_km(lon1, lat1, lon2, lat2, z);
            if z.is_wall() {
                if clr <= 1e-9 || clr < inflation_km {
                    return false;
                }
            } else if let crate::config::ZoneShape::Circle { center, radius_km } = &z.shape {
                // restricted 圆：与 verify 完全同口径，**两层**判定都做——
                // 1) 解析二次方程得到穿圆参数区间 [t1,t2]，**区间内**采样高度
                //    （0..N 等距采样会漏掉浅穿/短弦：段擦圆边缘穿入仅 0.03 宽，
                //    16 个等距点可能全在圆外 → check 放行 verify 会拒的穿区段，
                //    2026-08-06 zigzag9 theta_star 拉直段擦过 restricted 圆）；
                // 2) 整段等距采样 + haversine 点判定（verify 层 2 同口径）——解析
                //    区间用等距投影（固定中纬 cos），点判定用 Geo::distance_m（大圆），
                //    半径 100km 边缘偏差 ~±2% 可翻转"穿/不穿"，边缘浅穿场景 check
                //    放行 verify 拒（2026-08-07 zigzag16 restricted 圆心东移）。
                if let Some((t1, t2)) = crate::smooth::segment_circle_intersect_t(
                    lon1,
                    lat1,
                    lon2,
                    lat2,
                    center[0],
                    center[1],
                    *radius_km,
                ) {
                    for i in 0..=N {
                        let t = t1 + (t2 - t1) * i as f64 / N as f64;
                        let lon = lon1 + (lon2 - lon1) * t;
                        let lat = lat1 + (lat2 - lat1) * t;
                        let alt = alt1 + (alt2 - alt1) * t;
                        if let Ok(g) = Geo::new(lon, lat) {
                            if zone_contains_at(z, &g, alt, None) {
                                return false;
                            }
                        }
                    }
                }
                // 层 2：整段等距采样（与 verify 的 sample inside zone 完全同口径）
                for i in 0..=N {
                    let t = i as f64 / N as f64;
                    let lon = lon1 + (lon2 - lon1) * t;
                    let lat = lat1 + (lat2 - lat1) * t;
                    let alt = alt1 + (alt2 - alt1) * t;
                    if let Ok(g) = Geo::new(lon, lat) {
                        if zone_contains_at(z, &g, alt, None) {
                            return false;
                        }
                    }
                }
            } else if clr <= 1e-9 {
                // restricted 多边形：净距相交（段-边相交=0）→ 高度层采样
                for i in 0..=N {
                    let t = i as f64 / N as f64;
                    let lon = lon1 + (lon2 - lon1) * t;
                    let lat = lat1 + (lat2 - lat1) * t;
                    let alt = alt1 + (alt2 - alt1) * t;
                    if let Ok(g) = Geo::new(lon, lat) {
                        if zone_contains_at(z, &g, alt, None) {
                            return false;
                        }
                    }
                }
            }
        }
        if let Some(tm) = threat {
            // 雷达：仅当**两端点都在深区外**时，直连穿深区 = 破坏 FMM 绕行决策 → 拒绝。
            // 任一端点已在深区（目标/必经点落在雷达探测区内，无法绕开，如 2026-08-06
            // 主管 37 点场景 target 距雷达 61km < 0.7×100km）→ 允许拉直——该端点深穿
            // 不可避免，拉直只简化 FMM 网格伪影，不引入新的"绕行决策破坏"；雷达是软
            // 约束，最终由 verify 记录 P_cross（此前无条件拒绝深穿 → Theta* 无法拉直
            // 最后接近段 → 交付密集网格点伪影）。
            let deep_a = tm.static_penetration(lon1, lat1, alt1) < DEEP_RATIO;
            let deep_b = tm.static_penetration(lon2, lat2, alt2) < DEEP_RATIO;
            if !deep_a && !deep_b {
                for i in 0..=N {
                    let t = i as f64 / N as f64;
                    let lon = lon1 + (lon2 - lon1) * t;
                    let lat = lat1 + (lat2 - lat1) * t;
                    let alt = alt1 + (alt2 - alt1) * t;
                    if tm.static_penetration(lon, lat, alt) < DEEP_RATIO {
                        return false;
                    }
                }
            }
        }
        true
    }
}

/// Restricted 是否按该飞行高度视为禁行墙（底部可通行语义，主管 2026-08-06）：
/// 飞行高度落在 restricted 高度区间内 → 该机 FMM 画墙绕行（否则直穿）。
fn restricted_blocks_alt(z: &Zone, alt_m: f64) -> bool {
    matches!(z.zone_type, crate::config::ZoneType::Restricted)
        && alt_m >= z.alt_min_m
        && alt_m <= z.alt_max_m
}

/// 度制近似平面距离（km）。短距离（<100km）内精度足够（剖面可行性采样/锚点用）。
/// 纬度 111.32 km/°；经度按中纬度 cos 收缩（度单位直接换算，勿再 to_radians）。
fn dist_km(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let dlat = lat2 - lat1; // 度
    let dlon = lon2 - lon1; // 度
    let mlat = ((lat1 + lat2) / 2.0).to_radians();
    let x = dlon * mlat.cos() * 111.32;
    let y = dlat * 111.32;
    (x * x + y * y).sqrt()
}

/// 受限区穿行剖面高度决策（主管 2026-08-06 二轮：比较顶部绕飞与底部穿行的代价，选更优）。
/// 仅对高度区间内的 restricted 调用；返回：
/// - Some(pass_alt)：可剖面穿行——底部（pass=alt_min−500m）与顶部（pass=alt_max+500m）
///   都评估后选更优：两者水平路径同为直线（总水平距离相同），仅垂直机动总量不同
///   （底部 2×|alt−alt_min−500|，顶部 2×|alt_max+500−alt|）→ 垂直机动更少的底部恒优；
///   底部不可行（穿行区地形挡住底部 / 爬升距离不足）→ 顶部绕飞（高于任何地形，且须
///   ≤ 升限 ceiling_m）；
/// - None：底部与顶部都不可行 → 需画墙水平绕行（fallback 保底）。
/// 多边形 restricted 的剖面暂不支持（MVP：仅圆形直穿剖面），返回 None（水平绕行）。
fn restricted_pass_alt(
    z: &Zone,
    alt_m: f64,
    ceiling_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    max_climb_deg: f64,
    raw_band: Option<(&[RouterPoint], usize, usize)>,
) -> Option<f64> {
    let ZoneShape::Circle { center, radius_km } = z.shape else {
        return None;
    };
    let bottom = z.alt_min_m - 500.0;
    let top = z.alt_max_m + 500.0;
    let climb_dist = |pass: f64| -> f64 {
        if max_climb_deg > 0.1 {
            (alt_m - pass).abs() / max_climb_deg.to_radians().tan() * 1.25
        } else {
            f64::INFINITY
        }
    };
    // start/target 到圆边界留有爬升距离（d 单位 km，climb_dist 单位 m → 统一换算）
    let d_in = dist_km(center[0], center[1], start.lon, start.lat) - radius_km;
    let d_out = dist_km(center[0], center[1], target.lon, target.lat) - radius_km;
    let fit = |pass: f64| d_in * 1000.0 >= climb_dist(pass) && d_out * 1000.0 >= climb_dist(pass);
    // 顶部绕飞可行性：爬升距离 + 升限（alt_max + 500 ≤ ceiling）
    let top_ok = fit(top) && ceiling_m.map_or(true, |c| top <= c);
    // 底部穿行可行性：爬升距离 + **底部严格低于 alt_min**（穿行高度必须在 restricted
    // 高度区间外；alt_min=0 时 bottom=-500 负高不可行——0m 仍在 [0,alt_max] 区间内且
    // 撞地形）+ 穿行带（直线穿圆段）地形 ≤ 底部 − 净空
    let bottom_ok = bottom >= 0.0
        && bottom < z.alt_min_m
        && fit(bottom)
        && bottom_terrain_ok(z, terrain, bottom, start, target, raw_band);
    match (bottom_ok, top_ok) {
        (true, _) => Some(bottom), // 底部垂直机动总量更小 → 恒更优（显式代价比较结论）
        (false, true) => Some(top), // 底部不可行 → 顶部绕飞（优于水平绕行：水平距离不增加）
        (false, false) => None,
    }
}

/// 底部通道地形可行性（主管 2026-08-06 三轮纠偏）：判据 = **直线穿行带**，不是整个圆面。
/// 穿行剖面沿 start→target 直线穿圆，飞机只经过圆内的一条**线**（穿行段），因此地形
/// 检查只采样该直线在圆内的穿行段沿线（步长 ~2.2km，取最高点）——圆面角落的高山
/// （飞机不经过）不应把底部判为不可行（圆面判据过度保守，会把"穿行带 46m、圆角落
/// 685m"的场景错误导向顶部绕飞）。
/// 无地形（平面 0m）→ 恒可行。顶部绕飞不查地形（高于任何地形）。
/// 2026-08-08 主管 zigzag21：传入 raw 穿行段（raw_band=Some）时，改采样
/// raw[i_in..=i_out] 实际穿行子段地形（FMM 绕行后的真实路径）——start→target 直线
/// 不穿圆时旧逻辑直接放行底部，但 raw 绕行后穿圆段可能过高（rz1 穿行段地形 1496m，
/// 1500m 剖面净空 4m < 100m → verify 拒 → 回退 157 点锯齿）。
fn bottom_terrain_ok(
    z: &Zone,
    terrain: Option<&dyn TerrainSource>,
    bottom: f64,
    start: &Geo,
    target: &Geo,
    raw_band: Option<(&[RouterPoint], usize, usize)>,
) -> bool {
    let Some(t) = terrain else {
        return true;
    };
    let ZoneShape::Circle { center, radius_km } = z.shape else {
        return false;
    };
    // raw 穿行段优先：底部剖面实际沿 raw 子段平飞，地形按该子段采样（含进出点）
    if let Some((raw, i_a, i_b)) = raw_band {
        let (lo, hi) = (i_a.min(i_b), i_a.max(i_b));
        let mut max_terr: Option<f64> = None;
        for k in lo..=hi {
            if let Sample::Land(h) = t.sample_at(raw[k].lon, raw[k].lat) {
                max_terr = Some(max_terr.map_or(h, |m: f64| m.max(h)));
            }
        }
        return match max_terr {
            Some(h) => h + 100.0 <= bottom, // 净空满足 → 底部可行
            None => true,                    // 穿行段无陆地（水面/无数据）→ 直穿
        };
    }
    // 平面近似：start→target 直线与圆交点（同 build_restricted_profiles 的解析二次方程）
    let mlat = ((start.lat + target.lat) / 2.0).to_radians();
    let kx = mlat.cos() * 111.32;
    let ky = 111.32;
    let dx = (target.lon - start.lon) * kx;
    let dy = (target.lat - start.lat) * ky;
    let cx = (center[0] - start.lon) * kx;
    let cy = (center[1] - start.lat) * ky;
    let a = dx * dx + dy * dy;
    if a < 1e-9 {
        return true; // start/target 重合（调用方已过滤）
    }
    let b = -2.0 * (dx * cx + dy * cy);
    let c = cx * cx + cy * cy - radius_km * radius_km;
    let disc = b * b - 4.0 * a * c;
    if disc <= 0.0 {
        return true; // 直线不穿圆
    }
    let sq = disc.sqrt();
    let (u_in, u_out) = ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a));
    let (u_in, u_out) = (u_in.max(0.0), u_out.min(1.0));
    if u_out <= u_in {
        return true; // 圆在线段端点之外（FMM 直穿不经过）
    }
    // 沿穿行段采样地形：穿行段长 ≈ 圆直径（40km），步长 ~2.2km → n ≈ 18；clamp [8,64]
    let seg_km = (u_out - u_in) * (dx * dx + dy * dy).sqrt();
    let n = ((seg_km / 2.2).round() as usize).clamp(8, 64);
    let mut max_terr: Option<f64> = None;
    for k in 0..=n {
        let u = u_in + (u_out - u_in) * (k as f64 / n as f64);
        let lon = start.lon + (target.lon - start.lon) * u;
        let lat = start.lat + (target.lat - start.lat) * u;
        if let Sample::Land(h) = t.sample_at(lon, lat) {
            max_terr = Some(max_terr.map_or(h, |m: f64| m.max(h)));
        }
    }
    match max_terr {
        Some(h) => h + 100.0 <= bottom, // 净空满足 → 底部可行
        None => true,                    // 穿行段无陆地（水面/无数据）→ 直穿
    }
}

/// 该机飞行高度落在 restricted 高度区间内时，是否必须在 FMM 层画墙**水平绕行**：
/// 高度在区间外 → 不拦截直穿；区间内 → `restricted_pass_alt` 决策：底部穿行 / 顶部
/// 绕飞（均可行时底部更优，不画墙，FMM 直穿后由 `build_restricted_profiles` 生成
/// 对应剖面）；底部与顶部都不可行 → 画墙水平绕行（fallback 保底，不产生失败路径）。
fn restricted_detour_required(
    z: &Zone,
    alt_m: f64,
    ceiling_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    max_climb_deg: f64,
) -> bool {
    if !restricted_blocks_alt(z, alt_m) {
        return false;
    }
    restricted_pass_alt(z, alt_m, ceiling_m, terrain, start, target, max_climb_deg, None).is_none()
}

/// 直线段是否穿/贴任一硬墙（NoFly/Obstacle，全高度墙）：净距 ≤ 0 或 < inflation。
/// 用于直线剖面 fallback（raw 未穿圆时用 start→target 直线参数化剖面，需直线避开硬墙）
/// 与 build 剖面段防御。与 verify 的墙判定口径一致。
fn line_hits_wall_km(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    zones: &[Zone],
    inflation_km: f64,
) -> bool {
    zones.iter().any(|z| {
        if !z.is_wall() {
            return false;
        }
        let clr = crate::config::zone_segment_clearance_km(lon1, lat1, lon2, lat2, z);
        clr <= 1e-9 || clr < inflation_km
    })
}

/// 过渡直线（desc_in / out_climb）是否穿任一 restricted 圆高度带：
/// 线段与圆求交区间内采样 8 点，线性高度插值落入 [alt_min, alt_max] 即命中。
/// 用于 build 剖面段防御——进入任何 restricted 圆时高度必须已在带外/带下
/// （2026-08-08 主管 zigzag23：rz1 的 desc1 从圆外 0.25km 处爬升，进入圆时
/// 高度 ~3008m 在带内 [1000,4000] → 拼接后终检拒 → 回退 1967 点锯齿）。
/// 与 verify 的圆/高度采样口径一致（含 segment_circle_intersect_t 的投影 slack）。
/// 退化段（desc_in 起点 = in_idx 点 → 零长度，只爬高度，如 zigzag23 desc1）
/// 求交返回 None——端点本身在圆内带内同样命中（i_desc=i_in 时 desc 起点即圆内
/// 带内点，爬升穿带内 → 非法剖面 → need_wall）。
fn line_hits_restricted_band_km(
    lon1: f64,
    lat1: f64,
    alt1: f64,
    lon2: f64,
    lat2: f64,
    alt2: f64,
    zones: &[Zone],
) -> bool {
    zones.iter().any(|z| {
        if z.zone_type != crate::config::ZoneType::Restricted {
            return false;
        }
        let crate::config::ZoneShape::Circle { center, radius_km } = z.shape else {
            return false;
        };
        // 端点本身在圆内带内（退化/零长度段覆盖）
        let in_band_at = |lon: f64, lat: f64, alt: f64| -> bool {
            match Geo::new(lon, lat) {
                Ok(g) => crate::config::zone_contains(z, &g) && alt >= z.alt_min_m && alt <= z.alt_max_m,
                Err(_) => false,
            }
        };
        if in_band_at(lon1, lat1, alt1) || in_band_at(lon2, lat2, alt2) {
            return true;
        }
        let Some((t1, t2)) =
            segment_circle_intersect_t(lon1, lat1, lon2, lat2, center[0], center[1], radius_km)
        else {
            return false;
        };
        for kk in 0..=8 {
            let tt = t1 + (t2 - t1) * kk as f64 / 8.0;
            let alt = alt1 + (alt2 - alt1) * tt;
            if alt >= z.alt_min_m && alt <= z.alt_max_m {
                return true;
            }
        }
        false
    })
}

/// 受限区穿行剖面（主管 2026-08-06 二轮+三轮架构增强）：FMM 直穿圆形 restricted
/// （只绕硬墙）后，沿 raw 路径找穿行区间（进入点 in / 穿出点 out）：
/// [首段 raw[0..=i_desc]@alt_m, desc→in 过渡直线(mask=true 跳过平滑,15°),
///  in→out raw 子段@pass_alt（绕行弧平飞，走平滑链）,
///  out→climb 过渡直线(mask=true), 尾段 raw[i_climb..]@alt_m]。
/// 过渡段跳过平滑（拉直会缩短水平距离→爬升角超 15°）；in→out 平飞段（含 no_fly
/// 与 restricted 重叠时深入圆内的绕行弧）以 pass_alt 飞行，即"先绕 no_fly 再剖面
/// 直穿 restricted"组合机动。in→out 平滑前做硬墙外扩（FMM 贴墙 clearance≈inflation，
/// 平滑内切后不足 verify 阈值）。穿行高度由 `restricted_pass_alt` 决策：
/// 底部可行恒选底部；底部被穿行带地形挡住 → 顶部绕飞；都不可行 → 已画墙绕行
/// （`restricted_detour_required`），raw 不穿它，此处自动跳过。
///
/// 返回 (切分段, 段掩码 true=跳过平滑, need_wall_fallback)。
/// 剖面段 = raw 子段（FMM 已避硬墙）+ 新构造过渡直线；过渡直线（desc_in/out_climb）
/// 可能穿硬墙（组合机动锚点伸到墙另一侧）→ need_wall=true → 画墙水平绕行兜底。
/// 2026-08-07 主管 zigzag12：rz1 顶部剖面 out→climb 直线穿 no_fly 多边形。
#[allow(clippy::too_many_arguments)]
fn build_restricted_profiles(
    seg: &Path,
    zones: &[Zone],
    alt_m: f64,
    max_climb_deg: f64,
    ceiling_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    inflation_km: f64,
) -> (Vec<Path>, Vec<bool>, bool) {
    let n = seg.points.len();
    if n < 2 {
        return (vec![seg.clone()], vec![false], false);
    }
    // 该机高度拦截的圆形 restricted（底部/顶部剖面穿行类型）
    let hits: Vec<&Zone> = zones
        .iter()
        .filter(|z| matches!(z.shape, ZoneShape::Circle { .. }) && restricted_blocks_alt(z, alt_m))
        .collect();
    if hits.is_empty() {
        return (vec![seg.clone()], vec![false], false);
    }
    // 多 restricted 处理顺序：按沿路径的穿行起点（第一次进入点）升序，先处理靠 start
    // 的圆。否则按 zones 输入顺序处理时，靠后的圆（地理上更靠 start）会被先处理的
    // 前圆切进 head，后续只搜 tail → 漏剖面（2026-08-08 主管 zigzag20：rz1（118°E）
    // 先处理把 rz2（124°E，更靠 start）锁进 head1，rz2 在 tail1 上找不到 → 跳过 →
    // head1 含 rz2 带内 3000m 点 → 平滑链 inside zone 全败 → 回退 599 点锯齿）。
    let mut ordered_hits = hits;
    ordered_hits.sort_by_key(|z| {
        for i in 0..seg.points.len().saturating_sub(1) {
            let pa = &seg.points[i];
            let pb = &seg.points[i + 1];
            let d = crate::config::zone_segment_clearance_km(pa.lon, pa.lat, pb.lon, pb.lat, z);
            let a_in = Geo::new(pa.lon, pa.lat)
                .map_or(false, |g| crate::config::zone_contains(z, &g));
            let b_in = Geo::new(pb.lon, pb.lat)
                .map_or(false, |g| crate::config::zone_contains(z, &g));
            if d <= 1e-9 || a_in || b_in {
                return i;
            }
        }
        usize::MAX // 不穿圆 → 排最后（循环内 fallback/跳过）
    });
    let (p0, p1) = (seg.points[0], *seg.points.last().unwrap());
    let mut out_segs: Vec<Path> = vec![seg.clone()];
    let mut out_mask: Vec<bool> = vec![false];
    let need_wall = false;
    // 逐个 hit（已按穿行起点升序）：在当前的尾段上找穿行区间 → 切 [首段, 剖面段, 尾段]
    for z in ordered_hits {
        let tail = out_segs.last().unwrap();
        let ZoneShape::Circle { center, radius_km } = z.shape else {
            continue;
        };
        let (cx, cy, r) = (center[0], center[1], radius_km);
        // 找进入索引（第一个与圆相交段的起点）与最后圆内点（穿出点）：
        // 逐格点 dist<r 判定会漏掉浅穿（锯齿格点全在圆外但线段穿入，如 new_rz
        // 垂距 19.5km 仅穿入 0.5km）→ 改用"段与圆相交"（最近距离 < r）判定。
        let mut in_idx: Option<usize> = None;
        let mut out_idx: Option<usize> = None;
        let mut in_circle = false;
        for i in 0..tail.points.len().saturating_sub(1) {
            let pa = &tail.points[i];
            let pb = &tail.points[i + 1];
            let d = crate::config::zone_segment_clearance_km(pa.lon, pa.lat, pb.lon, pb.lat, z);
            // 端点 inside 判定与 verify（zone_contains_at）同口径：Geo::distance_m
            // （haversine）≤ r——dist_km（平面近似）在圆边界处有 ~0.1% 偏差，
            // 边界点（如距圆心 20.02 vs 19.99km）会漏判进入 → 首段含圆内 3000m 违规。
            let a_in = Geo::new(pa.lon, pa.lat)
                .map_or(false, |g| crate::config::zone_contains(z, &g));
            let b_in = Geo::new(pb.lon, pb.lat)
                .map_or(false, |g| crate::config::zone_contains(z, &g));
            let crossing = d <= 1e-9 || a_in || b_in; // 段与圆相交/端点在内
            if crossing && !in_circle {
                // 圆凸：in_idx 只取第一次进入（raw 锯齿在圆边界摆动时可能"出圆→再进"，
                // 覆盖 in_idx 会把剖面起点推迟到最后一个进入点 → 首段含圆内 3000m 违规）
                if in_idx.is_none() {
                    in_idx = Some(i);
                }
                in_circle = true;
            }
            if in_circle {
                if b_in {
                    out_idx = Some(i + 1); // 圆内区间持续（终点更新）
                } else if !crossing {
                    // 段已完全在圆外 → 圆凸，区间结束（out = 段起点，圆外）
                    out_idx = Some(i);
                    in_circle = false;
                } else {
                    // crossing && !b_in：段从圆内穿出 → out = 段终点（圆外点）。
                    // out 必须是圆外点——否则 out→climb 爬升过渡从圆内开始，
                    // 高度 500→1000+ 进入 restricted 区间违规（verify alt band 拒）。
                    out_idx = Some(i + 1);
                }
            }
        }
        let (Some(i_in), Some(i_out)) = (in_idx, out_idx) else {
            // raw 未穿该圆：FMM 网格离散擦边（浅穿深度 < 格距，格点全在圆外）时
            // 真实几何仍穿圆 → 平滑拉直会穿圆违规（verify 几何精确拦截）→ 回退锯齿。
            // fallback：start→target 直线穿圆（且直线避开全部硬墙）→ 直线参数化剖面
            // （1b1331b 旧方案；仅 raw 未穿圆时启用，主管 2026-08-06 三轮架构保留）。
            let Some(pass_alt) =
                restricted_pass_alt(z, alt_m, ceiling_m, terrain, start, target, max_climb_deg, None)
            else {
                continue;
            };
            if line_hits_wall_km(p0.lon, p0.lat, p1.lon, p1.lat, zones, inflation_km) {
                continue; // 直线穿硬墙（如 no_fly）→ 直线剖面不可用
            }
            let mlat = ((p0.lat + p1.lat) / 2.0).to_radians();
            let kx = mlat.cos() * 111.32;
            let ky = 111.32;
            let ddx = (p1.lon - p0.lon) * kx;
            let ddy = (p1.lat - p0.lat) * ky;
            let oox = (cx - p0.lon) * kx;
            let ooy = (cy - p0.lat) * ky;
            let aa = ddx * ddx + ddy * ddy;
            let bb = -2.0 * (ddx * oox + ddy * ooy);
            let cc = oox * oox + ooy * ooy - r * r;
            let disc = bb * bb - 4.0 * aa * cc;
            if disc <= 0.0 {
                continue;
            }
            let sq = disc.sqrt();
            let u1 = ((-bb - sq) / (2.0 * aa)).clamp(0.0, 1.0);
            let u2 = ((-bb + sq) / (2.0 * aa)).clamp(0.0, 1.0);
            if u2 <= 0.0 || u1 >= 1.0 {
                continue;
            }
            let climb_base_km = if max_climb_deg > 0.1 {
                (alt_m - pass_alt).abs() / max_climb_deg.to_radians().tan() / 1000.0 * 1.1
            } else {
                f64::INFINITY
            };
            let line_len_km = aa.sqrt();
            let u_desc = (u1 - climb_base_km / line_len_km).max(0.0);
            let u_climb = (u2 + climb_base_km / line_len_km).min(1.0);
            let pt_at = |u: f64, h: f64| {
                RouterPoint::new(
                    p0.lon + (p1.lon - p0.lon) * u,
                    p0.lat + (p1.lat - p0.lat) * u,
                    h,
                )
            };
            let desc_p = pt_at(u_desc, alt_m);
            let in_p = pt_at(u1, pass_alt);
            let out_p = pt_at(u2, pass_alt);
            let climb_p = pt_at(u_climb, alt_m);
            // 全直线 5 段（跳过平滑——本身是 max_climb 直线 + 平飞弦）
            let head_l = Path::new(vec![tail.points[0], desc_p]);
            let desc_in_l = Path::new(vec![desc_p, in_p]);
            let in_out_l = Path::new(vec![in_p, out_p]);
            let out_climb_l = Path::new(vec![out_p, climb_p]);
            let tail2_l = Path::new(vec![climb_p, *tail.points.last().unwrap()]);
            out_segs.pop();
            out_mask.pop();
            for (s, m) in [
                (head_l, true),
                (desc_in_l, true),
                (in_out_l, true),
                (out_climb_l, true),
                (tail2_l, true),
            ] {
                out_segs.push(s);
                out_mask.push(m);
            }
            break; // 直线剖面覆盖整段，后续 hit 不再处理
        };
        if i_out <= i_in {
            continue;
        }
        // 穿行高度决策（底部优先 / 顶部备选），不可行 → 已画墙绕行（raw 不穿它）。
        // raw 穿行段地形参与底部判定（bottom_terrain_ok 采样 raw[i_in..=i_out]）
        let Some(pass_alt) = restricted_pass_alt(
            z,
            alt_m,
            ceiling_m,
            terrain,
            start,
            target,
            max_climb_deg,
            Some((&tail.points, i_in, i_out)),
        ) else {
            continue; // 底部/顶部都不可行 → 已画墙绕行（raw 不穿它）
        };
        // 过渡段基线水平距离（无裕量）：高差 → 15° 爬升角所需最小水平距离。
        // desc/climb 锚点判定用"直线距离 ≥ 1.1×基线"（保证过渡直线爬升角 ≤ 13.6°），
        // 且锚点沿 raw 路径前移（直线段上，距硬墙远，避免拉直后爬升角超/穿墙）。
        let climb_base_km = if max_climb_deg > 0.1 {
            (alt_m - pass_alt).abs() / max_climb_deg.to_radians().tan() / 1000.0 * 1.1
        } else {
            f64::INFINITY
        };
        // in→out 平飞段 = raw[i_in..=i_out] @pass_alt（含绕行弧），平滑前硬墙外扩
        //（FMM 贴墙 clearance≈inflation，平滑内切后不足 verify 阈值 → 外扩到
        // radius+inflation+margin；margin=0.5km 吸收平滑内切与网格离散误差）。
        let in_out_raw: Vec<RouterPoint> = tail.points[i_in..=i_out]
            .iter()
            .map(|p| RouterPoint::new(p.lon, p.lat, pass_alt))
            .collect();
        let in_out = push_out_of_walls(&in_out_raw, zones, inflation_km, 0.5);
        let pin2 = in_out[0];
        let pout2 = *in_out.last().unwrap();
        // 方向辅助（局部等距投影 heading；相邻过渡段转角 ≤ 60°——固定翼最大转角，
        // 否则 join 后 final verify turn 拒 → 回退锯齿，如 zigzag6 vertex5 64.55°）。
        let heading = |lon1: f64, lat1: f64, lon2: f64, lat2: f64| {
            let mlat = ((lat1 + lat2) / 2.0).to_radians();
            let kx = mlat.cos() * 111.32;
            let dx = (lon2 - lon1) * kx;
            let dy = (lat2 - lat1) * 111.32;
            dy.atan2(dx).to_degrees().rem_euclid(360.0)
        };
        let angle_between = |h1: f64, h2: f64| {
            let d = (h1 - h2).abs() % 360.0;
            if d > 180.0 { 360.0 - d } else { d }
        };
        let p0pt = tail.points[0];
        let plast = *tail.points.last().unwrap();
        // 点是否位于**其他** restricted 圆带内（排除自身）。2026-08-08 主管 zigzag22：
        // rz2 的 out_climb 过渡完成点选在 rz1 圆内（距圆心 41km<50km，@3000m 在 rz1
        // 带内）→ rz1 处理时 tail 起点在圆内 → in_idx=0 → head 退化成单点带内段 →
        // 平滑全败 → join 后 FINAL 在 rz1 处带内直穿 → 1967 点锯齿。desc/climb 锚点
        // 都必须避开其他 restricted 圆带内点。
        let in_other_band = |lon: f64, lat: f64| -> bool {
            zones.iter().any(|z2| {
                if std::ptr::eq(z2, z) || !matches!(z2.shape, ZoneShape::Circle { .. }) {
                    return false;
                }
                if !restricted_blocks_alt(z2, alt_m) {
                    return false;
                }
                Geo::new(lon, lat).map_or(false, |g| crate::config::zone_contains(z2, &g))
            })
        };
        // desc：沿路径从 i_in 向前，找直线距离 ≥ climb_base 且 start→desc 与 desc→in
        // 转角 ≤ 55° 的点（首段平滑为 start→desc 直线后连接 desc→in 过渡；55° 留 5°
        // 余量——检查用局部投影近似，verify 用精确投影，边界会差 ~0.3°）。
        // **不做 line_hits_wall_km 前置预检**（2026-08-08 试错回退）：预检会让
        // 候选全被拒时 i_climb 静默退化（=i_out → out_climb 零长）而非触发后置
        // need_wall 兜底，产生单点 head 坏段；后置检查（desc_line_hits/climb_line_hits
        // → need_wall 画墙绕行）才是"宁丑勿违"正确兜底。
        let mut i_desc = i_in;
        for i in (0..i_in).rev() {
            if in_other_band(tail.points[i].lon, tail.points[i].lat) {
                continue;
            }
            if dist_km(tail.points[i].lon, tail.points[i].lat, pin2.lon, pin2.lat) >= climb_base_km {
                let h1 = heading(p0pt.lon, p0pt.lat, tail.points[i].lon, tail.points[i].lat);
                let h2 = heading(tail.points[i].lon, tail.points[i].lat, pin2.lon, pin2.lat);
                if angle_between(h1, h2) <= 55.0 {
                    i_desc = i;
                    break;
                }
            }
        }
        // climb：沿路径从 i_out 向后，找直线距离 ≥ climb_base 且 out→climb 与
        // climb→target 转角 ≤ 55° 的点（尾段平滑为 climb→target 直线）
        let mut i_climb = i_out;
        for i in (i_out + 1)..tail.points.len() {
            if in_other_band(tail.points[i].lon, tail.points[i].lat) {
                continue;
            }
            if dist_km(pout2.lon, pout2.lat, tail.points[i].lon, tail.points[i].lat) >= climb_base_km {
                let h1 = heading(pout2.lon, pout2.lat, tail.points[i].lon, tail.points[i].lat);
                let h2 = heading(tail.points[i].lon, tail.points[i].lat, plast.lon, plast.lat);
                if angle_between(h1, h2) <= 55.0 {
                    i_climb = i;
                    break;
                }
            }
        }
        // 切分 5 段前：过渡直线可能穿硬墙（组合机动 out→climb 直线从 rz 出口伸到
        // 墙另一侧，如 2026-08-07 主管 zigzag12：rz1 顶部剖面 out→climb 直线
        // (119.75,37.27)→(117.16,37.30) 穿 no_fly 多边形，clearance=0 → 拼接后
        // 终检拒 → 回退 1893 点网格楼梯）。desc_in / out_climb 是新构造直线（非
        // raw 子段），必须做硬墙净距检查；任一穿墙 → need_wall → 画墙水平绕行兜底。
        let desc_line_hits = line_hits_wall_km(
            tail.points[i_desc].lon,
            tail.points[i_desc].lat,
            pin2.lon,
            pin2.lat,
            zones,
            inflation_km,
        );
        let climb_line_hits = line_hits_wall_km(
            pout2.lon,
            pout2.lat,
            tail.points[i_climb].lon,
            tail.points[i_climb].lat,
            zones,
            inflation_km,
        );
        // 过渡直线穿 **restricted 圆高度带** 同样 need_wall（2026-08-08 主管
        // zigzag23：rz2 圆心南移后 out_climb 不穿 no_fly → 不触发硬墙兜底 →
        // rz1 的 desc1 从圆外 0.25km 处爬升，进入 rz1 圆时高度 ~3008m 在带内
        // [1000,4000] → FINAL verify 拒 → 全链回退 1967 点锯齿）。desc_in /
        // out_climb 是新构造直线，其进入任何 restricted 圆时高度必须已在带外
        // （或降穿到带下）；区间内采样 8 点高度插值 ∈ [alt_min, alt_max] 即拒。
        let desc_band_hits = line_hits_restricted_band_km(
            tail.points[i_desc].lon,
            tail.points[i_desc].lat,
            alt_m,
            pin2.lon,
            pin2.lat,
            pin2.alt_m,
            zones,
        );
        let climb_band_hits = line_hits_restricted_band_km(
            pout2.lon,
            pout2.lat,
            pout2.alt_m,
            tail.points[i_climb].lon,
            tail.points[i_climb].lat,
            alt_m,
            zones,
        );
        if desc_line_hits || climb_line_hits || desc_band_hits || climb_band_hits {
            return (vec![seg.clone()], vec![false], true);
        }
        // 过渡直线水平距离不足（< climb_base → 爬升角超 15°）：desc/climb 锚点
        // 搜索失败时 i_desc/i_climb 退化为 i_in/i_out（如 zigzag23 rz1：raw 在圆外
        // 仅 0.05km，climb_base 7.5km 内无合法锚点 → desc1 零长度垂直爬升 3000→4500）。
        // 零长度段本身在圆外不违例，但 join 去重（坐标相同）丢失 desc1 与 in_out 起点
        // → pt(3000 圆外)→pt(4500 圆内) 40km 大爬升穿 restricted 带内 → final verify 拒
        // → 回退 1967 点锯齿。过渡段必须有足够水平爬升距离，否则 need_wall 画墙兜底。
        let desc_len_km = dist_km(
            tail.points[i_desc].lon,
            tail.points[i_desc].lat,
            pin2.lon,
            pin2.lat,
        );
        let climb_len_km = dist_km(
            pout2.lon,
            pout2.lat,
            tail.points[i_climb].lon,
            tail.points[i_climb].lat,
        );
        if desc_len_km < climb_base_km || climb_len_km < climb_base_km {
            return (vec![seg.clone()], vec![false], true);
        }
        // 切分 5 段：
        //  [首段 raw[0..=i_desc]@alt_m, 过渡直线 desc→in(mask=true 跳过平滑,15°),
        //   in→out raw 子段@pass_alt（绕行弧平飞，走平滑链）,
        //   过渡直线 out→climb(mask=true), 尾段 raw[i_climb..]@alt_m]
        let head = Path::new(tail.points[..=i_desc].to_vec());
        let desc_in = Path::new(vec![
            RouterPoint::new(tail.points[i_desc].lon, tail.points[i_desc].lat, alt_m),
            pin2,
        ]);
        let out_climb = Path::new(vec![
            pout2,
            RouterPoint::new(tail.points[i_climb].lon, tail.points[i_climb].lat, alt_m),
        ]);
        let tail2 = Path::new(tail.points[i_climb..].to_vec());
        // 替换尾段为 [首段, desc→in, in→out, out→climb, 新尾段]（继续处理下一个 hit）
        out_segs.pop();
        out_mask.pop();
        out_segs.push(head);
        out_mask.push(false);
        out_segs.push(desc_in);
        out_mask.push(true);
        out_segs.push(Path::new(in_out));
        out_mask.push(false);
        out_segs.push(out_climb);
        out_mask.push(true);
        out_segs.push(tail2);
        out_mask.push(false);
    }
    (out_segs, out_mask, need_wall)
}

/// 逐点把路径外扩到所有硬墙（NoFly/Obstacle）安全距离外：
/// 距圆墙圆心 < radius+inflation+margin → 沿径向（远离圆心）外移到该距离。
/// 用于 in→out 平飞段（FMM 贴墙绕行 clearance≈inflation，平滑内切后不足 verify
/// 阈值；margin 吸收平滑内切与 FMM 网格离散误差）。多边形墙当前不处理（沿用 raw）。
fn push_out_of_walls(
    pts: &[RouterPoint],
    zones: &[Zone],
    inflation_km: f64,
    margin_km: f64,
) -> Vec<RouterPoint> {
    pts.iter()
        .map(|p| {
            let (mut lon, mut lat) = (p.lon, p.lat);
            for z in zones {
                if !z.is_wall() {
                    continue;
                }
                if let ZoneShape::Circle { center, radius_km } = &z.shape {
                    let (cx, cy) = (center[0], center[1]);
                    let d = dist_km(lon, lat, cx, cy);
                    let target = radius_km + inflation_km + margin_km;
                    if d < target {
                        if d < 1e-6 {
                            // 极端：点与圆心重合 → 沿经度方向外移
                            lon = cx + target / 111.32;
                            continue;
                        }
                        let f = target / d;
                        lon = cx + (lon - cx) * f;
                        lat = cy + (lat - cy) * f;
                    }
                }
            }
            RouterPoint::new(lon, lat, p.alt_m)
        })
        .collect()
}


/// 禁飞区墙膨胀 + 过渡带软罚（5c + 5c2）：
/// - 5c：NoFly/Obstacle 硬墙向外膨胀 inflation_cells 格（考虑飞机机动留转弯空间，
///   主管 2026-08-06：绕飞太贴边→考虑飞机机动——绕行需留物理转弯空间）；
/// - 5c2：膨胀墙外过渡带内代价渐变递增（墙边 ×1.5，带外 ×1），FMM 权衡代价后自然
///   走离墙更远的栅格，拉直后 clearance 余量充足（防贴墙锯齿，主管 2026-08-05
///   双禁飞区场景实测）。**软罚带按物理距离 1.5km（格数随 cell_m 自适应）**——
///   固定 2 格在细网格（自适应大 region，cell<0.75km）时物理宽度变窄 → FMM 贴墙
///   更近 → 绕行 clearance 余量不足 → 平滑内切后 verify 拒（主管 2026-08-06
///   双大雷达+多边形 no_fly 场景实测 clearance 0.00 < inflation）。窄缝（7.8km）
///   仍 > 2×1.5km，不挤缝（band=3 格/0.75km=2.25km 物理才挤，已弃）。
fn apply_inflation_and_band(
    field: &mut crate::costfield::CostField,
    inflation_cells: usize,
    cell_m: f64,
) {
    let grid = field.rows.max(field.cols);
    // 5c. 膨胀（栅格级多轮 8 邻域扩散）
    if inflation_cells > 0 {
        let mut expanded: Vec<bool> = (0..grid * grid)
            .map(|i| !field.cost[i].is_finite())
            .collect();
        for _ in 0..inflation_cells {
            let cur = expanded.clone();
            for r in 0..grid {
                for c in 0..grid {
                    let idx = r * grid + c;
                    if cur[idx] {
                        continue;
                    }
                    let mut near = false;
                    for dr in -1i32..=1 {
                        for dc in -1i32..=1 {
                            if dr == 0 && dc == 0 {
                                continue;
                            }
                            let nr = r as i32 + dr;
                            let nc = c as i32 + dc;
                            if nr >= 0
                                && nr < grid as i32
                                && nc >= 0
                                && nc < grid as i32
                                && cur[nr as usize * grid + nc as usize]
                            {
                                near = true;
                                break;
                            }
                        }
                        if near {
                            break;
                        }
                    }
                    if near {
                        expanded[idx] = true;
                    }
                }
            }
        }
        for i in 0..grid * grid {
            if expanded[i] {
                field.cost[i] = f32::INFINITY;
            }
        }
    }
    // 5c2. 过渡带软罚（BFS 距离变换，8 邻域，源 = 当前 INF 墙；物理带 1.5km）
    {
        use std::collections::VecDeque;
        const BAND_M: f64 = 1500.0;
        const BAND_COEF: f32 = 0.5;
        let band_cells = (BAND_M / cell_m.max(1.0)).ceil() as u32;
        let mut dist = vec![u32::MAX; grid * grid];
        let mut q = VecDeque::new();
        for i in 0..grid * grid {
            if !field.cost[i].is_finite() {
                dist[i] = 0;
                q.push_back(i);
            }
        }
        while let Some(idx) = q.pop_front() {
            let r = idx / grid;
            let c = idx % grid;
            let nd = dist[idx] + 1;
            for (dr, dc) in [
                (0i32, 1i32),
                (0, -1),
                (1, 0),
                (-1, 0),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ] {
                let nr = r as i32 + dr;
                let nc = c as i32 + dc;
                if nr >= 0 && nr < grid as i32 && nc >= 0 && nc < grid as i32 {
                    let ni = nr as usize * grid + nc as usize;
                    if dist[ni] > nd {
                        dist[ni] = nd;
                        q.push_back(ni);
                    }
                }
            }
        }
        for i in 0..grid * grid {
            if !field.cost[i].is_finite() {
                continue;
            }
            let d = dist[i];
            if d >= 1 && d <= band_cells {
                let t = 1.0 - (d as f32 - 1.0) / band_cells as f32; // d=1 → t=1.0；d=band → t≈0.33
                field.cost[i] *= 1.0 + BAND_COEF * t;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Input;

    fn parse(s: &str) -> Input {
        Input::from_json_str(s).unwrap()
    }

    const BASE: &str = r#"{
        "schema_version":"0.20",
        "mission":{
            "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
            "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
            "terrain":{"source":"none"}
        }
    }"#;

    #[test]
    fn m1_end_to_end_plain() {
        let input = parse(BASE);
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        assert_eq!(out.status, "success");
        assert_eq!(out.vehicles.len(), 1);
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(v.path.len() >= 2, "path 应 ≥2 点，实际 {}", v.path.len());
        assert!(v.distance_m > 100_000.0, "北京 115E,39N → 116.5E,39.9N ≈ 150km，实际 {}", v.distance_m);
        // 起终点在路径内（容差 0.02°≈2km——网格 256 格 1.5°≈210m/格）
        let (x0, y0) = (v.path.first().unwrap().x, v.path.first().unwrap().y);
        let (x1, y1) = (v.path.last().unwrap().x, v.path.last().unwrap().y);
        assert!((x0 - 115.0).abs() < 0.02 && (y0 - 39.0).abs() < 0.02);
        assert!((x1 - 116.5).abs() < 0.02 && (y1 - 39.9).abs() < 0.02);
    }

    #[test]
    fn m1_no_solution_when_target_blocked() {
        // 目标被巨型禁飞区完全覆盖 → 回溯失败 → no_solution
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[{"id":"wall","zone_type":"no_fly","shape":"circle",
                    "geometry":{"center":[116.5,39.9],"radius_km":30},
                    "alt_min_m":0,"alt_max_m":10000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "no_solution");
    }

    #[test]
    fn twin_zone_smooth_after_band_penalty() {
        // 双禁飞区并列（20km 圆 + 6 边形，主管 2026-08-05 真实场景无地形版）：
        // FMM 贴膨胀墙走 → Theta* 拉直 clearance 差 ~1 格 → 全链失败回退 362 点锯齿。
        // 5c2 过渡带软罚后必须平滑（≤10 点且无 smoothing_failed）。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":116.7188493150615,"lat":40.20313810412108,"alt_m":3000},
                "target":{"lon":115.44547741980215,"lat":38.63800678240428,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[
                    {"id":"c1","zone_type":"no_fly","shape":"circle",
                     "geometry":{"center":[115.90510756361434,39.1546051011457],"radius_km":20},
                     "alt_min_m":0,"alt_max_m":20000},
                    {"id":"p1","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[
                         [116.10919779988855,39.40629934778061],
                         [116.66748158411933,39.2812425371292],
                         [116.63856029155627,39.72638339128002],
                         [116.21277919960414,39.78606996932337],
                         [115.82845032806537,39.89839427431206],
                         [115.56298632103817,39.63798773661437]]},
                     "alt_min_m":0,"alt_max_m":20000}
                ]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 10,
            "双禁飞区并列应平滑（过渡带软罚），实际 {} 点（锯齿回退）",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应有 smoothing_failed，实际 {:?}",
            v.warnings
        );
    }

    #[test]
    fn bottom_pass_band_ignores_corner_mountain() {
        // 主管 2026-08-06 三轮纠偏：判据 = 直线穿行带（飞机实际走的线），不是整个圆面。
        // 圆西侧角落 700m 高山（飞机穿行不经过），穿行带地形 50m → 底部 1500m 应可行。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.14959340327005, 39.597263409766285],
                radius_km: 20.0,
            },
            alt_min_m: 2000.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target = Geo::new(115.28680713092322, 39.04668499383146).unwrap();
        // 地形网格：覆盖 start→target 走廊；lon < 115.95（圆西侧角落）700m，其余 50m
        let rows = 20usize;
        let cols = 24usize;
        let mut h = vec![0.0f32; rows * cols];
        for r in 0..rows {
            for c in 0..cols {
                let lon = 115.55 + 0.05 * c as f64;
                h[r * cols + c] = if lon < 115.95 { 700.0 } else { 50.0 };
            }
        }
        let terr = crate::terrain::memory::Terrain {
            rows,
            cols,
            origin_lon: 115.55,
            origin_lat: 39.30,
            cell_lon_deg: 0.05,
            cell_lat_deg: 0.05,
            h,
        };
        let pass = restricted_pass_alt(&z, 3000.0, None, Some(&terr), &start, &target, 15.0, None);
        assert_eq!(
            pass,
            Some(1500.0),
            "穿行带 50m → 底部 1500m 可行；圆角落 700m 山不应参与判据"
        );
    }

    #[test]
    fn restricted_pass_alt_top_flyover_vs_bottom_descent() {
        // 主管 2026-08-06 二轮：比较顶部绕飞与底部穿行代价选更优。
        // 圆 [2000,5000]msl，巡航 3000m；start/target 距圆 > 爬升距离。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.14959340327005, 39.597263409766285],
                radius_km: 20.0,
            },
            alt_min_m: 2000.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target = Geo::new(115.28680713092322, 39.04668499383146).unwrap();
        // 无地形 → 底部 1500m 可行（垂直机动少 → 恒优）
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, None, None, &start, &target, 15.0, None),
            Some(1500.0)
        );
        // 穿行区地形 1450m：底部 1500 − 净空100 = 1400 < 1450 → 底部不可行 → 顶部绕飞 5500m
        let terr = crate::terrain::memory::Terrain {
            rows: 10,
            cols: 10,
            origin_lon: 115.9,
            origin_lat: 39.3,
            cell_lon_deg: 0.05,
            cell_lat_deg: 0.05,
            h: vec![1450.0f32; 100],
        };
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, None, Some(&terr), &start, &target, 15.0, None),
            Some(5500.0)
        );
        // 升限 5000m → 顶部 5500 超升限且底部被地形挡 → 两者不可行 → None（画墙绕行）
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, Some(5000.0), Some(&terr), &start, &target, 15.0, None),
            None
        );
        // 巡航 1500m（区间外底部通道）→ 不拦截（restricted_blocks_alt false）
        assert!(!restricted_blocks_alt(&z, 1500.0));
    }

    #[test]
    fn restricted_pass_alt_zero_alt_min_forces_top() {
        // 主管 2026-08-06 场景：restricted [0,5000]msl → bottom = -500（负高）不可行
        //（0m 仍在 [0,5000] 区间内且撞地形）→ 必须顶部绕飞 5500m（而非 0m 剖面——
        // 0m 穿行被 verify alt band + 地形净空拒绝 → 回退锯齿）。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [115.1103270025858, 39.570299948815645],
                radius_km: 20.0,
            },
            alt_min_m: 0.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target = Geo::new(113.93832638409175, 38.5937625849369).unwrap();
        // 无地形：底部 -500 负高 → 不可行 → 顶部 5500（不能返回 0m）
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, None, None, &start, &target, 15.0, None),
            Some(5500.0)
        );
    }

    #[test]
    fn build_restricted_profiles_top_flyover_profile() {
        // 主管 2026-08-06 二轮：底部被 1450m 地形挡住 → 剖面应为顶部绕飞（5500m 平飞穿行）。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.14959340327005, 39.597263409766285],
                radius_km: 20.0,
            },
            alt_min_m: 2000.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start_geo = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target_geo = Geo::new(115.28680713092322, 39.04668499383146).unwrap();
        let start = RouterPoint::new(start_geo.lon, start_geo.lat, 3000.0);
        let target = RouterPoint::new(target_geo.lon, target_geo.lat, 3000.0);
        let pts: Vec<RouterPoint> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                RouterPoint::new(
                    start.lon + (target.lon - start.lon) * t,
                    start.lat + (target.lat - start.lat) * t,
                    3000.0,
                )
            })
            .collect();
        let seg = Path::new(pts);
        let terr = crate::terrain::memory::Terrain {
            rows: 10,
            cols: 10,
            origin_lon: 115.9,
            origin_lat: 39.3,
            cell_lon_deg: 0.05,
            cell_lat_deg: 0.05,
            h: vec![1450.0f32; 100],
        };
        let (segs, mask, _nw) = build_restricted_profiles(
            &seg,
            &[z.clone()],
            3000.0,
            15.0,
            None,
            Some(&terr),
            &start_geo,
            &target_geo,
            0.0, // 无硬墙 → 外扩不生效
        );
        assert_eq!(segs.len(), 5, "应切 [首段, desc→in, in→out, out→climb, 尾段]");
        assert_eq!(mask, vec![false, true, false, true, false]);
        // 平飞段 = segs[2]（in→out）：穿行高度 = alt_max + 500 = 5500
        let prof = &segs[2].points;
        assert!(
            prof.iter().any(|p| (p.alt_m - 5500.0).abs() < 1.0),
            "顶部平飞高度应为 5500m，实际 {:?}",
            prof.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
        assert!(
            prof.iter().all(|p| (p.alt_m - 5500.0).abs() < 1.0),
            "平飞段应全程 5500m，实际 {:?}",
            prof.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
        // 过渡段（segs[1] desc→in / segs[3] out→climb）：2 点直线，端点高度 3000/5500
        assert_eq!(segs[1].points.len(), 2, "desc→in 过渡应为 2 点直线");
        assert_eq!(segs[3].points.len(), 2, "out→climb 过渡应为 2 点直线");
        assert!(
            segs[1].points.iter().all(|p| p.alt_m == 3000.0 || (p.alt_m - 5500.0).abs() < 1.0),
            "desc→in 端点高度应为 3000/5500，实际 {:?}",
            segs[1].points.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
    }

    #[test]
    fn restricted_zone_height_layer_blocks_or_passes() {
        // 底部可通行的限飞区（主管 2026-08-06）：restricted 圆 [2000,5000]msl 挡在
        // start→target 直线上。3000m 飞行高度在区间内、底部通道地形可行 → **降高剖面
        // 直穿**（下降→底部平飞→爬升，不水平绕路——航路允许变高度）；
        // 1500m 在区间外（底部通道）→ 直穿（不绕行）。
        let mk = |alt: f64| {
            format!(
                r#"{{
                    "schema_version":"0.20",
                    "mission":{{
                        "start":{{"lon":116.82168446499925,"lat":40.23810827713887,"alt_m":{alt}}},
                        "target":{{"lon":115.28680713092322,"lat":39.04668499383146,"alt_m":{alt}}},
                        "terrain":{{"source":"none"}},
                        "restricted_zones":[{{"id":"rz","zone_type":"restricted","shape":"circle",
                            "geometry":{{"center":[116.14959340327005,39.597263409766285],"radius_km":20}},
                            "alt_min_m":2000,"alt_max_m":5000}}]
                    }}
                }}"#
            )
        };
        // 3000m：区间内 + 底部可行 → 降高剖面直穿（不再水平绕行；不得回退密集锯齿）
        let input = parse(&mk(3000.0));
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 10,
            "3000m 剖面直穿应 ≈6 点（desc/in/out/climb + 端点），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.distance_m < 190_000.0,
            "3000m 底部剖面直穿距离应 ≈187km（不绕行），实际 {}km",
            v.distance_m / 1000.0
        );
        assert!(
            v.path.iter().any(|p| p.alt_m < 2000.0),
            "3000m 剖面应在 restricted 穿行区降高到 <2000m，实际最小高度 {}m",
            v.path.iter().map(|p| p.alt_m).fold(f64::MAX, f64::min)
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "3000m 不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 1500m：区间外（底部通道）→ 直穿，≤3 点且距离 ≈ 直线
        let input = parse(&mk(1500.0));
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 3,
            "1500m 底部可通行应直穿，实际 {} 点",
            v.path.len()
        );
        assert!(
            v.distance_m < 192_000.0,
            "1500m 直穿距离应 ≈187km，实际 {}km",
            v.distance_m / 1000.0
        );
    }

    #[test]
    fn segment_check_geometry_catches_diagonal_polygon_crossing() {
        // 梯形禁飞区（主管 2026-08-06 场景）：直线斜切穿内部（16 点采样会漏——
        // 几何判定必须拒绝）；绕行折线（先下后右）必须放行。
        use crate::config::{HeightSemantics, ZoneShape, ZoneType};
        let z = Zone {
            id: "trap".into(),
            zone_type: ZoneType::NoFly,
            shape: ZoneShape::Polygon {
                vertices: vec![
                    [116.2, 39.9],
                    [116.5, 39.9],
                    [116.5, 40.2],
                    [116.35, 40.2],
                ],
            },
            alt_min_m: 0.0,
            alt_max_m: 12000.0,
            height_semantics: HeightSemantics::Msl,
        };
        let zones = vec![z];
        let check = make_segment_check(&zones, None, 0.0, None, 0.0);
        // 斜切直线：start → 接近 target 的直线，穿过梯形内部 → 拒绝拉直
        assert!(!check(115.9, 39.8, 3000.0, 116.48, 40.3, 3000.0));
        // 绕行折线两段：先向下绕过梯形下边（y<39.9），再从右侧上行（x>116.5）→ 放行
        assert!(check(115.9, 39.8, 3000.0, 116.55, 39.85, 3000.0));
        assert!(check(116.55, 39.85, 3000.0, 116.8, 40.3, 3000.0));
        // 机动膨胀（主管 2026-08-06：绕飞太贴边→考虑飞机机动）：贴边绕行段
        // （距下边 ~5.5km < 膨胀 6km）被拒；远离段放行。
        let check_infl = make_segment_check(&zones, None, 6.0, None, 0.0);
        assert!(
            !check_infl(115.9, 39.8, 3000.0, 116.55, 39.85, 3000.0),
            "绕行段距梯形下边 < 膨胀 6km 应被拒绝（留转弯空间）"
        );
        assert!(
            check_infl(115.0, 39.0, 3000.0, 117.0, 39.5, 3000.0),
            "远离段应放行"
        );
    }

    #[test]
    fn segment_check_restricted_shallow_graze_rejected() {
        // zigzag9 根因（2026-08-06）：theta_star 拉直段"擦过" restricted 圆边缘——
        // 段-圆相交区间仅 ~0.03 宽（t∈[0.592,0.622]），16 点等距采样可能全在圆外
        // → 旧 check（净距 clr≤1e-9 或等距采样）放行 verify 会拒的穿区段。
        // 修复：check 与 verify 同口径（解析二次方程 + [t1,t2] 区间内采样）。
        use crate::config::{HeightSemantics, ZoneShape, ZoneType};
        let z = Zone {
            id: "rz".into(),
            zone_type: ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.27050736818683, 41.08978345198258],
                radius_km: 50.0,
            },
            alt_min_m: 0.0,
            alt_max_m: 5000.0,
            height_semantics: HeightSemantics::Msl,
        };
        let zones = vec![z];
        let check = make_segment_check(&zones, None, 0.0, None, 0.0);
        // zigzag9 theta_star 拉直段 start→(114.2076,42.3648) 擦过 rz1 圆边缘
        // （浅穿）→ 高度 3000 在带内 → 必须拒绝拉直
        assert!(
            !check(
                118.28982699875671,
                38.42208802408725,
                3000.0,
                114.2076,
                42.3648,
                3000.0
            ),
            "擦边浅穿 restricted 圆必须拒绝（与 verify 同口径）"
        );
        // 同段但高度 6000（带外）→ 放行
        assert!(check(
            118.28982699875671,
            38.42208802408725,
            6000.0,
            114.2076,
            42.3648,
            6000.0
        ));
    }

    #[test]
    fn m1_detours_around_zone() {        // 挡路禁飞区（圆心在中点）→ 路径绕行（折线长度 > 直线）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[{"id":"mid","zone_type":"no_fly","shape":"circle",
                    "geometry":{"center":[115.75,39.45],"radius_km":25},
                    "alt_min_m":0,"alt_max_m":10000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let v = &out.vehicles[0];
        // 直线 ≈ 153km；绕行应更长（25km 半径圆挡在中点）
        assert!(v.distance_m > 155_000.0, "应绕行，距离 {}", v.distance_m);
    }

    #[test]
    fn m2_restricted_band_does_not_wall() {
        // Restricted 高度层 [0, 2000]m：巡航 3000m 在区间外 → 可穿越（直达，不绕行）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "restricted_zones":[{"id":"band","zone_type":"restricted","shape":"circle",
                    "geometry":{"center":[115.75,39.45],"radius_km":25},
                    "alt_min_m":0,"alt_max_m":2000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 高度区间外 → 路径可直穿（Theta* 截直）→ 距离 ≈ 直线（≈164km），远小于绕行
        let d = out.vehicles[0].distance_m;
        assert!(d < 167_000.0, "应直穿 restricted 高度层，距离 {}", d);
    }

    #[test]
    fn m2_nofly_wall_blocks_regardless_of_altitude() {
        // NoFly 同位置：全高度墙 → 绕行（距离显著大于直线）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[{"id":"wall","zone_type":"no_fly","shape":"circle",
                    "geometry":{"center":[115.75,39.45],"radius_km":25},
                    "alt_min_m":0,"alt_max_m":2000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let d = out.vehicles[0].distance_m;
        assert!(d > 160_000.0, "NoFly 应绕行，距离 {}", d);
    }

    #[test]
    fn m3_radar_threat_detour_and_warning() {
        // 场景 A：大雷达（40km）挡在中点。radar_cost_coef=50（占位）→ 高概率区代价显著
        // （中心 ×6）→ FMM 明确绕行躲避（主管 2026-08-05：探测概率应明显影响航路规划）。
        // 绕行后最近点应完全绕出有效半径（48km），累计探测概率应低于阈值。
        let big = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "red_forces":{"radars":[{"id":"r1","lon":115.75,"lat":39.45,"radar_type":"tracking","radius_km":40}]}
            }
        }"#;
        let out = solve(&parse(big), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 绕行：显著大于直线 164km（不得回到直穿 <170km）
        let d = out.vehicles[0].distance_m;
        assert!(d > 170_000.0, "40km 雷达应绕行躲避，距离 {}", d);
        // 绕行平滑（Theta* 转角受限 + Catmull-Rom）：不得交付网格锯齿（>50 点）
        assert!(out.vehicles[0].path.len() <= 50, "绕行应平滑，点数 {}", out.vehicles[0].path.len());
        // 绕行成功 → 累计探测概率低于阈值（不超 P_cross=0.1）
        let over = out.vehicles[0]
            .warnings
            .iter()
            .chain(out.stats.degradations.iter())
            .any(|s| s.contains("radar: cumulative detection p") && s.contains("> threshold"));
        assert!(!over, "绕行后探测概率应 <0.1: warnings={:?} degradations={:?}",
            out.vehicles[0].warnings, out.stats.degradations);
    }

    #[test]
    fn m3_target_in_radar_deep_zone_straightens_approach() {
        // 主管 2026-08-06 37 点场景：target 落在雷达深区（距圆心 61km < 0.7×100km），
        // Theta* check 无条件拒绝"直连穿深区"→ 最后接近段无法拉直 → 交付 FMM 网格
        // 伪影（33 点共线等距，总路径 37 点）。修复：段两端点任一已在深区（目标/必经
        // 点本身在雷达探测区内，绕不开）→ 允许拉直；雷达软约束由 verify 记录 P_cross。
        // 本用例：target 在 100km 雷达深区内，start 在深区外 → 路径应平滑（无网格伪影）。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":118.0,"lat":42.0,"alt_m":3000},
                "target":{"lon":111.26,"lat":43.83,"alt_m":3000},
                "terrain":{"source":"none"},
                "red_forces":{"radars":[{"id":"r1","lon":111.0,"lat":44.0,"radar_type":"early_warning","radius_km":100}]}
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 平滑：接近段必须拉直（修复前 Theta* 拒深区直连 → 数十~上百点网格伪影）
        assert!(
            out.vehicles[0].path.len() <= 12,
            "深区 target 接近段应拉直平滑，点数 {}",
            out.vehicles[0].path.len()
        );
        assert!(
            !out.vehicles[0].warnings.iter().any(|w| w.contains("smoothing_failed")),
            "不应 smoothing_failed: {:?}",
            out.vehicles[0].warnings
        );
    }

    #[test]
    fn m3_small_radar_crossing_reports_probability() {
        // 场景 B：小雷达（5km）代价极小 → FMM 微绕（≈直线，<170km）或直穿。
        // 微绕时探测概率≈0 可不报告；直穿时必须报告。A2 未标定（见上）。
        let small = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "red_forces":{"radars":[{"id":"r1","lon":115.75,"lat":39.45,"radar_type":"tracking","radius_km":5}]}
            }
        }"#;
        let out = solve(&parse(small), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let d = out.vehicles[0].distance_m;
        assert!(d < 170_000.0, "小雷达应直穿或微绕，距离 {}", d);
        let radar_reported = out.vehicles[0]
            .warnings
            .iter()
            .chain(out.stats.degradations.iter())
            .any(|s| s.contains("radar"));
        // 若接近直线（直穿）则必须报告；微绕（>+1.5km）可免
        if d < 165_500.0 {
            assert!(radar_reported, "直穿应报告雷达概率: warnings={:?} degradations={:?}",
                out.vehicles[0].warnings, out.stats.degradations);
        }
    }

    #[test]
    fn m5_mid_waypoint_passes_through() {
        // 单机 mid_waypoints：路径应经过必经点附近（分段 FMM 拼接 + 段端点保留）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[{"lon":115.3,"lat":39.8,"alt_m":3000}]}
                ]
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 经过必经点（格距内 < 0.05°≈5km）
        let near_mid = out.vehicles[0].path.iter().any(|p| {
            (p.x - 115.3).abs() < 0.05 && (p.y - 39.8).abs() < 0.05
        });
        assert!(near_mid, "应经过必经点 (115.3,39.8): {:?}",
            out.vehicles[0].path.iter().map(|p| format!("({:.2},{:.2})", p.x, p.y)).take(6).collect::<Vec<_>>());
        // 北侧绕行 → 距离显著大于直线 164km（2026-08-07 Theta* 改大圆口径后
        // 拉直更彻底，4 点 196km；阈值 180km 保留"显著大于直线"语义）
        assert!(out.vehicles[0].distance_m > 180_000.0, "dist {}", out.vehicles[0].distance_m);
    }

    #[test]
    fn m5_per_vehicle_independent_waypoints() {
        // 多机各自 mid_waypoints（主管拍板：每机独立序列）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[{"lon":115.3,"lat":39.8,"alt_m":3000}]},
                    {"id":"uav2","profile":{"aircraft_type":"ROTORCRAFT","cruise_speed_mps":60},
                     "start_pose":{"lon":115.2,"lat":39.1,"alt_m":2000,"heading_deg":90},
                     "mid_waypoints":[{"lon":116.1,"lat":39.2,"alt_m":2000}]}
                ]
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles.len(), 2);
        for v in &out.vehicles {
            assert_eq!(v.status, "planned");
        }
        // uav1 经 (115.3,39.8)；uav2 经 (116.1,39.2)（各自必经点独立）
        let u1_near = out.vehicles[0].path.iter().any(|p| (p.x - 115.3).abs() < 0.05 && (p.y - 39.8).abs() < 0.05);
        let u2_near = out.vehicles[1].path.iter().any(|p| (p.x - 116.1).abs() < 0.05 && (p.y - 39.2).abs() < 0.05);
        assert!(u1_near, "uav1 应经过 (115.3,39.8)");
        assert!(u2_near, "uav2 应经过 (116.1,39.2)");
        // 旋翼机无 smoothing 告警
        assert!(out.vehicles[1].warnings.is_empty(), "{:?}", out.vehicles[1].warnings);
    }

    #[test]
    fn m5_multiple_mid_waypoints_sequence() {
        // 多个必经点顺序经过（三段时间拼接）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[
                        {"lon":115.3,"lat":39.8,"alt_m":3000},
                        {"lon":116.0,"lat":39.7,"alt_m":3000}
                     ]}
                ]
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let p = &out.vehicles[0].path;
        // 两个必经点都经过（格距内）
        let m1 = p.iter().position(|q| (q.x - 115.3).abs() < 0.05 && (q.y - 39.8).abs() < 0.05);
        let m2 = p.iter().position(|q| (q.x - 116.0).abs() < 0.05 && (q.y - 39.7).abs() < 0.05);
        assert!(m1.is_some() && m2.is_some(), "必经点应都经过: m1={m1:?} m2={m2:?}");
        // 顺序：m1 在 m2 前
        assert!(m1.unwrap() < m2.unwrap(), "顺序应 m1 < m2");
    }

    #[test]
    fn m1_multi_vehicle() {
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":100},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45}},
                    {"id":"uav2","profile":{"aircraft_type":"ROTORCRAFT","cruise_speed_mps":50},
                     "start_pose":{"lon":115.5,"lat":39.2,"alt_m":2000,"heading_deg":90}}
                ]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles.len(), 2);
        assert_eq!(out.vehicles[0].id, "uav1");
        assert_eq!(out.vehicles[1].id, "uav2");
        for v in &out.vehicles {
            assert_eq!(v.status, "planned");
            assert!(v.path.len() >= 2);
        }
        // 旋翼机（uav2）无 Dubins 链（急转合法）——路径点应不因拟合失败回退
        assert!(out.vehicles[1].warnings.is_empty(), "旋翼机不应有 smoothing 告警");
    }

    #[test]
    fn m1_vehicle_per_target_ref() {
        // Demo 每机独立终点（主管 2026-08-10）：target_ref = "lon,lat[,alt]" 自定义
        // 坐标 → 每机路径终点 = 各自 target；缺省 / "mission.target" → mission.target；
        // 未识别引用 → InputInvalid。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":100},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45},
                     "target_ref":"117.0,40.2,3000"},
                    {"id":"v2","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":100},
                     "start_pose":{"lon":115.5,"lat":39.5,"alt_m":3000,"heading_deg":45},
                     "target_ref":"mission.target"}
                ]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles.len(), 2);
        for v in &out.vehicles {
            assert_eq!(v.status, "planned");
        }
        // v1 自定义目标（117.0, 40.2）；v2 mission.target（116.5, 39.9）
        let last1 = out.vehicles[0].path.last().unwrap();
        let last2 = out.vehicles[1].path.last().unwrap();
        assert!(
            (last1.x - 117.0).abs() < 0.05 && (last1.y - 40.2).abs() < 0.05,
            "v1 终点应 ≈ (117.0,40.2)，实际 ({},{})",
            last1.x,
            last1.y
        );
        assert!(
            (last2.x - 116.5).abs() < 0.05 && (last2.y - 39.9).abs() < 0.05,
            "v2 终点应 ≈ mission.target (116.5,39.9)，实际 ({},{})",
            last2.x,
            last2.y
        );
        // 未识别引用 → input_invalid（非法坐标错误）
        let bad = s.replace("117.0,40.2,3000", "not-a-ref");
        let input_bad = parse(&bad);
        let out_bad = solve(&input_bad, &SolveParams::default(), 0);
        assert!(
            out_bad.is_err(),
            "未识别 target_ref 应 InputInvalid，实际 {:?}",
            out_bad
        );
    }

    #[test]
    fn zigzag19_boundary_arc_between_profile_and_cruise() {
        // 主管 2026-08-08 输入（China DEM L12 真实地形）：start 东海上 → target 内蒙，
        // 2 个 restricted 圆（rz1 顶部绕飞 6500m、rz2 底部穿行 1500m）+ 5 个 no_fly
        // + 2 雷达。build_restricted_profiles 切出 desc_in/in_out/out_climb 剖面段，
        // out_climb（mask=true 固定直线）方向只约束"out→climb vs climb→tail 终点"
        // （build 时近似），与 in_out 平滑（Theta*）后末段方向偏差大 → 段边界
        // pout2 处转角 65.9° 超 max_turn → final verify 拒 → 全链回退 1975 点
        // 网格楼梯（4048km）。修复：平滑循环中每段（含 mask=true）push 前检查
        // 与前一段输出的边界转角，超限 → arc_transition 插入过渡弧（弹出边界点、
        // 后续段起点同步到弧末点 E）→ 22 点平滑路径 2982km。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag19: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":126.56263413053458,"lat":30.32884201287228,"alt_m":3000},
                "target":{"lon":106.37660123285819,"lat":51.14912421163358,"alt_m":3000},
                "vehicles":[{"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                    "min_turn_radius_m":442,"max_climb_angle_deg":15},
                    "start_pose":{"lon":126.56263413053458,"lat":30.32884201287228,"alt_m":3000,"heading_deg":45},
                    "mid_waypoints":[]}],
                "red_forces":{"radars":[
                    {"id":"radar_1786151025411","lon":113.98758157631866,"lat":40.493383922561435,"radar_type":"early_warning","radius_km":100,"alt_m":10},
                    {"id":"radar_1786151487443","lon":109.16900472287948,"lat":46.82742249911229,"radar_type":"tracking","radius_km":100,"alt_m":10}]},
                "no_fly_zones":[
                    {"id":"zone_1786150842284","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[114.40468979438141,42.65722021983792],[110.62088557896168,38.988188308928834],[112.81262578575785,39.7164462210201]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786150865059","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[118.0683305776591,44.76739249108195],[114.57960742739121,41.61506064153377],[117.27464739502655,42.648718862161275]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786150891051","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[117.0676237684419,46.341013239879814],[113.7376472130683,43.26605614623792],[116.16387600630392,43.34838640726238]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786151204171","zone_type":"no_fly","shape":"circle","geometry":{"center":[111.525601573293,45.97116185501782],"radius_km":50},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786151327667","zone_type":"no_fly","shape":"circle","geometry":{"center":[107.23020272260999,49.498587971424286],"radius_km":100},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786150991459","zone_type":"restricted","shape":"circle","geometry":{"center":[115.86408593793746,40.956574434371106],"radius_km":100},"alt_min_m":0,"alt_max_m":6000,"height_semantics":"msl"},
                    {"id":"rz_1786151584275","zone_type":"restricted","shape":"circle","geometry":{"center":[113.31470106587476,42.70043214171731],"radius_km":100},"alt_min_m":2000,"alt_max_m":8000,"height_semantics":"msl"}],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{"p_cross":0.9}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "段边界弧修复后不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        assert!(
            v.path.len() <= 60,
            "应交付平滑路径（修复前 1975 点），实际 {} 点",
            v.path.len()
        );
        // 修复前 raw 楼梯 4048km；平滑交付应显著更短
        assert!(
            v.distance_m < 3_200_000.0,
            "应 ≈ 平滑 2982km，实际 {}km",
            v.distance_m / 1000.0
        );
        // 剖面语义保持：rz1 顶部绕飞（6500m）与 rz2 底部穿行（1500m）都应在路径中
        let has_6500 = v.path.iter().any(|p| (p.alt_m - 6500.0).abs() < 1.0);
        let has_1500 = v.path.iter().any(|p| (p.alt_m - 1500.0).abs() < 1.0);
        assert!(has_6500, "rz1 顶部剖面 6500m 应保留，实际 {:?}", v.path);
        assert!(has_1500, "rz2 底部剖面 1500m 应保留，实际 {:?}", v.path);
    }

    #[test]
    fn zigzag22_adjacent_rz_climb_anchor_avoid_band() {
        // 主管 2026-08-08 输入（China DEM L12 真实地形）：start 黄海 → target 蒙古，
        // 2 个**相邻** restricted 圆：rz2（116.30°E r100，alt[500,4500]→顶部 5000m）
        // 先处理，其 out_climb 过渡完成点沿 raw 落在 rz1（115.55°E r50，
        // alt[1000,4000]）圆内（距圆心 41km<50km，@3000 在 rz1 带内）→ rz1 处理时
        // tail 起点在圆内 → in_idx=0 → head 退化成**单点带内段** → 平滑全败 → join
        // 后 FINAL 在 rz1 处带内直穿 → 1967 点网格楼梯（2129.9km）。
        // 修复：desc/climb 锚点搜索拒绝位于**其他** restricted 圆带内的点（in_other_band）
        // → 剖面方案仍无解（rz2 out_climb 起点在 rz1 圆内 + 出口直线穿 no_fly）→
        // 后置 need_wall 画墙水平绕行兜底（宁丑勿违）→ 6 点 3000m 平滑路径 1685km。
        // 另修复 verify/check 圆判定投影误差（segment_circle_intersect_t slack）：
        // 1200km 长段投影偏差 ~0.5km，贴圆擦过（穿入 0.5km）被漏判交付 → 拒。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag22: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":122.9207839850354,"lat":34.08860812240517,"alt_m":3000},
                "target":{"lon":112.42397536890363,"lat":44.77935701405758,"alt_m":3000},
                "vehicles":[{"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                    "min_turn_radius_m":442,"max_climb_angle_deg":15},
                    "start_pose":{"lon":122.9207839850354,"lat":34.08860812240517,"alt_m":3000,"heading_deg":45},
                    "mid_waypoints":[]}],
                "red_forces":{"radars":[
                    {"id":"radar_1786161064845","lon":112.831513368294,"lat":43.96813072477223,"radar_type":"early_warning","radius_km":50,"alt_m":10},
                    {"id":"radar_1786161114205","lon":113.3041431982615,"lat":41.11010411517771,"radar_type":"tracking","radius_km":50,"alt_m":10}]},
                "no_fly_zones":[
                    {"id":"zone_1786160882933","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[115.8706565717089,39.78344941000123],[114.7159667887149,38.93519569445941],[115.2867088987491,39.07797084088842]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786160896845","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[118.18296859187262,39.84855590795146],[116.61186105129381,38.74876499516125],[117.56074185629024,38.71484400273334]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786160926653","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[115.2637148745787,42.16339247467029],[113.72404261848064,41.18665997544362],[114.42958291361323,41.129943488801175],[115.10906620487548,41.58646379784794]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786160994133","zone_type":"restricted","shape":"circle","geometry":{"center":[115.55242377844469,39.53680791293585],"radius_km":50},"alt_min_m":1000,"alt_max_m":4000,"height_semantics":"msl"},
                    {"id":"rz_1786161169725","zone_type":"restricted","shape":"circle","geometry":{"center":[116.3037514968375,38.447119717716134],"radius_km":100},"alt_min_m":500,"alt_max_m":4500,"height_semantics":"msl"}],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{"p_cross":0.9}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "相邻 restricted 修复后不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        assert!(
            v.path.len() <= 60,
            "应交付平滑路径（修复前 1967 点），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.distance_m < 1_800_000.0,
            "应 ≈ 平滑 1685km（画墙绕行兜底），实际 {}km",
            v.distance_m / 1000.0
        );
        // 交付路径不得穿 restricted 高度带（verify 圆判定 slack 修复）：
        // 全 3000m 平飞绕行 → 圆内采样点高度都不在 [alt_min, alt_max] 带内
        for z in &input.mission.restricted_zones {
            let crate::config::ZoneShape::Circle { center, radius_km } = z.shape else {
                continue;
            };
            for w in v.path.windows(2) {
                for k in 0..=20 {
                    let u = k as f64 / 20.0;
                    let (lon, lat) = (
                        w[0].x + (w[1].x - w[0].x) * u,
                        w[0].y + (w[1].y - w[0].y) * u,
                    );
                    let d = dist_km(lon, lat, center[0], center[1]);
                    if d <= radius_km && w[0].alt_m >= z.alt_min_m && w[0].alt_m <= z.alt_max_m {
                        panic!(
                            "路径穿 restricted {} 带内 ({:.3},{:.3}) d={:.1}km alt={:.0}",
                            z.id, lon, lat, d, w[0].alt_m
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn zigzag23_desc_anchor_no_space_need_wall() {
        // 主管 2026-08-08 输入（China DEM L12 真实地形，zigzag22 变体）：rz2 圆心南移
        // 至（116.044,38.196）→ rz2 out_climb 过渡直线不再穿 no_fly → 不触发硬墙兜底 →
        // 剖面方案继续。rz1 处理时 raw 在 rz1 圆外仅 0.05km（climb_base 7.5km 内无合法
        // desc 锚点）→ i_desc=i_in → desc1 零长度垂直爬升 3000→4500（起点在圆外不违例）。
        // 但 join 去重（坐标相同）丢失 desc1 与 in_out1 起点 → pt(3000 圆外)→pt(4500
        // 圆内) 40km 大爬升穿 rz1 带内 [1000,4000] → final verify 拒 → 回退 1967 点
        // 网格楼梯（2129.9km）。
        // 修复：desc_in/out_climb 过渡直线穿任何 restricted 圆带内（含端点本身在圆内
        // 带内，退化/零长度段也覆盖）+ 过渡段水平距离 < climb_base（爬升角超 15°）→
        // need_wall 画墙水平绕行兜底 → 6 点 3000m 平滑路径 1712km。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag23: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":122.9207839850354,"lat":34.08860812240517,"alt_m":3000},
                "target":{"lon":112.42397536890363,"lat":44.77935701405758,"alt_m":3000},
                "vehicles":[{"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                    "min_turn_radius_m":442,"max_climb_angle_deg":15},
                    "start_pose":{"lon":122.9207839850354,"lat":34.08860812240517,"alt_m":3000,"heading_deg":45},
                    "mid_waypoints":[]}],
                "red_forces":{"radars":[
                    {"id":"radar_1786161064845","lon":112.831513368294,"lat":43.96813072477223,"radar_type":"early_warning","radius_km":50,"alt_m":10},
                    {"id":"radar_1786161114205","lon":113.3041431982615,"lat":41.11010411517771,"radar_type":"tracking","radius_km":50,"alt_m":10}]},
                "no_fly_zones":[
                    {"id":"zone_1786160882933","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[115.8706565717089,39.78344941000123],[114.7159667887149,38.93519569445941],[115.2867088987491,39.07797084088842]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786160896845","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[118.18296859187262,39.84855590795146],[116.61186105129381,38.74876499516125],[117.56074185629024,38.71484400273334]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"},
                    {"id":"zone_1786160926653","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[115.2637148745787,42.16339247467029],[113.72404261848064,41.18665997544362],[114.42958291361323,41.129943488801175],[115.10906620487548,41.58646379784794]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786160994133","zone_type":"restricted","shape":"circle","geometry":{"center":[115.55242377844469,39.53680791293585],"radius_km":50},"alt_min_m":1000,"alt_max_m":4000,"height_semantics":"msl"},
                    {"id":"rz_1786161169725","zone_type":"restricted","shape":"circle","geometry":{"center":[116.04433455541918,38.19552566014898],"radius_km":100},"alt_min_m":500,"alt_max_m":4500,"height_semantics":"msl"}],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{"p_cross":0.9}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "过渡段穿 restricted 带内/长度不足应 need_wall 兜底而非回退，实际 {:?}",
            v.warnings
        );
        assert!(
            v.path.len() <= 60,
            "应交付平滑路径（修复前 1967 点），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.distance_m < 1_800_000.0,
            "应 ≈ 平滑 1712km（画墙绕行兜底），实际 {}km",
            v.distance_m / 1000.0
        );
        // 交付路径不得穿 restricted 高度带（同 zigzag22 检查）
        for z in &input.mission.restricted_zones {
            let crate::config::ZoneShape::Circle { center, radius_km } = z.shape else {
                continue;
            };
            for w in v.path.windows(2) {
                for k in 0..=20 {
                    let u = k as f64 / 20.0;
                    let (lon, lat) = (
                        w[0].x + (w[1].x - w[0].x) * u,
                        w[0].y + (w[1].y - w[0].y) * u,
                    );
                    let d = dist_km(lon, lat, center[0], center[1]);
                    if d <= radius_km && w[0].alt_m >= z.alt_min_m && w[0].alt_m <= z.alt_max_m {
                        panic!(
                            "路径穿 restricted {} 带内 ({:.3},{:.3}) d={:.1}km alt={:.0}",
                            z.id, lon, lat, d, w[0].alt_m
                        );
                    }
                }
            }
        }
    }

    #[test]
    fn zigzag24_low_alt_terrain_raise_no_staircase() {
        // 主管 2026-08-10 输入（east_asia 7.5as 真实地形）：双机独立 target_ref。
        // v2 起点 1000m（116.55°E,38.55°N）→ 目标 1500m（116.07°E,42.00°N）——路径
        // 穿内蒙古高原（地形 2050m+）——FMM 2D 不感知飞行高度 vs 地形 → 1000m raw
        // 穿山 → Theta*/verify 全拒 → 回退 734 点网格楼梯（smoothing_failed）。
        // 修复：代价场按本机高度把「Land + 净空 ≥ alt + 300m」格点置 INF（FMM 绕山）；
        // 过滤后无解 → 无过滤场探测路径 → 路径撞山（2050m）→ 抬升到「路径地形最高 +
        // 净空 + 100m」=2250m 重跑 → 平滑 7 点；restricted 底部 1500m 剖面保持。
        // 回归保护：① 抬升不破坏 v1（3000m 不动）；② restricted 墙膨胀/软罚带不因
        // terrain 存在而误调（zigzag21 回归）；③ 抬升只发生在路径撞山（非区域级）。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag24: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":117.5708068837583,"lat":38.97929027731468,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[],"target_ref":"114.62855523087296,41.481418330201244,3000"},
                    {"id":"v2","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":116.55201554900877,"lat":38.54836682471938,"alt_m":1000,"heading_deg":45},
                     "mid_waypoints":[],"target_ref":"116.06634800292873,42.00165253451988,1500"}],
                "red_forces":{"radars":[]},
                "no_fly_zones":[
                    {"id":"zone_1786326750206","zone_type":"no_fly","shape":"polygon","geometry":{"vertices":[[116.29703178143244,40.659872167707796],[115.20946822222263,39.507896098494335],[115.73653221684515,39.54513536978776]]},"alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786326782863","zone_type":"restricted","shape":"circle","geometry":{"center":[116.60669517425168,39.793491838843366],"radius_km":40},"alt_min_m":2000,"alt_max_m":6000,"height_semantics":"msl"}],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v1 = &out.vehicles[0];
        let v2 = &out.vehicles[1];
        assert_eq!(v1.status, "planned");
        assert_eq!(v2.status, "planned");
        // v2：抬升 + 平滑（修复前 734 点楼梯 + smoothing_failed）
        assert!(
            v2.path.len() <= 20,
            "v2 应平滑交付（修复前 734 点网格楼梯），实际 {} 点",
            v2.path.len()
        );
        assert!(
            v2.warnings.iter().any(|w| w.contains("cruise altitude raised")),
            "v2 应提示巡航高度抬升，实际 {:?}",
            v2.warnings
        );
        assert!(
            v2.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "v2 不应 smoothing_failed，实际 {:?}",
            v2.warnings
        );
        // v2 抬升后 restricted 底部 1500m 剖面保持（穿过 rz 带内时下降底部穿行）
        assert!(
            v2.path.iter().any(|p| (p.alt_m - 1500.0).abs() < 1.0),
            "v2 restricted 底部 1500m 剖面应保持，实际 {:?}",
            v2.path
        );
        // v1：3000m 不受抬升影响，平滑且无 smoothing_failed
        assert!(
            v1.path.len() <= 20,
            "v1 应平滑交付，实际 {} 点",
            v1.path.len()
        );
        assert!(
            v1.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "v1 不应 smoothing_failed，实际 {:?}",
            v1.warnings
        );
    }

    #[test]
    fn zigzag26_straight_line_peak_raise_no_collision() {
        // 主管 2026-08-10 输入（east_asia 7.5as 真实地形）：v1 start_pose
        // （117.57,38.98）→ target_ref（115.46,40.87），直线 276km，巡航 1892m。
        // 直线后半段经 (115.8132,40.5606) 2137m 尖峰。修复前：抬升决策采样 FMM
        // 网格点（间隔 ~1km 恰好错过尖峰 → 只抬到 1692m→1892m），Theta* 把绕山
        // 楼梯拉直成 2 点直线穿山，verify 固定 9 点/段采样间隔 ~30km 漏峰 → 撞山
        // 路径通过复验交付（clearance -245m）。
        // 修复三层：① verify_path 地形/禁飞采样按段长自适应（间隔 ≤1km）——
        // 长段不再漏峰；② make_segment_check 加地形净空（Theta* 拉直不得穿山）；
        // ③ 抬升决策采样分段直线（seg_ends，~1km 密度）而非 FMM 网格点——
        // 直线地形最高 2040m → 抬升到 2240m 越过山峰。
        // 断言：交付路径沿直线密采样（2000 点）最小净空 ≥ 100m（不撞山）；
        // 平滑交付（≤3 点）；无 smoothing_failed。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag26: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":117.57051750925365,"lat":38.9816070217835,"alt_m":500,"heading_deg":45},
                     "mid_waypoints":[],"target_ref":"115.46093981532285,40.8762471499978,2000"}],
                "red_forces":{"radars":[]},
                "no_fly_zones":[],"restricted_zones":[],"obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 3,
            "应平滑交付直线，实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 抬升提示存在（低空任务被地形抬升）
        assert!(
            v.warnings.iter().any(|w| w.contains("cruise altitude raised")),
            "应提示巡航高度抬升，实际 {:?}",
            v.warnings
        );
        // 密采样复核：输出路径任何点净空 ≥ 100m（不撞山）
        let t = crate::terrain::open_source(&cand).unwrap();
        let a = &v.path[0];
        let b = v.path.last().unwrap();
        let n = 2000;
        let mut min_clr = f64::INFINITY;
        for i in 0..=n {
            let tt = i as f64 / n as f64;
            let lon = a.x + (b.x - a.x) * tt;
            let lat = a.y + (b.y - a.y) * tt;
            let alt = a.alt_m + (b.alt_m - a.alt_m) * tt;
            if let crate::terrain::Sample::Land(h) = t.sample_at(lon, lat) {
                min_clr = min_clr.min(alt - h);
            }
        }
        assert!(
            min_clr >= 100.0,
            "交付路径撞山：最小净空 {min_clr:.1}m < 100m"
        );
    }

    #[test]
    fn zigzag25_entry_heading_first_jump_uturn_band_arc() {
        // 主管 2026-08-10 输入（east_asia 7.5as 真实地形）：双机独立起终点 + 必经点 +
        // 单 tracking 雷达 50km。v2 起点（116.63,40.66）→ 必经点（116.45,39.56）→ 目标
        // （115.50,39.58）。必经点 mid 恰在雷达盘内（距中心 49.6km<50km）→ SEG0 从
        // 南侧绕雷达盘向北进入 mid（末段方向 0°），SEG1 需向西离开 → 首跳 88.5° 被
        // entry_heading 60° 约束全拒 → theta_star 只能走相邻点（b→c 800m 向南）→
        // 段边界 180° 掉头 → arc tan(90°) 发散退化 → FINAL 拒 → 回退 471 点网格楼梯。
        // 修复三层：① theta_star 首跳 entry 约束放宽到 95°（必要大转向直接拉直，
        // zigzag11 首跳 61.94° 场景仍受约束）→ b→target 直线（check 已通过）直接拉直；
        // ② solver 必经点识别改容差匹配（段端点网格离散偏离必经点 ~0.5 cell，1e-9
        // 精确匹配漏判 → 必经点被弧弹出 → E 越过出段节点折返 178°）；
        // ③ arc_transition d_m ≤ 0.75×|bc| 截断（弧末点不越过后段下一节点），
        // keep_b（必经点）用物理转弯半径（切点偏差 r·tan(θ/2)≈431m，满足必经点
        // 容差 0.05°≈5.5km 测试断言）。结果：v2 471→10 点平滑，v1 5 点不变。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag25: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":116.92977366719182,"lat":40.4410563361943,"alt_m":300,"heading_deg":45},
                     "mid_waypoints":[{"lon":116.5031580680477,"lat":39.61768508544036,"alt_m":300}],
                     "target_ref":"115.49957776796042,39.577093261324855,3000"},
                    {"id":"v2","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":116.63273135458661,"lat":40.66380307890914,"alt_m":300,"heading_deg":45},
                     "mid_waypoints":[{"lon":116.44919168873186,"lat":39.56474547305474,"alt_m":300}],
                     "target_ref":"115.49957776796042,39.577093261324855,3000"}],
                "red_forces":{"radars":[
                    {"id":"radar_1786332337392","lon":116.76131203011676,"lat":39.93279959421326,"radar_type":"tracking","radius_km":50,"alt_m":10}]},
                "no_fly_zones":[],
                "restricted_zones":[],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        for (vi, v) in out.vehicles.iter().enumerate() {
            assert_eq!(v.status, "planned", "v{} 应 planned", vi + 1);
            assert!(
                v.path.len() <= 20,
                "v{} 应平滑交付（修复前 v2 471 点网格楼梯），实际 {} 点",
                vi + 1,
                v.path.len()
            );
            assert!(
                v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
                "v{} 不应 smoothing_failed，实际 {:?}",
                vi + 1,
                v.warnings
            );
        }
        // 必经点语义：交付路径经过必经点容差邻域（0.05°≈5.5km，同 m5 测试口径；
        // 实测切点偏差 r·tan(θ/2)≈431m）。v2 必经点 mid=(116.4492,39.5647)。
        let v2 = &out.vehicles[1];
        let (mlon, mlat) = (116.44919168873186_f64, 39.56474547305474_f64);
        assert!(
            v2.path.iter().any(|p| (p.x - mlon).abs() < 0.05 && (p.y - mlat).abs() < 0.05),
            "v2 路径应经过必经点邻域，实际 {:?}",
            v2.path
        );
    }

    #[test]
    fn zigzag27_waypoint_uturn_160deg_entry_relax_hard_only() {
        // 主管 2026-08-10 输入（east_asia 7.5as 真实地形）：单机独立 start_pose + 4 必经点
        // + target_ref，无 zone/雷达。必经点之字形：wp2(116.973,39.269)→wp3(117.355,39.254)
        // 方向 ~84°（东），wp3→wp4(116.829,39.840) 方向 ~250°（西北）→ wp3 处几何需
        // ~160° 大转向。zigzag25 首跳 entry 放宽只到 95° → SEG3 拉直全拒 → 只能走相邻点
        // → 段边界 179.9° 掉头 → arc tan(89.95°)≈114.6 发散退化（d_m 截断 0.75×|bc| 后
        // r_eff≈5m，弧点间距 <1m）→ FINAL radius 1m 拒 → 回退 541 点网格楼梯 + 516.7km。
        // 修复（zigzag27）：必经点段（起点落在 hard_boundary 容差内）首跳 entry 上限放宽
        // 到 175°（必经点大转向合法，solver 段边界 arc 切弧处理 ~128°，r×tan(64°)≈906m
        // 物理可行）；非必经点段保持 95°（zigzag11/25 语义保护）。结果：541→14 点平滑，
        // 381.1km（直线拉直），smoothing_failed 消失。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag27: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":117.57051750925365,"lat":38.9816070217835,"alt_m":500,"heading_deg":45},
                     "mid_waypoints":[
                        {"lon":117.38220298919144,"lat":39.522437439199145,"alt_m":500},
                        {"lon":116.97334657489195,"lat":39.26884298384281,"alt_m":500},
                        {"lon":117.35493646681157,"lat":39.25372350847514,"alt_m":500},
                        {"lon":116.82900640317098,"lat":39.839899568556554,"alt_m":500}],
                     "target_ref":"115.46093981532285,40.8762471499978,2000"}],
                "red_forces":{"radars":[]},
                "no_fly_zones":[],
                "restricted_zones":[],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned", "v1 应 planned");
        assert!(
            v.path.len() <= 20,
            "应平滑交付（修复前 541 点网格楼梯），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 必经点语义：4 个必经点均需经过（容差 0.05°≈5.5km，同 m5 口径）。
        let mids = [
            (117.38220298919144_f64, 39.522437439199145_f64),
            (116.97334657489195_f64, 39.26884298384281_f64),
            (117.35493646681157_f64, 39.25372350847514_f64),
            (116.82900640317098_f64, 39.839899568556554_f64),
        ];
        for (mi, (mlon, mlat)) in mids.iter().enumerate() {
            assert!(
                v.path.iter().any(|p| (p.x - mlon).abs() < 0.05 && (p.y - mlat).abs() < 0.05),
                "wp{} 必经点应经过邻域，实际 {:?}",
                mi + 1,
                v.path
            );
        }
    }
}


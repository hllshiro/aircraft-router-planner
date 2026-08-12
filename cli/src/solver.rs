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
    point_in_polygon_xy, pt_seg_dist_km, resolve_target_ref, zone_contains, zone_contains_at,
    Input, Output, PathPoint, Stats, TerrainSourceType, VehicleOutput, Zone, ZoneShape,
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
    /// 端到端时间预算（ms，含 solve 前地形加载等 main 已计耗时）：0 = 无限
    /// （测试/CI 双跑确定性用；docs/07 §5 3s 预算硬护栏）。超预算：
    /// 已有过闸候选（前面车辆已完成）→ `degraded_timeout` + 部分结果；
    /// 无候选 → `AppError::DegradedTimeout`。
    pub time_budget_ms: u64,
}

impl Default for SolveParams {
    fn default() -> Self {
        Self {
            terrain_path: None,
            mask_path: None,
            grid: 256,
            time_budget_ms: 0,
        }
    }
}

/// 地形打开中间态：ARPK1（BuiltinSource，预取）或外部格式（trait object，带锁）。
enum InnerSource {
    Builtin(BuiltinSource),
    Dyn(Box<dyn TerrainSource>),
}

/// 地形句柄：无地形 / ARPK1 / 外部格式 / 掩膜包装。
/// - `Plain`/`Masked`：ARPK1（BuiltinSource）——field build 走 BulkPrefetch 并行无锁预取
///   （候选③，3.71×，对比测试验证）；
/// - `External`/`MaskedExternal`：外部格式直读（GeoTIFF/DTED/SRTM，`open_source` 分派
///   对应解析库，2026-08-11 主管：外部格式不需要转换）——无 BulkPrefetch → 带锁采样；
/// - 掩膜包装（Phase 2 水体判定）：海洋 → Sample::Water（0 高程）、内陆湖 → Sample::Lake(DEM)、
/// 陆地 → 委托内层；平滑链/代价场统一走 TerrainSource/BulkPrefetch 抽象。
enum TerrainHandle {
    None,
    Plain(BuiltinSource),
    External(Box<dyn TerrainSource>),
    Masked(MaskedSource<BuiltinSource>),
    MaskedExternal(MaskedSource<Box<dyn TerrainSource>>),
}

impl TerrainHandle {
    fn as_source(&self) -> Option<&dyn TerrainSource> {
        match self {
            Self::None => None,
            Self::Plain(t) => Some(t),
            Self::External(t) => Some(t.as_ref()),
            Self::Masked(t) => Some(t),
            Self::MaskedExternal(t) => Some(t),
        }
    }
    fn as_bulk(&self) -> Option<&(dyn BulkPrefetch + Sync)> {
        match self {
            Self::None => None,
            Self::Plain(t) => Some(t),
            Self::External(_) => None,
            Self::Masked(t) => Some(t),
            Self::MaskedExternal(_) => None,
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
    /// 每机目标高度（2026-08-12 垂直剖面：路径终点高度；target_ref 第 3 段或 mission.target）。
    target_alt_m: f64,
    /// 机型配置（Phase 4 M4：平滑参数派生输入）。
    profile: crate::config::VehicleProfile,
    /// 中途必经点（Phase 4 M5：start → mid[0..] → target 分段拼接）。
    mid_waypoints: Vec<Geo>,
}

/// 端到端解算。elapsed_ms 为端到端耗时（main 计时传入）。
pub fn solve(input: &Input, params: &SolveParams, elapsed_ms: u64) -> Result<Output, AppError> {
    // P6-B（docs/07 §5）：3s 预算硬护栏。预算含 solve 前耗时（地形加载等）——
    // 总耗时 ≈ elapsed_ms + solve 内耗时；0 = 无限（测试/CI 确定性，永不触发）。
    let budget_ms = params.time_budget_ms;
    let solve_t0 = std::time::Instant::now();
    let over_budget = |elapsed_ms: u64| -> bool {
        budget_ms != 0 && elapsed_ms + solve_t0.elapsed().as_millis() as u64 >= budget_ms
    };
    // 1. 地形源（none = 无地形平面；path/builtin = ARPK1 文件或外部格式）。
    //    主管 2026-08-11：外部格式（GeoTIFF/DTED/SRTM）不需要转换，直接调对应
    //    解析库取数（`open_source` 按扩展名分派）；ARPK1 走 BuiltinSource（预取）。
    let terrain: TerrainHandle = match input.mission.terrain.source {
        TerrainSourceType::None => TerrainHandle::None,
        _ => {
            let p = params
                .terrain_path
                .clone()
                .or_else(|| input.mission.terrain.path.clone().map(PathBuf::from));
            let inner: InnerSource = match p {
                Some(p) => {
                    let ext = p
                        .extension()
                        .and_then(|e| e.to_str())
                        .unwrap_or("")
                        .to_ascii_lowercase();
                    match ext.as_str() {
                        // ARPK1/zstd → BuiltinSource（BulkPrefetch 预取路径）
                        "arpack" | "zstd" => {
                            InnerSource::Builtin(BuiltinSource::open(&p)?)
                        }
                        // 外部格式 → open_source 分派对应解析库（GeoTiff/Dted/Srtm/目录）
                        _ => InnerSource::Dyn(crate::terrain::open_source(&p)?),
                    }
                }
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
                        InnerSource::Builtin(BuiltinSource::open(c)?)
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
                    match inner {
                        InnerSource::Builtin(b) => {
                            TerrainHandle::Masked(MaskedSource::new(b, gm))
                        }
                        InnerSource::Dyn(d) => {
                            TerrainHandle::MaskedExternal(MaskedSource::new(d, gm))
                        }
                    }
                }
                None if !explicit_terrain => match default_mask_candidates() {
                    Some(mp) => {
                        let gm = GeoMask::open(&mp)?;
                        match inner {
                            InnerSource::Builtin(b) => {
                                TerrainHandle::Masked(MaskedSource::new(b, gm))
                            }
                            InnerSource::Dyn(d) => {
                                TerrainHandle::MaskedExternal(MaskedSource::new(d, gm))
                            }
                        }
                    }
                    None => match inner {
                        InnerSource::Builtin(b) => TerrainHandle::Plain(b),
                        InnerSource::Dyn(d) => TerrainHandle::External(d),
                    },
                },
                None => match inner {
                    InnerSource::Builtin(b) => TerrainHandle::Plain(b),
                    InnerSource::Dyn(d) => TerrainHandle::External(d),
                },
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
            target_alt_m: input.mission.target.alt_m,
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
                    target_alt_m: crate::config::resolve_target_alt(
                        v.target_ref.as_deref(),
                        input.mission.target.alt_m,
                    ),
                    profile: v.profile.clone(),
                    mid_waypoints: mid,
                })
            })
            .collect::<Result<Vec<_>, AppError>>()?
    };

    // 2b. 起终点/必经点不再做地形范围硬拒（主管 2026-08-11 拍板：放开输入点限制）。
    //     输入点落在数据范围外时，交给既有空洞/无效数据处理流程：FMM 种子格点
    //     （OOB 格点代价 INF 墙）照常传播 → 目标/必经点在数据内则出路径（OOB 段走
    //     墙格，verify OOB 硬拒 → 平滑失败回退 raw 交付）；全被墙挡则 no_solution
    //     + warning——均为四态内可用结果，不再返回 data_error（旧 8e5e64e 预检取消）。

    // 3. 任务区域（所有起点 + target 包围盒 + 缓冲）
    let target = input.mission.target.to_geo()?;
    let base_region = region_of(&specs, &target);
    // 3b. 障碍感知外扩（2026-08-11 zz_region_block2）：硬墙（NoFly/Obstacle）bbox
    // 超出任务区域时并入——墙占满 region 短边方向时绕行被迫出 region（region 外
    // 无代价场 → coarse FMM no path → 误报 no_solution）。Restricted 不画墙不纳入。
    let wall_zones = input
        .mission
        .no_fly_zones
        .iter()
        .chain(input.mission.obstacles.iter())
        .filter(|z| z.is_wall());
    let region = expand_region_for_walls(base_region, wall_zones, REGION_PAD_DEG);
    // 3c. 网格自适应（主管 2026-08-06 双大雷达/多边形场景）：**仅大区域**（span > 2.5°）时
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
    // 3c2. 障碍感知外扩后 grid 等比保持 cell（2026-08-11）：region 变大时按 base_grid
    // 的 cell 等比提 grid——cell 不变则 5c2 软罚带物理宽度不变（不触发 real_bad 教训），
    // 也不变粗（无锯齿/贴墙 clearance 风险）。
    let base_span_km = base_region.span_deg * 111.32;
    let has_poly_wall = input
        .mission
        .no_fly_zones
        .iter()
        .chain(input.mission.restricted_zones.iter())
        .chain(input.mission.obstacles.iter())
        .any(|z| z.is_wall() && matches!(z.shape, ZoneShape::Polygon { .. }));
    let target_cell = if has_poly_wall { 600.0 } else { 1100.0 };
    let auto_grid = if base_region.span_deg > 2.5 {
        ((base_span_km * 1000.0) / target_cell).ceil() as usize
    } else {
        0
    };
    let base_grid = params.grid.max(8).max(auto_grid).min(1024);
    let grid = if region.span_deg > base_region.span_deg + 1e-9 {
        let cell_deg = base_region.span_deg / base_grid as f64;
        ((region.span_deg / cell_deg).round() as usize)
            .max(base_grid)
            .min(1024)
    } else {
        base_grid
    };
    eprintln!("[debug] region span={:.2}deg grid={} cell_m={:.0} (base {:.2})", region.span_deg, grid, region.span_deg * 111_320.0 / grid as f64, base_region.span_deg);

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

    // 5. 语义代价场（Land=1 / Water=1 / Lake=1 / NoData=5 / OOB=5（2026-08-11 放开）/
    //    Forbidden=INF；NoFly/Obstacle 墙用 Forbidden——OOB 不再表达墙）
    //    硬墙判定闭包（每格：墙内 → Forbidden 禁行）——par_local 与串行回退共用。
    //    多边形墙用"格子矩形与多边形相交"（2026-08-11 zz_nosolution_case：中心点
    //    采样漏掉尖角/斜边 < 1 格窄带 → FMM 贴顶点穿入 → verify 精确几何拒 →
    //    误报 no_solution；矩形相交保证任何分辨率不漏窄带）；圆形连续无窄带问题，
    //    保持中心点语义（与 zone_contains 解析一致）。
    let cell_deg = region.span_deg / grid as f64;
    let walled = |lon: f64, lat: f64| -> bool {
        let Ok(g) = Geo::new(lon, lat) else {
            return false;
        };
        let half = cell_deg * 0.5;
        let (rx0, ry0, rx1, ry1) = (lon - half, lat - half, lon + half, lat + half);
        all_zones.iter().any(|z| {
            if !z.is_wall() {
                return false;
            }
            match &z.shape {
                ZoneShape::Circle { .. } => zone_contains(z, &g),
                ZoneShape::Polygon { vertices } => {
                    crate::config::rect_intersects_polygon(rx0, ry0, rx1, ry1, vertices)
                }
            }
        })
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
        // 外部格式（GeoTIFF/DTED/SRTM）无 BulkPrefetch → 带锁采样回退
        TerrainHandle::External(t) => build_semantic_cost_field(grid, grid, |r, c| {
            let (lon, lat) = cell_lonlat(r, c, &region, grid);
            if walled(lon, lat) {
                return Sample::Forbidden;
            }
            t.sample_at(lon, lat)
        }, 5.0),
        TerrainHandle::MaskedExternal(t) => build_semantic_cost_field(grid, grid, |r, c| {
            let (lon, lat) = cell_lonlat(r, c, &region, grid);
            if walled(lon, lat) {
                return Sample::Forbidden;
            }
            t.sample_at(lon, lat)
        }, 5.0),
        TerrainHandle::None => build_semantic_cost_field(grid, grid, |r, c| {
            let (lon, lat) = cell_lonlat(r, c, &region, grid);
            if walled(lon, lat) {
                return Sample::Forbidden;
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
        // P6-B 检查点①（每机开始）：预算耗尽 → 有部分结果返回 degraded_timeout，
        // 无候选返回 DegradedTimeout（docs/07 §5：warm best-so-far 超时降级）。
        if over_budget(elapsed_ms) {
            if out_vehicles.is_empty() {
                return Err(AppError::DegradedTimeout(format!(
                    "time budget {budget_ms}ms exceeded; no verified candidate yet"
                )));
            }
            out_vehicles.push(VehicleOutput {
                id: v.id.clone(),
                status: "degraded".into(),
                path: Vec::new(),
                distance_m: 0.0,
                warnings: vec![format!(
                    "time budget {budget_ms}ms exceeded; no path for this vehicle"
                )],
            });
            return Ok(Output {
                schema_version: crate::config::SCHEMA_VERSION.into(),
                status: "degraded_timeout".into(),
                error: Some(crate::error::ErrorBody {
                    code: "degraded_timeout".into(),
                    message: format!(
                        "time budget {budget_ms}ms exceeded; partial results for completed vehicles"
                    ),
                }),
                elapsed_ms: Some(elapsed_ms),
                vehicles: out_vehicles,
                stats: Stats {
                    fmm_ms,
                    los_checks: 0,
                    degradations,
                },
            });
        }
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
        // 平滑终检产物（循环内填充、循环外输出）：
        let mut warnings: Vec<String> = Vec::new();
        // loop 无条件进入且每个 break 前必赋值（attempts>6 兜底 / final_rep.ok /
        // 失败回退），故无需初始值
        let mut pts: Vec<crate::path::PathPoint>;
        // 2026-08-11 主管输入（zz30）：抬升决策只采样 seg_ends 直线，Theta* 绕行
        // 走廊可能经过更高山峰（(107.33,48.36) 2480m > 抬升 2555m 的净空阈值
        // 2455m；check 长段采样 1024 上限截断 → 928m 间隔漏窄峰、verify ~200m
        // 采样抓到）→ 平滑链全败回退 1789 点锯齿。修复：平滑+终检移入解算循环，
        // final_rep 报地形净空不足 → 按 verify issue 地形高度抬升重跑（根治
        // "走廊地形 > 抬升假设"；verify 采样最密为最终裁判）。抬升严格递增
        // （>alt_eff+0.5 且 ≤ceiling）单调有界；attempts 上限兜底（FMM 4 次 +
        // 平滑抬升重跑 2 次）。
        let mut attempts = 0usize;
        // 阶段1-C 失败诊断：最近 3 次 attempt 的 final verify 失败原因（attempts>6
        // 耗尽时输出 JSON，供定位"为什么平滑链始终不过"）。
        let mut last_failures: Vec<(usize, Vec<String>)> = Vec::new();
        'fmm_attempt: loop {
            attempts += 1;
            // P6-B 检查点②（每 attempt 前）：预算耗尽 → 直接按超时处理（同①口径）。
            if over_budget(elapsed_ms) {
                if out_vehicles.is_empty() {
                    return Err(AppError::DegradedTimeout(format!(
                        "time budget {budget_ms}ms exceeded; no verified candidate yet"
                    )));
                }
                out_vehicles.push(VehicleOutput {
                    id: v.id.clone(),
                    status: "degraded".into(),
                    path: Vec::new(),
                    distance_m: 0.0,
                    warnings: vec![format!(
                        "time budget {budget_ms}ms exceeded; no path for this vehicle"
                    )],
                });
                return Ok(Output {
                    schema_version: crate::config::SCHEMA_VERSION.into(),
                    status: "degraded_timeout".into(),
                    error: Some(crate::error::ErrorBody {
                        code: "degraded_timeout".into(),
                        message: format!(
                            "time budget {budget_ms}ms exceeded; partial results for completed vehicles"
                        ),
                    }),
                    elapsed_ms: Some(elapsed_ms),
                    vehicles: out_vehicles,
                    stats: Stats {
                        fmm_ms,
                        los_checks: 0,
                        degradations,
                    },
                });
            }
            if attempts > 6 {
                // 诊断输出（阶段1-C）：最后一次 verify 失败原因 JSON 到 stderr。
                if !last_failures.is_empty() {
                    let json = serde_json::json!({
                        "event": "fmm_attempts_exhausted",
                        "vehicle": v.id,
                        "attempts": attempts - 1,
                        "failures": last_failures.iter().map(|(a, iss)| {
                            serde_json::json!({ "attempt": a, "issues": iss })
                        }).collect::<Vec<_>>(),
                    });
                    eprintln!("{}", serde_json::to_string(&json).unwrap_or_else(|_| {
                        r#"{"event":"fmm_attempts_exhausted","serialize_error":true}"#.into()
                    }));
                }
                pts = raw_joined.points.clone();
                warnings.push("smoothing_failed: max attempts exhausted".into());
                break;
            }
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
            eprintln!("[debug] fmm attempt {} field ready (veh={})", attempts, veh_field.is_some());
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
                // P3 分类结论出口（docs/12 §3.4/§12.4）：粗层真无通道 → 几何无解分类
                emit_classified(
                    &v.id,
                    "geometrically_impossible",
                    "coarse FMM no path (docs/12 §11.2)",
                    &mut degradations,
                );
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
            pts = raw_joined.points.clone();
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
            // boundary arc 因净距（zone clearance）失败而回退的边界点坐标：final verify
            // 的 turn 检查对该边界豁免（≤65°；arc 会压到膨胀线内 → 宁可不转，机动空间
            // 优先，宁丑勿违）。2026-08-11 主管输入：wp1 必经点转角 60.7°>60°，U 形弧
            // 采样点偏墙 ~386m → arc 后段距墙 1.90km < 2.00km → 全链回退 687 点锯齿。
            let mut turn_exempt: Vec<(f64, f64)> = Vec::new();
            let mut seg_warnings = Vec::new();
            let mut entry_heading: Option<f64> = None;
            // 段级平滑中间阶段的地形净空不足最大高度（smooth.rs SmoothResult.
            // terrain_gap_m）：theta_star 拉直段穿山被回退楼梯吞掉时，final verify
            // 无 terrain issue，靠这里触发抬升重跑（2026-08-11 zz30 2480m 峰）。
            let mut seg_terr_max: f64 = 0.0;
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
                    if let Some(t) = result.terrain_gap_m {
                        seg_terr_max = seg_terr_max.max(t);
                    }
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
                                0,
                            ) {
                                let arc_len = arc_pts.len();
                                let e = *arc_pts.last().unwrap();
                                // 净距预检（2026-08-11 主管输入）：arc 使弧点偏出原直线
                                // （U 形弧采样偏墙 ~386m），arc 后段 E→next 可能压到
                                // zone 膨胀线内（1.90km < 2.00km）→ final verify 拒 →
                                // 全链回退 raw 网格楼梯。插入前逐段检查弧段 + E→next 的
                                // 墙净距（zone_segment_clearance_km 与 verify 同口径）；
                                // 不足 → 回退 arc（保持必经点 b，宁丑勿违），该边界转角
                                // ≤65° 记入豁免（机动空间优先）。
                                let c2 = out_seg.points.get(1).copied().unwrap_or(c);
                                let arc_ok = seg_zone_clearance_ok_arc(
                                    &arc_pts, &e, &c2, &all_zones, inflation_m,
                                );
                                if arc_ok {
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
                                } else {
                                    // 弧末点外推到出段直线（E' 距 b = 4×r，clamp 0.75×|bc|）：
                                    // U 形弧末点偏墙（~386m）使 arc 后段压到膨胀线内；E' 落在
                                    // b→c 直线上后，段 E'→next 恢复为 b→c 子段净距（≥ 原值）。
                                    // E'=2.2r 时弧内部转角 63.8°>60°（pt5→E' 短弦偏出段方向）；
                                    // 4r 使 pt5→E' 趋近出段方向（turn≈3θ/4<60，θ≤80），
                                    // verify radius（b, 弧中点, E'）≥442（θ=90° 最差 ~902m）。
                                    let bc_m = crate::path::haversine_m(b.lon, b.lat, c.lon, c.lat);
                                    let ext_m = (4.0 * opts.turn_radius_m).min(bc_m * 0.75);
                                    if ext_m > opts.turn_radius_m {
                                        let h_bc = crate::path::bearing_deg(b.lon, b.lat, c.lon, c.lat);
                                        let lat0 = b.lat.to_radians();
                                        let kx = 111_320.0 * lat0.cos();
                                        let ky = 111_320.0;
                                        let e2 = crate::path::PathPoint::new(
                                            b.lon + ext_m * h_bc.to_radians().sin() / kx,
                                            b.lat + ext_m * h_bc.to_radians().cos() / ky,
                                            b.alt_m,
                                        );
                                        // 外推后弧末段 p_{n-1}→E' 可能偏离弧方向（转角超限，
                                        // verify 拒）——细分弧重试：n 增大 → 末段步进减小 →
                                        // p_{n-1}→E' 趋近出段方向（2026-08-11 zz33：θ=166.8°
                                        // 掉头 n=3 时 p2→E' 101°；n=5 时 p4→E' 33°）。
                                        let mut accepted = false;
                                        for min_steps in 4..=8usize {
                                            let Some((arc_pts_sub, _)) =
                                                crate::smooth::arc_transition(
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
                                                    min_steps,
                                                )
                                            else {
                                                continue;
                                            };
                                            let mut arc_pts2 = arc_pts_sub.clone();
                                            if let Some(last) = arc_pts2.last_mut() {
                                                *last = e2;
                                            }
                                            if !seg_zone_clearance_ok_arc(
                                                &arc_pts2, &e2, &c2, &all_zones, inflation_m,
                                            ) {
                                                continue;
                                            }
                                            // 弧点转角（与 verify 同口径 bearing）：入段 a→S
                                            // 及弧内各段均 ≤ max_turn。外推 E' 只影响末段
                                            // p_{n-1}→E'（细分后 ≈ 出段方向）。
                                            let mut prev_h = crate::path::bearing_deg(
                                                a.lon, a.lat, arc_pts2[0].lon, arc_pts2[0].lat,
                                            );
                                            let mut turn_ok = true;
                                            for w in arc_pts2.windows(2) {
                                                let h = crate::path::bearing_deg(
                                                    w[0].lon, w[0].lat, w[1].lon, w[1].lat,
                                                );
                                                if crate::path::angle_diff_deg(prev_h, h).abs()
                                                    > opts.max_turn_deg + 1e-6
                                                {
                                                    turn_ok = false;
                                                    break;
                                                }
                                                prev_h = h;
                                            }
                                            if !turn_ok {
                                                continue;
                                            }
                                            let arc_len2 = arc_pts2.len();
                                            prev.points.truncate(n - 1);
                                            prev.points.extend(arc_pts2);
                                            if !out_seg.points.is_empty() {
                                                let p0 = &out_seg.points[0];
                                                if (p0.lon - b.lon).abs() < 1e-9
                                                    && (p0.lat - b.lat).abs() < 1e-9
                                                {
                                                    out_seg.points[0] = e2;
                                                }
                                            }
                                            if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                                eprintln!(
                                                    "[smooth-dbg] boundary arc ext at ({:.4},{:.4}) turn {:.1}->{} pts (E' {:.0}m, steps {min_steps})",
                                                    b.lon, b.lat, d, arc_len2, ext_m
                                                );
                                            }
                                            accepted = true;
                                            break;
                                        }
                                        if !accepted && d <= 65.0 {
                                            // arc 会破坏净空 → 不插弧，保持 b；该边界转角 ≤65
                                            // 豁免（final verify 后过滤，宁丑勿违）。
                                            turn_exempt.push((b.lon, b.lat));
                                            if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                                eprintln!(
                                                    "[smooth-dbg] boundary arc SKIP (clearance) at ({:.4},{:.4}) turn {:.1} exempt",
                                                    b.lon, b.lat, d
                                                );
                                            }
                                        } else if !accepted && std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                            eprintln!(
                                                "[smooth-dbg] boundary arc FAIL (clearance, turn {:.1} > 65) at ({:.4},{:.4})",
                                                d, b.lon, b.lat
                                            );
                                        }
                                    } else if d <= 65.0 {
                                        turn_exempt.push((b.lon, b.lat));
                                        if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                            eprintln!(
                                                "[smooth-dbg] boundary arc SKIP (clearance, short seg) at ({:.4},{:.4}) turn {:.1} exempt",
                                                b.lon, b.lat, d
                                            );
                                        }
                                    } else if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                        eprintln!(
                                            "[smooth-dbg] boundary arc FAIL (clearance, turn {:.1} > 65) at ({:.4},{:.4})",
                                            d, b.lon, b.lat
                                        );
                                    }
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
            // 阶段1-C：收集失败原因（仅失败的 attempt；保留最近 3 次）
            if !final_rep.ok {
                last_failures.push((attempts, final_rep.issues.clone()));
                if last_failures.len() > 3 {
                    last_failures.remove(0);
                }
            }
            if final_rep.ok {
                pts = joined.points;
                // extend 而非覆盖：保留 profile 级降速提示（turn_radius 信任输入）
                warnings.extend(seg_warnings.iter().cloned());
                break 'fmm_attempt;
            } else {
                // arc 失败边界的 turn 豁免（2026-08-11 主管输入）：boundary arc 因净距
                // 预检回退（arc 会压到膨胀线内）后保持必经点 b 原样 → final verify 仅剩
                // 该边界 turn 超限（≤65°）——机动空间优先，宁可不转（宁丑勿违）。过滤
                // 掉这些 turn issue（转 warning）；若其余 issues 为空 → 交付拼接路径。
                if !turn_exempt.is_empty() {
                    let mut kept: Vec<String> = Vec::new();
                    for iss in final_rep.issues.iter() {
                        let exempted = iss.strip_prefix("vertex ").is_some_and(|rest| {
                            let Some(colon) = rest.find(": turn ") else {
                                return false;
                            };
                            let Ok(idx) = rest[..colon].trim().parse::<usize>() else {
                                return false;
                            };
                            joined.points.get(idx).is_some_and(|p| {
                                turn_exempt
                                    .iter()
                                    .any(|(lo, la)| dist_km(*lo, *la, p.lon, p.lat) < 1.0)
                            })
                        });
                        if !exempted {
                            kept.push(iss.clone());
                        }
                    }
                    if kept.len() != final_rep.issues.len() {
                        warnings.push(format!(
                            "boundary turn at ({:.4},{:.4}) exceeds {}deg but arc would violate zone clearance; kept as-is (机动空间优先)",
                            turn_exempt[0].0, turn_exempt[0].1, opts.max_turn_deg
                        ));
                        if kept.is_empty() {
                            pts = joined.points;
                            warnings.extend(seg_warnings.iter().cloned());
                            break 'fmm_attempt;
                        }
                    }
                }
                // 地形净空不足 → 抬升重跑（2026-08-11 主管输入 2480m 峰）：verify
                // issue 采样密（~200m），以其地形高度为准；段级平滑中间阶段的
                // terrain issue（回退楼梯吞掉，见 seg_terr_max）取 max 并集。
                // 抬升严格递增（>alt_eff+0.5 且 ≤ceiling）单调有界，attempts 上限
                // 兜底。原 FAIL 分支（smoothing_failed + 直线替代 + 雷达替代）仅在
                // 抬升不可行/超限后执行。
                let terr_max = final_rep.issues.iter().filter_map(|s| {
                    let pos = s.find("(terrain ")?;
                    let tail = s[pos + "(terrain ".len()..].trim_end_matches(')').trim();
                    tail.trim_end_matches('m').trim().parse::<f64>().ok()
                }).fold(0.0_f64, f64::max).max(seg_terr_max);
                if terr_max > 0.0 {
                    let clearance = opts.clearance_m.max(1.0);
                    let new_alt = (terr_max + clearance + 100.0).max(v.alt_m);
                    let ceiling_ok = v.profile.ceiling_m.is_none_or(|c| new_alt <= c);
                    if new_alt > alt_eff + 0.5 && ceiling_ok {
                        terrain_alt_raised = true;
                        alt_eff = new_alt;
                        eprintln!(
                            "[debug] smooth terrain clearance -> raise cruise alt {:.0}->{:.0}m (terrain {:.0}m, v={})",
                            v.alt_m, alt_eff, terr_max, v.id
                        );
                        continue 'fmm_attempt;
                    }
                }
                // 终检失败 → 回退未平滑拼接（必经点保留，宁丑勿违）
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
                // P2 可见图 patch 绕过（docs/12 §8/§13；feature-flag 默认关，C6）
                // 触发：终检硬闸失败 → 失败点多簇（PATCH_R=30km 合并，字典序 tie-break）
                // → 每簇 patch 矩形 → 边界锚点（骨架首个可通交点，§7）→ 可见图
                // （多障碍 + 限飞区弦判据接线 + 雷达同源边权）→ 拼接回骨架 →
                // 接缝复验（C7 扩张重试硬上限 + 固定步长）。成功交付 + degradations
                // 标注；失败按 C3 归因分层（截断 ≠ 几何无解，C2），归原 raw 回退。
                if crate::patch::patch_enabled()
                    && crate::patch::patch_applicable(
                        &input
                            .mission
                            .no_fly_zones
                            .iter()
                            .chain(input.mission.obstacles.iter())
                            .cloned()
                            .collect::<Vec<_>>(),
                        ctx.terrain.is_some(),
                    )
                {
                    let skeleton: Vec<[f64; 2]> =
                        raw_joined.points.iter().map(|p| [p.lon, p.lat]).collect();
                    let clusters =
                        crate::patch::cluster_failures(&final_rep.issues, crate::patch::PATCH_R_DEG);
                    if !clusters.is_empty() && skeleton.len() >= 2 {
                        let wall_polys: Vec<&Zone> = input
                            .mission
                            .no_fly_zones
                            .iter()
                            .chain(input.mission.obstacles.iter())
                            .filter(|z| z.is_wall() && matches!(z.shape, ZoneShape::Polygon { .. }))
                            .collect();
                        let circle_walls: Vec<&Zone> = input
                            .mission
                            .no_fly_zones
                            .iter()
                            .chain(input.mission.obstacles.iter())
                            .filter(|z| z.is_wall() && matches!(z.shape, ZoneShape::Circle { .. }))
                            .collect();
                        let restricted: Vec<&Zone> = input.mission.restricted_zones.iter().collect();
                        let inflation_m = (opts.turn_radius_m * 0.5).clamp(2_000.0, 10_000.0);
                        let radar_opt = if input.mission.red_forces.radars.is_empty() {
                            None
                        } else {
                            Some((&threat, params_merged.radar_cost_coef))
                        };
                        // P4：地形净空 + NODATA 5x 进 patch（§3.3 P2 口径完整化）
                        let terrain_opt = ctx.terrain.map(|t| (t, opts.clearance_m.max(1.0)));
                        // P4-M4：真·多 patch 串接（§11.2 第 3 条）——失败点沿全程分布的
                        // 100km+ 贴墙走廊：所有簇依次 patch，每簇成功后把 patch 拼入当前
                        // 路径（stitch），下一簇锚点取"当前拼接路径"与簇 rect 的交点
                        // （迭代拼接，边界锚点 tie-break 同单簇）；每步整链 verify
                        // （阶段 1-D 硬闸）通过才拼入。全部簇处理完 → 交付拼接路径。
                        let mut current: Vec<[f64; 2]> = skeleton;
                        let mut used_any = false;
                        for c in &clusters {
                            let mut rect =
                                crate::patch::PatchRect::from_center(*c, crate::patch::PATCH_R_DEG);
                            // P5-M3：必经点安全子集（§13.2 R3）——patch 不得改必经点位置：
                            // 簇矩形含必经点 → 跳过该簇（degradations 标注）；不含 →
                            // patch 正常（必经点骨架段不受影响）。
                            if v.mid_waypoints
                                .iter()
                                .any(|wp| rect.contains([wp.lon, wp.lat]))
                            {
                                degradations.push(format!(
                                    "patch: cluster at ({:.4},{:.4}) contains mid_waypoint, skipped (R3)",
                                    c[0], c[1]
                                ));
                                continue;
                            }
                            let mut retry = 0;
                            loop {
                                let (ein, eout) = crate::patch::boundary_anchors(&current, &rect);
                                if let (Some(a), Some(b)) = (ein, eout) {
                                    let obstacles: Vec<Vec<[f64; 2]>> = wall_polys
                                        .iter()
                                        .filter_map(|z| match &z.shape {
                                            ZoneShape::Polygon { vertices } => {
                                                if vertices.iter().any(|v| rect.contains([v[0], v[1]])) {
                                                    Some(vertices.clone())
                                                } else {
                                                    None
                                                }
                                            }
                                            _ => None,
                                        })
                                        .collect();
                                    // P4：圆硬墙（膨胀后）进入可见图切点锚点
                                    let circle_obs: Vec<crate::patch::CircleObs> = circle_walls
                                        .iter()
                                        .filter_map(|z| match &z.shape {
                                            ZoneShape::Circle { center, radius_km } => {
                                                if rect.contains([center[0], center[1]]) {
                                                    Some(crate::patch::CircleObs {
                                                        center: *center,
                                                        r_eff_m: radius_km * 1000.0 + inflation_m,
                                                    })
                                                } else {
                                                    None
                                                }
                                            }
                                            _ => None,
                                        })
                                        .collect();
                                    if !obstacles.is_empty() || !circle_obs.is_empty() || !restricted.is_empty() {
                                        match crate::patch::plan_patch_multi(
                                            a,
                                            b,
                                            &obstacles,
                                            &circle_obs,
                                            &restricted,
                                            inflation_m,
                                            alt_eff,
                                            radar_opt,
                                            terrain_opt,
                                        ) {
                                            crate::patch::PatchOutcome::Path(patch_pts, _len_km) => {
                                                let joined_pts = crate::patch::stitch(
                                                    &current, &patch_pts, a, b,
                                                );
                                                let joined_path = crate::path::Path::new(
                                                    joined_pts
                                                        .iter()
                                                        .map(|[lon, lat]| {
                                                            crate::path::PathPoint::new(
                                                                *lon, *lat, alt_eff,
                                                            )
                                                        })
                                                        .collect(),
                                                );
                                                let rep_j = crate::smooth::verify_path(
                                                    &joined_path,
                                                    None,
                                                    &opts,
                                                    &ctx,
                                                    Some(phys_min_radius_m),
                                                );
                                                if rep_j.ok {
                                                    current = joined_pts;
                                                    used_any = true;
                                                    degradations.push(
                                                        "patch: visibility-graph bypass adopted (docs/12 §3.3/§3.5)"
                                                            .into(),
                                                    );
                                                    warnings.extend(rep_j.warnings.iter().cloned());
                                                    break;
                                                }
                                                // 接缝违规 → C7：扩张 patch 重试（硬上限 + 固定步长）
                                                if retry < crate::patch::PATCH_RETRY_MAX {
                                                    rect.half_deg *= crate::patch::PATCH_RETRY_EXPAND;
                                                    retry += 1;
                                                    continue;
                                                }
                                                // C3 归因：verify 硬闸失败 → 拟合缺陷 vs 几何无解
                                                let all_inflated: Vec<Vec<[f64; 2]>> = obstacles
                                                    .iter()
                                                    .map(|v| {
                                                        let hull = crate::patch::convex_hull(v);
                                                        crate::patch::inflate_convex(
                                                            &hull,
                                                            inflation_m / 111_320.0,
                                                        )
                                                    })
                                                    .filter(|h| h.len() >= 3)
                                                    .collect();
                                                match crate::patch::classify_verify_failure(
                                                    &patch_pts,
                                                    &rep_j.issues,
                                                    all_inflated
                                                        .first()
                                                        .map_or(&[], |h| h.as_slice()),
                                                    &circle_obs,
                                                    2.0 * inflation_m / 111_320.0,
                                                ) {
                                                    crate::patch::PatchFailureClass::FittingDefect => {
                                                        degradations.push(
                                                            "patch: fitting_defect (turn margin, docs/12 §13.1 C3)"
                                                                .into(),
                                                        )
                                                    }
                                                    crate::patch::PatchFailureClass::GeometricImpossible => {
                                                        degradations.push(
                                                            "patch: geometrically_impossible (docs/12 §13.1 C3)"
                                                                .into(),
                                                        )
                                                    }
                                                }
                                            }
                                            crate::patch::PatchOutcome::SearchTruncated => {
                                                // C2：截断 ≠ 几何无解；标注降级，归原 raw 回退
                                                degradations.push(
                                                    "patch: search_truncated (visible-graph cap, docs/12 §13.1 C2)"
                                                        .into(),
                                                );
                                            }
                                            crate::patch::PatchOutcome::GeometricImpossible => {}
                                        }
                                    }
                                }
                                break;
                            }
                        }
                        if used_any {
                            pts = current
                                .iter()
                                .map(|[lon, lat]| crate::path::PathPoint::new(*lon, *lat, alt_eff))
                                .collect();
                            warnings.extend(seg_warnings.iter().cloned());
                            break 'fmm_attempt;
                        }
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
            break 'fmm_attempt;
        }
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
        // 阶段1-D 回退过硬闸（主管 2026-08-11 拍板：不需要输出无法实现的路径）：
        // raw 回退（smoothing_failed / max attempts exhausted）后若直线替代与雷达
        // 直穿替代都未成功（未撤销 smoothing_failed），则路径未过完整复验硬闸
        // （地形净空/禁飞/受限区）——不再交付 raw，明确输出 no_solution。
        // "宁可不给路径，也不给违禁路径"；失败原因保留在 warnings，
        // 详细 issue JSON 已由 smooth_path_chain / attempts 耗尽分支输出到 stderr。
        let smoothing_failed = warnings.iter().any(|w| w.starts_with("smoothing_failed"));
        if smoothing_failed {
            let mut diag = warnings.clone();
            diag.push("no valid smoothed path; raw fallback withheld (hard-gate)".into());
            // P3 分类结论出口（docs/12 §3.4/§12.4）：smoothing_failed 硬闸拒交付 →
            // 复用 P2 patch 归因（C2/C3 标注）或默认 fitting_defect（可迭代缺陷）。
            emit_classified(
                &v.id,
                category_from_degradations(&degradations),
                "smoothing_failed; raw fallback withheld (hard-gate)",
                &mut degradations,
            );
            out_vehicles.push(VehicleOutput {
                id: v.id.clone(),
                status: "no_solution".into(),
                path: Vec::new(),
                distance_m: 0.0,
                warnings: diag,
            });
            continue 'veh;
        }
        // 垂直剖面（2026-08-12 主管 demo 轨迹倾斜）：输出高度从起点高度按累计
        // 距离线性过渡到目标高度（起终点不同高时轨迹呈现爬升/下降，而非恒为
        // 巡航高度的水平直线）；地形可用时保底（下降段不穿山）。起终点同高 →
        // 曲线水平，行为与既往一致（含受限区剖面/抬升巡航语义）。
        apply_vertical_profile(
            &mut pts,
            v.alt_m,
            v.target_alt_m,
            alt_eff,
            terrain.as_source(),
            opts.clearance_m,
        );
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

    // P6-C（docs/01 §7.1）：多机路径空间交叉检测——输出后处理，检出显式告警；
    // 时间维 out-of-scope（不判是否同时到达）。
    detect_multi_vehicle_crossings(&mut out_vehicles);

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

/// 任务区域缓冲（度）：保证源/目标不贴边；同时是障碍感知外扩的机动余量
/// （2026-08-11 zz_region_block2：墙占满 region 短边方向时绕行被迫出 region）。
const REGION_PAD_DEG: f64 = 0.15;

// ==================== P6-C 多机交叉检测 ====================
// docs/01 §7.1 方案：输出后处理检测多机路径空间交叉，检出显式告警；
// 时间维 out-of-scope（不判是否同时到达，仅空间接近告警）。
const CROSS_H_KM: f64 = 1.0; // 水平间隔阈值（km）
const CROSS_V_M: f64 = 150.0; // 垂直间隔阈值（m）
const CROSS_BBOX_PAD_DEG: f64 = 0.02; // 段包围盒粗筛延展（~2km）

/// 点 p 到段 q1-q2 的最近点（2D 局部平面，经度按 cos(mid_lat) 折算 km）。
/// 返回 (最近距离 km, 段参数 t, 垂足经度, 垂足纬度)。
fn closest_on_seg(
    px: f64,
    py: f64,
    q1x: f64,
    q1y: f64,
    q2x: f64,
    q2y: f64,
    mid_lat: f64,
) -> (f64, f64, f64, f64) {
    let kx = 111.32 * mid_lat.to_radians().cos().max(1e-6);
    let ky = 111.32;
    let dx = (q2x - q1x) * kx;
    let dy = (q2y - q1y) * ky;
    let len2 = dx * dx + dy * dy;
    let t = if len2 <= 0.0 {
        0.0
    } else {
        ((((px - q1x) * kx) * dx + ((py - q1y) * ky) * dy) / len2).clamp(0.0, 1.0)
    };
    let qx = q1x + (q2x - q1x) * t;
    let qy = q1y + (q2y - q1y) * t;
    let h = (((px - qx) * kx).powi(2) + ((py - qy) * ky).powi(2)).sqrt();
    (h, t, qx, qy)
}

/// 段-段水平最近距离与最近点参数（2D；候选 = 4 端点距离 + 4 端点-对段投影）。
/// 返回 (最近水平距离 km, t_a, t_b)。
fn seg_seg_closest(
    a1: &PathPoint,
    a2: &PathPoint,
    b1: &PathPoint,
    b2: &PathPoint,
    mid_lat: f64,
) -> (f64, f64, f64) {
    let mut best_h = f64::INFINITY;
    let mut best_ta = 0.0;
    let mut best_tb = 0.0;
    let mut consider = |h: f64, ta: f64, tb: f64| {
        if h < best_h {
            best_h = h;
            best_ta = ta;
            best_tb = tb;
        }
    };
    // 端点-对段投影
    for (ta0, p) in [(0.0, a1), (1.0, a2)] {
        let (h, tb, _, _) = closest_on_seg(p.x, p.y, b1.x, b1.y, b2.x, b2.y, mid_lat);
        consider(h, ta0, tb);
    }
    for (tb0, p) in [(0.0, b1), (1.0, b2)] {
        let (h, ta, _, _) = closest_on_seg(p.x, p.y, a1.x, a1.y, a2.x, a2.y, mid_lat);
        consider(h, ta, tb0);
    }
    // 端点-端点
    for (ta0, p) in [(0.0, a1), (1.0, a2)] {
        for (tb0, q) in [(0.0, b1), (1.0, b2)] {
            let kx = 111.32 * mid_lat.to_radians().cos().max(1e-6);
            let h = (((p.x - q.x) * kx).powi(2) + ((p.y - q.y) * 111.32).powi(2)).sqrt();
            consider(h, ta0, tb0);
        }
    }
    // 段-段内部最近（两直线最近点都在段内；相交 → 交点距离 0）。
    // 线性坐标（km）下解 da·s − db·t = b1 − a1。
    let kx = 111.32 * mid_lat.to_radians().cos().max(1e-6);
    let ky = 111.32;
    let ax = (a2.x - a1.x) * kx;
    let ay = (a2.y - a1.y) * ky;
    let bx = (b2.x - b1.x) * kx;
    let by = (b2.y - b1.y) * ky;
    let wx = (b1.x - a1.x) * kx;
    let wy = (b1.y - a1.y) * ky;
    let det = bx * ay - ax * by;
    if det.abs() > 1e-9 {
        let s = (bx * wy - wx * by) / det;
        let t = (ax * wy - ay * wx) / det;
        if (0.0..=1.0).contains(&s) && (0.0..=1.0).contains(&t) {
            let hx = wx + bx * t - ax * s;
            let hy = wy + by * t - ay * s;
            consider((hx * hx + hy * hy).sqrt(), s, t);
        }
    }
    (best_h, best_ta, best_tb)
}

/// 垂直剖面（2026-08-12 主管 demo 轨迹倾斜）：输出路径高度从起点高度
/// （start_pose.alt_m）按累计水平距离线性过渡到目标高度（resolve 目标 alt）。
/// - 起终点同高（|Δ|<0.5m）→ 保持现状（受限区剖面/抬升巡航语义不变）；
/// - **剖面段保持**：插值前路径中间点高度 = 巡航高度（alt_eff）；显著偏离
///   巡航高度的点 = 受限区底部穿行/顶部绕飞剖面（build_restricted_profiles 已按
///   语义生成并经验证）→ 垂直剖面不覆盖（zigzag24 底部 1500m 剖面保持）；
/// - **抬升场景**（alt_eff > start_alt，地形要求）→ 巡航段不得低于抬升巡航高度
///   （既有地形安全裁决；zigzag26 起点 500→2240 的既有行为保持），终点仍爬升；
/// - 非抬升场景（alt_eff == start_alt）→ 平滑线性过渡（下降场景也生效）；
/// - 终点 = 用户指定目标高度（保底不覆盖用户意图）；
/// - 地形可用时逐点保底：高度 ≥ 地形 + 净空 + 100m（下降段不穿山）；
/// - 不重跑 verify：高度调整只升不降（相对既有 verify 高度），净空结论保持。
fn apply_vertical_profile(
    pts: &mut [crate::path::PathPoint],
    start_alt: f64,
    target_alt: f64,
    cruise_alt: f64,
    terrain: Option<&dyn TerrainSource>,
    clearance_m: f64,
) {
    if pts.len() < 2 || (target_alt - start_alt).abs() < 0.5 {
        return;
    }
    // 累计水平距离（等距投影近似，仅作剖面插值参数）
    let lat0 = pts[0].lat.to_radians();
    let kx = 111_320.0 * lat0.cos();
    let ky = 111_320.0;
    let mut cum = vec![0.0_f64; pts.len()];
    for i in 1..pts.len() {
        let dx = (pts[i].lon - pts[i - 1].lon) * kx;
        let dy = (pts[i].lat - pts[i - 1].lat) * ky;
        cum[i] = cum[i - 1] + dx.hypot(dy);
    }
    let total = cum[pts.len() - 1].max(1.0);
    let clearance = clearance_m.max(1.0);
    // 抬升场景下限：巡航段高度不得低于抬升巡航高度（地形安全裁决）；
    // 非抬升（cruise == start）→ 无下限，线性插值平滑过渡（下降生效）。
    let raise_floor = if cruise_alt > start_alt + 0.5 {
        cruise_alt
    } else {
        f64::NEG_INFINITY
    };
    for i in 0..pts.len() {
        // 剖面段保持：原高度显著偏离巡航高度 → 受限区底部/顶部剖面（主动升降高），
        // 垂直剖面不覆盖（其高度已经 build_restricted_profiles 语义验证）。
        // 注意：插值前路径点高度 = 巡航高度或剖面段高度，不存在"插值中间点"，
        // 因此下降/爬升场景的巡航段（orig == cruise）不会误判为剖面段。
        if (pts[i].alt_m - cruise_alt).abs() > 0.5 {
            continue;
        }
        let t = (cum[i] / total).clamp(0.0, 1.0);
        let mut h = (start_alt + (target_alt - start_alt) * t).max(raise_floor);
        // 地形保底（含终点）：目标点在地形以下（物理不可达）→ 抬到安全高度
        // （zigzag26 目标 2000m、终点地形 2097m → 保底；宁高勿穿山）
        if let Some(tsrc) = terrain {
            if let Sample::Land(g) = tsrc.sample_at(pts[i].lon, pts[i].lat) {
                h = h.max(g + clearance + 100.0);
            }
        }
        pts[i].alt_m = h;
    }
}

/// P6-C：多机路径空间交叉检测（输出后处理）。对 path 非空的车辆两两检测：
/// 段-段水平最近距离 < CROSS_H_KM 且垂直最近差 < CROSS_V_M → 视为交叉，
/// 两车 warnings 各追加一条 `multi_vehicle_conflict`（时间维 out-of-scope）。
pub(crate) fn detect_multi_vehicle_crossings(vehicles: &mut [VehicleOutput]) {
    for i in 0..vehicles.len() {
        for j in (i + 1)..vehicles.len() {
            let pi = vehicles[i].path.clone();
            let pj = vehicles[j].path.clone();
            if pi.len() < 2 || pj.len() < 2 {
                continue;
            }
            let mid_lat = pi.iter().chain(pj.iter()).map(|p| p.y).sum::<f64>()
                / (pi.len() + pj.len()) as f64;
            let mut best: Option<(f64, f64, f64, f64)> = None; // (h_km, v_m, lon, lat)
            for a in pi.windows(2) {
                for b in pj.windows(2) {
                    // 包围盒粗筛（经纬度延展 pad；垂直在候选命中后再判）
                    if a[0].x.max(a[1].x) + CROSS_BBOX_PAD_DEG < b[0].x.min(b[1].x)
                        || b[0].x.max(b[1].x) + CROSS_BBOX_PAD_DEG < a[0].x.min(a[1].x)
                        || a[0].y.max(a[1].y) + CROSS_BBOX_PAD_DEG < b[0].y.min(b[1].y)
                        || b[0].y.max(b[1].y) + CROSS_BBOX_PAD_DEG < a[0].y.min(a[1].y)
                    {
                        continue;
                    }
                    let (h, ta, tb) = seg_seg_closest(&a[0], &a[1], &b[0], &b[1], mid_lat);
                    if h > CROSS_H_KM {
                        continue;
                    }
                    let alt_a = a[0].alt_m + (a[1].alt_m - a[0].alt_m) * ta;
                    let alt_b = b[0].alt_m + (b[1].alt_m - b[0].alt_m) * tb;
                    let v = (alt_a - alt_b).abs();
                    if v > CROSS_V_M {
                        continue;
                    }
                    let lon = a[0].x + (a[1].x - a[0].x) * ta;
                    let lat = a[0].y + (a[1].y - a[0].y) * ta;
                    match best {
                        Some((bh, bv, _, _)) if (h, v) >= (bh, bv) => {}
                        _ => best = Some((h, v, lon, lat)),
                    }
                }
            }
            if let Some((h, v, lon, lat)) = best {
                let msg = format!(
                    "multi_vehicle_conflict: crossing with {} near ({:.4},{:.4}) h={:.1}km v={:.0}m (time-dimension out-of-scope)",
                    vehicles[j].id, lon, lat, h, v
                );
                let msg2 = format!(
                    "multi_vehicle_conflict: crossing with {} near ({:.4},{:.4}) h={:.1}km v={:.0}m (time-dimension out-of-scope)",
                    vehicles[i].id, lon, lat, h, v
                );
                vehicles[i].warnings.push(msg);
                vehicles[j].warnings.push(msg2);
            }
        }
    }
}

/// 圆墙外扩机动余量（度）：绕圆墙需求 ≈ inflation（2km）+ 转弯半径（~0.5km）+
/// 净空 ≈ 0.03°（3.3km）。比多边形墙的 REGION_PAD_DEG 小——圆墙 bbox 大（半径
/// 数十公里），大余量会让本就接近任务边界的圆撑出 region（zz19 教训，见
/// expand_region_for_walls 注释）。
const REGION_CIRCLE_MARGIN_DEG: f64 = 0.03;

/// 任务区域：所有起点 + 每机目标 + mission.target 的方形包围盒 + 缓冲
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
    let pad = REGION_PAD_DEG;
    min_lon -= pad;
    min_lat -= pad;
    let span = (max_lon - min_lon).max(max_lat - min_lat) + 2.0 * pad;
    Region {
        min_lon,
        min_lat,
        span_deg: span,
    }
}

/// P3 分类结论出口（docs/12 §3.4/§12.4 拍板）：stderr 分类 JSON + stdout
/// stats.degradations 汇总；不升 schema 0.20（零 schema 负担）。
/// 类别（C2/C3 决策树）：geometrically_impossible（真无解）/ search_truncated
/// （可见图上限截断，≠几何无解）/ fitting_defect（拟合缺陷，可迭代）。
pub(crate) fn emit_classified(
    vehicle_id: &str,
    category: &str,
    detail: &str,
    degradations: &mut Vec<String>,
) {
    let json = serde_json::json!({
        "event": "classified",
        "vehicle": vehicle_id,
        "category": category,
        "detail": detail,
    });
    eprintln!(
        "{}",
        serde_json::to_string(&json)
            .unwrap_or_else(|_| r#"{"event":"classified","serialize_error":true}"#.into())
    );
    degradations.push(format!("classified: {category}"));
}

/// 从已收集的 patch 归因标注（P2 挂接 push 的 C2/C3 标注）映射分类类别；
/// 无标注 → 默认 fitting_defect（平滑链失败默认可迭代拟合缺陷）。
fn category_from_degradations(degradations: &[String]) -> &'static str {
    if degradations
        .iter()
        .any(|d| d.contains("geometrically_impossible"))
    {
        "geometrically_impossible"
    } else if degradations.iter().any(|d| d.contains("search_truncated")) {
        "search_truncated"
    } else {
        "fitting_defect"
    }
}

/// 障碍感知区域外扩（2026-08-11 zz_region_block2）：任务 bbox 近正方形时两个方向
/// 都只有 REGION_PAD_DEG 缓冲——硬墙（NoFly/Obstacle）若占满 region 一个方向（墙
/// bbox 延伸到 region 边界外），绕行路径被迫超出 region（region 外无代价场 →
/// coarse FMM no path → 误报 no_solution；实际绕出 region 0.1° 即通）。
/// 把硬墙 bbox（+ pad 机动余量）并入任务 region，保持方形（Region 只有单值 span）。
/// Restricted 不画墙、不阻碍 FMM 水平传播 → 不纳入。
fn expand_region_for_walls<'a>(
    region: Region,
    wall_zones: impl Iterator<Item = &'a Zone>,
    pad: f64,
) -> Region {
    let r_min_lon = region.min_lon;
    let r_max_lon = region.min_lon + region.span_deg;
    let r_min_lat = region.min_lat;
    let r_max_lat = region.min_lat + region.span_deg;
    let mut min_lon = r_min_lon;
    let mut max_lon = r_max_lon;
    let mut min_lat = r_min_lat;
    let mut max_lat = r_max_lat;
    let mut has_wall = false;
    for z in wall_zones {
        // 墙 bbox 单独补机动余量（任务 region 已含 pad，不再重复加）：
        // 多边形墙绕行在顶点外侧（路径需离顶点 ≥ inflation+转弯），余量取 pad（0.15°≈17km）
        // ——zz_region_block2 矩形墙占满 region 短边时，绕行要出墙 bbox 0.05°+；
        // 圆墙绕行贴圆边（FMM 绕膨胀圆，需求 ≈ inflation 2km + 转弯 ≈ 0.03°）——
        // 大余量会把 bbox 本就接近任务边界的圆撑出 region，触发 terrain 墙格点翻转
        // （zz19：r100 圆西缘 +0.15° 超任务西缘 0.045° → masked 场无解 → probe unmasked
        // 丢 restricted 剖面语义）。圆墙用小余量 CIRCLE_MARGIN。
        match &z.shape {
            ZoneShape::Circle { center, radius_km } => {
                let r = radius_km / 111.32 + REGION_CIRCLE_MARGIN_DEG;
                min_lon = min_lon.min(center[0] - r);
                max_lon = max_lon.max(center[0] + r);
                min_lat = min_lat.min(center[1] - r);
                max_lat = max_lat.max(center[1] + r);
            }
            ZoneShape::Polygon { vertices } => {
                for v in vertices {
                    min_lon = min_lon.min(v[0] - pad);
                    max_lon = max_lon.max(v[0] + pad);
                    min_lat = min_lat.min(v[1] - pad);
                    max_lat = max_lat.max(v[1] + pad);
                }
            }
        }
        has_wall = true;
    }
    if !has_wall {
        return region;
    }
    // 并集保持方形（span 取长边；两侧已含 pad，不再额外加）
    let span = (max_lon - min_lon).max(max_lat - min_lat);
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
        // 采样点数与单点判定共用 smooth::terrain_sample_count / terrain_point_clearance
        // 原语（阶段1-A，2026-08-11）——两处口径单一来源，杜绝 zz29 相位差类漏检。
        if let Some(t) = terrain {
            let seg_len_m = crate::path::haversine_m(lon1, lat1, lon2, lat2);
            // 目标间隔 ~200m（7.5as 地形 ~230m 分辨率），下限 2（同 verify 下限语义
            // min_samples.max(2)），上限 1024（≈205km，防超长段退化）——与 verify
            // 同函数同间隔，仅上限收紧（Theta* 每候选段都查地形，性能关键）。
            let n_t = crate::smooth::terrain_sample_count(seg_len_m, 2, 1024);
            for i in 0..=n_t {
                let tt = i as f64 / n_t as f64;
                let lon = lon1 + (lon2 - lon1) * tt;
                let lat = lat1 + (lat2 - lat1) * tt;
                let alt = alt1 + (alt2 - alt1) * tt;
                match crate::smooth::terrain_point_clearance(t, lon, lat, alt, clearance_m) {
                    crate::smooth::TerrainPointClearance::Fail(_) => return false,
                    // 空洞不硬拒（降级警告由 verify 汇总）；OOB 同空洞（2026-08-11）
                    crate::smooth::TerrainPointClearance::NoData
                    | crate::smooth::TerrainPointClearance::OutOfBounds => {}
                    crate::smooth::TerrainPointClearance::Ok => {}
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
                // restricted 多边形：解析求交成带（线段×各边交点参数）→ 带内加密采样。
                // 旧 N=16 整段等距采样会漏掉长段上的短穿带（2026-08-12 主管 rz_poly2：
                // 250km theta_star 拉直弦穿多边形北端 ~15km，16 等距点恰全落带外 →
                // check 放行直穿弦 → verify 拒 → 全链 smooth 失败 → 无解）。
                let ZoneShape::Polygon { vertices } = &z.shape else {
                    continue; // 非圆非墙（理论不可达）
                };
                let bands = segment_polygon_bands_t(lon1, lat1, lon2, lat2, vertices);
                for (t1, t2) in bands {
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
                // 层 2：整段等距采样（与 verify 的 sample inside zone 完全同口径；
                // 求交退化/共线兜底，双保险）
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

/// 线段与多边形（经纬度平面，中纬等距缩放，同 zone_segment_clearance_km 口径）
/// 各边求交 → 穿行参数区间列表（进/出成对）。端点在内 → 补 0/1。共线退化保守
/// 并入边端点投影参数。2026-08-12 主管 rz_poly2：make_segment_check 多边形分支
/// 用此函数替代 N=16 整段等距采样（长段短穿带漏检）。
fn segment_polygon_bands_t(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    vertices: &[[f64; 2]],
) -> Vec<(f64, f64)> {
    if vertices.len() < 3 {
        return Vec::new();
    }
    let mlat = ((lat1 + lat2) / 2.0).to_radians();
    let kx = 111.320 * mlat.cos();
    let ky = 111.0;
    let (ax, ay) = (lon1 * kx, lat1 * ky);
    let (bx, by) = (lon2 * kx, lat2 * ky);
    let (dx1, dy1) = (bx - ax, by - ay);
    let len2 = dx1 * dx1 + dy1 * dy1;
    if len2 < 1e-12 {
        return Vec::new();
    }
    let mut ts: Vec<f64> = Vec::new();
    let mut j = vertices.len() - 1;
    for i in 0..vertices.len() {
        let (cx, cy) = (vertices[j][0] * kx, vertices[j][1] * ky);
        let (dx2, dy2) = (vertices[i][0] * kx - cx, vertices[i][1] * ky - cy);
        let denom = dx1 * dy2 - dy1 * dx2;
        let (rx, ry) = (cx - ax, cy - ay);
        if denom.abs() > 1e-9 {
            let t = (rx * dy2 - ry * dx2) / denom;
            let u = (rx * dy1 - ry * dx1) / denom;
            if t >= -1e-9 && t <= 1.0 + 1e-9 && u >= -1e-9 && u <= 1.0 + 1e-9 {
                ts.push(t.clamp(0.0, 1.0));
            }
        } else if (rx * dy1 - ry * dx1).abs() < 1e-6 {
            // 平行且共线：边两端点投影到段上的参数并入（重合段保守采样）
            let te0 = ((cx - ax) * dx1 + (cy - ay) * dy1) / len2;
            let te1 = ((cx + dx2 - ax) * dx1 + (cy + dy2 - ay) * dy1) / len2;
            ts.push(te0.clamp(0.0, 1.0));
            ts.push(te1.clamp(0.0, 1.0));
        }
        j = i;
    }
    if crate::config::point_in_polygon_xy(lon1, lat1, vertices) {
        ts.push(0.0);
    }
    if crate::config::point_in_polygon_xy(lon2, lat2, vertices) {
        ts.push(1.0);
    }
    ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    let mut bands: Vec<(f64, f64)> = Vec::new();
    let mut k = 0;
    while k + 1 < ts.len() {
        let (t0, t1) = (ts[k], ts[k + 1]);
        if t1 - t0 > 1e-9 {
            bands.push((t0.max(0.0), t1.min(1.0)));
        }
        k += 2;
    }
    bands
}

/// Restricted 是否按该飞行高度视为禁行墙（底部可通行语义，主管 2026-08-06）：
/// 飞行高度落在 restricted 高度区间内 → 该机 FMM 画墙绕行（否则直穿）。
/// pub(crate)：patch.rs 限飞区弦判据边检查复用（docs/12 §3.3）。
pub(crate) fn restricted_blocks_alt(z: &Zone, alt_m: f64) -> bool {
    matches!(z.zone_type, crate::config::ZoneType::Restricted)
        && z.alt_min_m.is_some_and(|lo| alt_m >= lo)
        && z.alt_max_m.is_some_and(|hi| alt_m <= hi)
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

/// 段对全部硬墙 zone（NoFly/Obstacle）的净距是否 ≥ inflation（与 smooth verify 的
/// clearance 检查同口径：zone_segment_clearance_km）。穿入（clr≤1e-9）或不足 → false。
fn seg_zone_clearance_ok(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    zones: &[crate::config::Zone],
    infl_m: f64,
) -> bool {
    let infl_km = infl_m / 1000.0;
    zones
        .iter()
        .filter(|z| z.is_wall())
        .all(|z| {
            let clr = crate::config::zone_segment_clearance_km(lon1, lat1, lon2, lat2, z);
            clr > 1e-9 && clr >= infl_km
        })
}

/// boundary arc 插入前净距预检：arc 各相邻段 + 弧末点 E→出段第二点 c2 都必须满足
/// 墙净距 ≥ inflation（2026-08-11 主管输入：U 形弧采样偏墙 → arc 后段 1.90km < 2.00km
/// → final verify 拒 → 全链回退 raw 网格楼梯）。arc_pts[0] 通常为 b（keep_b 弧）。
fn seg_zone_clearance_ok_arc(
    arc_pts: &[crate::path::PathPoint],
    e: &crate::path::PathPoint,
    c2: &crate::path::PathPoint,
    zones: &[crate::config::Zone],
    infl_m: f64,
) -> bool {
    let mut prev = arc_pts.first().copied();
    for p in arc_pts.iter().skip(1) {
        if let Some(pp) = prev {
            if !seg_zone_clearance_ok(pp.lon, pp.lat, p.lon, p.lat, zones, infl_m) {
                return false;
            }
        }
        prev = Some(*p);
    }
    seg_zone_clearance_ok(e.lon, e.lat, c2.lon, c2.lat, zones, infl_m)
}


/// 受限区穿行剖面高度决策（主管 2026-08-06 二轮：比较顶部绕飞与底部穿行的代价，选更优）。
/// 仅对高度区间内的 restricted 调用；返回：
/// - Some(pass_alt)：可剖面穿行——底部（pass=alt_min−500m）与顶部（pass=alt_max+500m）
///   都评估后选更优：两者水平路径同为直线（总水平距离相同），仅垂直机动总量不同
///   （底部 2×|alt−alt_min−500|，顶部 2×|alt_max+500−alt|）→ 垂直机动更少的底部恒优；
///   底部不可行（穿行区地形挡住底部 / 爬升距离不足）→ 顶部绕飞（高于任何地形，且须
///   ≤ 升限 ceiling_m）；
/// 点到多边形边界最近距离（km；平面近似，与 pt_seg_dist_km 同口径）。
/// 点在多边形内部（含边界）→ 0（穿行起点在带内 → 爬升过渡距离不满足）。
fn pt_polygon_boundary_km(lon: f64, lat: f64, vertices: &[[f64; 2]]) -> f64 {
    if vertices.len() < 3 {
        return f64::MAX;
    }
    if point_in_polygon_xy(lon, lat, vertices) {
        return 0.0;
    }
    let mut best = f64::MAX;
    let mut j = vertices.len() - 1;
    for i in 0..vertices.len() {
        let d = pt_seg_dist_km(
            vertices[j][0],
            vertices[j][1],
            vertices[i][0],
            vertices[i][1],
            lon,
            lat,
        );
        best = best.min(d);
        j = i;
    }
    best
}

/// 直线段在多边形**内部**的归一化参数带列表（u∈[0,1]；凸多边形 0~1 个，凹多边形多个）。
/// 局部平面近似（中点纬度等距投影，与圆分支的解析二次方程同口径）。返回空 = 直线不穿
/// 多边形。算法：直线与每条边求交参数 t 收集去重 → 相邻交点对中点判定多边形内 → 成带；
/// start/end 端点本身在内时补 [0,first]/[last,1]。
fn line_polygon_inside_bands(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    vertices: &[[f64; 2]],
) -> Vec<(f64, f64)> {
    if vertices.len() < 3 {
        return Vec::new();
    }
    let mlat = ((lat1 + lat2) / 2.0).to_radians();
    let kx = mlat.cos() * 111.32;
    let ky = 111.32;
    let (ax, ay) = (lon1 * kx, lat1 * ky);
    let (bx, by) = (lon2 * kx, lat2 * ky);
    let (rx, ry) = (bx - ax, by - ay);
    let mut ts: Vec<f64> = Vec::new();
    let n = vertices.len();
    let mut j = n - 1;
    for i in 0..n {
        let (cx, cy) = (vertices[j][0] * kx, vertices[j][1] * ky);
        let (dx, dy) = (vertices[i][0] * kx, vertices[i][1] * ky);
        let (sx, sy) = (dx - cx, dy - cy);
        let denom = rx * sy - ry * sx;
        if denom.abs() < 1e-12 {
            j = i; // 平行/共线 → 无唯一交点（贴边保守由中点判定兜底）
            continue;
        }
        let (qpx, qpy) = (cx - ax, cy - ay);
        let t = (qpx * sy - qpy * sx) / denom;
        let s = (qpx * ry - qpy * rx) / denom;
        if t >= -1e-9 && t <= 1.0 + 1e-9 && s >= -1e-9 && s <= 1.0 + 1e-9 {
            ts.push(t.clamp(0.0, 1.0));
        }
        j = i;
    }
    ts.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
    ts.dedup_by(|a, b| (*a - *b).abs() < 1e-9);
    if ts.is_empty() {
        // 无边交点：全段在内（start/target 都在多边形内）或全段在外
        return if point_in_polygon_xy(lon1, lat1, vertices) {
            vec![(0.0, 1.0)]
        } else {
            Vec::new()
        };
    }
    let start_in = point_in_polygon_xy(lon1, lat1, vertices);
    let end_in = point_in_polygon_xy(lon2, lat2, vertices);
    let mut bands: Vec<(f64, f64)> = Vec::new();
    if start_in {
        bands.push((0.0, ts[0]));
    }
    for k in 0..ts.len().saturating_sub(1) {
        let u = (ts[k] + ts[k + 1]) / 2.0;
        let (lon, lat) = (lon1 + (lon2 - lon1) * u, lat1 + (lat2 - lat1) * u);
        if point_in_polygon_xy(lon, lat, vertices) {
            bands.push((ts[k], ts[k + 1]));
        }
    }
    if end_in {
        bands.push((*ts.last().unwrap(), 1.0));
    }
    bands
}

/// - None：底部与顶部都不可行 → 需画墙水平绕行（fallback 保底）。
/// 圆/多边形 restricted 同语义：底部穿行（alt_min−500m）恒优于顶部绕飞
/// （alt_max+500m）；底部被穿行带地形挡住 → 顶部；都不可行 → None（水平绕行）。
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
    let ZoneShape::Circle { center, radius_km } = &z.shape else {
        // 多边形：d_in/d_out = 点到多边形边界最近距离（点在内 → 0 → 过渡距离不足）
        let ZoneShape::Polygon { vertices } = &z.shape else {
            return None;
        };
        // 限飞区高度区间必须存在（validate 已强制 Restricted 提供 [alt_min, alt_max]）；
        // 缺失时按全高度处理（退化输入兜底，不 panic）。
        let Some(min_alt) = z.alt_min_m else { return None };
        let Some(max_alt) = z.alt_max_m else { return None };
        let bottom = min_alt - 500.0;
        let top = max_alt + 500.0;
        let climb_dist = |pass: f64| -> f64 {
            if max_climb_deg > 0.1 {
                (alt_m - pass).abs() / max_climb_deg.to_radians().tan() * 1.25
            } else {
                f64::INFINITY
            }
        };
        let d_in = pt_polygon_boundary_km(start.lon, start.lat, vertices);
        let d_out = pt_polygon_boundary_km(target.lon, target.lat, vertices);
        let fit = |pass: f64| d_in * 1000.0 >= climb_dist(pass) && d_out * 1000.0 >= climb_dist(pass);
        // 顶部绕飞可行性：爬升距离 + 升限（alt_max + 500 ≤ ceiling）
        let top_ok = fit(top) && ceiling_m.map_or(true, |c| top <= c);
        // 底部穿行可行性：爬升距离 + 底部严格低于 alt_min + 穿行带（直线穿多边形段）地形
        let bottom_ok = bottom >= 0.0
            && bottom < min_alt
            && fit(bottom)
            && bottom_terrain_ok(z, terrain, bottom, start, target, raw_band);
        return match (bottom_ok, top_ok) {
            (true, _) => Some(bottom), // 底部垂直机动总量更小 → 恒更优
            (false, true) => Some(top),
            (false, false) => None,
        };
    };
    // 限飞区高度区间必须存在（validate 已强制 Restricted 提供 [alt_min, alt_max]）；
    // 缺失时按全高度处理（退化输入兜底，不 panic）。
    let Some(min_alt) = z.alt_min_m else { return None };
    let Some(max_alt) = z.alt_max_m else { return None };
    let bottom = min_alt - 500.0;
    let top = max_alt + 500.0;
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
        && bottom < min_alt
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
    // raw 穿行段优先：底部剖面实际沿 raw 子段平飞，地形按该子段采样（含进出点）。
    // 2026-08-11 zz31：raw 网格点间隔 ≈ cell（大 span → 2325m），窄山峰（7.5as
    // ~230m 格）落在点间 → 漏检 → 底部误判可行 → 1500m 剖面穿 1421m 峰（净空
    // 79-98m < 100m）→ final verify 拒 → 5468 点锯齿。改沿 raw 子段加密采样
    // （间隔 ≤200m，同 verify 口径）。
    if let Some((raw, i_a, i_b)) = raw_band {
        let (lo, hi) = (i_a.min(i_b), i_a.max(i_b));
        let mut seg_len_m = 0.0_f64;
        for k in lo + 1..=hi {
            seg_len_m += crate::path::haversine_m(
                raw[k - 1].lon,
                raw[k - 1].lat,
                raw[k].lon,
                raw[k].lat,
            );
        }
        // raw 相邻点间距 ≈ cell（8 邻域回溯），按点序号线性插值足够
        let n = ((seg_len_m / 200.0).ceil() as usize).clamp(8, 2048);
        let mut max_terr: Option<f64> = None;
        for j in 0..=n {
            let u = j as f64 / n as f64;
            let idx = lo as f64 + u * (hi - lo) as f64;
            let i0 = idx.floor() as usize;
            let i1 = (i0 + 1).min(hi);
            let f = idx - i0 as f64;
            let lon = raw[i0].lon + (raw[i1].lon - raw[i0].lon) * f;
            let lat = raw[i0].lat + (raw[i1].lat - raw[i0].lat) * f;
            if let Sample::Land(h) = t.sample_at(lon, lat) {
                max_terr = Some(max_terr.map_or(h, |m: f64| m.max(h)));
            }
        }
        return match max_terr {
            Some(h) => h + 100.0 <= bottom, // 净空满足 → 底部可行
            None => true,                    // 穿行段无陆地（水面/无数据）→ 直穿
        };
    }
    // 平面近似：start→target 直线穿行带（圆：解析二次方程；多边形：边交点成带）
    let mlat = ((start.lat + target.lat) / 2.0).to_radians();
    let kx = mlat.cos() * 111.32;
    let ky = 111.32;
    let dx = (target.lon - start.lon) * kx;
    let dy = (target.lat - start.lat) * ky;
    let a = dx * dx + dy * dy;
    if a < 1e-9 {
        return true; // start/target 重合（调用方已过滤）
    }
    let bands: Vec<(f64, f64)> = match &z.shape {
        ZoneShape::Circle { center, radius_km } => {
            let cx = (center[0] - start.lon) * kx;
            let cy = (center[1] - start.lat) * ky;
            let b = -2.0 * (dx * cx + dy * cy);
            let c = cx * cx + cy * cy - radius_km * radius_km;
            let disc = b * b - 4.0 * a * c;
            if disc <= 0.0 {
                Vec::new()
            } else {
                let sq = disc.sqrt();
                let u_in = ((-b - sq) / (2.0 * a)).max(0.0);
                let u_out = ((-b + sq) / (2.0 * a)).min(1.0);
                if u_out <= u_in { Vec::new() } else { vec![(u_in, u_out)] }
            }
        }
        ZoneShape::Polygon { vertices } => line_polygon_inside_bands(
            start.lon,
            start.lat,
            target.lon,
            target.lat,
            vertices,
        ),
    };
    if bands.is_empty() {
        return true; // 直线不穿 restricted（FMM 直穿不经过）
    }
    // 沿穿行带采样地形：步长 ~200m（同 verify 口径；2.2km 会漏窄峰——2026-08-11
    // zz31 1421m 峰 ~230m 格；clamp [8,2048] 覆盖 ≤400km 穿行段）
    let mut max_terr: Option<f64> = None;
    for (u_in, u_out) in bands {
        let seg_km = (u_out - u_in) * a.sqrt();
        let n = ((seg_km / 0.2).round() as usize).clamp(8, 2048);
        for k in 0..=n {
            let u = u_in + (u_out - u_in) * (k as f64 / n as f64);
            let lon = start.lon + (target.lon - start.lon) * u;
            let lat = start.lat + (target.lat - start.lat) * u;
            if let Sample::Land(h) = t.sample_at(lon, lat) {
                max_terr = Some(max_terr.map_or(h, |m: f64| m.max(h)));
            }
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

/// 过渡直线（desc_in / out_climb）是否穿任一 restricted 圆/多边形高度带：
/// 线段与圆/多边形求交区间内采样 8 点，线性高度插值落入 [alt_min, alt_max] 即命中。
/// 用于 build 剖面段防御——进入任何 restricted 圆/多边形时高度必须已在带外/带下
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
        // 端点本身在带内（退化/零长度段覆盖；圆/多边形同口径 zone_contains）
        let in_band_at = |lon: f64, lat: f64, alt: f64| -> bool {
            match Geo::new(lon, lat) {
                Ok(g) => crate::config::zone_contains(z, &g)
                    && z.alt_min_m.is_some_and(|lo| alt >= lo)
                    && z.alt_max_m.is_some_and(|hi| alt <= hi),
                Err(_) => false,
            }
        };
        if in_band_at(lon1, lat1, alt1) || in_band_at(lon2, lat2, alt2) {
            return true;
        }
        let bands: Vec<(f64, f64)> = match &z.shape {
            crate::config::ZoneShape::Circle { center, radius_km } => {
                match segment_circle_intersect_t(
                    lon1,
                    lat1,
                    lon2,
                    lat2,
                    center[0],
                    center[1],
                    *radius_km,
                ) {
                    Some((t1, t2)) => vec![(t1, t2)],
                    None => Vec::new(),
                }
            }
            crate::config::ZoneShape::Polygon { vertices } => {
                line_polygon_inside_bands(lon1, lat1, lon2, lat2, vertices)
            }
        };
        for (b1, b2) in bands {
            for kk in 0..=8 {
                let tt = b1 + (b2 - b1) * kk as f64 / 8.0;
                let alt = alt1 + (alt2 - alt1) * tt;
                if z
                    .alt_min_m
                    .is_some_and(|lo| alt >= lo)
                    && z.alt_max_m.is_some_and(|hi| alt <= hi)
                {
                    return true;
                }
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
    // 该机高度拦截的 restricted 圆/多边形（底部/顶部剖面穿行类型；多边形 2026-08-12
    // 主管 rz_poly 场景：三角形 [2000,6000]msl，巡航 2282m 直穿带内 → 顶部/底部剖面）
    let hits: Vec<&Zone> = zones
        .iter()
        .filter(|z| restricted_blocks_alt(z, alt_m))
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
        // 找进入索引（第一个与圆/多边形相交段的起点）与最后带内点（穿出点）：
        // 逐格点判定会漏掉浅穿（锯齿格点全在外但线段穿入，如 new_rz 垂距 19.5km
        // 仅穿入 0.5km）→ 改用"段与 zone 相交"（zone_segment_clearance_km ≤ 0）。
        // 多边形凹形时取 [首次进入, 最后穿出] 覆盖段（凹口外以 pass_alt 平飞合法）。
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
            let crossing = d <= 1e-9 || a_in || b_in; // 段与 zone 相交/端点在内
            if crossing && !in_circle {
                // 凸 zone：in_idx 只取第一次进入（raw 锯齿在边界摆动时可能"出→再进"，
                // 覆盖 in_idx 会把剖面起点推迟到最后一个进入点 → 首段含带内 3000m 违规）
                if in_idx.is_none() {
                    in_idx = Some(i);
                }
                in_circle = true;
            }
            if in_circle {
                if b_in {
                    out_idx = Some(i + 1); // 带内区间持续（终点更新）
                } else if !crossing {
                    // 段已完全在 zone 外 → 凸，区间结束（out = 段起点，带外）
                    out_idx = Some(i);
                    in_circle = false;
                } else {
                    // crossing && !b_in：段从带内穿出 → out = 段终点（带外点）。
                    // out 必须是带外点——否则 out→climb 爬升过渡从带内开始，
                    // 高度 500→1000+ 进入 restricted 区间违规（verify alt band 拒）。
                    out_idx = Some(i + 1);
                }
            }
        }
        let (Some(i_in), Some(i_out)) = (in_idx, out_idx) else {
            // raw 未穿该 zone：FMM 网格离散擦边（浅穿深度 < 格距，格点全在带外）时
            // 真实几何仍穿 zone → 平滑拉直会穿 zone 违规（verify 几何精确拦截）→ 回退锯齿。
            // fallback：start→target 直线穿 zone（且直线避开全部硬墙）→ 直线参数化剖面
            // （1b1331b 旧方案；仅 raw 未穿时启用，主管 2026-08-06 三轮架构保留）。
            // 2026-08-11 zz31：底部判定必须用**段首尾**（p0/p1，与剖面 in_out 一致）。
            // 传全局 start/target 时，若全局直线不穿 zone（v2 (103.8,32.5)→(124.7,53.3)
            // 不穿 rz2）→ bottom_terrain_ok 平面分支不穿 → 直接放行底部 1500m，
            // 但剖面沿段直线（wp4→wp5）穿 zone 经过 1421m 峰 → 净空 79-98m < 100m →
            // final verify 拒 → 5468 点锯齿（zigzag21 只修了 raw_band 分支，此分支漏）。
            let p0g = Geo::new(p0.lon, p0.lat).unwrap_or(*start);
            let p1g = Geo::new(p1.lon, p1.lat).unwrap_or(*target);
            let Some(pass_alt) = restricted_pass_alt(
                z,
                alt_m,
                ceiling_m,
                terrain,
                &p0g,
                &p1g,
                max_climb_deg,
                None,
            ) else {
                continue;
            };
            if line_hits_wall_km(p0.lon, p0.lat, p1.lon, p1.lat, zones, inflation_km) {
                continue; // 直线穿硬墙（如 no_fly）→ 直线剖面不可用
            }
            // 穿行带：圆 = 解析二次方程（同 bottom_terrain_ok 口径）；多边形 = 边交点成带
            let mlat = ((p0.lat + p1.lat) / 2.0).to_radians();
            let kx = mlat.cos() * 111.32;
            let ky = 111.32;
            let ddx = (p1.lon - p0.lon) * kx;
            let ddy = (p1.lat - p0.lat) * ky;
            let aa = ddx * ddx + ddy * ddy;
            let pass_bands: Vec<(f64, f64)> = match &z.shape {
                ZoneShape::Circle { center, radius_km } => {
                    let oox = (center[0] - p0.lon) * kx;
                    let ooy = (center[1] - p0.lat) * ky;
                    let bb = -2.0 * (ddx * oox + ddy * ooy);
                    let cc = oox * oox + ooy * ooy - radius_km * radius_km;
                    let disc = bb * bb - 4.0 * aa * cc;
                    if disc <= 0.0 {
                        Vec::new()
                    } else {
                        let sq = disc.sqrt();
                        let u1v = ((-bb - sq) / (2.0 * aa)).clamp(0.0, 1.0);
                        let u2v = ((-bb + sq) / (2.0 * aa)).clamp(0.0, 1.0);
                        vec![(u1v, u2v)]
                    }
                }
                ZoneShape::Polygon { vertices } => {
                    line_polygon_inside_bands(p0.lon, p0.lat, p1.lon, p1.lat, vertices)
                }
            };
            let Some((fst, lst)) = pass_bands.first().zip(pass_bands.last()) else {
                continue;
            };
            let (u1, u2) = (fst.0, lst.1);
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
        // 点是否位于**其他** restricted 圆/多边形带内（排除自身）。2026-08-08 主管 zigzag22：
        // rz2 的 out_climb 过渡完成点选在 rz1 圆内（距圆心 41km<50km，@3000m 在 rz1
        // 带内）→ rz1 处理时 tail 起点在圆内 → in_idx=0 → head 退化成单点带内段 →
        // 平滑全败 → join 后 FINAL 在 rz1 处带内直穿 → 1967 点锯齿。desc/climb 锚点
        // 都必须避开其他 restricted 圆/多边形带内点。
        let in_other_band = |lon: f64, lat: f64| -> bool {
            zones.iter().any(|z2| {
                if std::ptr::eq(z2, z) {
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

    // ---------- P6-C 多机交叉检测 ----------
    fn vo(id: &str, pts: &[(f64, f64, f64)]) -> VehicleOutput {
        VehicleOutput {
            id: id.into(),
            status: "planned".into(),
            path: pts
                .iter()
                .map(|&(x, y, alt)| PathPoint {
                    x,
                    y,
                    alt_m: alt,
                })
                .collect(),
            distance_m: 0.0,
            warnings: Vec::new(),
        }
    }

    // ---------- 垂直剖面（2026-08-12 demo 轨迹倾斜） ----------
    fn vpp(lon: f64, lat: f64, alt: f64) -> crate::path::PathPoint {
        crate::path::PathPoint::new(lon, lat, alt)
    }

    #[test]
    fn vertical_profile_linear_climb() {
        // 起点 3000 → 目标 8000：三点等距路径 → 中间点高度 = 线性中点 5500
        let mut pts = vec![
            vpp(115.0, 39.0, 3000.0),
            vpp(115.75, 39.45, 3000.0),
            vpp(116.5, 39.9, 3000.0),
        ];
        apply_vertical_profile(&mut pts, 3000.0, 8000.0, 3000.0, None, 100.0);
        assert!((pts[0].alt_m - 3000.0).abs() < 0.5, "start {}", pts[0].alt_m);
        assert!((pts[1].alt_m - 5500.0).abs() < 200.0, "mid {}", pts[1].alt_m);
        assert!((pts[2].alt_m - 8000.0).abs() < 0.5, "end {}", pts[2].alt_m);
    }

    #[test]
    fn vertical_profile_same_alt_unchanged() {
        // 起终点同高 → 高度保持原样（受限区剖面/抬升语义不变）
        let mut pts = vec![vpp(115.0, 39.0, 3000.0), vpp(116.5, 39.9, 3000.0)];
        apply_vertical_profile(&mut pts, 3000.0, 3000.0, 3000.0, None, 100.0);
        assert!(pts.iter().all(|p| (p.alt_m - 3000.0).abs() < 1e-9));
    }

    #[test]
    fn vertical_profile_descent_terrain_floor() {
        // 下降 8000→3000，中间点地形 6000m：插值 5500 被地形保底抬到 ≥ 6100（净空+100）
        struct FakeTerrain;
        impl TerrainSource for FakeTerrain {
            fn sample_at(&self, _lon: f64, _lat: f64) -> Sample {
                Sample::Land(6000.0)
            }
            fn height_at(&self, _lon: f64, _lat: f64) -> Option<f64> {
                Some(6000.0)
            }
            fn bounds(&self) -> Option<crate::terrain::GeoBounds> {
                None
            }
            fn resolution_desc(&self) -> String {
                "fake".into()
            }
        }
        let mut pts = vec![
            vpp(115.0, 39.0, 8000.0),
            vpp(115.75, 39.45, 8000.0),
            vpp(116.5, 39.9, 8000.0),
        ];
        apply_vertical_profile(&mut pts, 8000.0, 3000.0, 8000.0, Some(&FakeTerrain), 100.0);
        assert!((pts[0].alt_m - 8000.0).abs() < 0.5, "start {}", pts[0].alt_m);
        assert!(pts[1].alt_m >= 6100.0, "mid floor {}", pts[1].alt_m);
        // 终点 3000 < 地形 6000（物理不可达）→ 保底抬到 ≥ 6100（宁高勿穿山）
        assert!(pts[2].alt_m >= 6100.0, "end floor {}", pts[2].alt_m);
    }

    #[test]
    fn vertical_profile_raised_cruise_floor() {
        // 抬升场景：start 500 目标 3000、巡航已抬升 2240（raw 路径点高度 = 2240）
        // → 中间高度不得低于 2240；终点 = 目标 3000。
        let mut pts = vec![
            vpp(115.0, 39.0, 2240.0),
            vpp(115.75, 39.45, 2240.0),
            vpp(116.5, 39.9, 2240.0),
        ];
        apply_vertical_profile(&mut pts, 500.0, 3000.0, 2240.0, None, 100.0);
        assert!(pts[0].alt_m >= 2240.0 - 0.5, "start floor {}", pts[0].alt_m);
        assert!(pts[1].alt_m >= 2240.0 - 0.5, "mid floor {}", pts[1].alt_m);
        assert!((pts[2].alt_m - 3000.0).abs() < 0.5, "end {}", pts[2].alt_m);
    }

    #[test]
    fn vertical_profile_keeps_restricted_profile_segment() {
        // 受限区底部穿行剖面段（1500m，zigzag24）：原高度显著偏离巡航 2242 →
        // 保持原高度，垂直剖面不覆盖；巡航段（== 2242）正常插值（含终点 raise_floor）。
        let mut pts = vec![
            vpp(115.0, 39.0, 2242.0),   // 巡航段
            vpp(115.5, 39.5, 1500.0),   // 剖面段（底部穿行）
            vpp(116.0, 40.0, 2242.0),   // 巡航段
            vpp(116.5, 40.5, 2242.0),   // 终点（raw 巡航）
        ];
        apply_vertical_profile(&mut pts, 1000.0, 1500.0, 2242.0, None, 100.0);
        assert!(pts[0].alt_m >= 2242.0 - 0.5, "cruise start {}", pts[0].alt_m);
        assert!(
            (pts[1].alt_m - 1500.0).abs() < 0.5,
            "profile seg kept {}",
            pts[1].alt_m
        );
        assert!(pts[2].alt_m >= 2242.0 - 0.5, "cruise mid {}", pts[2].alt_m);
        // 终点 = max(target 1500, raise_floor 2242) = 2242（目标低于抬升巡航 → 保持抬升）
        assert!(pts[3].alt_m >= 2242.0 - 0.5, "end {}", pts[3].alt_m);
    }

    #[test]
    fn p6c_detect_crossing() {
        // 两条路径在中段交叉（x 形），高度相同 → 两车都应出 multi_vehicle_conflict
        let mut vs = vec![
            vo("v1", &[(115.0, 39.0, 3000.0), (116.5, 40.0, 3000.0)]),
            vo("v2", &[(116.5, 39.0, 3000.0), (115.0, 40.0, 3000.0)]),
        ];
        detect_multi_vehicle_crossings(&mut vs);
        assert!(
            vs[0]
                .warnings
                .iter()
                .any(|w| w.contains("multi_vehicle_conflict") && w.contains("v2")),
            "v1 warnings: {:?}",
            vs[0].warnings
        );
        assert!(
            vs[1]
                .warnings
                .iter()
                .any(|w| w.contains("multi_vehicle_conflict") && w.contains("v1")),
            "v2 warnings: {:?}",
            vs[1].warnings
        );
    }

    #[test]
    fn p6c_no_crossing_far_apart() {
        // 平行远离（水平 ~133km > 1km）→ 无告警
        let mut vs = vec![
            vo("v1", &[(115.0, 39.0, 3000.0), (116.0, 39.0, 3000.0)]),
            vo("v2", &[(115.0, 40.2, 3000.0), (116.0, 40.2, 3000.0)]),
        ];
        detect_multi_vehicle_crossings(&mut vs);
        assert!(vs[0].warnings.is_empty(), "{:?}", vs[0].warnings);
        assert!(vs[1].warnings.is_empty(), "{:?}", vs[1].warnings);
    }

    #[test]
    fn p6c_no_crossing_alt_separated() {
        // 同水平位置但高度差 2000m > 150m → 无告警
        let mut vs = vec![
            vo("v1", &[(115.5, 39.5, 3000.0), (116.0, 39.5, 3000.0)]),
            vo("v2", &[(115.5, 39.5, 5000.0), (116.0, 39.5, 5000.0)]),
        ];
        detect_multi_vehicle_crossings(&mut vs);
        assert!(vs[0].warnings.is_empty(), "{:?}", vs[0].warnings);
        assert!(vs[1].warnings.is_empty(), "{:?}", vs[1].warnings);
    }

    #[test]
    fn p6c_single_vehicle_no_detection() {
        let mut vs = vec![vo("v1", &[(115.0, 39.0, 3000.0), (116.0, 39.0, 3000.0)])];
        detect_multi_vehicle_crossings(&mut vs);
        assert!(vs[0].warnings.is_empty(), "{:?}", vs[0].warnings);
    }

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
            alt_min_m: Some(2000.0),
            alt_max_m: Some(5000.0),
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
            alt_min_m: Some(2000.0),
            alt_max_m: Some(5000.0),
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
            alt_min_m: Some(0.0),
            alt_max_m: Some(5000.0),
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
            alt_min_m: Some(2000.0),
            alt_max_m: Some(5000.0),
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
    fn line_polygon_inside_bands_triangle_and_concave() {
        // 三角形（主管 rz_poly 场景）：直线斜穿 → 单个带；带端点应在三角形边上
        let tri = [[116.90767296501929, 40.99465146600213], [115.36066171773774, 40.05644513239613], [116.34715136271842, 40.336691478802884]];
        let bands = line_polygon_inside_bands(117.5633, 38.9892, 115.0644, 41.1679, &tri);
        assert_eq!(bands.len(), 1, "三角形穿行应单带，实际 {:?}", bands);
        let (u1, u2) = bands[0];
        assert!(u1 > 0.0 && u1 < 1.0 && u2 > u1 && u2 < 1.0);
        // 带端点（穿行出入口）应落在三角形边上：入口在 BC 边、出口在 AB 边
        let (lon_in, lat_in) = (117.5633 + (115.0644 - 117.5633) * u1, 38.9892 + (41.1679 - 38.9892) * u1);
        let (lon_out, lat_out) = (117.5633 + (115.0644 - 117.5633) * u2, 38.9892 + (41.1679 - 38.9892) * u2);
        // BC 边：B(115.3607,40.0564)→C(116.3472,40.3367)；AB 边：A(116.9077,40.9947)→B(115.3607,40.0564)
        let on_edge = |lon: f64, lat: f64, a: [f64; 2], b: [f64; 2]| {
            let d = pt_seg_dist_km(a[0], a[1], b[0], b[1], lon, lat);
            d < 0.5
        };
        assert!(
            on_edge(lon_in, lat_in, tri[1], tri[2]) || on_edge(lon_in, lat_in, tri[2], tri[0]),
            "入口 ({:.4},{:.4}) 不在三角形边上",
            lon_in,
            lat_in
        );
        assert!(
            on_edge(lon_out, lat_out, tri[0], tri[1]) || on_edge(lon_out, lat_out, tri[1], tri[2]),
            "出口 ({:.4},{:.4}) 不在三角形边上",
            lon_out,
            lat_out
        );
        // 不穿（直线在多边形外）→ 空
        let no_bands = line_polygon_inside_bands(117.5633, 38.9892, 118.5, 39.0, &tri);
        assert!(no_bands.is_empty(), "直线不穿三角形应无带，实际 {:?}", no_bands);
        // 凹四边形（U 形：底部条带 + 左右柱，中间凹口外部）：水平线 y=5 从 -1→11
        // 穿左柱 [0,4] → 凹口外 → 右柱 [6,10] → 两个带
        let concave = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0], [6.0, 10.0], [6.0, 4.0], [4.0, 4.0], [4.0, 10.0], [0.0, 10.0]];
        let cb = line_polygon_inside_bands(-1.0, 5.0, 11.0, 5.0, &concave);
        assert_eq!(cb.len(), 2, "凹多边形水平线两次进入应双带，实际 {:?}", cb);
        let (a1, a2) = cb[0];
        let (b1, b2) = cb[1];
        assert!(a1 < a2 && a2 < b1 && b1 < b2, "带序异常 {:?}", cb);
        assert!((a1 - 1.0 / 12.0).abs() < 1e-3 && (a2 - 5.0 / 12.0).abs() < 1e-3, "左柱带异常 {:?}", cb[0]);
        assert!((b1 - 7.0 / 12.0).abs() < 1e-3 && (b2 - 11.0 / 12.0).abs() < 1e-3, "右柱带异常 {:?}", cb[1]);
    }

    #[test]
    fn pt_polygon_boundary_km_basic() {
        let tri = [[0.0, 0.0], [10.0, 0.0], [10.0, 10.0]];
        // 外部点 (11,5)：到直边 x=10 距离 1°（lat≈0 → 1° 经度 ≈ 111.32km）
        let d = pt_polygon_boundary_km(11.0, 5.0, &tri);
        assert!((d - 111.32).abs() < 0.5, "外部点距直边应 ≈111.32km，实际 {d}");
        // 内部点 → 0
        let d2 = pt_polygon_boundary_km(5.0, 5.0, &tri);
        assert_eq!(d2, 0.0);
    }

    #[test]
    fn segment_polygon_bands_t_catches_short_crossing() {
        // 主管 rz_poly2（2026-08-12）：三角形 restricted [2000,6000]msl，
        // theta_star 拉直弦 start→(115.812,41.505) 穿多边形北端 ~15km。
        // 旧 N=16 整段等距采样恰好漏检（穿带 t∈[0.9385,0.9978]，16 等距点
        // 0.9375 差 250m）→ 解析求交必须捕捉到。
        let tri = [
            [116.11516863940805, 41.612788219567264],
            [114.56815739212651, 40.67458188596126],
            [115.55464703710719, 40.954828232368016],
        ];
        let bands = segment_polygon_bands_t(117.5625, 38.9921, 115.812, 41.505, &tri);
        assert!(!bands.is_empty(), "穿北端弦应有带，实际 {:?}", bands);
        let (t1, t2) = bands[0];
        assert!(t1 > 0.9 && t2 > t1, "穿带应在弦末段，实际 ({t1:.4},{t2:.4})");
        // 北端外水平弦（lat 41.7 > 顶点 41.613）→ 无带
        let outer = segment_polygon_bands_t(116.5, 41.7, 114.0, 41.7, &tri);
        assert!(outer.is_empty(), "多边形上方水平弦应无带，实际 {:?}", outer);
        // 端点在内 → 补 0/1 带（lat 41.0 内范围 [115.104,115.593]、lat 41.2 内 [115.434,115.764]）
        let inside = segment_polygon_bands_t(115.3, 41.0, 115.6, 41.2, &tri);
        assert!(!inside.is_empty(), "端点在内应有带，实际 {:?}", inside);
        let (i1, i2) = inside[0];
        assert!(i1 <= 1e-9, "起点在内应补 0，实际 {i1}");
        assert!(i2 >= 1.0 - 1e-9, "终点在内应补 1，实际 {i2}");
        // 不穿（远离多边形）→ 空
        let far = segment_polygon_bands_t(117.5, 38.9, 118.5, 39.5, &tri);
        assert!(far.is_empty(), "远离多边形应无带，实际 {:?}", far);
    }

    #[test]
    fn restricted_pass_alt_polygon_decides_bottom_or_top() {
        // 多边形 restricted [2000,6000]msl：巡航 2282m，起点/终点距多边形 > 爬升距离
        // → 底部 1500m 可行（无地形）→ 恒选底部。
        let z = Zone {
            id: "rzp".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Polygon {
                vertices: vec![
                    [116.90767296501929, 40.99465146600213],
                    [115.36066171773774, 40.05644513239613],
                    [116.34715136271842, 40.336691478802884],
                ],
            },
            alt_min_m: Some(2000.0),
            alt_max_m: Some(6000.0),
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let s = Geo::new(117.5633, 38.9892).unwrap();
        let t = Geo::new(115.0644, 41.1679).unwrap();
        let pass = restricted_pass_alt(&z, 2282.0, None, None, &s, &t, 15.0, None);
        assert_eq!(pass, Some(1500.0), "底部穿行应恒优（无地形），实际 {:?}", pass);
        // alt_min=0 → 底部 = -500 负高不可行 → 顶部 6500m
        let z0 = Zone {
            id: "rzp0".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: z.shape.clone(),
            alt_min_m: Some(0.0),
            alt_max_m: Some(6000.0),
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let pass0 = restricted_pass_alt(&z0, 2282.0, None, None, &s, &t, 15.0, None);
        assert_eq!(pass0, Some(6500.0), "alt_min=0 底部负高 → 顶部绕飞，实际 {:?}", pass0);
        // 升限低于顶部 → 顶部也不可行 → None（水平绕行兜底）
        let pass_c = restricted_pass_alt(&z0, 2282.0, Some(5000.0), None, &s, &t, 15.0, None);
        assert_eq!(pass_c, None, "升限 5000 < 顶部 6500 → 不可行，实际 {:?}", pass_c);
    }

    #[test]
    fn polygon_restricted_profile_bottom_pass() {
        // 主管 2026-08-12 rz_poly 场景（无地形版）：多边形 restricted [2000,6000]msl
        // 三角形挡在 start→target 直线上，巡航 3000m 在区间内 → 底部 1500m 剖面直穿
        //（desc→1500 平飞→climb），不水平绕行、不回退密集锯齿。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":117.56330714245705,"lat":38.98919664864976,"alt_m":3000},
                "target":{"lon":115.0643644570711,"lat":41.16789261835432,"alt_m":3000},
                "terrain":{"source":"none"},
                "restricted_zones":[{"id":"rzp","zone_type":"restricted","shape":"polygon",
                    "geometry":{"vertices":[
                        [116.90767296501929,40.99465146600213],
                        [115.36066171773774,40.05644513239613],
                        [116.34715136271842,40.336691478802884]]},
                    "alt_min_m":2000,"alt_max_m":6000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 8,
            "多边形底部剖面直穿应 ≈6 点，实际 {} 点",
            v.path.len()
        );
        assert!(
            v.path.iter().any(|p| p.alt_m < 2000.0),
            "剖面应在穿行区降高到 <2000m，实际最小 {}m",
            v.path.iter().map(|p| p.alt_m).fold(f64::MAX, f64::min)
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 穿行段长度 = 直线穿多边形的弦长（~62km）→ 总距离 ≈ 直线（323km）+ 机动余量
        assert!(
            v.distance_m < 340_000.0,
            "剖面直穿距离应 <340km（直线 323km），实际 {}km",
            v.distance_m / 1000.0
        );
    }

    #[test]
    fn polygon_restricted_profile_top_flyover() {
        // 多边形 restricted [0,6000]msl：底部 -500 负高不可行 → 顶部 6500m 绕飞剖面
        //（多边形内部 6500m 平飞，高于区间上界 6000）。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":117.56330714245705,"lat":38.98919664864976,"alt_m":3000},
                "target":{"lon":115.0643644570711,"lat":41.16789261835432,"alt_m":3000},
                "terrain":{"source":"none"},
                "restricted_zones":[{"id":"rzp","zone_type":"restricted","shape":"polygon",
                    "geometry":{"vertices":[
                        [116.90767296501929,40.99465146600213],
                        [115.36066171773774,40.05644513239613],
                        [116.34715136271842,40.336691478802884]]},
                    "alt_min_m":0,"alt_max_m":6000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.iter().any(|p| p.alt_m > 6000.0),
            "剖面应在穿行区升到 >6000m（顶部 6500），实际最大 {}m",
            v.path.iter().map(|p| p.alt_m).fold(0.0, f64::max)
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
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
            alt_min_m: Some(0.0),
            alt_max_m: Some(12000.0),
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
            alt_min_m: Some(0.0),
            alt_max_m: Some(5000.0),
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
    fn p6_time_budget_exhausted_returns_degraded_timeout() {
        // P6-B（docs/07 §5 3s 预算硬护栏）：预算=1ms（solve 前场构建即超）→
        // 无候选 → Err(DegradedTimeout) → 顶层 status=degraded_timeout。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"}
            }
        }"#;
        let input = parse(s);
        let params = SolveParams { time_budget_ms: 1, ..SolveParams::default() };
        match solve(&input, &params, 0) {
            Err(AppError::DegradedTimeout(_)) => {}
            other => panic!("expected DegradedTimeout, got {other:?}"),
        }
        // 对照：预算=0（无限）→ 正常 success（现有所有测试默认路径不受影响）
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.status, "success");
        assert_eq!(out.vehicles[0].status, "planned");
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
        // 旋翼机无 smoothing 告警（垂直剖面后可能触发 P6-C 交叉告警——真实行为）
        assert!(
            out.vehicles[1]
                .warnings
                .iter()
                .all(|w| !w.contains("smoothing")),
            "旋翼机不应有 smoothing 告警: {:?}",
            out.vehicles[1].warnings
        );
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
        // 旋翼机（uav2）无 Dubins 链（急转合法）——路径点应不因拟合失败回退；
        // 垂直剖面后两车高度靠近可能触发 P6-C 交叉告警（真实行为），仅过滤 smoothing。
        assert!(
            out.vehicles[1]
                .warnings
                .iter()
                .all(|w| !w.contains("smoothing")),
            "旋翼机不应有 smoothing 告警: {:?}",
            out.vehicles[1].warnings
        );
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
                    if d <= radius_km
                        && z.alt_min_m.is_some_and(|lo| w[0].alt_m >= lo)
                        && z.alt_max_m.is_some_and(|hi| w[0].alt_m <= hi)
                    {
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
                    if d <= radius_km
                        && z.alt_min_m.is_some_and(|lo| w[0].alt_m >= lo)
                        && z.alt_max_m.is_some_and(|hi| w[0].alt_m <= hi)
                    {
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

    #[test]
    fn zigzag28_waypoint_zigzag_verify_local_projection_radius() {
        // 主管 2026-08-10 输入（east_asia 7.5as 真实地形）：单机独立 start_pose + 10 必经点
        // + target_ref，无 zone/雷达。10 个必经点大之字形（经度 116.7~117.7、纬度 38.9~39.6
        // 交错），段边界多 >140° 大转角（wp2 144°、wp3 149.7°、wp4 116.2°、wp5 148°、wp6
        // 141.7°、wp7 158.4°、wp8 143.7°、wp9 87.4°、wp10 63.5°）。zigzag27 entry 放宽后
        // theta_star 全部拉直成功、段边界 arc 全部插入成功 → FINAL 36 点，但 verify 报
        // `vertex 23: radius 436m < min 442m` 误拒 → 全链回退 1017 点网格楼梯 + 995km。
        // 根因（zigzag28）：verify_path 用**全路径中点投影**（path.points[n/2]，lat 39.56°
        // → cos=0.7712）测三点外接圆半径，而 arc_transition 生成弧用 **b 点局部投影**
        // （lat 38.93° → cos=0.7776）；wp7 处 158.4° 切弧远离路径中点（~70km），经度缩放
        // 失配 0.8% 被放大 → 物理 442m 弧被量成 434m < 437.58（442×0.99）→ 误拒。
        // 修复：radius 检查改三点局部投影（LocalProjection 以三点均值为中心，与
        // arc_transition 生成口径一致）。结果：1017→36 点平滑，770km（之字形直线拉直），
        // smoothing_failed 消失。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag28: real terrain missing ({})", cand.display());
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
                        {"lon":117.5310926755737,"lat":39.36711249439876,"alt_m":500},
                        {"lon":117.27112137218157,"lat":39.56318138239763,"alt_m":500},
                        {"lon":117.34900609147277,"lat":39.17993766070357,"alt_m":500},
                        {"lon":117.02306561461393,"lat":39.4989067016085,"alt_m":500},
                        {"lon":116.81964533225553,"lat":39.16459697832345,"alt_m":500},
                        {"lon":117.61912491286566,"lat":39.56928673239517,"alt_m":500},
                        {"lon":117.34789241305992,"lat":38.91485862893048,"alt_m":500},
                        {"lon":117.73424167310449,"lat":39.2810150397907,"alt_m":500},
                        {"lon":117.71147898319053,"lat":38.91563411936639,"alt_m":500},
                        {"lon":116.69698769455375,"lat":38.92250726143136,"alt_m":500}],
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
            v.path.len() <= 60,
            "应平滑交付（修复前 1017 点网格楼梯），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 必经点语义：10 个必经点均需经过（容差 0.05°≈5.5km，同 m5/zigzag27 口径）。
        let mids = [
            (117.5310926755737_f64, 39.36711249439876_f64),
            (117.27112137218157_f64, 39.56318138239763_f64),
            (117.34900609147277_f64, 39.17993766070357_f64),
            (117.02306561461393_f64, 39.4989067016085_f64),
            (116.81964533225553_f64, 39.16459697832345_f64),
            (117.61912491286566_f64, 39.56928673239517_f64),
            (117.34789241305992_f64, 38.91485862893048_f64),
            (117.73424167310449_f64, 39.2810150397907_f64),
            (117.71147898319053_f64, 38.91563411936639_f64),
            (116.69698769455375_f64, 38.92250726143136_f64),
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

    #[test]
    fn zigzag29_waypoint_zigzag_uturn_arc_from_waypoint() {
        // 主管 2026-08-10 输入 3（east_asia 7.5as 真实地形）：单机独立 start_pose + 23
        // 必经点 + target_ref，无 zone/雷达。23 必经点密集大之字形，含多处 >170° 接近
        // 180° 的掉头（wp1 174.9°、wp7 174.8°、wp11 165.9°、wp13 167.9°、wp15 172.5°、
        // wp17 167°、wp21 162.4°）。zigzag27 entry 放宽 + zigzag28 局部投影 radius 后，
        // theta_star 首跳 177.8° > 175° 被挡 → SEG1 输出 3 点 → solver 段边界 arc 的
        // c=中间点（距 b 仅 11.5km）→ 切弧模型 d_m=min(r·tan(87.45°)≈10km, 0.75×11.5km)
        // 截断 → r_eff≈383m < 442m → verify radius 拒；且切弧 S/E 距 b≈10km 把必经点
        // 甩出弧外（远超 0.05°≈5.5km 容差）→ 全链回退 2549 点网格楼梯。
        // 根因（zigzag29）：**切弧模型对接近 180° 的转角几何发散**——切点距离
        // r·tan(θ/2) 在 θ→180° 时 →∞，必经点 b 被甩到弧外。修复：keep_b（必经点/段
        // 端点）改用 **U 形弧（S=b）**——弧从必经点出发（b 在弧上精确经过），半径恒
        // min_r_m，转 θ 后 E 为弧上自然点（距 b=2r·sin(θ/2)，174.9° 时仅 883m，E→c
        // 方向偏差 <3°）。同时地形采样 1km→200m（7.5as ~230m 格距），消除弧后段起点
        // 偏移导致的采样网格相位差漏检窄山峰。结果：2549→95 点平滑（1821km 之字形
        // 拉直），smoothing_failed 消失，23 必经点全部在 5.5km 容差内。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag29: real terrain missing ({})", cand.display());
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
                        {"lon":117.11783474006214,"lat":39.46984164454213,"alt_m":500},
                        {"lon":117.47656872164231,"lat":39.05202611607821,"alt_m":500},
                        {"lon":116.90637588798378,"lat":39.4496563699438,"alt_m":500},
                        {"lon":117.36422085502315,"lat":38.99039201054334,"alt_m":500},
                        {"lon":116.79495413353128,"lat":39.3882830409556,"alt_m":500},
                        {"lon":117.2970385812442,"lat":38.92409889966998,"alt_m":500},
                        {"lon":116.64313344054925,"lat":39.34130967422113,"alt_m":500},
                        {"lon":117.21571916145582,"lat":38.88579602285712,"alt_m":500},
                        {"lon":117.0496252060799,"lat":39.69033181045541,"alt_m":500},
                        {"lon":117.59846930948892,"lat":39.14600125973928,"alt_m":500},
                        {"lon":117.19239519871157,"lat":39.77515291467246,"alt_m":500},
                        {"lon":117.75328904873285,"lat":39.265771413168174,"alt_m":500},
                        {"lon":117.37639341129983,"lat":39.899859443956274,"alt_m":500},
                        {"lon":117.82723727925679,"lat":39.4482470117936,"alt_m":500},
                        {"lon":117.4781831630707,"lat":40.02988393385804,"alt_m":500},
                        {"lon":117.91417815346816,"lat":39.566639775283306,"alt_m":500},
                        {"lon":116.35343384512255,"lat":39.21191424808533,"alt_m":500},
                        {"lon":117.13763274937696,"lat":40.007607829090794,"alt_m":500},
                        {"lon":116.54022244694409,"lat":39.61959713191777,"alt_m":500},
                        {"lon":116.94054105824308,"lat":40.15250255474903,"alt_m":500},
                        {"lon":116.33463652719644,"lat":39.77092757783807,"alt_m":500},
                        {"lon":116.77908228189388,"lat":40.31455957166121,"alt_m":500},
                        {"lon":116.40841749032751,"lat":40.11731272727076,"alt_m":500}],
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
            v.path.len() <= 150,
            "应平滑交付（修复前 2549 点网格楼梯），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 必经点语义：23 个必经点均需经过（球面距离容差 0.05°≈5.5km；zigzag29 用
        // 球面距离而非矩形 dx/dy——23 点密集时 wp snap 的经度偏差可达 4-5km，lat 39.5°
        // 处 0.05° 经度只 ≈4.3km，矩形会误拒；注释口径同为 5.5km）。
        let mids = [
            (117.11783474006214_f64, 39.46984164454213_f64),
            (117.47656872164231_f64, 39.05202611607821_f64),
            (116.90637588798378_f64, 39.4496563699438_f64),
            (117.36422085502315_f64, 38.99039201054334_f64),
            (116.79495413353128_f64, 39.3882830409556_f64),
            (117.2970385812442_f64, 38.92409889966998_f64),
            (116.64313344054925_f64, 39.34130967422113_f64),
            (117.21571916145582_f64, 38.88579602285712_f64),
            (117.0496252060799_f64, 39.69033181045541_f64),
            (117.59846930948892_f64, 39.14600125973928_f64),
            (117.19239519871157_f64, 39.77515291467246_f64),
            (117.75328904873285_f64, 39.265771413168174_f64),
            (117.37639341129983_f64, 39.899859443956274_f64),
            (117.82723727925679_f64, 39.4482470117936_f64),
            (117.4781831630707_f64, 40.02988393385804_f64),
            (117.91417815346816_f64, 39.566639775283306_f64),
            (116.35343384512255_f64, 39.21191424808533_f64),
            (117.13763274937696_f64, 40.007607829090794_f64),
            (116.54022244694409_f64, 39.61959713191777_f64),
            (116.94054105824308_f64, 40.15250255474903_f64),
            (116.33463652719644_f64, 39.77092757783807_f64),
            (116.77908228189388_f64, 40.31455957166121_f64),
            (116.40841749032751_f64, 40.11731272727076_f64),
        ];
        for (mi, (mlon, mlat)) in mids.iter().enumerate() {
            let near = v.path.iter().any(|p| {
                let d = crate::path::haversine_m(p.x, p.y, *mlon, *mlat);
                d <= 5_500.0
            });
            assert!(near, "wp{} 必经点应经过邻域，实际 {:?}", mi + 1, v.path);
        }
    }

    #[test]
    fn zigzag30_terrain_gap_raise_on_smoothed_segment() {
        // 主管 2026-08-11 输入（east_asia 7.5as 真实地形）：单机独立 start_pose +
        // 3 必经点 + target_ref + no_fly 三角形 + 2 restricted 圆 + 1 雷达。
        // v1 从 (121.32,35.35) 到 (106.68,49.89) 横跨 21.39° → FMM grid 1024
        // → cell 2325m，绕行走廊粗楼梯；Theta* 拉直段 (109.86,42.54)→(106.69,49.85)
        // 经过 2480m 山峰（不在 seg_ends 直线上）。
        // 根因（zigzag30）：抬升决策只采样 seg_ends 直线（最高 2355m）→ 抬到 2555m，
        // 但 Theta* 绕行走廊的拉直段穿 2480m 峰 → 净空 75-99m < 100m → verify 拒 →
        // 全链回退 1789 点网格楼梯（smoothing_failed）。且段级 theta_star 阶段的地形
        // issue 被"回退楼梯阶段"（沿 FMM 走廊地形 OK）吞掉 → 最终 verify 无 terrain
        // issue → 原抬升逻辑不触发。
        // 修复：① 平滑链 + 终检移入 'fmm_attempt 解算循环（final_rep FAIL 时抬升重跑
        // FMM）；② smooth.rs SmoothResult 新增 terrain_gap_m 记录段级中间阶段最大
        // 地形高度（verify ~200m 采样密于 check，窄峰不漏检），solver 抬升决策与
        // final_rep issues 解析取 max。结果：1789→14 点平滑，1000→2680m（抬升链
        // 2555m→段级 2480m 峰→2680m），smoothing_failed 消失。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag30: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":121.32158437921957,"lat":35.345605078916044,"alt_m":1000,"heading_deg":45},
                     "mid_waypoints":[
                        {"lon":122.3852641802933,"lat":37.91301080822844,"alt_m":1000},
                        {"lon":118.45106783248693,"lat":36.09407976700374,"alt_m":1000},
                        {"lon":119.64410040053079,"lat":38.42608749463845,"alt_m":1000}],
                     "target_ref":"106.6800083196283,49.891931490296315,3000"},
                    {"id":"v2","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":500,"min_turn_radius_m":800,"max_climb_angle_deg":15},
                     "start_pose":{"lon":103.80007749527002,"lat":32.492118851168414,"alt_m":5000,"heading_deg":45},
                     "mid_waypoints":[],
                     "target_ref":"124.7360092361736,53.31522760700038,3000"}],
                "red_forces":{"radars":[
                    {"id":"radar_1786409721004","lon":113.00786729664442,"lat":46.21627287139257,"radar_type":"early_warning","radius_km":200,"alt_m":10}]},
                "no_fly_zones":[
                    {"id":"zone_1786409515324","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[112.56383008070333,44.4855055439424],[117.93020756431244,40.51747800244875],[110.16677625965431,42.442633787371435]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786409547965","zone_type":"restricted","shape":"circle",
                     "geometry":{"center":[106.7843768348038,36.24901461339551],"radius_km":100},
                     "alt_min_m":3000,"alt_max_m":6000,"height_semantics":"msl"},
                    {"id":"rz_1786409606028","zone_type":"restricted","shape":"circle",
                     "geometry":{"center":[112.99788589051144,45.6335833104839],"radius_km":100},
                     "alt_min_m":2000,"alt_max_m":6000,"height_semantics":"msl"}],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{"p_cross":0.9}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned", "v1 应 planned");
        assert!(
            v.path.len() <= 60,
            "应平滑交付（修复前 1789 点网格楼梯），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 地形抬升必须显式报告（段级山峰 → 抬升；具体峰值随墙光栅化/路径微调
        // 而变，2026-08-11 墙精确化后 2480m→2472m，只断言抬升发生且 terrain 在列）
        assert!(
            v.warnings
                .iter()
                .any(|w| w.contains("cruise altitude raised") && w.contains("terrain")),
            "应报告地形抬升（段级山峰），实际 {:?}",
            v.warnings
        );
        // 必经点语义：3 个必经点均需经过（球面距离容差 5.5km，同 zigzag29）
        let mids = [
            (122.3852641802933_f64, 37.91301080822844_f64),
            (118.45106783248693_f64, 36.09407976700374_f64),
            (119.64410040053079_f64, 38.42608749463845_f64),
        ];
        for (mi, (mlon, mlat)) in mids.iter().enumerate() {
            let near = v.path.iter().any(|p| {
                let d = crate::path::haversine_m(p.x, p.y, *mlon, *mlat);
                d <= 5_500.0
            });
            assert!(near, "wp{} 必经点应经过邻域，实际 {:?}", mi + 1, v.path);
        }
    }

    #[test]
    fn zigzag31_fallback_pass_alt_uses_segment_ends() {
        // 主管 2026-08-11 输入（zz31）：v1 同 zigzag30（3 必经点 + no_fly + 2 restricted
        // + 1 radar），v2 新增 **13 必经点**（跨 21°×21° 大区域），no_fly 三角形换位置、
        // rz1 圆心微移。v1 复现 13 点平滑（zz30 机制），v2 复现 **5468 点网格楼梯
        // smoothing_failed**。
        // 根因（zigzag31）：v2 多段往返穿过 rz2（112.998,45.634）。wp4→wp5 段 raw
        // 穿圆 → bottom_terrain_ok raw_band 采样 raw 网格点（间隔 ~cell 2325m）漏
        // 1421m 窄峰（7.5as ~230m 格）→ 底部判可行 → 1500m 剖面穿峰（净空 79-98m
        // < 100m）；wp12→wp13 段 raw 未穿圆（网格离散擦边）→ fallback 直线参数化
        // 剖面，但 restricted_pass_alt 传**全局** start/target（v2 (103.8,32.5)→
        // (124.7,53.3) 直线不穿 rz2 → bottom_terrain_ok 平面分支 disc<=0 直接放行
        // 底部 1500m）→ 剖面沿段直线穿圆经过 1421m 峰 → 判定与剖面不一致 → final
        // verify 拒 → 全链回退 5468 点。
        // 修复：① bottom_terrain_ok raw_band 分支改沿 raw 子段加密采样（间隔 ≤200m，
        // 同 verify 口径）；平面分支步长 2.2km→200m（clamp [8,2048]）；② fallback
        // 分支 restricted_pass_alt 传**段首尾**（p0/p1，与剖面 in_out 一致）。
        // 结果：v1 13 点平滑（不变）；v2 5468→37 点平滑，13 必经点全过；wp4→wp5 段
        // rz2 底部→顶部 6500m（1421m 峰），wp12→wp13 段 1500m（该穿行带地形 ≤1400m）。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag31: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":121.32158437921957,"lat":35.345605078916044,"alt_m":1000,"heading_deg":45},
                     "mid_waypoints":[
                        {"lon":122.3852641802933,"lat":37.91301080822844,"alt_m":1000},
                        {"lon":118.45106783248693,"lat":36.09407976700374,"alt_m":1000},
                        {"lon":119.64410040053079,"lat":38.42608749463845,"alt_m":1000}],
                     "target_ref":"106.6800083196283,49.891931490296315,3000"},
                    {"id":"v2","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":500,"min_turn_radius_m":800,"max_climb_angle_deg":15},
                     "start_pose":{"lon":103.80007749527002,"lat":32.492118851168414,"alt_m":5000,"heading_deg":45},
                     "mid_waypoints":[
                        {"lon":109.97496463623962,"lat":40.89240709612281,"alt_m":5000},
                        {"lon":105.05176607466912,"lat":45.12560078345386,"alt_m":5000},
                        {"lon":107.45453837531355,"lat":47.33920209644489,"alt_m":5000},
                        {"lon":117.35985238233839,"lat":44.39524022248522,"alt_m":5000},
                        {"lon":123.01092994596729,"lat":42.94575095184553,"alt_m":5000},
                        {"lon":117.84672376061431,"lat":38.80107785870063,"alt_m":5000},
                        {"lon":112.72724669014056,"lat":37.192877051433356,"alt_m":5000},
                        {"lon":108.30767125147409,"lat":38.39729311871755,"alt_m":5000},
                        {"lon":105.43569482275211,"lat":40.72490772213813,"alt_m":5000},
                        {"lon":105.07165169750732,"lat":44.17689617497171,"alt_m":5000},
                        {"lon":106.27781114030425,"lat":46.905803587744295,"alt_m":5000},
                        {"lon":109.75971033327446,"lat":46.304227978356174,"alt_m":5000},
                        {"lon":118.23811461480165,"lat":44.09728961713801,"alt_m":5000}],
                     "target_ref":"124.7360092361736,53.31522760700038,3000"}],
                "red_forces":{"radars":[
                    {"id":"radar_1786409721004","lon":113.00786729664442,"lat":46.21627287139257,"radar_type":"early_warning","radius_km":200,"alt_m":10}]},
                "no_fly_zones":[
                    {"id":"zone_1786409515324","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[118.17769472280284,40.45820431372051],[107.63203137395895,44.11758744295661],[112.15118325335442,40.40008219131184]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786409547965","zone_type":"restricted","shape":"circle",
                     "geometry":{"center":[106.78464665022119,36.00663895579695],"radius_km":100},
                     "alt_min_m":3000,"alt_max_m":6000,"height_semantics":"msl"},
                    {"id":"rz_1786409606028","zone_type":"restricted","shape":"circle",
                     "geometry":{"center":[112.99788589051144,45.6335833104839],"radius_km":100},
                     "alt_min_m":2000,"alt_max_m":6000,"height_semantics":"msl"}],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{"p_cross":0.9}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        // v1：3 必经点（同 zigzag30，抬升 warning）
        let v1 = &out.vehicles[0];
        assert_eq!(v1.status, "planned", "v1 应 planned");
        assert!(
            v1.path.len() <= 60,
            "v1 应平滑交付（修复前 13 点），实际 {} 点",
            v1.path.len()
        );
        assert!(
            v1.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "v1 不应 smoothing_failed，实际 {:?}",
            v1.warnings
        );
        // v2：13 必经点全部经过（球面距离容差 5.5km），平滑交付
        let v2 = &out.vehicles[1];
        assert_eq!(v2.status, "planned", "v2 应 planned");
        assert!(
            v2.path.len() <= 120,
            "v2 应平滑交付（修复前 5468 点网格楼梯），实际 {} 点",
            v2.path.len()
        );
        assert!(
            v2.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "v2 不应 smoothing_failed，实际 {:?}",
            v2.warnings
        );
        let mids2 = [
            (109.97496463623962_f64, 40.89240709612281_f64),
            (105.05176607466912_f64, 45.12560078345386_f64),
            (107.45453837531355_f64, 47.33920209644489_f64),
            (117.35985238233839_f64, 44.39524022248522_f64),
            (123.01092994596729_f64, 42.94575095184553_f64),
            (117.84672376061431_f64, 38.80107785870063_f64),
            (112.72724669014056_f64, 37.192877051433356_f64),
            (108.30767125147409_f64, 38.39729311871755_f64),
            (105.43569482275211_f64, 40.72490772213813_f64),
            (105.07165169750732_f64, 44.17689617497171_f64),
            (106.27781114030425_f64, 46.905803587744295_f64),
            (109.75971033327446_f64, 46.304227978356174_f64),
            (118.23811461480165_f64, 44.09728961713801_f64),
        ];
        for (mi, (mlon, mlat)) in mids2.iter().enumerate() {
            let near = v2.path.iter().any(|p| {
                let d = crate::path::haversine_m(p.x, p.y, *mlon, *mlat);
                d <= 5_500.0
            });
            assert!(near, "v2 wp{} 必经点应经过邻域，实际 {:?}", mi + 1, v2.path);
        }
    }

    #[test]
    fn zigzag32_boundary_arc_extend_keeps_clearance() {
        // 主管 2026-08-11 输入（zz32）：单机 + 1 必经点（wp1 在 no_fly 三角形东北
        // 角外） + no_fly 三角形 + restricted 圆，地形为外部 Beijing_DEM.tif（west
        // 界外 170m/北界外 400m——OOB 5x 通行，降级警告）。
        // 根因：① FMM raw 贴 no_fly 膨胀墙（2km）走 → 600m 网格楼梯；② Theta*
        // 拉直 SEG1 = wp1→(116.6035,40.8452) 距墙 2.10km（刚过 2.00 膨胀线，合法）；
        // ③ wp1 必经点处转角 67°（球面）> 60° → 插 boundary arc（keep_b U 形弧
        // r=442m）；④ U 形弧采样点偏墙 ~386m → 段(弧末点→116.6035) 距墙 1.90km
        // < 2.00km → final verify 拒 → 全链回退 raw 687 点网格楼梯（密集锯齿）。
        // 修复：arc 插入前逐段净距预检（zone_segment_clearance_km 同 verify 口径）；
        // 不足 → 弧末点外推到出段直线（E' 距 b = 4×r，clamp 0.75×|bc|）——段 E'→next
        // 恢复为 b→c 子段净距（≥ 原值），弧内部转角 ≈ 3θ/4 < 60（θ≤80），verify
        // radius（b,弧中点,E'）≥442（θ=90° 最差 ~902m）；仍不足（转角 ≤65）才豁免。
        // 结果：687→9 点平滑，必经点经过，277.9km（修复前 354.9km）。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag32: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":117.49643196710215,"lat":39.45217964261854,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[{"lon":116.92011401843381,"lat":40.280864859008126,"alt_m":3000}],
                     "target_ref":"115.41519624070744,41.063105449335495,3000"}],
                "red_forces":{"radars":[]},
                "no_fly_zones":[
                    {"id":"zone_1786418099258","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[116.74744508792169,40.54250033772123],[116.04051071102833,39.87068759296977],[116.58825219982235,40.13334649202884]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786418172746","zone_type":"restricted","shape":"circle",
                     "geometry":{"center":[116.86855354211916,40.0244769648816],"radius_km":20},
                     "alt_min_m":1000,"alt_max_m":6000,"height_semantics":"msl"}],
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
            v.path.len() <= 100,
            "应平滑交付（修复前 687 点网格楼梯），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        let near = v.path.iter().any(|p| {
            crate::path::haversine_m(p.x, p.y, 116.92011401843381, 40.280864859008126) <= 5_500.0
        });
        assert!(near, "wp1 必经点应经过邻域，实际 {:?}", v.path);
    }

    #[test]
    fn zigzag33_two_waypoints_big_u_turn_at_wp2() {
        // 主管 2026-08-11 输入（zz33）：单机 + 2 必经点（wp1 在 no_fly 三角形西侧
        // 31km、wp2 在三角形东北） + no_fly 三角形 + restricted 圆（圆心微移）。
        // 根因：① SEG1（wp1→wp2）北绕 no_fly 三角形（y 爬到 40.58），raw 折返段
        // 与拉直方向冲突——i=192 处任何跳点拉直转角 91°+>60°，j 递减到相邻点
        // （raw 折返段微步 ~270m）才插弧 → |bc| 短 → d_m 截断 → r_eff 塌缩 178m
        // <442m → verify radius 拒 → SEG1 theta_star FAIL → 全链回退 1061 点；
        // ② wp2 必经点处 166.8° 大掉头（SEG1 东南进、SEG2 西北出）keep_b U 形弧
        // 末点 E 偏出段线南侧 0.88km → E→c2 在 no_fly A 顶点高度净距 1.75km<2km
        // 膨胀线 → 净距预检拒 → 外推 E'（出段直线上 4×r）修净距 → 但 n=3 弧倒数
        // 第二点→E' 转角 101°>60° → final verify 拒。
        // 修复：① theta_star 跳点（j>i+1）转角超限时也尝试 arc_transition（c=j，
        // |bc| 大，r_eff 保持 ≥442）——跳点插弧；② boundary arc 外推 E' 前细分弧
        // 重试（min_steps 4..=8：n 增大 → 末段步进减小 → p_{n-1}→E' 趋近出段
        // 方向，n=8 时 33.4°<60）——净距+转角双检。
        // 结果：1061→21 点平滑，必经点偏差 <0.2km，429.6km。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag33: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":117.49643196710215,"lat":39.45217964261854,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[{"lon":116.04302827980699,"lat":40.30245861424856,"alt_m":3000},{"lon":116.87502139486968,"lat":40.43880896740148,"alt_m":3000}],
                     "target_ref":"115.41519624070744,41.063105449335495,3000"}],
                "red_forces":{"radars":[]},
                "no_fly_zones":[
                    {"id":"zone_1786418099258","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[116.74744508792169,40.54250033772123],[116.04051071102833,39.87068759296977],[116.58825219982235,40.13334649202884]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786418172746","zone_type":"restricted","shape":"circle",
                     "geometry":{"center":[116.87035584365995,40.01756770083293],"radius_km":20},
                     "alt_min_m":1000,"alt_max_m":6000,"height_semantics":"msl"}],
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
            v.path.len() <= 100,
            "应平滑交付（修复前 1061 点网格楼梯），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        for (lo, la) in [
            (116.04302827980699, 40.30245861424856),
            (116.87502139486968, 40.43880896740148),
        ] {
            let near = v.path.iter().any(|p| crate::path::haversine_m(p.x, p.y, lo, la) <= 5_500.0);
            assert!(near, "必经点 ({lo},{la}) 应经过邻域，实际 {:?}", v.path);
        }
    }

    #[test]
    fn zigzag34_theta_star_check_uses_output_endpoint_after_jump_arc() {
        // 主管 2026-08-11 输入（zz34）：单机 + 2 必经点 + radar（early_warning
        // 50km，wp2 在盘内 37.5km）+ no_fly 三角形 + restricted 圆（圆顶与三角形
        // C 顶点同高，膨胀后走廊闭合）。
        // 根因：zz33 跳点插弧成功后 out 末点是弧点 E（≠ raw path[i]，i=k=231），
        // 后续 Theta* 跳点 check 起点仍用 path.points[i]（raw 走廊内点）——漏检
        // E→path[j] 穿 restricted 圆（SEG1 E→(117.0828,39.8165) 在 y≈40.02 处穿圆
        // 17km 深处）→ theta_star 输出 9 点但 final verify 拒（issues=96）→ 全链
        // 回退 raw 1679 点网格楼梯（871km）。
        // 修复：Theta* 跳点 check 起点改用 out 实际末点（插弧后为 E）——与 verify
        // 同口径检查真实输出段。
        // 结果：1679→20 点平滑，必经点偏差 <0.3km，562.4km，radar 累计探测
        // 0.0154（路径绕开雷达盘）。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag34: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":117.49643196710215,"lat":39.45217964261854,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[{"lon":116.30222159303027,"lat":40.52501663038863,"alt_m":3000},{"lon":116.5195046089279,"lat":40.00372676035467,"alt_m":3000}],
                     "target_ref":"115.41519624070744,41.063105449335495,3000"}],
                "red_forces":{"radars":[
                    {"id":"radar_1786430183478","lon":116.10054207161798,"lat":39.893963015168175,"radar_type":"early_warning","radius_km":50,"alt_m":10}
                ]},
                "no_fly_zones":[
                    {"id":"zone_1786418099258","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[116.13619653711649,40.07186918292957],[116.91994987492495,40.58747146826297],[116.5682729278144,40.19722929127514]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[
                    {"id":"rz_1786418172746","zone_type":"restricted","shape":"circle",
                     "geometry":{"center":[116.87035584365995,40.01756770083293],"radius_km":20},
                     "alt_min_m":1000,"alt_max_m":6000,"height_semantics":"msl"}],
                "obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{"p_cross":0.9}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned", "v1 应 planned");
        assert!(
            v.path.len() <= 100,
            "应平滑交付（修复前 1679 点网格楼梯），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        for (lo, la) in [
            (116.30222159303027, 40.52501663038863),
            (116.5195046089279, 40.00372676035467),
        ] {
            let near = v.path.iter().any(|p| crate::path::haversine_m(p.x, p.y, lo, la) <= 5_500.0);
            assert!(near, "必经点 ({lo},{la}) 应经过邻域，实际 {:?}", v.path);
        }
    }

    #[test]
    fn oob_input_point_no_error_and_plans() {
        // 主管 2026-08-11：放开输入点限制——起点/必经点落在地形数据范围外
        // （east_asia_7p5as 东界 135E 之外 152E）不再报 data_error（旧 8e5e64e
        // 预检），走空洞/无效数据处理流程：OOB 按 NODATA 5x 高代价**通行**（非
        // 禁行墙），目标在数据内 → 出路径（planned + OOB 降级警告）；全被挡 →
        // no_solution——均为四态可用结果。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip oob_input_point: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":152.0,"lat":35.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.8,"alt_m":3000},
                "terrain":{"source":"path","path":"__P__"}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        // 不再 data_error（旧预检 return Err 已删除）
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert!(
            v.status == "planned" || v.status == "no_solution",
            "OOB 输入点应给 planned/no_solution（四态可用结果），实际 {}",
            v.status
        );
        if v.status == "planned" {
            assert!(!v.path.is_empty(), "planned 路径不应为空");
            assert!(
                v.warnings.iter().any(|w| w.contains("out of terrain bounds")),
                "应带 OOB 降级警告，实际 {:?}",
                v.warnings
            );
        }
    }

    #[test]
    fn smoothing_failed_withholds_raw_fallback_no_solution() {
        // 阶段1-D 护栏（主管 2026-08-11 拍板：不需要输出无法实现的路径）：
        // FMM 有解但所有平滑阶段都失败（min_turn_radius_m=100km 使任何绕行
        // 转角曲率半径都不足 → Theta*/Catmull/Dubins/greedy 全拒），且直线
        // 替代穿禁飞方块被拒 → 不得再回退 raw 交付（旧行为 status=planned +
        // 密集网格楼梯），必须明确 no_solution + 保留失败原因。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":100000,"max_climb_angle_deg":15},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":90},
                     "mid_waypoints":[
                        {"lon":116.0,"lat":39.0,"alt_m":3000},
                        {"lon":116.0,"lat":39.5,"alt_m":3000},
                        {"lon":117.0,"lat":39.5,"alt_m":3000}],
                     "target_ref":"117.0,39.0,3000"}
                ],
                "red_forces":{"radars":[]},
                "no_fly_zones":[
                    {"id":"wall","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[116.2,39.25],[116.8,39.25],[116.8,39.75],[116.2,39.75]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[],"obstacles":[],
                "terrain":{"source":"none"}
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "no_solution", "平滑全失败应 no_solution，实际 {:?}", v.status);
        assert!(v.path.is_empty(), "no_solution 不应携带 raw 路径");
        assert!(
            v.warnings
                .iter()
                .any(|w| w.contains("smoothing_failed") || w.contains("hard-gate")),
            "应保留失败原因，实际 {:?}",
            v.warnings
        );
    }

    #[test]
    fn classified_on_smoothing_failed() {
        // P3 验收 1（docs/12 §8/§12.4）：C 类失败（smoothing_failed 硬闸拒交付）
        // 100% 获得分类结论——stdout stats.degradations 汇总（stderr classified
        // JSON 由 emit_classified 输出）。该场景 patch 不适用（3 必经点 +
        // turn_radius 100km）→ 默认 fitting_defect。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":100000,"max_climb_angle_deg":15},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":90},
                     "mid_waypoints":[
                        {"lon":116.0,"lat":39.0,"alt_m":3000},
                        {"lon":116.0,"lat":39.5,"alt_m":3000},
                        {"lon":117.0,"lat":39.5,"alt_m":3000}],
                     "target_ref":"117.0,39.0,3000"}
                ],
                "red_forces":{"radars":[]},
                "no_fly_zones":[
                    {"id":"wall","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[116.2,39.25],[116.8,39.25],[116.8,39.75],[116.2,39.75]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[],"obstacles":[],
                "terrain":{"source":"none"}
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "no_solution");
        assert!(
            out.stats.degradations
                .iter()
                .any(|d| d == "classified: fitting_defect"),
            "degradations 应有 classified 汇总，实际 {:?}",
            out.stats.degradations
        );
    }

    #[test]
    fn classified_on_coarse_no_path() {
        // P3 验收 1：coarse FMM 真无通道 → geometrically_impossible 分类汇总。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":90}}
                ],
                "red_forces":{"radars":[]},
                "no_fly_zones":[
                    {"id":"wall","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[115.2,39.0],[117.0,39.0],[117.0,40.5],[115.2,40.5]]},
                     "alt_min_m":0,"alt_max_m":10000,"height_semantics":"msl"}],
                "restricted_zones":[],"obstacles":[],
                "terrain":{"source":"none"}
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "no_solution");
        assert!(
            out.stats.degradations
                .iter()
                .any(|d| d == "classified: geometrically_impossible"),
            "degradations 应有 geometrically_impossible 分类，实际 {:?}",
            out.stats.degradations
        );
    }

    #[test]
    fn category_from_degradations_maps_patch_labels() {
        // P3 归因映射（docs/12 §3.4 C3 决策树）：patch 归因标注 → 分类类别。
        assert_eq!(category_from_degradations(&[]), "fitting_defect");
        assert_eq!(
            category_from_degradations(&["patch: geometrically_impossible (docs/12 §13.1 C3)".into()]),
            "geometrically_impossible"
        );
        assert_eq!(
            category_from_degradations(&["patch: search_truncated (visible-graph cap, docs/12 §13.1 C2)".into()]),
            "search_truncated"
        );
        assert_eq!(
            category_from_degradations(&["patch: fitting_defect (turn margin, docs/12 §13.1 C3)".into()]),
            "fitting_defect"
        );
    }

    #[test]
    fn zigzag35_polygon_narrow_tip_wall_rasterization_no_false_no_solution() {
        // 主管 2026-08-11 输入（zz_nosolution_case）：v1 起点 (117.56,38.96) 目标
        // (115.51,40.72) + no_fly 三角形（西顶点 A(116.4596,39.5413)）+ 1 雷达 50km。
        // 修复前 grid256：墙光栅化"格子中心采样"漏掉 A 点附近 <1 格窄带（三角形
        // 截面 ~30m），墙从列 116 才开始 → FMM 沿列 112（x=116.4605，离 A 84m）走，
        // 路径段穿过三角形窄带 → verify 精确几何 clearance 0.00 < 2km → 平滑链全败
        // → 误报 no_solution（grid512 cell 更细恰好命中窄带才成功）。
        // 修复：walled 多边形改"格子矩形与多边形相交"（rect_intersects_polygon），
        // 任何分辨率不漏窄带。grid384/320/288/272/260/256/512 全部 planned。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag35: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                "target":{"lon":116.8,"lat":40.3,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":117.560718576047,"lat":38.96247381822325,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[],
                     "target_ref":"115.51277784181079,40.72094100704186,3000"}],
                "red_forces":{"radars":[
                    {"id":"radar_1786439379325","lon":115.9099176949948,"lat":40.11567473682985,"radar_type":"tracking","radius_km":50,"alt_m":10}]},
                "no_fly_zones":[
                    {"id":"zone_1786439400716","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[116.45960372107142,39.54130857882594],[117.72747798452006,39.878961668275586],[117.2682607976188,39.54988915269062]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[],"obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned", "窄带多边形不应误报 no_solution，实际 {:?}", v.status);
        assert!(
            v.path.len() <= 30,
            "应平滑交付（修复前 423 点网格楼梯 + no_solution），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 绕行路径必须离禁飞区多边形 ≥ 2km（inflation）：v1 路径任何段不得进入
        // 三角形近邻（用 zone_segment_clearance_km 复验，与 verify 同口径）。
        let zone = &input.mission.no_fly_zones[0];
        for w in v.path.windows(2) {
            let clr = crate::config::zone_segment_clearance_km(w[0].x, w[0].y, w[1].x, w[1].y, zone);
            assert!(
                clr >= 2.0 - 1e-6,
                "路径段应离禁飞区 ≥2km，实际 {:.3}km ({},{})->({},{})",
                clr, w[0].x, w[0].y, w[1].x, w[1].y
            );
        }
    }

    #[test]
    fn zigzag36_region_expand_when_wall_fills_bbox() {
        // 2026-08-11 主管问题 2（zz_region_block2）：bbox 近正方形（起点 (115.8,39.8)
        // → 目标 (117.2,40.2)，span 1.85°）+ 矩形 no_fly（[115.9,117.1]×[39.65,41.6]）
        // 占满 region 南北方向（北界 41.6 超 region 41.35、南界 39.65 顶到 region 底）：
        // 绕行只能出 region（绕南端 y<39.65 或北端 y>41.6）→ region 外无代价场 →
        // coarse FMM no path → 误报 no_solution（实际绕出 region 0.05° 即通）。
        // 修复：region 纳入硬墙（NoFly/Obstacle）bbox + 机动余量（多边形 pad 0.15°、
        // 圆墙 0.03°——zz19 教训：圆墙大余量会让本就贴近任务边界的圆撑出 region，
        // 触发 terrain 墙格点翻转丢 restricted 剖面语义）；grid 等比保持 cell
        // （region 变大 grid 按 base_grid cell 等比上调，软罚带物理宽度不变）。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("data/east_asia_7p5as.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag36: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.8,"lat":39.8,"alt_m":3000},
                "target":{"lon":117.2,"lat":40.2,"alt_m":3000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,"min_turn_radius_m":442,"max_climb_angle_deg":15},
                     "start_pose":{"lon":115.8,"lat":39.8,"alt_m":3000,"heading_deg":0},
                     "mid_waypoints":[]}],
                "red_forces":{"radars":[]},
                "no_fly_zones":[
                    {"id":"zone_block2","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[[115.9,39.65],[117.1,39.65],[117.1,41.6],[115.9,41.6]]},
                     "alt_min_m":0,"alt_max_m":12000,"height_semantics":"msl"}],
                "restricted_zones":[],"obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(
            v.status, "planned",
            "墙占满 region 短边不应误报 no_solution（修复前 coarse FMM no path），实际 {:?}",
            v.status
        );
        assert!(
            v.path.len() <= 30,
            "应平滑交付（修复前 no_solution），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 修复语义：路径必须绕出任务 bbox（南端 y < 39.65 或北端 y > 41.6——
        // 矩形墙占满两方向，绕行只能出墙 bbox，证明 region 外扩生效而非碰巧直穿）。
        let y_min = v.path.iter().map(|p| p.y).fold(f64::INFINITY, f64::min);
        let y_max = v.path.iter().map(|p| p.y).fold(f64::NEG_INFINITY, f64::max);
        assert!(
            y_min < 39.65 - 0.005 || y_max > 41.6 + 0.005,
            "路径应绕出墙 bbox（南绕 y<39.65 或北绕 y>41.6），实际 y∈[{:.3},{:.3}]",
            y_min, y_max
        );
        // 硬约束复验：路径段不得穿入禁飞区（与 verify 同口径 clearance ≥ 2km）。
        let zone = &input.mission.no_fly_zones[0];
        for w in v.path.windows(2) {
            let clr = crate::config::zone_segment_clearance_km(w[0].x, w[0].y, w[1].x, w[1].y, zone);
            assert!(
                clr >= 2.0 - 1e-6,
                "路径段应离禁飞区 ≥2km，实际 {:.3}km ({},{})->({},{})",
                clr, w[0].x, w[0].y, w[1].x, w[1].y
            );
        }
    }
}

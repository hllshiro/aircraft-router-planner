//! 端到端 solver（Phase 4 M1）：把 Phase 0-3 库层组件串成主流程。
//!
//! parse → validate（main）→ TerrainSource（ARPK1/无地形）→ 语义代价场
//! （Land/Water/NoData/NODATA 5x + Zone 硬墙 INF）→ FMM → 回溯 → 平滑链
//! （Theta* 去锯齿 + 抽稀 + Dubins 拟合 + 复验门）→ AircraftOutput 契约。
//!
//! M1 范围（主管拍板 2026-08-05）：单机/多机（共享代价场，每机独立 FMM）；
//! Zone 水平几何禁入（高度层 M2）；SmoothOptions 默认值（AircraftProfile 派生 M4）；
//! 无雷达代价（M3）；逐机显式 start/target（M5 每机 waypoints）。

use std::time::Instant;

use crate::config::{
    Input, Output, PathPoint, Stats, TerrainIndex, AircraftOutput, Zone, ZoneShape,
    point_in_polygon_xy, pt_seg_dist_km, zone_contains, zone_contains_at
};
use crate::coord::Geo;
use crate::costfield::{
    backtrack_path, build_semantic_cost_field, build_semantic_cost_field_par_local, fmm_propagate
};
use crate::error::{AppError, InputInvalidReason};
use crate::path::{Path, PathPoint as RouterPoint};
use crate::smooth::{VerifyContext, default_chain, segment_circle_intersect_t, smooth_path_chain};
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

/// 解算参数。
#[derive(Debug, Clone)]
pub struct SolveParams {
    /// 端到端时间预算（ms，含 solve 前地形加载等 main 已计耗时）：0 = 无限
    /// （测试/CI 双跑确定性用；docs/07 §5 3s 预算硬护栏）。超预算：
    /// 已有过闸候选（前面飞行器已完成）→ `degraded_timeout` + 部分结果；
    /// 无候选 → `AppError::DegradedTimeout`。
    pub time_budget_ms: u64
}

impl Default for SolveParams {
    fn default() -> Self {
        Self {
            time_budget_ms: 0
        }
    }
}

/// 地形打开中间态：ARPK1（BuiltinSource，预取）或外部格式（trait object，带锁）。
enum InnerSource {
    Builtin(BuiltinSource),
    Dyn(Box<dyn TerrainSource>)
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
    MaskedExternal(MaskedSource<Box<dyn TerrainSource>>)
}

impl TerrainHandle {
    fn as_source(&self) -> Option<&dyn TerrainSource> {
        match self {
            Self::None => None,
            Self::Plain(t) => Some(t),
            Self::External(t) => Some(t.as_ref()),
            Self::Masked(t) => Some(t),
            Self::MaskedExternal(t) => Some(t)
        }
    }
    fn as_bulk(&self) -> Option<&(dyn BulkPrefetch + Sync)> {
        match self {
            Self::None => None,
            Self::Plain(t) => Some(t),
            Self::External(_) => None,
            Self::Masked(t) => Some(t),
            Self::MaskedExternal(_) => None
        }
    }
}

/// 解算任务区域（方形经纬度包围盒 + 缓冲）。
#[derive(Debug, Clone, Copy)]
struct Region {
    min_lon: f64,
    min_lat: f64,
    span_deg: f64
}

/// 待解算的飞行器规格（逐机显式；aircraft 空数组已在 validate 拦截）。
struct AircraftSpec {
    id: String,
    start: Geo,
    /// 每机目标（逐机显式）。
    target: Geo,
    alt_m: f64,
    /// 每机目标高度（2026-08-12 垂直剖面：路径终点高度；来自 aircraft.target.alt_m）。
    target_alt_m: f64,
    /// 机型配置（Phase 4 M4：平滑参数派生输入）。
    profile: crate::config::AircraftProfile,
    /// 中途必经点（Phase 4 M5：start → mid[0..] → target 分段拼接）。
    mid_waypoints: Vec<Geo>,
    /// 必经点高度（P8 M2：与 mid_waypoints 对齐；垂直剖面分段锚点，缺省同 start 无效果）。
    mid_alts: Vec<f64>,
    /// 关联武器（P7：逐机显式；出现即启用——`effective_range_km()` 恒 Some
    /// （weapon_type 必填））。None = 无武器 → 点目标语义。
    weapon: Option<crate::config::Weapon>
}

/// 查找 data 目录（index.yaml 所在目录）。候选顺序：
///   1) cwd/data
///   2) exe 同级/data
///   3) exe 上级/data
///   4) workspace 根 data（相对路径 ./data）
pub fn find_data_dir() -> Option<std::path::PathBuf> {
    let mut candidates: Vec<std::path::PathBuf> = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        candidates.push(cwd.join("data"));
        // workspace root（cargo test 从 cli/ 运行时 cwd=cli/，data 在上级）
        candidates.push(cwd.join("..").join("data"));
    }
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            candidates.push(dir.join("data"));
            candidates.push(dir.join("..").join("data"));
        }
    }
    // 相对路径回退
    candidates.push(std::path::PathBuf::from("data"));
    candidates
        .into_iter()
        .find(|p| p.join("index.yaml").exists())
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
    // 1. 地形源（索引 id 模式：data/index.yaml 解析路径）。
    //    arpack/mask 字段为索引 id，查找 index.yaml 获取文件名，拼接 data_dir 得到完整路径。
    let mut terrain_warnings: Vec<String> = Vec::new();
    let data_dir = find_data_dir();
    let index = data_dir.as_deref().and_then(TerrainIndex::load);
    let terrain: TerrainHandle = {
        // 解析 arpack 路径
        let arpack_path = match (&input.terrain.arpack, &index) {
            (Some(id), Some(idx)) => idx.find_arpack(id).map(|f| {
                let mut p = data_dir.clone().unwrap();
                p.push(f);
                p
            }),
            (Some(_), None) => {
                let msg = "terrain arpack 索引文件不存在（data/index.yaml），无法加载地形".into();
                eprintln!("[warn] {msg}");
                terrain_warnings.push(msg);
                None
            }
            (None, _) => None
        };
        // 解析 mask 路径
        let mask_path = match (&input.terrain.mask, &index) {
            (Some(id), Some(idx)) => idx.find_mask(id).map(|f| {
                let mut p = data_dir.clone().unwrap();
                p.push(f);
                p
            }),
            (Some(_), None) => {
                let msg = "terrain mask 索引文件不存在（data/index.yaml），无法加载掩膜".into();
                eprintln!("[warn] {msg}");
                terrain_warnings.push(msg);
                None
            }
            (None, _) => None
        };
        // 加载地形
        let inner: Option<InnerSource> = match arpack_path {
            Some(ref p) => {
                let ext = p
                    .extension()
                    .and_then(|e| e.to_str())
                    .unwrap_or("")
                    .to_ascii_lowercase();
                let result = match ext.as_str() {
                    "arpack" | "zstd" => BuiltinSource::open(p).map(InnerSource::Builtin),
                    _ => crate::terrain::open_source(p).map(InnerSource::Dyn)
                };
                match result {
                    Ok(src) => Some(src),
                    Err(e) => {
                        let msg = format!(
                            "terrain arpack '{}' 加载失败（{}），降级为无地形",
                            p.display(),
                            e
                        );
                        eprintln!("[warn] {msg}");
                        terrain_warnings.push(msg);
                        None
                    }
                }
            }
            None => None
        };
        match inner {
            None => TerrainHandle::None,
            Some(inner) => match mask_path {
                Some(mp) => {
                    if !mp.exists() {
                        return Err(AppError::Data(format!(
                            "mask file not found: {}（terrain.mask）",
                            mp.display()
                        )));
                    }
                    let gm = GeoMask::open(&mp)?;
                    match inner {
                        InnerSource::Builtin(b) => TerrainHandle::Masked(MaskedSource::new(b, gm)),
                        InnerSource::Dyn(d) => {
                            TerrainHandle::MaskedExternal(MaskedSource::new(d, gm))
                        }
                    }
                }
                None => match inner {
                    InnerSource::Builtin(b) => TerrainHandle::Plain(b),
                    InnerSource::Dyn(d) => TerrainHandle::External(d)
                }
            }
        }
    };

    // 2. 飞行器规格（逐机显式；aircraft 空数组已在 validate 拦截）
    let specs: Vec<AircraftSpec> = input
        .aircraft
        .iter()
        .map(|a| {
            let mid = a
                .mid_waypoints
                .iter()
                .map(|w| {
                    Geo::new(w.lon, w.lat).map_err(|_| {
                        AppError::InputInvalid(InputInvalidReason::IllegalCoordinate)
                    })
                })
                .collect::<Result<Vec<_>, AppError>>()?;
            let mid_alts = a.mid_waypoints.iter().map(|w| w.alt_m).collect::<Vec<_>>();
            Ok(AircraftSpec {
                id: a.id.clone(),
                start: a.start.to_geo()?,
                target: a.target.to_geo()?,
                alt_m: a.start.alt_m,
                target_alt_m: a.target.alt_m,
                profile: a.profile.clone(),
                mid_waypoints: mid,
                mid_alts,
                weapon: a.weapon.clone()
            })
        })
        .collect::<Result<Vec<_>, AppError>>()?;

    // 2b. 起终点/必经点不再做地形范围硬拒（主管 2026-08-11 拍板：放开输入点限制）。
    //     输入点落在数据范围外时，交给既有空洞/无效数据处理流程：FMM 种子格点
    //     （OOB 格点代价 INF 墙）照常传播 → 目标/必经点在数据内则出路径（OOB 段走
    //     墙格，verify OOB 硬拒 → 平滑失败回退 raw 交付）；全被墙挡则 no_solution
    //     + warning——均为四态内可用结果，不再返回 data_error（旧 8e5e64e 预检取消）。

    // 3. 任务区域（所有起点 + 每机目标包围盒 + 缓冲）
    let base_region = region_of(&specs);
    // 3b. 障碍感知外扩（2026-08-11 zz_region_block2）：硬墙（NoFly/Obstacle）bbox
    // 超出任务区域时并入——墙占满 region 短边方向时绕行被迫出 region（region 外
    // 无代价场 → coarse FMM no path → 误报 no_solution）。Restricted 不画墙不纳入。
    // 2026-08-12 rz_poly3：restricted **画墙绕行**时同样必须外扩——多边形两端
    // （尖角顶点 B(114.691,40.625)/A(116.941,42.027)）超出任务 bbox+pad → 东绕
    // （A 北侧 lat>41.56）/南绕（B 西侧 lon<114.97）走廊全被 region 边界截断 →
    // coarse FMM no path → 误报 geometrically_impossible；同形状禁飞区因 is_wall()
    // 纳入外扩能绕行。画墙判定与 aircraft 循环内 restricted_wall_for 同源
    // （restricted_blocks_alt + restricted_pass_alt 不可行）；region 构建在抬升
    // 前，用初始 alt_m（低高度更可能 blocks → 外扩保守侧）。
    let params_merged = crate::config::DefaultParams::default().merge(&input.parameters);
    let spec_climb: Vec<(f64, Option<f64>)> = specs
        .iter()
        .map(|s| {
            let (opts, _) = crate::smooth::smooth_options_for(&s.profile, &params_merged);
            (opts.max_climb_deg, s.profile.maximum_altitude_m)
        })
        .collect();
    let restricted_wall_zs: Vec<&Zone> = input
        .zones
        .iter()
        .filter(|z| !z.is_wall())
        .filter(|z| {
            specs.iter().zip(&spec_climb).any(|(s, (mcd, ceil))| {
                restricted_blocks_alt(z, s.alt_m)
                    && restricted_pass_alt(
                        z,
                        s.alt_m,
                        *ceil,
                        terrain.as_source(),
                        &s.start,
                        &s.target,
                        *mcd,
                        None,
                    )
                    .is_none()
            })
        })
        .collect();
    let wall_zones = input
        .zones
        .iter()
        .filter(|z| z.is_wall())
        .chain(restricted_wall_zs.iter().copied());
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
        .zones
        .iter()
        .any(|z| z.is_wall() && matches!(z.shape, ZoneShape::Polygon { .. }));
    let target_cell = if has_poly_wall { 600.0 } else { 1100.0 };
    let auto_grid = if base_region.span_deg > 2.5 {
        ((base_span_km * 1000.0) / target_cell).ceil() as usize
    } else {
        0
    };
    let base_grid = params_merged.default_grid_resolution.max(8).max(auto_grid).min(1024);
    let grid = if region.span_deg > base_region.span_deg + 1e-9 {
        let cell_deg = base_region.span_deg / base_grid as f64;
        ((region.span_deg / cell_deg).round() as usize)
            .max(base_grid)
            .min(1024)
    } else {
        base_grid
    };
    eprintln!(
        "[debug] region span={:.2}deg grid={} cell_m={:.0} (base {:.2})",
        region.span_deg,
        grid,
        region.span_deg * 111_320.0 / grid as f64,
        base_region.span_deg
    );

    // 4. Zone 集合
    //    代价场墙策略（M2 高度层）：
    //    - NoFly/Obstacle → 全高度水平墙（代价场 INF）——保守禁入；
    //    - Restricted → 不画墙（高度区间外可穿越），由 Theta* check + verify 高度判定。
    let all_zones: Vec<Zone> = input.zones.clone();
    let nofly = circle_index(&all_zones.iter().collect::<Vec<_>>());

    // 4b. 参数合并 + 禁飞区膨胀距离（主管 2026-08-06：绕飞太贴边→考虑飞机机动）。
    //     规划转弯半径 r = turn_radius（信任输入/默认表，2026-08-07 起不再钳巡航物理
    //     下限；小半径经转弯段降速实现）；绕行弧需要 ≥r 的转弯空间，把 NoFly/Obstacle
    //     硬墙向外膨胀 max(0.5×r)（clamp [2km, 10km]）——FMM 绕行自然远离边界，
    //     Dubins 转弯弧留足空间（不再因贴边急弯被物理复验拒绝）。
    let mut degradations = Vec::new();
    degradations.extend(terrain_warnings);
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
    //    + 5c 膨胀/软罚带 + 5b 雷达静态代价（+ P8 LOS mask）→ 见 build_cost_field。
    //    提取为函数：无解出口链④（docs/01 §5）网格细分重试复用同一套墙/膨胀/雷达
    //    语义（grid 翻倍重算，见 no_solution 分支）。
    let threat_params = radar_threat_params(&params_merged);
    let threat = SphericalRadarThreat::new(&input.red_forces.radars, threat_params.clone());
    let mut grid = grid;
    let mut cell_m = region.span_deg * 111_320.0 / grid as f64;
    // 膨胀格数（terrain 过滤场复用；细分重试时随 cell_m 同步——与 build_cost_field
    // 内部口径一致）
    let mut inflation_cells = if region.span_deg > 2.5 {
        ((inflation_m + 0.71 * cell_m) / cell_m.max(1.0)).ceil() as usize + 1
    } else {
        (inflation_m / cell_m.max(1.0)).ceil() as usize
    };
    let mut field = build_cost_field(
        &region,
        grid,
        &terrain,
        &all_zones,
        inflation_m,
        &threat,
        params_merged.radar_cost_coef,
        params_merged.los_mask_coef,
        !input.red_forces.radars.is_empty(),
    );
    let mut grid_refined = false; // ④ 已细分重试（每机最多一次）

    // 6. 每机：分段 FMM（start → mid[0..] → target，共享代价场）→ 拼接 → 平滑 → 输出
    let mut out_aircraft = Vec::new();
    let mut fmm_ms = 0.0f64;
    'aircraft: for v in &specs {
        // P6-B 检查点①（每机开始）：预算耗尽 → 有部分结果返回 degraded_timeout，
        // 无候选返回 DegradedTimeout（docs/07 §5：warm best-so-far 超时降级）。
        if over_budget(elapsed_ms) {
            if out_aircraft.is_empty() {
                return Err(AppError::DegradedTimeout(format!(
                    "time budget {budget_ms}ms exceeded; no verified candidate yet"
                )));
            }
            out_aircraft.push(AircraftOutput {
                id: v.id.clone(),
                status: "degraded".into(),
                path: Vec::new(),
                distance_m: 0.0,
                warnings: vec![format!(
                    "time budget {budget_ms}ms exceeded; no path for this aircraft"
                )]
            });
            return Ok(Output {
                status: "degraded_timeout".into(),
                error: Some(crate::error::ErrorBody {
                    code: "degraded_timeout".into(),
                    message: format!(
                        "time budget {budget_ms}ms exceeded; partial results for completed aircraft"
                    )
                }),
                elapsed_ms: Some(elapsed_ms),
                aircraft: out_aircraft,
                stats: Stats {
                    fmm_ms,
                    los_checks: 0,
                    degradations
                }
            });
        }
        // 每机目标（shadow：闭包与剖面切分统一使用 v.target）
        let target = v.target;
        // 段序列：起点 + 必经点 + 目标
        let mut seg_ends: Vec<Geo> = Vec::with_capacity(v.mid_waypoints.len() + 2);
        seg_ends.push(v.start);
        seg_ends.extend(v.mid_waypoints.iter().copied());
        seg_ends.push(target);
        // 起终点地面抬升（主管 2026-08-14）：起点/终点 < 当地地面海拔 → 强制抬到
        // 地面 MSL **再计算**（物理可达底线）；高于地面则保持用户设定（任务硬约束）。
        // 起终点是任务约束，不因中间地形抬升——中间段高度由抬升决策自动插入的
        // 中间锚点（terrain_anchor）+ 逐点地形保底解决。
        let ground_at = |lon: f64, lat: f64| -> f64 {
            terrain
                .as_source()
                .and_then(|t| match t.sample_at(lon, lat) {
                    crate::terrain::Sample::Land(h) => Some(h),
                    _ => None
                })
                .unwrap_or(0.0)
        };
        // 机型平滑参数提前（受限区剖面需要 max_climb：决定下降/爬升距离）
        let (opts, phys_min_radius_m) =
            crate::smooth::smooth_options_for(&v.profile, &params_merged);
        // 起点低于当地地面 → 抬到地面 MSL（主管 2026-08-14 三反：起终点贴地合理，
        // 不加净空；起飞/降落段允许贴近地形，中间巡航段才保净空）
        let start_alt_norm = v.alt_m.max(ground_at(v.start.lon, v.start.lat));
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
        let mut terrain_probe_done = false; // 已用无过滤场探测路径
        let mut terrain_alt_raised = false; // 已抬升巡航高度
        let mut terrain_fallback_done = false; // 已回退无过滤场（保底）
        // 有效巡航高度（可被抬升逻辑更新；初始 = 起终点地面抬升后的起点高度）
        let mut alt_eff = start_alt_norm;
        // 方案 B（主管 2026-08-14）：抬升决策记录撞山区段最高地形点 → 自动中间锚点
        // (lon, lat, 抬升高度)；apply_vertical_profile 据此在中间爬升/下降，
        // 起点/终点保持用户高度（不再整条路径强制抬到抬升巡航高度）。
        let mut terrain_anchor: Option<(f64, f64, f64)> = None;
        // 平滑终检产物（循环内填充、循环外输出）：
        let mut warnings: Vec<String> = Vec::new();
        // 起点低于当地地面 → 抬到地面 MSL（主管 2026-08-14 三反：起终点是起飞/降落
        // 场景，贴地合理、不叠加净空余量；仅低于地面时物理保底抬到地面）。
        if start_alt_norm > v.alt_m + 0.5 {
            warnings.push(format!(
                "start altitude below terrain: raised {:.0}->{:.0}m (terrain {:.0}m)",
                v.alt_m,
                start_alt_norm,
                ground_at(v.start.lon, v.start.lat)
            ));
        }
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
                if out_aircraft.is_empty() {
                    return Err(AppError::DegradedTimeout(format!(
                        "time budget {budget_ms}ms exceeded; no verified candidate yet"
                    )));
                }
                out_aircraft.push(AircraftOutput {
                    id: v.id.clone(),
                    status: "degraded".into(),
                    path: Vec::new(),
                    distance_m: 0.0,
                    warnings: vec![format!(
                        "time budget {budget_ms}ms exceeded; no path for this aircraft"
                    )]
                });
                return Ok(Output {
                    status: "degraded_timeout".into(),
                    error: Some(crate::error::ErrorBody {
                        code: "degraded_timeout".into(),
                        message: format!(
                            "time budget {budget_ms}ms exceeded; partial results for completed aircraft"
                        )
                    }),
                    elapsed_ms: Some(elapsed_ms),
                    aircraft: out_aircraft,
                    stats: Stats {
                        fmm_ms,
                        los_checks: 0,
                        degradations
                    }
                });
            }
            if attempts > 6 {
                // 诊断输出（阶段1-C）：最后一次 verify 失败原因 JSON 到 stderr。
                if !last_failures.is_empty() {
                    let json = serde_json::json!({
                        "event": "fmm_attempts_exhausted",
                        "aircraft": v.id,
                        "attempts": attempts - 1,
                        "failures": last_failures.iter().map(|(a, iss)| {
                            serde_json::json!({ "attempt": a, "issues": iss })
                        }).collect::<Vec<_>>()
                    });
                    eprintln!(
                        "{}",
                        serde_json::to_string(&json).unwrap_or_else(|_| {
                            r#"{"event":"fmm_attempts_exhausted","serialize_error":true}"#.into()
                        })
                    );
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
                        v.profile.maximum_altitude_m,
                        terrain.as_source(),
                        &v.start,
                        &target,
                        opts.max_climb_deg,
                    )
            };
            let has_restricted_wall = all_zones.iter().any(|z| restricted_wall_for(z));
            let has_terrain_mask = use_terrain_mask && terrain.as_source().is_some();
            let aircraft_field: Option<crate::costfield::CostField> =
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
            let field_ref = aircraft_field.as_ref().unwrap_or(&field);
            eprintln!(
                "[debug] fmm attempt {} field ready (aircraft={})",
                attempts,
                aircraft_field.is_some()
            );
            // 逐段 FMM → 回溯 → 拼接（去重段端点）
            let mut raw_segs: Vec<Path> = Vec::new();
            let mut no_solution = false;
            // P7：环带目标集只作用于最后一段（→ target）；有武器时 FMM 传播
            // 到环带 [Rmin, Rmax] 内 T 最小可达 cell 即停（docs/技术方案 §4.2）。
            let ring_range: Option<[f64; 2]> =
                v.weapon.as_ref().and_then(|w| w.effective_range_km());
            let seg_total = seg_ends.windows(2).len();
            for (si, seg) in seg_ends.windows(2).enumerate() {
                let (s, e) = (seg[0], seg[1]);
                let is_target_seg = si == seg_total - 1;
                let (sr, sc) = lonlat_cell(s.lon, s.lat, &region, grid);
                let (dr, dc) = lonlat_cell(e.lon, e.lat, &region, grid);
                let t0 = Instant::now();
                let res = fmm_propagate(field_ref, sr, sc);
                fmm_ms += t0.elapsed().as_secs_f64() * 1000.0;
                let mut cells_opt: Option<Vec<(usize, usize)>> = None;
                if is_target_seg {
                    if let Some([rmin_km, rmax_km]) = ring_range {
                        if let Some((rr, rc)) =
                            ring_target_cell(&res, &v.target, &region, grid, rmin_km, rmax_km)
                        {
                            cells_opt = backtrack_path(field_ref, &res, rr, rc, sr, sc);
                        }
                        if cells_opt.is_none() {
                            // 环带内无可达 cell → 仅当目标点本身落在环带内（Rmin≈0）
                            // 才回退点目标；否则"带最小射程的武器不得停在 Rmin 内"
                            // → 几何无解（degradations 标注，后续随 no_solution 出口）。
                            let d_km =
                                crate::path::haversine_m(e.lon, e.lat, v.target.lon, v.target.lat)
                                    / 1000.0;
                            if d_km >= rmin_km && d_km <= rmax_km {
                                cells_opt = backtrack_path(field_ref, &res, dr, dc, sr, sc);
                            }
                            if cells_opt.is_none() {
                                degradations.push(format!(
                                    "ring target unreachable: no cell in [{rmin_km}, {rmax_km}] km of target (v={})",
                                    v.id
                                ));
                            }
                        }
                    } else {
                        // 无武器 → 点目标：目标 cell 回溯；失败 → 无解出口链③
                        // （docs/01 §5：目标半径放宽——在目标附近容差内找 T 最小
                        // 可达 cell，到达并显式降级标注，不静默无解）。
                        cells_opt = backtrack_path(field_ref, &res, dr, dc, sr, sc);
                        if cells_opt.is_none() {
                            if let Some((rr, rc, relax_km)) = relaxed_target_cell(
                                &res,
                                &region,
                                grid,
                                e.lon,
                                e.lat,
                                RELAX_TARGET_MAX_KM,
                            ) {
                                cells_opt = backtrack_path(field_ref, &res, rr, rc, sr, sc);
                                if cells_opt.is_some() {
                                    degradations.push(format!(
                                        "target point unreachable; radius relaxed to {relax_km:.1} km (v={})",
                                        v.id
                                    ));
                                }
                            }
                        }
                    }
                } else {
                    cells_opt = backtrack_path(field_ref, &res, dr, dc, sr, sc);
                }
                let Some(mut cells) = cells_opt else {
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
                if !grid_refined && grid < 1024 {
                    // 无解出口链④（docs/01 §5）：走廊细分/重路由——grid 翻倍重建
                    // 代价场重试一次。粗层无解可能是网格离散导致的窄通道漏检
                    // （FMM 格点采样漏窄缝/尖角 → 细分后找到）；细分后仍无解 →
                    // 几何无解（no_solution，原因码 coarse FMM no path）。
                    let old_grid = grid;
                    grid_refined = true;
                    grid = (grid * 2).min(1024);
                    cell_m = region.span_deg * 111_320.0 / grid as f64;
                    inflation_cells = if region.span_deg > 2.5 {
                        ((inflation_m + 0.71 * cell_m) / cell_m.max(1.0)).ceil() as usize + 1
                    } else {
                        (inflation_m / cell_m.max(1.0)).ceil() as usize
                    };
                    field = build_cost_field(
                        &region,
                        grid,
                        &terrain,
                        &all_zones,
                        inflation_m,
                        &threat,
                        params_merged.radar_cost_coef,
                        params_merged.los_mask_coef,
                        !input.red_forces.radars.is_empty(),
                    );
                    degradations.push(format!(
                        "coarse FMM no path at grid {old_grid}; corridor refined to grid {grid} (v={})",
                        v.id
                    ));
                    eprintln!(
                        "[debug] coarse FMM no path -> refined grid {old_grid}->{grid} (v={})",
                        v.id
                    );
                    continue 'fmm_attempt;
                }
                out_aircraft.push(AircraftOutput {
                    id: v.id.clone(),
                    status: "no_solution".into(),
                    path: Vec::new(),
                    distance_m: 0.0,
                    warnings: vec!["coarse FMM no path".into()]
                });
                // P3 分类结论出口（docs/12 §3.4/§12.4）：粗层真无通道 → 几何无解分类
                emit_classified(
                    &v.id,
                    "geometrically_impossible",
                    "coarse FMM no path (docs/12 §11.2)",
                    &mut degradations,
                );
                continue 'aircraft;
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
                // 记录撞山区段最高地形点坐标（方案 B 自动中间锚点）
                let mut path_max_lon = 0.0;
                let mut path_max_lat = 0.0;
                for ends in seg_ends.windows(2) {
                    let (a, b) = (ends[0], ends[1]);
                    let seg_len_m = crate::path::haversine_m(a.lon, a.lat, b.lon, b.lat);
                    let n = ((seg_len_m / 1_000.0).ceil() as usize).max(2);
                    for i in 0..=n {
                        let tt = i as f64 / n as f64;
                        let lon = a.lon + (b.lon - a.lon) * tt;
                        let lat = a.lat + (b.lat - a.lat) * tt;
                        if let Sample::Land(h) = t.sample_at(lon, lat) {
                            if h > path_max_terr {
                                path_max_terr = h;
                                path_max_lon = lon;
                                path_max_lat = lat;
                            }
                        }
                    }
                }
                let clearance = opts.clearance_m.max(1.0);
                if path_max_terr > 0.0
                    && path_max_terr + clearance >= alt_eff + TERRAIN_MASK_SLACK_M
                {
                    let new_alt = (path_max_terr + clearance + 100.0).max(v.alt_m);
                    let ceiling_ok = v.profile.maximum_altitude_m.is_none_or(|c| new_alt <= c);
                    if new_alt > alt_eff + 0.5 && ceiling_ok {
                        terrain_alt_raised = true;
                        alt_eff = new_alt;
                        terrain_anchor = Some((path_max_lon, path_max_lat, new_alt));
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
                    v.profile.maximum_altitude_m,                    terrain.as_source(),
                    &v.start,
                    &target,
                    inflation_m / 1000.0,
                    &mut degradations,
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
                    zone_inflation_m: inflation_m
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
                // P7：发射包线终端航向下放平滑级（docs/技术方案 §4.2：终端姿态不只是
                // 到达判据，作为平滑级输入）——最后一段末点 heading_deg = 窗口中心，
                // Dubins 拟合天然吃终端 pose（docs/08：heading 已支持）。不提供 heading
                // 窗口 → 不约束（现状点目标语义）。
                if let Some([lo, hi]) = v
                    .weapon
                    .as_ref()
                    .and_then(|w| w.envelope.as_ref())
                    .and_then(|e| e.heading_deg)
                {
                    if let Some(last_seg) = smooth_src.last_mut() {
                        if let Some(p) = last_seg.points.last_mut() {
                            p.heading_deg = Some(heading_window_center(lo, hi));
                        }
                    }
                }
                // 段级平滑中间阶段的地形净空不足最大高度（smooth.rs SmoothResult.
                // terrain_gap_m）：theta_star 拉直段穿山被回退楼梯吞掉时，final verify
                // 无 terrain issue，靠这里触发抬升重跑（2026-08-11 zz30 2480m 峰）。
                let mut seg_terr_max: f64 = 0.0;
                // 段边界硬约束点（起点/必经点/目标）：arc 修复会弹出边界点 b，必经点不得
                // 被替代（user 硬约束），否则违反"任何平滑不得移除必经点"。
                let hard_boundary: Vec<(f64, f64)> =
                    seg_ends.iter().map(|g| (g.lon, g.lat)).collect();
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
                        let result =
                            smooth_path_chain(seg, &chain, &opts, &ctx, Some(phys_min_radius_m));
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
                                        &arc_pts,
                                        &e,
                                        &c2,
                                        &all_zones,
                                        inflation_m,
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
                                        let bc_m =
                                            crate::path::haversine_m(b.lon, b.lat, c.lon, c.lat);
                                        let ext_m = (4.0 * opts.turn_radius_m).min(bc_m * 0.75);
                                        if ext_m > opts.turn_radius_m {
                                            let h_bc = crate::path::bearing_deg(
                                                b.lon, b.lat, c.lon, c.lat,
                                            );
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
                                                    &arc_pts2,
                                                    &e2,
                                                    &c2,
                                                    &all_zones,
                                                    inflation_m,
                                                ) {
                                                    continue;
                                                }
                                                // 弧点转角（与 verify 同口径 bearing）：入段 a→S
                                                // 及弧内各段均 ≤ max_turn。外推 E' 只影响末段
                                                // p_{n-1}→E'（细分后 ≈ 出段方向）。
                                                let mut prev_h = crate::path::bearing_deg(
                                                    a.lon,
                                                    a.lat,
                                                    arc_pts2[0].lon,
                                                    arc_pts2[0].lat,
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
                                            } else if !accepted
                                                && std::env::var_os("ARP_DEBUG_SMOOTH").is_some()
                                            {
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
                let final_rep =
                    crate::smooth::verify_path(&joined, None, &opts, &ctx, Some(phys_min_radius_m));
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
                    let mut terr_anchor: Option<(f64, f64, f64)> = None; // (lon, lat, terrain h)
                    let mut terr_max = 0.0_f64;
                    for iss in &final_rep.issues {
                        let Some(pos) = iss.find("(terrain ") else {
                            continue;
                        };
                        let tail = iss[pos + "(terrain ".len()..].trim_end_matches(')').trim();
                        let Ok(h) = tail.trim_end_matches('m').trim().parse::<f64>() else {
                            continue;
                        };
                        if h > terr_max {
                            terr_max = h;
                            // issue 前缀含 "sample (lon=..,lat=..)" → 解析坐标（自动锚点用）
                            let coord = (|| {
                                let lon = iss.find("lon=").and_then(|p| {
                                    let s: String = iss[p + 4..]
                                        .chars()
                                        .take_while(|c| {
                                            c.is_ascii_digit() || *c == '.' || *c == '-'
                                        })
                                        .collect();
                                    s.parse().ok()
                                })?;
                                let lat = iss.find("lat=").and_then(|p| {
                                    let s: String = iss[p + 4..]
                                        .chars()
                                        .take_while(|c| {
                                            c.is_ascii_digit() || *c == '.' || *c == '-'
                                        })
                                        .collect();
                                    s.parse().ok()
                                })?;
                                Some((lon, lat, h))
                            })();
                            terr_anchor = coord;
                        }
                    }
                    let terr_max = terr_max.max(seg_terr_max);
                    if terr_max > 0.0 {
                        let clearance = opts.clearance_m.max(1.0);
                        let new_alt = (terr_max + clearance + 100.0).max(v.alt_m);
                        let ceiling_ok = v.profile.maximum_altitude_m.is_none_or(|c| new_alt <= c);
                        if new_alt > alt_eff + 0.5 && ceiling_ok {
                            terrain_alt_raised = true;
                            alt_eff = new_alt;
                            // 自动中间锚点：优先 verify issue 坐标；缺失（seg_terr_max 更大 /
                            // issue 无坐标）→ 沿当前走廊 raw_joined 采样地形找最高点（近似）。
                            let anchor = terr_anchor.or_else(|| {
                                let t = terrain.as_source()?;
                                let mut best: Option<(f64, f64, f64)> = None;
                                let mut bh = 0.0_f64;
                                for p in &raw_joined.points {
                                    if let Sample::Land(h) = t.sample_at(p.lon, p.lat) {
                                        if h > bh {
                                            bh = h;
                                            best = Some((p.lon, p.lat, h));
                                        }
                                    }
                                }
                                best.map(|(lo, la, _)| (lo, la, new_alt))
                            });
                            if let Some(a) = anchor {
                                terrain_anchor = Some(a);
                            }
                            eprintln!(
                                "[debug] smooth terrain clearance -> raise cruise alt {:.0}->{:.0}m (terrain {:.0}m, v={})",
                                start_alt_norm, alt_eff, terr_max, v.id
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
                                .zones
                                .iter()
                                .filter(|z| z.is_wall())
                                .cloned()
                                .collect::<Vec<_>>(),
                            ctx.terrain.is_some(),
                        )
                    {
                        let skeleton: Vec<[f64; 2]> =
                            raw_joined.points.iter().map(|p| [p.lon, p.lat]).collect();
                        let clusters = crate::patch::cluster_failures(
                            &final_rep.issues,
                            crate::patch::PATCH_R_DEG,
                        );
                        if !clusters.is_empty() && skeleton.len() >= 2 {
                            let wall_polys: Vec<&Zone> = input
                                .zones
                                .iter()
                                .filter(|z| {
                                    z.is_wall() && matches!(z.shape, ZoneShape::Polygon { .. })
                                })
                                .collect();
                            let circle_walls: Vec<&Zone> = input
                                .zones
                                .iter()
                                .filter(|z| {
                                    z.is_wall() && matches!(z.shape, ZoneShape::Circle { .. })
                                })
                                .collect();
                            let restricted: Vec<&Zone> = input
                                .zones
                                .iter()
                                .filter(|z| !z.is_wall())
                                .collect();
                            let inflation_m = (opts.turn_radius_m * 0.5).clamp(2_000.0, 10_000.0);
                            let radar_opt = if input.red_forces.radars.is_empty() {
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
                                let mut rect = crate::patch::PatchRect::from_center(
                                    *c,
                                    crate::patch::PATCH_R_DEG,
                                );
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
                                    let (ein, eout) =
                                        crate::patch::boundary_anchors(&current, &rect);
                                    if let (Some(a), Some(b)) = (ein, eout) {
                                        let obstacles: Vec<Vec<[f64; 2]>> = wall_polys
                                            .iter()
                                            .filter_map(|z| match &z.shape {
                                                ZoneShape::Polygon { vertices } => {
                                                    if vertices
                                                        .iter()
                                                        .any(|v| rect.contains([v[0], v[1]]))
                                                    {
                                                        Some(vertices.clone())
                                                    } else {
                                                        None
                                                    }
                                                }
                                                _ => None
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
                                                            r_eff_m: radius_km * 1000.0
                                                                + inflation_m
                                                        })
                                                    } else {
                                                        None
                                                    }
                                                }
                                                _ => None
                                            })
                                            .collect();
                                        if !obstacles.is_empty()
                                            || !circle_obs.is_empty()
                                            || !restricted.is_empty()
                                        {
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
                                                crate::patch::PatchOutcome::Path(
                                                    patch_pts,
                                                    _len_km,
                                                ) => {
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
                                                        warnings
                                                            .extend(rep_j.warnings.iter().cloned());
                                                        break;
                                                    }
                                                    // 接缝违规 → C7：扩张 patch 重试（硬上限 + 固定步长）
                                                    if retry < crate::patch::PATCH_RETRY_MAX {
                                                        rect.half_deg *=
                                                            crate::patch::PATCH_RETRY_EXPAND;
                                                        retry += 1;
                                                        continue;
                                                    }
                                                    // C3 归因：verify 硬闸失败 → 拟合缺陷 vs 几何无解
                                                    let all_inflated: Vec<Vec<[f64; 2]>> =
                                                        obstacles
                                                            .iter()
                                                            .map(|v| {
                                                                let hull =
                                                                    crate::patch::convex_hull(v);
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
                                                crate::patch::PatchOutcome::GeometricImpossible => {
                                                }
                                            }
                                        }
                                    }
                                    break;
                                }
                            }
                            if used_any {
                                pts = current
                                    .iter()
                                    .map(|[lon, lat]| {
                                        crate::path::PathPoint::new(*lon, *lat, alt_eff)
                                    })
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
                    let threat_ok = straight
                        .points
                        .iter()
                        .all(|p| threat.static_penetration(p.lon, p.lat, p.alt_m) >= 0.7);
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
        // 抬升提示（2026-08-10 / 2026-08-14）：中间段巡航高度被地形抬升必须显式告知；
        // 起点/终点保持用户高度（任务硬约束，不因中间障碍抬升——中间段由自动锚点爬升）。
        if terrain_alt_raised && alt_eff > start_alt_norm + 0.5 {
            let terr_max = (alt_eff - opts.clearance_m.max(1.0) - 100.0).max(0.0);
            let msg = format!(
                "terrain clearance: cruise altitude raised {:.0}->{:.0}m (terrain up to {:.0}m); start/target kept at user altitudes",
                start_alt_norm, alt_eff, terr_max
            );
            warnings.push(msg.clone());
            degradations.push(msg);
        }
        // 降速提示（主管 2026-08-07：速度非锁定，转弯段可降速实现小半径）：
        // turn_radius < 巡航物理下限 → 转弯段需降到 v_turn = sqrt(r·g·tanφ)。
        if opts.turn_radius_m > 0.0 {
            let bank = params_merged.default_max_bank_deg;
            let v_turn = (opts.turn_radius_m * 9.81 * bank.to_radians().tan()).sqrt();
            let cruise_v = match v.profile.aircraft_type {
                crate::config::AircraftType::FixedWing => {
                    params_merged.default_fixed_wing_cruise_speed_mps
                }
                crate::config::AircraftType::Rotorcraft => {
                    params_merged.default_rotorcraft_cruise_speed_mps
                }
            };
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
            out_aircraft.push(AircraftOutput {
                id: v.id.clone(),
                status: "no_solution".into(),
                path: Vec::new(),
                distance_m: 0.0,
                warnings: diag
            });
            continue 'aircraft;
        }
        // 垂直剖面（2026-08-12 主管 demo 轨迹倾斜）：输出高度从起点高度按累计
        // 距离线性过渡到目标高度（起终点不同高时轨迹呈现爬升/下降，而非恒为
        // 巡航高度的水平直线）；地形可用时保底（下降段不穿山）。起终点同高 →
        // 曲线水平，行为与既往一致（含受限区剖面/抬升巡航语义）。
        // P7：发射包线高度窗口优先（docs/技术方案 §4.2：区域与包线冲突时以包线
        // 优先）——终点目标高度 clamp 到 [lo, hi] 窗口（若提供）。
        let target_alt_eff = v
            .weapon
            .as_ref()
            .and_then(|w| w.envelope.as_ref())
            .and_then(|e| e.alt_m)
            .map_or(v.target_alt_m, |[lo, hi]| v.target_alt_m.clamp(lo, hi));
        // 终点地面抬升（主管 2026-08-14 三反）：终点 < 当地地面 → 抬到地面 MSL（物理
        // 保底，不加净空——目标点是降落场景，贴地合理；在武器包线 clamp 之后兜底）。
        let target_alt_norm = target_alt_eff.max(ground_at(target.lon, target.lat));
        if target_alt_norm > target_alt_eff + 0.5 {
            warnings.push(format!(
                "target altitude below terrain: raised {:.0}->{:.0}m (terrain {:.0}m)",
                target_alt_eff,
                target_alt_norm,
                ground_at(target.lon, target.lat)
            ));
        }
        // 垂直剖面锚点 = 用户必经点 + 抬升决策自动插入的撞山最高点锚点（方案 B）
        let mut mid_anchors = v
            .mid_waypoints
            .iter()
            .zip(v.mid_alts.iter())
            .map(|(g, &a)| (g.lon, g.lat, a))
            .collect::<Vec<_>>();
        if let Some((alon, alat, aalt)) = terrain_anchor {
            mid_anchors.push((alon, alat, aalt));
        }
        apply_vertical_profile(
            &mut pts,
            start_alt_norm,
            target_alt_norm,
            alt_eff,
            &mid_anchors,
            terrain.as_source(),
            opts.clearance_m,
        );
        // 地形跟随细化（方案 B 2026-08-14）：中间段沿地形缓爬升/下降，起终点保持
        // 用户高度；插值不足以覆盖地形处插入段内细化点（净空 ≥ clearance）；起点段/
        // 终点段（起飞/降落）插入点按 max_climb_angle 爬升/下降线（净空 < 100 允许、
        // 不穿地，主管 2026-08-14 三反）。
        terrain_follow_insert(
            &mut pts,
            terrain.as_source(),
            opts.clearance_m,
            opts.max_climb_deg,
        );
        // 爬升率平滑（主管 2026-08-14 低空场景）：起终点/绕障处抬升下降近乎垂直，
        // 固定翼巡航不可行 → 限制相邻点坡度 ≤ max_climb_angle_deg（净空优先）。
        apply_climb_rate(
            &mut pts,
            opts.max_climb_deg,
            terrain.as_source(),
            opts.clearance_m,
        );
        // P7：发射包线到达判定（docs/技术方案 §4.2：落点 ∈ [Rmin,Rmax] ∧ 发射包线
        // 都满足才算到达）。heading/alt/环带距离 = 硬校验（不满足 → 未到达 →
        // no_solution，宁可不给路径，不给违禁路径）；speed 是常量输入（规划不可
        // 调）→ 软校验（degradation 告警）。无武器 / 无 envelope → 不校验（现状
        // 点目标语义，零回归）。
        if let Some(w) = &v.weapon {
            // Rmin 未定（lo ≤ 0）→ 显式告警（docs/技术方案 §4.2：不静默当无下限处理）
            if let Some([lo, _]) = w.effective_range_km() {
                if lo <= 0.0 {
                    degradations.push(format!(
                        "weapon on {}: Rmin {lo} km undefined (treated as no minimum, ring = [0, Rmax])",
                        v.id
                    ));
                }
            }
            let mut env_fail: Vec<String> = Vec::new();
            if let Some(last) = pts.last() {
                if let Some([rmin_km, rmax_km]) = w.effective_range_km() {
                    let d_km =
                        crate::path::haversine_m(last.lon, last.lat, v.target.lon, v.target.lat)
                            / 1000.0;
                    if !(d_km >= rmin_km - 1e-6 && d_km <= rmax_km + 1e-6) {
                        env_fail.push(format!(
                            "terminal {d_km:.1} km from target outside weapon ring [{rmin_km}, {rmax_km}] km"
                        ));
                    }
                }
            }
            if let Some(env) = &w.envelope {
                if let Some([lo, hi]) = env.heading_deg {
                    let h_last = if pts.len() >= 2 {
                        let a = &pts[pts.len() - 2];
                        let b = pts.last().unwrap();
                        crate::path::bearing_deg(a.lon, a.lat, b.lon, b.lat)
                    } else {
                        f64::NAN
                    };
                    if h_last.is_finite() && !heading_in_window(h_last, lo, hi) {
                        env_fail.push(format!(
                            "terminal heading {h_last:.1} deg outside [{lo}, {hi}]"
                        ));
                    }
                }
                if let Some([lo, hi]) = env.alt_m {
                    let alt_last = pts.last().map_or(f64::NAN, |p| p.alt_m);
                    if alt_last.is_finite() && !(alt_last >= lo && alt_last <= hi) {
                        env_fail.push(format!("terminal alt {alt_last:.0} m outside [{lo}, {hi}]"));
                    }
                }
                if let Some([lo, hi]) = env.speed_mps {
                    let cruise = v
                        .profile
                        .maximum_speed_mps
                        .unwrap_or(match v.profile.aircraft_type {
                            crate::config::AircraftType::FixedWing => {
                                params_merged.default_fixed_wing_cruise_speed_mps
                            }
                            crate::config::AircraftType::Rotorcraft => {
                                params_merged.default_rotorcraft_cruise_speed_mps
                            }
                        });
                    if !(cruise >= lo && cruise <= hi) {
                        degradations.push(format!(
                            "launch envelope: cruise speed {cruise:.0} m/s outside weapon speed window [{lo}, {hi}] (speed is input constant, soft)"
                        ));
                    }
                }
            }
            if !env_fail.is_empty() {
                warnings.push(format!(
                    "launch envelope not satisfied: {}",
                    env_fail.join("; ")
                ));
                emit_classified(
                    &v.id,
                    "geometrically_impossible",
                    "launch envelope not satisfied (docs/技术方案 §4.2)",
                    &mut degradations,
                );
                out_aircraft.push(AircraftOutput {
                    id: v.id.clone(),
                    status: "no_solution".into(),
                    path: Vec::new(),
                    distance_m: 0.0,
                    warnings
                });
                continue 'aircraft;
            }
        }
        let dist = Path::new(pts.clone()).length_m();
        out_aircraft.push(AircraftOutput {
            id: v.id.clone(),
            status: "planned".into(),
            path: pts
                .iter()
                .map(|p| PathPoint {
                    x: p.lon,
                    y: p.lat,
                    alt_m: p.alt_m
                })
                .collect(),
            distance_m: dist,
            warnings
        });
    }

    // P6-C（docs/01 §7.1）：多机路径空间交叉检测——输出后处理，检出显式告警；
    // 时间维 out-of-scope（不判是否同时到达）。
    detect_multi_aircraft_crossings(&mut out_aircraft);

    Ok(Output {
        status: "success".into(),
        error: None,
        elapsed_ms: Some(elapsed_ms),
        aircraft: out_aircraft,
        stats: Stats {
            fmm_ms,
            los_checks: 0,
            degradations
        }
    })
}

// ==================== 辅助 ====================

/// 任务区域缓冲（度）：保证源/目标不贴边；同时是障碍感知外扩的机动余量
/// （2026-08-11 zz_region_block2：墙占满 region 短边方向时绕行被迫出 region）。
const REGION_PAD_DEG: f64 = 0.15;

// ==================== P8 无解出口链③ 目标半径放宽 ====================
// docs/01 §5 回退层③：点目标 cell 不可达（墙/地形挡）→ 在目标附近容差内
// 找 T 最小可达 cell 到达并显式降级标注（不静默无解）。上限保守 10km
// （≈9 个默认格距；越过则判几何无解 → no_solution）。
const RELAX_TARGET_MAX_KM: f64 = 10.0;

/// P8 M5 LOS mask 静态代价场参考高度（米，MSL）：代价场多机共享，LOS 判定用
/// 固定参考高度近似（地形遮蔽随高度变化小；verify 威胁评估仍用实际路径高度精确
/// 判定）。默认巡航量级，与各机 start.alt_m 同量级。
const LOS_REF_ALT_M: f64 = 3000.0;

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

/// 垂直剖面（2026-08-12 主管 demo 轨迹倾斜 + 2026-08-14 起终点约束修正）：
/// 输出路径高度从起点高度按累计水平距离线性过渡到目标高度。
/// - **起终点 = 任务硬约束**：起点/终点锚点 = 用户设定（低于地面时已在输入侧抬到
///   地面 MSL），**不因中间地形抬升**——不再整条路径强制抬到抬升巡航高度
///   （主管 2026-08-14：中间高度变化靠改变航路中间点，不强制改起终点）；
/// - **抬升场景**（cruise_alt > start_alt，地形要求）→ 中间段按 mid_anchors
///   （含抬升决策自动插入的撞山最高点锚点，方案 B）插值爬升/下降，逐点地形保底
///   兜底净空；**起终点同高也不早退**（起终点钉用户高度、中间段仍按锚点/保底抬升）；
/// - **剖面段保持**：原高度显著偏离巡航高度的点 = 受限区底部穿行/顶部绕飞剖面
///   （build_restricted_profiles 已按语义生成并经验证）→ 垂直剖面不覆盖
///   （zigzag24 底部 1500m 剖面保持）；
/// - 非抬升场景（alt_eff == start_alt）→ 平滑线性过渡（下降场景也生效）；
/// - 中间点地形保底：高度 ≥ 地形 + 净空 + 100m（下降段不穿山）；
///   起终点地形保底：≥ 地面（不叠加净空余量——地面是物理可达底线，输入侧已抬升）；
/// - 不重跑 verify：中间段高度由保底保证净空（≥ 既有 verify 高度或更高）；
///   起终点按用户约束（输入侧已保证 ≥ 地面）。
fn apply_vertical_profile(
    pts: &mut [crate::path::PathPoint],
    start_alt: f64,
    target_alt: f64,
    cruise_alt: f64,
    mid_anchors: &[(f64, f64, f64)],
    terrain: Option<&dyn TerrainSource>,
    clearance_m: f64,
) {
    // 抬升场景（cruise > start）：即使起终点同高也必须处理（起终点钉用户高度、
    // 中间段按自动锚点/保底抬升）；非抬升且同高 → 无高度调整需求（保持现状）。
    let raised = cruise_alt > start_alt + 0.5;
    if pts.len() < 2 || ((target_alt - start_alt).abs() < 0.5 && !raised) {
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
    let clearance = clearance_m.max(1.0);
    // 方案 B（2026-08-14）：不再强制中间点 ≥ 抬升巡航高度（raise_floor）——
    // 中间段高度 = 锚点线性插值（起终点 + 用户必经点 + 自动撞山锚点）+ 地形保底，
    // 形成"起点爬升 → 过山 → 下降回目标"的自然轮廓，起终点不被抬升。
    // P8 M2：必经点高度锚点分段（mid_anchors = (lon, lat, alt) 序列）。
    // 分段插值节点表 [(idx, alt)]：起点 (0, start_alt) → 必经点最近点 → 终点。
    // 段端点（必经点/目标）是硬约束不被平滑移除 → 最近点顺序匹配可靠（index 递增）。
    let mut anchors: Vec<(usize, f64)> = Vec::with_capacity(mid_anchors.len() + 2);
    anchors.push((0, start_alt));
    if !mid_anchors.is_empty() {
        let mut search_from = 0usize;
        for &(alon, alat, aalt) in mid_anchors {
            let mut best = search_from;
            let mut best_d = f64::MAX;
            for i in search_from..pts.len() {
                let dx = (pts[i].lon - alon) * kx;
                let dy = (pts[i].lat - alat) * ky;
                let d = dx * dx + dy * dy;
                if d < best_d {
                    best_d = d;
                    best = i;
                }
            }
            search_from = best + 1;
            // 重复 idx（必经点与上一锚点/终点重合）→ 跳过（高度被覆盖，无独立效果）
            if best > anchors.last().unwrap().0 {
                anchors.push((best, aalt));
            }
        }
    }
    anchors.push((pts.len() - 1, target_alt));
    for i in 0..pts.len() {
        // 剖面段保持：原高度显著偏离巡航高度 → 受限区底部/顶部剖面（主动升降高），
        // 垂直剖面不覆盖（其高度已经 build_restricted_profiles 语义验证）。
        // 注意：插值前路径点高度 = 巡航高度或剖面段高度，不存在"插值中间点"，
        // 因此下降/爬升场景的巡航段（orig == cruise）不会误判为剖面段。
        if (pts[i].alt_m - cruise_alt).abs() > 0.5 {
            continue;
        }
        // 找 i 所在锚点区间 [anchors[k], anchors[k+1]]（anchors 严格递增；
        // i == 最后锚点（终点）时 k+1 越界 → clamp 到自身 = 精确锚点高度）
        let k = anchors.partition_point(|&(idx, _)| idx <= i) - 1;
        let (lo, h_lo) = anchors[k];
        let (hi, h_hi) = anchors[(k + 1).min(anchors.len() - 1)];
        let seg_len = (cum[hi] - cum[lo]).max(1.0);
        let t = ((cum[i] - cum[lo]) / seg_len).clamp(0.0, 1.0);
        let mut h = h_lo + (h_hi - h_lo) * t;
        // 地形保底：中间点 ≥ 地形 + 净空（2026-08-14 主管低空场景：保底余量 +100
        // 会让起点段形成"余量高台阶"（如起点 500m、地形 377m → 保底 578m，起点旁
        // 250m 内爬升 78m ≈ 17° 超过 15° 爬升率）→ 保底 = 地形 + 净空，恰为 verify
        // 口径；更高的余量由抬升锚点（new_alt = 地形+净空+100）与 apply_climb_rate
        // 兜底）；起终点 ≥ 地面（主管 2026-08-14 三反：起终点是起飞/降落场景，贴地
        // 合理、不叠加净空余量——起终点是任务约束，地面是物理可达底线；起飞段允许
        // 贴近地形，中间巡航段才保净空；输入侧已规范化）。
        if let Some(tsrc) = terrain {
            if let Sample::Land(g) = tsrc.sample_at(pts[i].lon, pts[i].lat) {
                if i == 0 || i == pts.len() - 1 {
                    h = h.max(g);
                } else {
                    h = h.max(g + clearance);
                }
            }
        }
        pts[i].alt_m = h;
    }
}

/// 地形跟随细化（方案 B，主管 2026-08-14）：沿路径各段 250m 细采样找段内
/// **最差峰**（插值高度 − 地形+净空 缺口最大处），插值不足以覆盖时在峰顶插入
/// 1 个新路径点（高度 = 峰顶 + 净空）。
/// 每段每轮最多插入 1 点 → 递归后每个独立峰各 1 点：既保证净空（7.5as 网格
/// ~230m，250m 采样基本全覆盖，不再像 1000m 相位错过 500m 宽尖峰 → 78m 穿山），
/// 又避免逐 500m 采样导致的避障段航路点过密（主管 2026-08-14 反馈：62 点）。
/// 插入点坐标在原段直线上（纯细分，不改变水平路径形状/转弯角）；起终点保持
/// 用户高度（任务硬约束），中间段沿地形缓爬升/下降；更高的余量由抬升锚点
/// （new_alt = 地形+净空+100）与 apply_climb_rate 兜底。
fn terrain_follow_insert(
    pts: &mut Vec<crate::path::PathPoint>,
    terrain: Option<&dyn TerrainSource>,
    clearance_m: f64,
    max_climb_angle_deg: f64,
) {
    let Some(tsrc) = terrain else { return };
    let clearance = clearance_m.max(1.0);
    let tan_a = max_climb_angle_deg.max(1.0).to_radians().tan();
    let mut guard = 0;
    loop {
        guard += 1;
        if guard > 64 {
            break;
        }
        let mut inserted = false;
        let mut i = 0;
        while i + 1 < pts.len() {
            let a = pts[i];
            let b = pts[i + 1];
            let seg_len_m = crate::path::haversine_m(a.lon, a.lat, b.lon, b.lat);
            // 短段（< 2 个采样间隔）端点已保底，无内部峰可插
            if seg_len_m < 600.0 {
                i += 1;
                continue;
            }
            let n = ((seg_len_m / 250.0).ceil() as usize).max(1);
            // 段内最差峰：need(地形+净空) − 插值 最大处（含缺口为负 = 净空充足）
            let mut worst_tt = -1.0;
            let mut worst_gap = f64::NEG_INFINITY;
            let mut worst_need = 0.0;
            for k in 1..n {
                let tt = k as f64 / n as f64;
                let lon = a.lon + (b.lon - a.lon) * tt;
                let lat = a.lat + (b.lat - a.lat) * tt;
                let alt_interp = a.alt_m + (b.alt_m - a.alt_m) * tt;
                if let Sample::Land(h) = tsrc.sample_at(lon, lat) {
                    let need = h + clearance;
                    let gap = need - alt_interp;
                    if gap > worst_gap {
                        worst_gap = gap;
                        worst_tt = tt;
                        worst_need = need;
                    }
                }
            }
            if worst_tt > 0.0 && worst_gap > 0.5 {
                let lon = a.lon + (b.lon - a.lon) * worst_tt;
                let lat = a.lat + (b.lat - a.lat) * worst_tt;
                let mut new_alt = worst_need;
                // 起点段/终点段 = 起飞/降落段（主管 2026-08-14 三反：起终点贴地合理，
                // 不加净空；爬升/下降坡度 ≤ max_climb_angle 优先，净空 < 100 允许但
                // ≥ 地形不穿地）→ 插入点高度 = min(地形+净空, 15° 爬升/下降线)，
                // 保证起点段按 15° 平滑起飞（不再 22° 陡爬 + 17m 贴山）、
                // 终点段按 15° 平滑降落。
                let is_start_seg = i == 0;
                let is_end_seg = i + 2 == pts.len();
                if is_start_seg || is_end_seg {
                    let end_ref = if is_start_seg {
                        pts[0]
                    } else {
                        pts[pts.len() - 1]
                    };
                    let dist_to_ref = crate::path::haversine_m(lon, lat, end_ref.lon, end_ref.lat);
                    new_alt = new_alt.min(end_ref.alt_m + tan_a * dist_to_ref);
                    // 不穿地（起飞/降落段净空 ≥ 0 底线）
                    if let Sample::Land(h) = tsrc.sample_at(lon, lat) {
                        new_alt = new_alt.max(h);
                    }
                }
                pts.insert(i + 1, crate::path::PathPoint::new(lon, lat, new_alt));
                inserted = true;
            } else {
                i += 1;
            }
        }
        if !inserted {
            break;
        }
    }
}

/// 爬升率平滑（主管 2026-08-14：起终点/绕障处抬升下降近乎垂直，固定翼巡航不可行）：
/// 限制相邻路径点坡度 ≤ max_climb_angle_deg（上坡与下坡同限）。
/// - 前向（起点→终点）：超陡爬升段压低到「前点 + tan×距离」，但不低于地形保底
///   （净空优先，压不低则保持保底——物理上该段只能绕飞或贴地爬升）；
/// - 后向（终点→起点）：超陡下降段压低前点到「后点 + tan×距离」（同样不低于
///   地形保底；压低只会让该点净空更紧张但保底兜底，不会穿山）；
/// - 迭代直到收敛（前向压低可能改变后向可行性）。
fn apply_climb_rate(
    pts: &mut [crate::path::PathPoint],
    max_climb_angle_deg: f64,
    terrain: Option<&dyn TerrainSource>,
    clearance_m: f64,
) {
    if pts.len() < 2 {
        return;
    }
    let tan_a = max_climb_angle_deg.max(1.0).to_radians().tan();
    let clearance = clearance_m.max(1.0);
    let floor_at = |lon: f64, lat: f64| -> f64 {
        terrain.map_or(f64::NEG_INFINITY, |t| match t.sample_at(lon, lat) {
            Sample::Land(h) => h + clearance,
            _ => f64::NEG_INFINITY
        })
    };
    let mut guard = 0;
    loop {
        guard += 1;
        if guard > 32 {
            break;
        }
        let mut changed = false;
        // 前向：限制上升率（pts[i] ≤ pts[i-1] + tan×dist）
        for i in 1..pts.len() {
            let (a, b) = (pts[i - 1], pts[i]);
            let dist = crate::path::haversine_m(a.lon, a.lat, b.lon, b.lat);
            let max_alt = a.alt_m + tan_a * dist;
            if pts[i].alt_m > max_alt + 0.5 {
                let new = max_alt.max(floor_at(pts[i].lon, pts[i].lat));
                if new < pts[i].alt_m - 0.5 {
                    pts[i].alt_m = new;
                    changed = true;
                }
            }
        }
        // 后向：限制下降率（pts[i] ≤ pts[i+1] + tan×dist，前点不能比后点高太多）
        for i in (0..pts.len() - 1).rev() {
            let (a, b) = (pts[i], pts[i + 1]);
            let dist = crate::path::haversine_m(a.lon, a.lat, b.lon, b.lat);
            let max_alt = b.alt_m + tan_a * dist;
            if pts[i].alt_m > max_alt + 0.5 {
                let new = max_alt.max(floor_at(pts[i].lon, pts[i].lat));
                if new < pts[i].alt_m - 0.5 {
                    pts[i].alt_m = new;
                    changed = true;
                }
            }
        }
        if !changed {
            break;
        }
    }
}

/// P6-C：多机路径空间交叉检测（输出后处理）。对 path 非空的飞行器两两检测：
/// 段-段水平最近距离 < CROSS_H_KM 且垂直最近差 < CROSS_V_M → 视为交叉，
/// 两机 warnings 各追加一条 `multi_vehicle_conflict`（时间维 out-of-scope）。
pub(crate) fn detect_multi_aircraft_crossings(aircraft: &mut [AircraftOutput]) {
    for i in 0..aircraft.len() {
        for j in (i + 1)..aircraft.len() {
            let pi = aircraft[i].path.clone();
            let pj = aircraft[j].path.clone();
            if pi.len() < 2 || pj.len() < 2 {
                continue;
            }
            let mid_lat =
                pi.iter().chain(pj.iter()).map(|p| p.y).sum::<f64>() / (pi.len() + pj.len()) as f64;
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
                        _ => best = Some((h, v, lon, lat))
                    }
                }
            }
            if let Some((h, v, lon, lat)) = best {
                let msg = format!(
                    "multi_vehicle_conflict: crossing with {} near ({:.4},{:.4}) h={:.1}km v={:.0}m (time-dimension out-of-scope)",
                    aircraft[j].id, lon, lat, h, v
                );
                let msg2 = format!(
                    "multi_vehicle_conflict: crossing with {} near ({:.4},{:.4}) h={:.1}km v={:.0}m (time-dimension out-of-scope)",
                    aircraft[i].id, lon, lat, h, v
                );
                aircraft[i].warnings.push(msg);
                aircraft[j].warnings.push(msg2);
            }
        }
    }
}

/// 圆墙外扩机动余量（度）：绕圆墙需求 ≈ inflation（2km）+ 转弯半径（~0.5km）+
/// 净空 ≈ 0.03°（3.3km）。比多边形墙的 REGION_PAD_DEG 小——圆墙 bbox 大（半径
/// 数十公里），大余量会让本就接近任务边界的圆撑出 region（zz19 教训，见
/// expand_region_for_walls 注释）。
const REGION_CIRCLE_MARGIN_DEG: f64 = 0.03;

/// P7：环带目标集——在距目标 ∈ [rmin_km, rmax_km] 的网格 cell 中选 FMM 到达时间
/// 最小的可达 cell（等价"传播到环带即停"，docs/技术方案 §4.2）。环带超出 region
/// 的 cell 自然 clip（cell 索引在 grid 内）。环带内无可达 → None。
fn ring_target_cell(
    res: &crate::costfield::FmmResult,
    target: &Geo,
    region: &Region,
    grid: usize,
    rmin_km: f64,
    rmax_km: f64,
) -> Option<(usize, usize)> {
    let mut best: Option<(f32, usize, usize)> = None;
    for r in 0..grid {
        for c in 0..grid {
            let idx = r * grid + c;
            let t = res.times[idx];
            if !t.is_finite() {
                continue;
            }
            let (lon, lat) = cell_lonlat(r, c, region, grid);
            let d_km = crate::path::haversine_m(lon, lat, target.lon, target.lat) / 1000.0;
            if d_km >= rmin_km && d_km <= rmax_km && best.is_none_or(|(bt, _, _)| t < bt) {
                best = Some((t, r, c));
            }
        }
    }
    best.map(|(_, r, c)| (r, c))
}

/// P7：发射包线 heading 窗口判定。窗口语义 = 从 lo 顺时针扫到 hi 的角距
/// （`rem_euclid` 处理跨 0°，如 [350,10] = 20° 宽窗；[10,350] = 340° 宽窗）。
fn heading_in_window(h: f64, lo: f64, hi: f64) -> bool {
    let d = (h - lo).rem_euclid(360.0);
    let span = (hi - lo).rem_euclid(360.0);
    d <= span + 1e-6
}

/// P7：heading 窗口中心（顺时针中点，归一到 [0,360)）。供 Dubins 终端 pose 下放
/// （docs/技术方案 §4.2：终端姿态不只是到达判据，作为平滑级输入）。
fn heading_window_center(lo: f64, hi: f64) -> f64 {
    (lo + (hi - lo).rem_euclid(360.0) / 2.0).rem_euclid(360.0)
}

/// 任务区域：所有起点 + 每机目标（逐机显式，含原 mission.target 语义）的方形包围盒 + 缓冲
/// （保证源/目标不贴边）。
fn region_of(specs: &[AircraftSpec]) -> Region {
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
    let pad = REGION_PAD_DEG;
    min_lon -= pad;
    min_lat -= pad;
    let span = (max_lon - min_lon).max(max_lat - min_lat) + 2.0 * pad;
    Region {
        min_lon,
        min_lat,
        span_deg: span
    }
}

/// P3 分类结论出口（docs/12 §3.4/§12.4 拍板）：stderr 分类 JSON + stdout
/// stats.degradations 汇总（v0.21 起契约无 schema 版本字段）。
/// 类别（C2/C3 决策树）：geometrically_impossible（真无解）/ search_truncated
/// （可见图上限截断，≠几何无解）/ fitting_defect（拟合缺陷，可迭代）。
pub(crate) fn emit_classified(
    aircraft_id: &str,
    category: &str,
    detail: &str,
    degradations: &mut Vec<String>,
) {
    let json = serde_json::json!({
        "event": "classified",
        "aircraft": aircraft_id,
        "category": category,
        "detail": detail
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
        span_deg: span
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

/// 无解出口链③（docs/01 §5 回退层）：目标点 cell 不可达 → 在目标附近
/// `max_relax_km` 容差内搜索 T 最小（times 有限 = 可达）且距目标最近的 cell。
/// 返回 `(r, c, 距目标 km)`；容差内无可达 cell → None（几何无解 → no_solution）。
/// 语义：目标半径放宽是显式降级（调用方记 degradation），不是静默改目标。
fn relaxed_target_cell(
    res: &crate::costfield::FmmResult,
    region: &Region,
    grid: usize,
    tlon: f64,
    tlat: f64,
    max_relax_km: f64,
) -> Option<(usize, usize, f64)> {
    let (tr, tc) = lonlat_cell(tlon, tlat, region, grid);
    let cell_km = (region.span_deg * 111_320.0 / grid as f64 / 1000.0).max(1e-9);
    let radius = (max_relax_km / cell_km).ceil() as isize;
    let mut best: Option<(usize, usize, f32, f64)> = None; // (r, c, t, dist_km)
    for dr in -radius..=radius {
        for dc in -radius..=radius {
            let r = tr as isize + dr;
            let c = tc as isize + dc;
            if r < 0 || c < 0 || r >= grid as isize || c >= grid as isize {
                continue;
            }
            let (ru, cu) = (r as usize, c as usize);
            if ru == tr && cu == tc {
                continue; // 目标 cell 本身不可达（否则不会进入放宽分支）
            }
            let t = res.times[ru * grid + cu];
            if !t.is_finite() {
                continue;
            }
            let (lon, lat) = cell_lonlat(ru, cu, region, grid);
            let d_km = crate::path::haversine_m(tlon, tlat, lon, lat) / 1000.0;
            if d_km > max_relax_km + cell_km {
                continue;
            }
            // 确定性 tie-break：T 最小优先，同 T 距目标近优先
            if best.map_or(true, |b| t < b.2 || (t == b.2 && d_km < b.3)) {
                best = Some((ru, cu, t, d_km));
            }
        }
    }
    best.map(|(r, c, _, d)| (r, c, d))
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
                radius_m: radius_km * 1000.0
            }),
            ZoneShape::Polygon { .. } => None
        })
        .collect();
    CircleIndex::build(entries)
}

/// 雷达威胁参数（默认参数表落默认值；输入覆盖合并）。
/// base_p（探测概率基准）已标定（2026-08-13 方案 A：Swerling I 典型监视雷达模型，
/// R_eff 处 = 0.9），**与 p_cross 解耦**（主管 2026-08-05 反馈：
/// P_cross 是验收阈值——调高只放宽"容忍探测"的评估/拉直判定，不应把物理探测概率
/// 一起抬高导致代价爆炸 + 强绕行 + 锯齿；base_p 从 DefaultParams 默认表读取，不进外部覆盖）。
fn radar_threat_params(d: &crate::config::DefaultParams) -> ThreatParams {
    ThreatParams {
        radar_inflation: d.radar_inflation,
        detection_curve: d.detection_curve,
        p_cross: d.p_cross,
        suppression_delta: d.suppression_delta,
        base_p: d.base_p, // 标定值（Swerling I：R_eff 处 0.9）
    }
}

/// 无效参数回落默认的降级报告（主管决策 2026-08-05：无外部参数或参数无效使用默认值）。
/// merge 已回落；此处把"输入无效"事实记入 stats.degradations 供验收可见。
fn radar_param_degradations(input: &Input, out: &mut Vec<String>) {
    let p = &input.parameters;
    if let Some(v) = p.radar_inflation
        && !(v.is_finite() && v > 1.0)
    {
        out.push(format!(
            "parameter radar_inflation={v} invalid -> default 1.2"
        ));
    }
    if let Some(v) = p.p_cross
        && !(v.is_finite() && v >= 0.0 && v <= 1.0)
    {
        out.push(format!("parameter p_cross={v} invalid -> default 0.1"));
    }
    if let Some(v) = p.suppression_delta
        && !(v.is_finite() && v >= 0.0 && v < 1.0)
    {
        out.push(format!(
            "parameter suppression_delta={v} invalid -> default 0.5"
        ));
    }
    if let Some(v) = p.radar_cost_coef
        && !(v.is_finite() && v > 0.0)
    {
        out.push(format!(
            "parameter radar_cost_coef={v} invalid -> default 200"
        ));
    }
    if let Some(v) = p.los_mask_coef
        && !(v.is_finite() && v >= 0.0 && v <= 1.0)
    {
        out.push(format!(
            "parameter los_mask_coef={v} invalid -> default 0.08"
        ));
    }
    if let Some(s) = &p.detection_curve
        && !matches!(s.to_ascii_lowercase().as_str(), "exponential" | "linear")
    {
        out.push(format!(
            "parameter detection_curve={s} invalid -> default exponential"
        ));
    }
}

/// 拼接多段路径（去重相邻重复点；段端点保留——必经点/目标硬约束）。
fn join_paths(segs: &[Path]) -> Path {
    let mut pts: Vec<RouterPoint> = Vec::new();
    for seg in segs {
        for p in &seg.points {
            let dup = pts
                .last()
                .map(|q: &RouterPoint| {
                    (q.lon - p.lon).abs() < 1e-12 && (q.lat - p.lat).abs() < 1e-12
                })
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
                    lon1, lat1, lon2, lat2, center[0], center[1], *radius_km,
                ) {
                    for i in 0..=N {
                        let t = t1 + (t2 - t1) * i as f64 / N as f64;
                        let lon = lon1 + (lon2 - lon1) * t;
                        let lat = lat1 + (lat2 - lat1) * t;
                        let alt = alt1 + (alt2 - alt1) * t;
                        if let Ok(g) = Geo::new(lon, lat) {
                            if zone_contains_at(z, &g, alt) {
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
                        if zone_contains_at(z, &g, alt) {
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
                            if zone_contains_at(z, &g, alt) {
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
                        if zone_contains_at(z, &g, alt) {
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
            // 2026-08-13 base_p 标定（Swerling1）追加概率判据：纯几何 <0.7R 只挡深穿，
            // 而 Swerling1 下 0.7R~1.0R 区间探测概率仍 0.84~0.96（远高 P_cross=0.1）——
            // 拉直段弦切圈边缘（端点在外、弦切 0.785R）会被放行 → verify 累计 p≈1.0。
            // 判据 = 几何深穿 <0.7R **或** 静态并集概率 > P_cross（P_cross 调高容忍
            // 场景：Swerling1 圈内 p≥base_p=0.9 恒 >0.7，语义=进入圈内即高概率被探测，
            // 绕行是硬性要求；P_cross 仅作验收阈值不再放宽拉直）。
            // 端点判定同义扩展：端点已在"探测圈内"（p>P_cross，含必经点/目标落在圈内
            // 无法绕开，如 zigzag25 必经点 mid 距雷达 49.6km<60km 有效半径 p≈0.93）
            // → 进圈不可避免 → 允许拉直（拉直只简化，不引入新的绕行决策破坏；软约束）。
            let deep_a = tm.static_penetration(lon1, lat1, alt1) < DEEP_RATIO
                || tm.static_union_probability(lon1, lat1) > tm.p_cross();
            let deep_b = tm.static_penetration(lon2, lat2, alt2) < DEEP_RATIO
                || tm.static_union_probability(lon2, lat2) > tm.p_cross();
            if !deep_a && !deep_b {
                for i in 0..=N {
                    let t = i as f64 / N as f64;
                    let lon = lon1 + (lon2 - lon1) * t;
                    let lat = lat1 + (lat2 - lat1) * t;
                    let alt = alt1 + (alt2 - alt1) * t;
                    if tm.static_penetration(lon, lat, alt) < DEEP_RATIO
                        || tm.static_union_probability(lon, lat) > tm.p_cross()
                    {
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
    !z.is_wall()
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
    zones.iter().filter(|z| z.is_wall()).all(|z| {
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
    maximum_altitude_m: Option<f64>,
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
        let Some(min_alt) = z.alt_min_m else {
            return None;
        };
        let Some(max_alt) = z.alt_max_m else {
            return None;
        };
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
        let fit =
            |pass: f64| d_in * 1000.0 >= climb_dist(pass) && d_out * 1000.0 >= climb_dist(pass);
        // 顶部绕飞可行性：爬升距离 + 升限（alt_max + 500 ≤ ceiling）
        let top_ok = fit(top) && maximum_altitude_m.map_or(true, |c| top <= c);
        // 底部穿行可行性：爬升距离 + 底部严格低于 alt_min + 穿行带（直线穿多边形段）地形
        let bottom_ok = bottom >= 0.0
            && bottom < min_alt
            && fit(bottom)
            && bottom_terrain_ok(z, terrain, bottom, start, target, raw_band);
        return match (bottom_ok, top_ok) {
            (true, _) => Some(bottom), // 底部垂直机动总量更小 → 恒更优
            (false, true) => Some(top),
            (false, false) => None
        };
    };
    // 限飞区高度区间必须存在（validate 已强制 Restricted 提供 [alt_min, alt_max]）；
    // 缺失时按全高度处理（退化输入兜底，不 panic）。
    let Some(min_alt) = z.alt_min_m else {
        return None;
    };
    let Some(max_alt) = z.alt_max_m else {
        return None;
    };
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
    let top_ok = fit(top) && maximum_altitude_m.map_or(true, |c| top <= c);
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
        (false, false) => None
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
            seg_len_m +=
                crate::path::haversine_m(raw[k - 1].lon, raw[k - 1].lat, raw[k].lon, raw[k].lat);
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
            None => true,                   // 穿行段无陆地（水面/无数据）→ 直穿
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
                if u_out <= u_in {
                    Vec::new()
                } else {
                    vec![(u_in, u_out)]
                }
            }
        }
        ZoneShape::Polygon { vertices } => {
            line_polygon_inside_bands(start.lon, start.lat, target.lon, target.lat, vertices)
        }
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
        None => true,                   // 穿行段无陆地（水面/无数据）→ 直穿
    }
}

/// 该机飞行高度落在 restricted 高度区间内时，是否必须在 FMM 层画墙**水平绕行**：
/// 高度在区间外 → 不拦截直穿；区间内 → `restricted_pass_alt` 决策：底部穿行 / 顶部
/// 绕飞（均可行时底部更优，不画墙，FMM 直穿后由 `build_restricted_profiles` 生成
/// 对应剖面）；底部与顶部都不可行 → 画墙水平绕行（fallback 保底，不产生失败路径）。
fn restricted_detour_required(
    z: &Zone,
    alt_m: f64,
    maximum_altitude_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    max_climb_deg: f64,
) -> bool {
    if !restricted_blocks_alt(z, alt_m) {
        return false;
    }
    restricted_pass_alt(
        z,
        alt_m,
        maximum_altitude_m,
        terrain,
        start,
        target,
        max_climb_deg,
        None,
    )
    .is_none()
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
        if z.is_wall() {
            return false;
        }
        // 端点本身在带内（退化/零长度段覆盖；圆/多边形同口径 zone_contains）
        let in_band_at = |lon: f64, lat: f64, alt: f64| -> bool {
            match Geo::new(lon, lat) {
                Ok(g) => {
                    crate::config::zone_contains(z, &g)
                        && z.alt_min_m.is_some_and(|lo| alt >= lo)
                        && z.alt_max_m.is_some_and(|hi| alt <= hi)
                }
                Err(_) => false
            }
        };
        if in_band_at(lon1, lat1, alt1) || in_band_at(lon2, lat2, alt2) {
            return true;
        }
        let bands: Vec<(f64, f64)> = match &z.shape {
            crate::config::ZoneShape::Circle { center, radius_km } => {
                match segment_circle_intersect_t(
                    lon1, lat1, lon2, lat2, center[0], center[1], *radius_km,
                ) {
                    Some((t1, t2)) => vec![(t1, t2)],
                    None => Vec::new()
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
                if z.alt_min_m.is_some_and(|lo| alt >= lo)
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
    maximum_altitude_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    inflation_km: f64,
    degradations: &mut Vec<String>,
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
            let a_in =
                Geo::new(pa.lon, pa.lat).map_or(false, |g| crate::config::zone_contains(z, &g));
            let b_in =
                Geo::new(pb.lon, pb.lat).map_or(false, |g| crate::config::zone_contains(z, &g));
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
            let a_in =
                Geo::new(pa.lon, pa.lat).map_or(false, |g| crate::config::zone_contains(z, &g));
            let b_in =
                Geo::new(pb.lon, pb.lat).map_or(false, |g| crate::config::zone_contains(z, &g));
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
                maximum_altitude_m,
                terrain,
                &p0g,
                &p1g,
                max_climb_deg,
                None,
            ) else {
                continue;
            };
            if line_hits_wall_km(p0.lon, p0.lat, p1.lon, p1.lat, zones, inflation_km) {
                // P8 M6（已知限制 #12）：fallback 直线剖面不可用（直线穿硬墙）→
                // **显式降级标注**（消除静默），保持 raw 浅穿交付（宁丑勿违）。
                // 不触发画墙绕行：need_wall 会全局画 restricted 墙，主管真实输入
                // zz33/zz34 中 rz 圆顶与 no_fly 三角顶点同高，膨胀后走廊闭合 → 无解
                // （比锯齿交付更坏）。取舍：显式标注可观测，路径行为不变（零回归）。
                degradations.push(format!(
                    "restricted fallback straight profile blocked by wall (zone={}); raw shallow-cross delivered",
                    z.id
                ));
                continue;
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
            maximum_altitude_m,
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
            if dist_km(tail.points[i].lon, tail.points[i].lat, pin2.lon, pin2.lat) >= climb_base_km
            {
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
            if dist_km(pout2.lon, pout2.lat, tail.points[i].lon, tail.points[i].lat)
                >= climb_base_km
            {
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
/// 距圆墙圆心 < radius+inflation+margin → 沿径向（远离圆心）外移到该距离；
/// 距多边形墙边界 < inflation+margin → 沿"点↔最近边界点"方向外移到该距离
/// （点在内部 → 向最近边界点外侧推；2026-08-13 P9 T3 补多边形，原仅 Circle）。
/// 用于 in→out 平飞段（FMM 贴墙绕行 clearance≈inflation，平滑内切后不足 verify
/// 阈值；margin 吸收平滑内切与 FMM 网格离散误差）。外层迭代：多边形外扩可能
/// 引入其他墙违反（凹多边形/墙重叠），至多 8 轮收敛；Circle 分支单遍幂等，
/// 第二轮起无变化 → 与既有行为一致。
fn push_out_of_walls(
    pts: &[RouterPoint],
    zones: &[Zone],
    inflation_km: f64,
    margin_km: f64,
) -> Vec<RouterPoint> {
    let mut out: Vec<RouterPoint> = pts.to_vec();
    let target = inflation_km + margin_km;
    for _round in 0..8 {
        let mut changed = false;
        for p in out.iter_mut() {
            let (mut lon, mut lat) = (p.lon, p.lat);
            let mut moved = false;
            for z in zones {
                if !z.is_wall() {
                    continue;
                }
                match &z.shape {
                    ZoneShape::Circle { center, radius_km } => {
                        let (cx, cy) = (center[0], center[1]);
                        let d = dist_km(lon, lat, cx, cy);
                        let ctarget = radius_km + target;
                        if d < ctarget {
                            if d < 1e-6 {
                                // 极端：点与圆心重合 → 沿经度方向外移
                                lon = cx + ctarget / 111.32;
                                changed = true;
                                moved = true;
                                continue;
                            }
                            let f = ctarget / d;
                            lon = cx + (lon - cx) * f;
                            lat = cy + (lat - cy) * f;
                            changed = true;
                            moved = true;
                        }
                    }
                    ZoneShape::Polygon { vertices } => {
                        if vertices.len() < 3 {
                            continue;
                        }
                        let (d, qx, qy, inside) = poly_nearest_edge_km(lon, lat, vertices);
                        if d < target {
                            let (dx, dy) = if inside {
                                (qx - lon, qy - lat) // 内部：向最近边界点外侧推
                            } else {
                                (lon - qx, lat - qy) // 外部：远离最近边界点
                            };
                            let dl = (dx * dx + dy * dy).sqrt();
                            let lat0 = lat.to_radians();
                            let kx = 111.320 * lat0.cos();
                            let ky = 111.0;
                            if dl < 1e-9 {
                                // 极端：点恰在边界上 → 沿"点→形心"反向（远离形心）外移
                                let (cx, cy) = poly_centroid(vertices);
                                let (dx2, dy2) = (lon - cx, lat - cy);
                                let dl2 = (dx2 * dx2 + dy2 * dy2).sqrt();
                                let (ux, uy) = if dl2 > 1e-9 {
                                    (dx2 / dl2, dy2 / dl2)
                                } else {
                                    (1.0, 0.0)
                                };
                                lon += ux * target / kx;
                                lat += uy * target / ky;
                            } else {
                                lon = qx + dx / dl * target / kx;
                                lat = qy + dy / dl * target / ky;
                            }
                            changed = true;
                            moved = true;
                        }
                    }
                }
            }
            if moved {
                *p = RouterPoint::new(lon, lat, p.alt_m);
            }
        }
        if !changed {
            break;
        }
    }
    out
}

/// 点到多边形边界最近距离（km，平面近似）与最近点（度）与内部判定。
/// 遍历各边取最近；点与边界重合（d≈0）时最近点 ≈ 点本身。
fn poly_nearest_edge_km(lon: f64, lat: f64, vertices: &[[f64; 2]]) -> (f64, f64, f64, bool) {
    let lat0 = lat.to_radians();
    let kx = 111.320 * lat0.cos();
    let ky = 111.0;
    let (px, py) = (lon * kx, lat * ky);
    let mut best = f64::MAX;
    let (mut qx, mut qy) = (lon, lat);
    for i in 0..vertices.len() {
        let a = vertices[i];
        let b = vertices[(i + 1) % vertices.len()];
        let (ax, ay) = (a[0] * kx, a[1] * ky);
        let (bx, by) = (b[0] * kx, b[1] * ky);
        let (vx, vy) = (bx - ax, by - ay);
        let (wx, wy) = (px - ax, py - ay);
        let l2 = vx * vx + vy * vy;
        let t = if l2 > 0.0 {
            ((wx * vx + wy * vy) / l2).clamp(0.0, 1.0)
        } else {
            0.0
        };
        let (qqx, qqy) = (ax + t * vx, ay + t * vy);
        let d = ((px - qqx).powi(2) + (py - qqy).powi(2)).sqrt();
        if d < best {
            best = d;
            qx = qqx / kx;
            qy = qqy / ky;
        }
    }
    let inside = crate::config::point_in_polygon_xy(lon, lat, vertices);
    (best, qx, qy, inside)
}

/// 多边形顶点平均（形心近似；仅用于边界退化兜底方向）。
fn poly_centroid(vertices: &[[f64; 2]]) -> (f64, f64) {
    let n = vertices.len() as f64;
    let sx = vertices.iter().map(|v| v[0]).sum::<f64>() / n;
    let sy = vertices.iter().map(|v| v[1]).sum::<f64>() / n;
    (sx, sy)
}

/// 语义代价场构建：5. 语义采样（Land=1/Water=1/Lake=1/NoData=5/OOB=5/Forbidden=INF）
/// + 5c 墙膨胀 + 过渡带软罚 + 5b 雷达静态代价（+ P8 M5 LOS mask）。
/// 提取为函数：无解出口链④（docs/01 §5）网格细分重试复用（同一套墙/膨胀/雷达
/// 语义，grid 翻倍重算）。
///
/// P8 M5 LOS mask（docs/06 §6）：`p_det = f(r)·mask_LOS`——有地形源时探测概率用
/// 带 LOS 的并集（`point_probability(lon, lat, LOS_REF_ALT_M, terrain)`），被地形
/// 遮挡（含 NoData 保守）→ 该雷达不探测 → p_los→0 → 无代价惩罚 → FMM 倾向走
/// 地形遮蔽区接近/绕过雷达。参考高度常量（多机共享静态近似，verify 威胁评估仍
/// 精确）；无地形（TerrainHandle::None）→ 无遮蔽（docs/06 §6"无地形文件 → 无遮蔽"，
/// 零回归）。
fn build_cost_field(
    region: &Region,
    grid: usize,
    terrain: &TerrainHandle,
    all_zones: &[Zone],
    inflation_m: f64,
    threat: &SphericalRadarThreat,
    radar_cost_coef: f64,
    los_mask_coef: f64,
    has_radars: bool,
) -> crate::costfield::CostField {
    // 硬墙判定闭包（每格：墙内 → Forbidden 禁行）——par_local 与串行回退共用。
    // 多边形墙用"格子矩形与多边形相交"（2026-08-11 zz_nosolution_case：中心点
    // 采样漏掉尖角/斜边 < 1 格窄带 → FMM 贴顶点穿入 → verify 精确几何拒 →
    // 误报 no_solution；矩形相交保证任何分辨率不漏窄带）；圆形连续无窄带问题，
    // 保持中心点语义（与 zone_contains 解析一致）。
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
    let cell_deg = region.span_deg / grid as f64;
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
        // 外部格式（GeoTIFF/DTED/SRTM）无 BulkPrefetch → 带锁采样回退。
        // P9 T4：传 cell 分辨率给 sample_at_res —— GeoTIFF 大文件带 Overview 时
        // FMM 粗层采样走低分辨率层（省高分辨率解压），verify 精查仍走主层。
        TerrainHandle::External(t) => build_semantic_cost_field(
            grid,
            grid,
            |r, c| {
                let (lon, lat) = cell_lonlat(r, c, region, grid);
                if walled(lon, lat) {
                    return Sample::Forbidden;
                }
                t.sample_at_res(lon, lat, cell_deg)
            },
            5.0,
        ),
        TerrainHandle::MaskedExternal(t) => build_semantic_cost_field(
            grid,
            grid,
            |r, c| {
                let (lon, lat) = cell_lonlat(r, c, region, grid);
                if walled(lon, lat) {
                    return Sample::Forbidden;
                }
                t.sample_at_res(lon, lat, cell_deg)
            },
            5.0,
        ),
        TerrainHandle::None => build_semantic_cost_field(
            grid,
            grid,
            |r, c| {
                let (lon, lat) = cell_lonlat(r, c, region, grid);
                if walled(lon, lat) {
                    return Sample::Forbidden;
                }
                Sample::Land(0.0)
            },
            5.0,
        )
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
    //     ——FMM 倾向绕行；coef = radar_cost_coef（默认 200）。几何深穿惩罚（u<1 时
    //     ×(1+coef·(1-u))，探测区外 u≥1 无几何项）：确保穿探测区明确绕行——主管
    //     2026-08-06：并排双雷达不得直穿探测区（即使 P_cross 调高，几何绕行与验收
    //     阈值解耦）。
    //     P8 M5 LOS mask（docs/06 §6）：有地形源时 p 用带 LOS 的并集概率
    //     （point_probability(lon, lat, LOS_REF_ALT_M, terrain)）；被地形遮挡的 cell
    //     p_los→0 → 无代价惩罚 → FMM 倾向走遮蔽区；无地形 → 无 LOS（零回归）。
    //     参考高度常量 LOS_REF_ALT_M：代价场多机共享静态近似（verify 仍精确）。
    let terrain_src = terrain.as_source();
    if has_radars {
        for r in 0..grid {
            for c in 0..grid {
                let (lon, lat) = cell_lonlat(r, c, region, grid);
                let p = if los_mask_coef > 0.0 && terrain_src.is_some() {
                    threat.point_probability(lon, lat, LOS_REF_ALT_M, terrain_src)
                } else {
                    threat.static_union_probability(lon, lat)
                };
                if p > 0.0 {
                    let idx = r * grid + c;
                    if field.cost[idx].is_finite() {
                        let u = threat.static_penetration(lon, lat, 0.0);
                        let geom = if u < 1.0 { 1.0 - u } else { 0.0 };
                        field.cost[idx] *= (1.0 + radar_cost_coef * (p + geom)) as f32;
                    }
                }
            }
        }
    }
    field
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

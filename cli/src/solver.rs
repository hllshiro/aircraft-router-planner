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
use crate::costfield::{backtrack_path, build_semantic_cost_field, fmm_propagate};
use crate::error::{AppError, InputInvalidReason};
use crate::path::{Path, PathPoint as RouterPoint};
use crate::smooth::{default_chain, smooth_path_chain, VerifyContext};
use crate::spatial::{CircleEntry, CircleIndex};
use crate::terrain::builtin::BuiltinSource;
use crate::terrain::{Sample, TerrainSource};
use crate::threat::{SphericalRadarThreat, ThreatModel, ThreatParams};

/// 解算参数（M1：地形路径 CLI/输入指定；grid 粗网格分辨率）。
#[derive(Debug, Clone)]
pub struct SolveParams {
    pub terrain_path: Option<PathBuf>,
    pub grid: usize,
}

impl Default for SolveParams {
    fn default() -> Self {
        Self {
            terrain_path: None,
            grid: 256,
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
    alt_m: f64,
    /// 机型配置（Phase 4 M4：平滑参数派生输入）。
    profile: crate::config::VehicleProfile,
    /// 中途必经点（Phase 4 M5：start → mid[0..] → target 分段拼接）。
    mid_waypoints: Vec<Geo>,
}

/// 端到端解算。elapsed_ms 为端到端耗时（main 计时传入）。
pub fn solve(input: &Input, params: &SolveParams, elapsed_ms: u64) -> Result<Output, AppError> {
    // 1. 地形源（none = 无地形平面；path/builtin = ARPK1 文件）
    let terrain: Option<Box<dyn TerrainSource>> = match input.mission.terrain.source {
        TerrainSourceType::None => None,
        _ => {
            let p = params
                .terrain_path
                .clone()
                .or_else(|| input.mission.terrain.path.clone().map(PathBuf::from));
            match p {
                Some(p) => Some(Box::new(BuiltinSource::open(&p)?)),
                None => {
                    // 主管决策 2026-08-05：默认低精度地形 = 量化中国数据（china_dem_l12.arpack）。
                    // 候选：exe 同目录 / exe 上溯 workspace 根 / 工作目录相对路径。
                    let mut candidates = vec![
                        PathBuf::from("china_dem_l12.arpack"),
                        PathBuf::from("phase0/data/pending/china_dem_l12.arpack"),
                        PathBuf::from("../phase0/data/pending/china_dem_l12.arpack"),
                    ];
                    if let Ok(exe) = std::env::current_exe() {
                        if let Some(dir) = exe.parent() {
                            candidates.insert(0, dir.join("china_dem_l12.arpack"));
                            // 上溯 3 层（target/release → target → workspace 根），逐层试开发路径
                            for anc in dir.ancestors().skip(1).take(3) {
                                candidates.push(anc.join("phase0/data/pending/china_dem_l12.arpack"));
                            }
                        }
                    }
                    if let Some(c) = candidates.iter().find(|c| c.exists()) {
                        Some(Box::new(BuiltinSource::open(c)?))
                    } else {
                        return Err(AppError::Data(
                            "terrain.source=path/builtin 但未提供地形文件，且默认低精度地形 \
                             (china_dem_l12.arpack) 未找到（--terrain / terrain.path / exe 同目录 / phase0/data/pending）"
                                .into(),
                        ));
                    }
                }
            }
        }
    };

    // 2. 车辆规格（vehicles 空 → 默认单机：mission.start → mission.target）
    let specs: Vec<VehicleSpec> = if input.mission.vehicles.is_empty() {
        vec![VehicleSpec {
            id: "v1".into(),
            start: input.mission.start.to_geo()?,
            alt_m: input.mission.start.alt_m,
            profile: crate::config::VehicleProfile::default(),
            mid_waypoints: Vec::new(),
        }]
    } else {
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
    //     物理转弯半径 r = v²/(g·tanφ)（与 smooth_options_for 同式）；绕行弧需要 ≥r 的
    //     转弯空间，把 NoFly/Obstacle 硬墙向外膨胀 max(0.5×r)（clamp [2km, 10km]）——
    //     FMM 绕行自然远离边界，Dubins 转弯弧留足空间（不再因贴边急弯被物理复验拒绝）。
    let params_merged = crate::config::DefaultParams::default().merge(&input.mission.parameters);
    let mut degradations = Vec::new();
    radar_param_degradations(input, &mut degradations);
    let inflation_m = specs
        .iter()
        .map(|v| {
            let (_opts, phys) = crate::smooth::smooth_options_for(&v.profile, &params_merged);
            (phys * 0.5).clamp(2_000.0, 10_000.0)
        })
        .fold(0.0f64, f64::max);

    // 5. 语义代价场（Land=1 / Water=1 / Lake=1 / NoData=5 / OOB=INF；NoFly/Obstacle 墙）
    let grid = params.grid.max(8);
    let mut field = build_semantic_cost_field(grid, grid, |r, c| {
        let (lon, lat) = cell_lonlat(r, c, &region, grid);
        if let Ok(g) = Geo::new(lon, lat) {
            if all_zones
                .iter()
                .any(|z| z.is_wall() && zone_contains(z, &g))
            {
                return Sample::OutOfBounds;
            }
        }
        match &terrain {
            Some(t) => t.sample_at(lon, lat),
            None => Sample::Land(0.0),
        }
    }, 5.0);

    // 5c. 禁飞区墙向外膨胀 + 过渡带软罚（见 apply_inflation_and_band）
    let cell_m = region.span_deg * 111_320.0 / grid as f64;
    let inflation_cells = (inflation_m / cell_m.max(1.0)).ceil() as usize;
    apply_inflation_and_band(&mut field, inflation_cells);

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
    for v in &specs {
        // 段序列：起点 + 必经点 + 目标
        let mut seg_ends: Vec<Geo> = Vec::with_capacity(v.mid_waypoints.len() + 2);
        seg_ends.push(v.start);
        seg_ends.extend(v.mid_waypoints.iter().copied());
        seg_ends.push(target);
        // 机型平滑参数提前（受限区剖面需要 max_climb：决定下降/爬升距离）
        let (opts, phys_min_radius_m) = crate::smooth::smooth_options_for(&v.profile, &params_merged);
        // 该机受限区墙（剖面直穿语义，主管 2026-08-06 二轮）：飞行高度落在 restricted
        // 高度区间内 → 比较底部穿行 / 顶部绕飞代价（底部可行恒更优，否则顶部）→ 可行
        // 则不画墙，FMM 直穿后由 build_restricted_profiles 生成对应剖面；两者都不可行
        // （地形过高且超升限 / 太贴边 / 多边形）→ 画墙水平绕行（fallback 保底）；
        // 高度在区间外（如低于 alt_min_m 的"底部通道"）→ 不画墙直穿（可通行）。
        let veh_field: Option<crate::costfield::CostField> = if all_zones.iter().any(|z| {
            restricted_detour_required(
                z,
                v.alt_m,
                v.profile.ceiling_m,
                terrain.as_deref(),
                &v.start,
                &target,
                opts.max_climb_deg,
            )
        }) {
            let mut f = field.clone();
            let g = f.rows;
            for r in 0..g {
                for c in 0..g {
                    let (lon, lat) = cell_lonlat(r, c, &region, g);
                    if let Ok(gg) = Geo::new(lon, lat) {
                        if all_zones.iter().any(|z| {
                            restricted_detour_required(
                                z,
                                v.alt_m,
                                v.profile.ceiling_m,
                                terrain.as_deref(),
                                &v.start,
                                &target,
                                opts.max_climb_deg,
                            ) && zone_contains(z, &gg)
                        }) {
                            f.cost[r * g + c] = f32::INFINITY;
                        }
                    }
                }
            }
            apply_inflation_and_band(&mut f, inflation_cells);
            Some(f)
        } else {
            None
        };
        let field_ref = veh_field.as_ref().unwrap_or(&field);
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
                        RouterPoint::new(lon, lat, v.alt_m)
                    })
                    .collect(),
            ));
        }
        if no_solution || raw_segs.is_empty() {
            out_vehicles.push(VehicleOutput {
                id: v.id.clone(),
                status: "no_solution".into(),
                path: Vec::new(),
                distance_m: 0.0,
                warnings: vec!["coarse FMM no path".into()],
            });
            continue;
        }
        // 段端点（必经点/目标）是硬约束：任何平滑不得移除
        let raw_joined = join_paths(&raw_segs);

        // 平滑链（≥2 点；VerifyContext 接地形净空 + Zone 高度层）
        // Phase 4 M4：SmoothOptions + A6 物理下限由机型配置派生
        // Phase 4 M5：分段平滑（段端点=必经点保留）→ 拼接 → 全路径终检复验
        // 受限区底部剖面：每段直穿的 circle restricted → 降高剖面切分
        // （剖面段跳过平滑链，防止抽稀破坏 15° 斜率）
        let mut smooth_src: Vec<Path> = Vec::new();
        let mut profile_mask: Vec<bool> = Vec::new();
        for seg in &raw_segs {
            let (sub, mask) = build_restricted_profiles(
                seg,
                &all_zones,
                v.alt_m,
                opts.max_climb_deg,
                v.profile.ceiling_m,
                terrain.as_deref(),
                &v.start,
                &target,
            );
            smooth_src.extend(sub);
            profile_mask.extend(mask);
        }
        let mut warnings = Vec::new();
        let mut pts = raw_joined.points.clone();
        if pts.len() >= 2 {
            let inflation_km = inflation_m / 1000.0;
            let check = make_segment_check(
                &all_zones,
                Some(&threat as &dyn crate::threat::ThreatModel),
                inflation_km,
            );
            let chain = default_chain(&opts, &check);
            let ctx = VerifyContext {
                terrain: terrain.as_deref(),
                nofly: Some(&nofly),
                zones: Some(&all_zones),
                threat: Some(&threat),
                zone_inflation_m: inflation_m,
            };
            // 每段独立平滑（首尾段端点保留——Theta* 截直不得移除必经点）
            let mut smooth_segs = Vec::new();
            let mut seg_warnings = Vec::new();
            for (idx, seg) in smooth_src.iter().enumerate() {
                if profile_mask[idx] {
                    // 受限区剖面段：已按 max_climb 生成下降/平飞/爬升，直接采用
                    smooth_segs.push(seg.clone());
                    continue;
                }
                let result = smooth_path_chain(seg, &chain, &opts, &ctx, Some(phys_min_radius_m));
                if let Some(w) = &result.warning {
                    seg_warnings.push(w.clone());
                }
                seg_warnings.extend(result.verify.warnings.iter().cloned());
                smooth_segs.push(result.path);
            }
            // 拼接 + 全路径终检（段间转角/整路径威胁在拼接后才可见）
            let joined = join_paths(&smooth_segs);
            let final_rep = crate::smooth::verify_path(
                &joined,
                None,
                &opts,
                &ctx,
                Some(phys_min_radius_m),
            );
            if final_rep.ok {
                pts = joined.points;
                warnings = seg_warnings.clone();
            } else {
                // 终检失败 → 回退未平滑拼接（必经点保留，宁丑勿违）
                pts = raw_joined.points;
                let msg = "smoothing_failed: no smoothed stage passed full verification";
                warnings.push(msg.into());
                degradations.push(msg.into());
                warnings.extend(final_rep.warnings.iter().cloned());
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
                    let mut straight_pts: Vec<crate::path::PathPoint> = Vec::new();
                    for seg in &raw_segs {
                        if let (Some(f), Some(l)) = (seg.points.first(), seg.points.last()) {
                            if straight_pts.is_empty() || straight_pts.last().map_or(true, |p| *p != *f) {
                                straight_pts.push(*f);
                            }
                            straight_pts.push(*l);
                        }
                    }
                    let straight = Path::new(straight_pts);
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
                            pts = straight.points;
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

/// 任务区域：所有起点 + target 的方形包围盒 + 0.15° 缓冲（保证源/目标不贴边）。
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
/// 而把绕行弧拉直成穿雷达区的直线；主管 2026-08-06：航路必须绕开雷达探测区域）；
/// 低概率边缘（≥0.7，即有效半径外）允许拉直 → 绕行路径可平滑。
/// 线段合法性检查（Theta* 去锯齿拉直用）。
/// Zone 水平判定：NoFly/Obstacle 全高度墙——段到 Zone 水平净距 < inflation_km 即拒绝
/// （主管 2026-08-06：绕飞太贴边→考虑飞机机动；膨胀距离按物理转弯半径 v²/(g·tanφ)
/// 的 0.5 倍（clamp [2,10]km），拉直不得贴进膨胀带，FMM 绕行留转弯空间）；
/// Restricted 保持"水平相交 + 段高度采样"（M2 高度层语义，不膨胀）。
/// 雷达威胁：直连"深穿"任一雷达（归一化深度 < 0.7，即深入有效半径 70% 以内）
/// → 拒绝拉直（保住 FMM 绕行决策——P_cross 只是验收阈值，不得因调高 P_cross
/// 而把绕行弧拉直成穿雷达区的直线；主管 2026-08-06：航路必须绕开雷达探测区域）；
/// 低概率边缘（≥0.7，即有效半径外）允许拉直 → 绕行路径可平滑。
fn make_segment_check<'a>(
    zones: &'a [Zone],
    threat: Option<&'a dyn crate::threat::ThreatModel>,
    inflation_km: f64,
) -> impl Fn(f64, f64, f64, f64, f64, f64) -> bool + 'a {
    move |lon1, lat1, alt1, lon2, lat2, alt2| {
        const N: usize = 16;
        const DEEP_RATIO: f64 = 0.7;
        for z in zones {
            let clr = crate::config::zone_segment_clearance_km(lon1, lat1, lon2, lat2, z);
            if z.is_wall() {
                if clr <= 1e-9 || clr < inflation_km {
                    return false;
                }
            } else if clr <= 1e-9 {
                // restricted：高度层采样（水平相交后，高度沿线段插值判定）
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
) -> Option<f64> {
    let ZoneShape::Circle { center, radius_km } = z.shape else {
        return None;
    };
    let bottom = (z.alt_min_m - 500.0).max(0.0);
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
    // 底部穿行可行性：爬升距离 + 穿行区地形 ≤ 底部 − 净空
    let bottom_ok = fit(bottom) && bottom_terrain_ok(z, terrain, bottom);
    match (bottom_ok, top_ok) {
        (true, _) => Some(bottom), // 底部垂直机动总量更小 → 恒更优（显式代价比较结论）
        (false, true) => Some(top), // 底部不可行 → 顶部绕飞（优于水平绕行：水平距离不增加）
        (false, false) => None,
    }
}

/// 底部通道地形可行性：穿行区（圆内）地形最高点 + 净空(100m) ≤ 底部高度。
/// 无地形（平面 0m）→ 恒可行。顶部绕飞不查地形（高于任何地形）。
fn bottom_terrain_ok(z: &Zone, terrain: Option<&dyn TerrainSource>, bottom: f64) -> bool {
    let Some(t) = terrain else {
        return true;
    };
    let ZoneShape::Circle { center, radius_km } = z.shape else {
        return false;
    };
    let (clon, clat) = (center[0], center[1]);
    let km_per_deg_lat = 111.32;
    let km_per_deg_lon = 111.32 * clat.to_radians().cos().max(1e-6);
    let span_lat = radius_km / km_per_deg_lat;
    let span_lon = radius_km / km_per_deg_lon;
    let step = 0.02; // 度 ≈ 2.2km 采样步长（穿行区地形粗判）
    let mut max_terr: Option<f64> = None;
    let mut lat = clat - span_lat;
    while lat <= clat + span_lat {
        let mut lon = clon - span_lon;
        while lon <= clon + span_lon {
            if dist_km(clon, clat, lon, lat) <= radius_km {
                if let Sample::Land(h) = t.sample_at(lon, lat) {
                    max_terr = Some(max_terr.map_or(h, |m: f64| m.max(h)));
                }
            }
            lon += step;
        }
        lat += step;
    }
    match max_terr {
        Some(h) => h + 100.0 <= bottom, // 净空满足 → 底部可行
        None => true,                    // 圆内无陆地（水面/无数据）→ 直穿
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
    restricted_pass_alt(z, alt_m, ceiling_m, terrain, start, target, max_climb_deg).is_none()
}

/// 单个 restricted 的剖面参数（沿直线距离 s 定义：过渡/平飞/过渡三段）。
/// pass_alt = 穿行高度：底部 = alt_min−500m（下降穿行）/ 顶部 = alt_max+500m（爬升绕飞）。
struct RestrictedProfile {
    s_desc: f64,
    s_in: f64,
    s_out: f64,
    s_climb: f64,
    pass_alt: f64,
}

/// 剖面高度函数：沿直线距离 ss → 高度（米）。
/// - ss < s_desc：巡航高度 alt_m；
/// - s_desc → s_in：线性过渡至 pass_alt（底部 = 下降 / 顶部 = 爬升，方向由差值自动决定）；
/// - s_in → s_out：pass_alt 平飞（穿 restricted 区间：底部 <alt_min 可通行 / 顶部 >alt_max 可通行）；
/// - s_out → s_climb：线性过渡回 alt_m；
/// - ss > s_climb：巡航高度。
fn profile_alt_at(pr: &RestrictedProfile, ss: f64, alt_m: f64) -> f64 {
    if ss < pr.s_desc {
        alt_m
    } else if ss < pr.s_in {
        let f = (ss - pr.s_desc) / (pr.s_in - pr.s_desc).max(1e-9);
        alt_m - f * (alt_m - pr.pass_alt)
    } else if ss <= pr.s_out {
        pr.pass_alt
    } else if ss < pr.s_climb {
        let f = (ss - pr.s_out) / (pr.s_climb - pr.s_out).max(1e-9);
        pr.pass_alt + f * (alt_m - pr.pass_alt)
    } else {
        alt_m
    }
}

/// 受限区穿行剖面（主管 2026-08-06 二轮：底部穿行 vs 顶部绕飞，比较代价选更优）。
/// FMM 直穿圆形 restricted 后，把穿行段切成 过渡(≤max_climb) → pass_alt 平飞 →
/// 过渡回巡航 高度剖面；剖面段压缩为锚点（desc/in/out/climb）并**跳过平滑链**
/// （抽稀会破坏斜率，而锚点间本身是 max_climb 直线，无需再平滑）。穿行高度由
/// `restricted_pass_alt` 决策：底部可行恒选底部（垂直机动更少代价更小）；底部被
/// 地形挡住 → 顶部绕飞（alt_max+500m）；都不可行 → 该 restricted 已在 FMM 前画墙
/// 绕行（`restricted_detour_required`），raw 不穿它，此处自动跳过。
///
/// 锚点全部用 **start→target 直线参数化**（解析圆交点、直线插值坐标）——不能沿用
/// FMM 锯齿路径的沿路径距离（锯齿使路径总长 264km vs 直线 187km，距离严重不成比例），
/// 且锯齿路径点会让剖面锚点不共线（desc/in/out 折角 → 固定翼转弯半径检查拒绝；
/// desc→in 直线距离缩短 → 爬升角超 max_climb）。
/// 返回：切分后的段（[首段, 剖面段, 尾段]）+ 剖面段掩码（true = 跳过平滑直接采用）。
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
) -> (Vec<Path>, Vec<bool>) {
    let n = seg.points.len();
    if n < 2 {
        return (vec![seg.clone()], vec![false]);
    }
    // 该机高度拦截的圆形 restricted（底部/顶部剖面穿行类型）
    let hits: Vec<&Zone> = zones
        .iter()
        .filter(|z| matches!(z.shape, ZoneShape::Circle { .. }) && restricted_blocks_alt(z, alt_m))
        .collect();
    if hits.is_empty() {
        return (vec![seg.clone()], vec![false]);
    }
    let (p0, p1) = (seg.points[0], seg.points[n - 1]);
    // 平面 km 坐标系（以 p0 为原点，度制近似）
    let mlat = ((p0.lat + p1.lat) / 2.0).to_radians();
    let kx = mlat.cos() * 111.32;
    let ky = 111.32;
    let dx = (p1.lon - p0.lon) * kx;
    let dy = (p1.lat - p0.lat) * ky;
    let l_line_km = (dx * dx + dy * dy).sqrt();
    if l_line_km < 0.1 {
        return (vec![seg.clone()], vec![false]);
    }
    let line_m = l_line_km * 1000.0;
    // 每个穿行 restricted：穿行高度决策（底部优先/顶部备选）→ 直线与圆交点 → 剖面参数
    let mut profiles: Vec<RestrictedProfile> = Vec::new();
    for z in hits {
        let Some(pass_alt) =
            restricted_pass_alt(z, alt_m, ceiling_m, terrain, start, target, max_climb_deg)
        else {
            continue; // 底部/顶部都不可行 → 已画墙绕行（raw 不穿它）
        };
        let ZoneShape::Circle { center, radius_km } = z.shape else { continue };
        let cx = (center[0] - p0.lon) * kx;
        let cy = (center[1] - p0.lat) * ky;
        let a = dx * dx + dy * dy;
        let b = -2.0 * (dx * cx + dy * cy);
        let c = cx * cx + cy * cy - radius_km * radius_km;
        let disc = b * b - 4.0 * a * c;
        if disc <= 0.0 {
            continue; // 直线不穿圆
        }
        let sq = disc.sqrt();
        let u1 = (-b - sq) / (2.0 * a);
        let u2 = (-b + sq) / (2.0 * a);
        let (u_in, u_out) = (u1.min(u2), u1.max(u2));
        if u_out <= 0.0 || u_in >= 1.0 {
            continue; // 圆在线段端点之外，FMM 直穿不经过该圆
        }
        // ×1.25 裕量（与 restricted_pass_alt 一致）：直线插值仍有微小近似误差，
        // 裕量保证 desc→in / out→climb 直线爬升角严格 ≤ max_climb。
        let climb_dist_m = if max_climb_deg > 0.1 {
            ((alt_m - pass_alt).abs()) / max_climb_deg.to_radians().tan() * 1.25
        } else {
            f64::INFINITY
        };
        let u_desc = (u_in - climb_dist_m / line_m).max(0.0);
        let u_climb = (u_out + climb_dist_m / line_m).min(1.0);
        profiles.push(RestrictedProfile {
            s_desc: u_desc * line_m,
            s_in: u_in * line_m,
            s_out: u_out * line_m,
            s_climb: u_climb * line_m,
            pass_alt,
        });
    }
    if profiles.is_empty() {
        return (vec![seg.clone()], vec![false]);
    }
    // 锚点（直线插值坐标，全部共线；kind: 0=desc 1=in 2=out 3=climb）
    let mut us: Vec<(f64, f64, f64, u8)> = Vec::new(); // (u, lon, lat, kind)
    for pr in &profiles {
        for (su, kind) in [
            (pr.s_desc / line_m, 0u8),
            (pr.s_in / line_m, 1),
            (pr.s_out / line_m, 2),
            (pr.s_climb / line_m, 3),
        ] {
            let lon = p0.lon + (p1.lon - p0.lon) * su;
            let lat = p0.lat + (p1.lat - p0.lat) * su;
            us.push((su, lon, lat, kind));
        }
    }
    us.sort_by(|x, y| x.0.partial_cmp(&y.0).unwrap());
    us.dedup_by(|x, y| (x.0 - y.0).abs() < 1e-6);
    // 剖面区间：所有 restricted 的 desc 最小值 ~ climb 最大值
    let u_desc_min = profiles.iter().map(|p| p.s_desc / line_m).fold(f64::MAX, f64::min);
    let u_climb_max = profiles.iter().map(|p| p.s_climb / line_m).fold(0.0, f64::max);
    let pt_at = |u: f64| -> RouterPoint {
        RouterPoint::new(
            p0.lon + (p1.lon - p0.lon) * u,
            p0.lat + (p1.lat - p0.lat) * u,
            alt_m,
        )
    };
    // 首段：start → desc_min（平飞 3000m，正常平滑拉直）
    let seg0 = Path::new(vec![p0, pt_at(u_desc_min)]);
    // 剖面段：desc_min..climb_max 内全部锚点（每个 restricted 的 desc/in/out/climb 保留，
    // 保证每个圆进入时高度已在区间外；多 restricted 时最后一个 profile 覆盖）
    let prof_pts: Vec<RouterPoint> = us
        .iter()
        .filter(|(su, _, _, _)| *su >= u_desc_min - 1e-9 && *su <= u_climb_max + 1e-9)
        .map(|(su, lon, lat, _)| {
            let mut alt = alt_m;
            for pr in &profiles {
                // 不能用 min：底部穿行（pass < alt）取更低，但顶部绕飞（pass > alt）
                // 取更高——直接覆盖，由 restricted_pass_alt 已决策好的 pass_alt 决定
                alt = profile_alt_at(pr, su * line_m, alt_m);
            }
            RouterPoint::new(*lon, *lat, alt)
        })
        .collect();
    let seg1 = Path::new(prof_pts);
    // 尾段：climb_max → target（平飞 3000m）
    let seg2 = Path::new(vec![pt_at(u_climb_max), p1]);
    (vec![seg0, seg1, seg2], vec![false, true, false])
}

/// 禁飞区墙膨胀 + 过渡带软罚（5c + 5c2）：
/// - 5c：NoFly/Obstacle 硬墙向外膨胀 inflation_cells 格（考虑飞机机动留转弯空间，
///   主管 2026-08-06：绕飞太贴边→考虑飞机机动——绕行需留物理转弯空间）；
/// - 5c2：膨胀墙外 2 格内代价渐变递增（墙边 ×1.5，带外 ×1），FMM 权衡代价后自然
///   走离墙更远的栅格，拉直后 clearance 余量充足（防贴墙锯齿，主管 2026-08-05
///   双禁飞区场景实测）。参数经实验标定 band=2/coef=0.5：窄缝（间隙 18.9km/7.8km）
///   与真实场景均平滑；band=3/coef=0.5 会把窄缝中间挤成锯齿（已弃）。
fn apply_inflation_and_band(field: &mut crate::costfield::CostField, inflation_cells: usize) {
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
    // 5c2. 过渡带软罚（BFS 距离变换，8 邻域，源 = 当前 INF 墙）
    {
        use std::collections::VecDeque;
        const BAND_CELLS: u32 = 2;
        const BAND_COEF: f32 = 0.5;
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
            if d >= 1 && d <= BAND_CELLS {
                let t = 1.0 - (d as f32 - 1.0) / BAND_CELLS as f32; // d=1 → t=1.0；d=BAND → t≈0.33
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
            restricted_pass_alt(&z, 3000.0, None, None, &start, &target, 15.0),
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
            restricted_pass_alt(&z, 3000.0, None, Some(&terr), &start, &target, 15.0),
            Some(5500.0)
        );
        // 升限 5000m → 顶部 5500 超升限且底部被地形挡 → 两者不可行 → None（画墙绕行）
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, Some(5000.0), Some(&terr), &start, &target, 15.0),
            None
        );
        // 巡航 1500m（区间外底部通道）→ 不拦截（restricted_blocks_alt false）
        assert!(!restricted_blocks_alt(&z, 1500.0));
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
        let (segs, mask) = build_restricted_profiles(
            &seg,
            &[z.clone()],
            3000.0,
            15.0,
            None,
            Some(&terr),
            &start_geo,
            &target_geo,
        );
        assert_eq!(segs.len(), 3, "应切 [首段, 剖面段, 尾段]");
        assert_eq!(mask, vec![false, true, false]);
        let prof = &segs[1].points;
        // 穿行高度 = alt_max + 500 = 5500（desc/climb 是过渡端点 = 巡航 3000，
        // in/out 是平飞穿行点 = 5500；过渡段为 max_climb 直线）
        assert!(
            prof.iter().any(|p| (p.alt_m - 5500.0).abs() < 1.0),
            "顶部平飞高度应为 5500m，实际 {:?}",
            prof.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
        assert!(
            prof.iter().all(|p| p.alt_m <= 5500.0 + 1.0),
            "顶部绕飞剖面不应低于巡航高度，实际 {:?}",
            prof.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
        // 剖面段锚点：desc/in/out/climb（4 点；端点重合时更少）
        assert!(prof.len() <= 6, "剖面段应压缩为锚点，实际 {} 点", prof.len());
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
        let check = make_segment_check(&zones, None, 0.0);
        // 斜切直线：start → 接近 target 的直线，穿过梯形内部 → 拒绝拉直
        assert!(!check(115.9, 39.8, 3000.0, 116.48, 40.3, 3000.0));
        // 绕行折线两段：先向下绕过梯形下边（y<39.9），再从右侧上行（x>116.5）→ 放行
        assert!(check(115.9, 39.8, 3000.0, 116.55, 39.85, 3000.0));
        assert!(check(116.55, 39.85, 3000.0, 116.8, 40.3, 3000.0));
        // 机动膨胀（主管 2026-08-06：绕飞太贴边→考虑飞机机动）：贴边绕行段
        // （距下边 ~5.5km < 膨胀 6km）被拒；远离段放行。
        let check_infl = make_segment_check(&zones, None, 6.0);
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
        // 北侧绕行 → 距离显著大于直线 164km
        assert!(out.vehicles[0].distance_m > 200_000.0, "dist {}", out.vehicles[0].distance_m);
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
}


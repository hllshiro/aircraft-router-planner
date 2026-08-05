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
use crate::threat::{SphericalRadarThreat, ThreatParams};

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
                    return Err(AppError::Data(
                        "terrain.source=path/builtin 但未提供地形文件（--terrain 或 terrain.path）".into(),
                    ))
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

    // 5b. 雷达静态代价（Phase 4 M3）：膨胀半径内 cost ×(1+几何并集概率)——FMM 倾向绕行；
    //     LOS 动态遮挡由 verify 威胁评估判定（不进静态代价场）。
    let params_merged = crate::config::DefaultParams::default().merge(&input.mission.parameters);
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
                        field.cost[idx] *= (1.0 + p) as f32;
                    }
                }
            }
        }
    }

    // 6. 每机：分段 FMM（start → mid[0..] → target，共享代价场）→ 拼接 → 平滑 → 输出
    let mut out_vehicles = Vec::new();
    let mut fmm_ms = 0.0f64;
    let mut degradations = Vec::new();
    for v in &specs {
        // 段序列：起点 + 必经点 + 目标
        let mut seg_ends: Vec<Geo> = Vec::with_capacity(v.mid_waypoints.len() + 2);
        seg_ends.push(v.start);
        seg_ends.extend(v.mid_waypoints.iter().copied());
        seg_ends.push(target);
        // 逐段 FMM → 回溯 → 拼接（去重段端点）
        let mut raw_segs: Vec<Path> = Vec::new();
        let mut no_solution = false;
        for seg in seg_ends.windows(2) {
            let (s, e) = (seg[0], seg[1]);
            let (sr, sc) = lonlat_cell(s.lon, s.lat, &region, grid);
            let (dr, dc) = lonlat_cell(e.lon, e.lat, &region, grid);
            let t0 = Instant::now();
            let res = fmm_propagate(&field, sr, sc);
            fmm_ms += t0.elapsed().as_secs_f64() * 1000.0;
            let Some(mut cells) = backtrack_path(&field, &res, dr, dc, sr, sc) else {
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
        let mut warnings = Vec::new();
        let mut pts = raw_joined.points.clone();
        if pts.len() >= 2 {
            let (opts, phys_min_radius_m) = crate::smooth::smooth_options_for(&v.profile, &params_merged);
            let check = make_segment_check(&all_zones);
            let chain = default_chain(&opts, &check);
            let ctx = VerifyContext {
                terrain: terrain.as_deref(),
                nofly: Some(&nofly),
                zones: Some(&all_zones),
                threat: Some(&threat),
            };
            // 每段独立平滑（首尾段端点保留——Theta* 截直不得移除必经点）
            let mut smooth_segs = Vec::new();
            let mut seg_warnings = Vec::new();
            for seg in &raw_segs {
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
            for i in final_rep.issues.iter().chain(seg_warnings.iter()) {
                if i.contains("radar") {
                    degradations.push(i.clone());
                }
            }        }
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
fn radar_threat_params(d: &crate::config::DefaultParams) -> ThreatParams {
    ThreatParams {
        radar_inflation: d.radar_inflation,
        detection_curve: d.detection_curve,
        p_cross: d.p_cross,
        suppression_delta: d.suppression_delta,
        base_p: d.p_cross,
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

/// Theta* 去锯齿段检查：直连 (a)→(b) 不穿任何 Zone（等距 16 点采样；
/// 水平 + 高度区间，高度沿线段线性插值——M2 高度层）。
fn make_segment_check<'a>(
    zones: &'a [Zone],
) -> impl Fn(f64, f64, f64, f64, f64, f64) -> bool + 'a {
    move |lon1, lat1, alt1, lon2, lat2, alt2| {
        const N: usize = 16;
        for i in 0..=N {
            let t = i as f64 / N as f64;
            let lon = lon1 + (lon2 - lon1) * t;
            let lat = lat1 + (lat2 - lat1) * t;
            let alt = alt1 + (alt2 - alt1) * t;
            if let Ok(g) = Geo::new(lon, lat) {
                if zones.iter().any(|z| zone_contains_at(z, &g, alt, None)) {
                    return false;
                }
            }
        }
        true
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
    fn m1_detours_around_zone() {
        // 挡路禁飞区（圆心在中点）→ 路径绕行（折线长度 > 直线）
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
        // 场景 A：大雷达（40km）挡在中点 → FMM 代价放大 → 绕行，探测概率≈0
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
        // 直线 ≈ 164km；雷达代价应导致绕行
        let d = out.vehicles[0].distance_m;
        assert!(d > 166_000.0, "大雷达应绕行，距离 {}", d);
        // 绕行成功 → 探测概率低 → 不应有超阈值警告
        assert!(
            !out.vehicles[0].warnings.iter().any(|s| s.contains("over threshold")),
            "绕行后不应超阈值: {:?}", out.vehicles[0].warnings
        );
    }

    #[test]
    fn m3_small_radar_crossing_reports_probability() {
        // 场景 B：小雷达（5km）代价不足 → FMM 直穿 → 复验累计探测概率超 P_cross →
        // 平滑链回退 + 雷达降级（degradations）
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
        // FMM 直穿（5km 雷达代价太小不值得绕）→ 距离 ≈ 直线（<170km）
        let d = out.vehicles[0].distance_m;
        assert!(d < 170_000.0, "小雷达应直穿，距离 {}", d);
        // 复验应报告雷达探测概率（软告警 warnings 或硬降级 degradations）
        let radar_reported = out.vehicles[0]
            .warnings
            .iter()
            .chain(out.stats.degradations.iter())
            .any(|s| s.contains("radar"));
        assert!(radar_reported, "应报告雷达探测概率: warnings={:?} degradations={:?}",
            out.vehicles[0].warnings, out.stats.degradations);
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


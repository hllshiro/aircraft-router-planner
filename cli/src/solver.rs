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
    zone_contains, Input, Output, PathPoint, Stats, TerrainSourceType, VehicleOutput, Zone,
    ZoneShape,
};
use crate::coord::Geo;
use crate::costfield::{backtrack_path, build_semantic_cost_field, fmm_propagate};
use crate::error::{AppError, InputInvalidReason};
use crate::path::{Path, PathPoint as RouterPoint};
use crate::smooth::{default_chain, smooth_path_chain, SmoothOptions, VerifyContext};
use crate::spatial::{CircleEntry, CircleIndex};
use crate::terrain::builtin::BuiltinSource;
use crate::terrain::{Sample, TerrainSource};

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
    aircraft_type: crate::config::AircraftType,
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
            aircraft_type: crate::config::AircraftType::FixedWing,
        }]
    } else {
        input
            .mission
            .vehicles
            .iter()
            .map(|v| {
                Ok(VehicleSpec {
                    id: v.id.clone(),
                    start: Geo::new(v.start_pose.lon, v.start_pose.lat)
                        .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))?,
                    alt_m: v.start_pose.alt_m,
                    aircraft_type: v.profile.aircraft_type,
                })
            })
            .collect::<Result<Vec<_>, AppError>>()?
    };

    // 3. 任务区域（所有起点 + target 包围盒 + 缓冲）
    let target = input.mission.target.to_geo()?;
    let region = region_of(&specs, &target);

    // 4. Zone 集合（no_fly + restricted + obstacles；M1 全部禁入，水平几何）
    let zones: Vec<&Zone> = input
        .mission
        .no_fly_zones
        .iter()
        .chain(input.mission.restricted_zones.iter())
        .chain(input.mission.obstacles.iter())
        .collect();
    let nofly = circle_index(&zones);

    // 5. 语义代价场（Land=1 / Water=1 / Lake=1 / NoData=5 / OOB=INF；Zone 覆盖 → OOB 墙）
    let grid = params.grid.max(8);
    let field = build_semantic_cost_field(grid, grid, |r, c| {
        let (lon, lat) = cell_lonlat(r, c, &region, grid);
        if let Ok(g) = Geo::new(lon, lat) {
            if zones.iter().any(|z| zone_contains(z, &g)) {
                return Sample::OutOfBounds;
            }
        }
        match &terrain {
            Some(t) => t.sample_at(lon, lat),
            None => Sample::Land(0.0),
        }
    }, 5.0);

    // 6. 每机：FMM → 回溯 → 平滑 → 输出
    let mut out_vehicles = Vec::new();
    let mut fmm_ms = 0.0f64;
    let mut degradations = Vec::new();
    for v in &specs {
        let (sr, sc) = lonlat_cell(v.start.lon, v.start.lat, &region, grid);
        let (dr, dc) = lonlat_cell(target.lon, target.lat, &region, grid);
        let t0 = Instant::now();
        let res = fmm_propagate(&field, sr, sc);
        fmm_ms += t0.elapsed().as_secs_f64() * 1000.0;
        let Some(mut cells) = backtrack_path(&field, &res, dr, dc, sr, sc) else {
            out_vehicles.push(VehicleOutput {
                id: v.id.clone(),
                status: "no_solution".into(),
                path: Vec::new(),
                distance_m: 0.0,
                warnings: vec!["coarse FMM no path".into()],
            });
            continue;
        };
        // backtrack 返回 dst→src 顺序 → 反转为 src→dst（路径语义）
        cells.reverse();
        let mut pts: Vec<RouterPoint> = cells
            .iter()
            .map(|&(r, c)| {
                let (lon, lat) = cell_lonlat(r, c, &region, grid);
                RouterPoint::new(lon, lat, v.alt_m)
            })
            .collect();

        // 平滑链（≥2 点；VerifyContext 接地形净空 + 禁飞圆）
        let mut warnings = Vec::new();
        if pts.len() >= 2 {
            let opts = SmoothOptions {
                aircraft_type: v.aircraft_type,
                ..Default::default()
            };
            let check = make_segment_check(&nofly, &zones);
            let chain = default_chain(&opts, &check);
            let ctx = VerifyContext {
                terrain: terrain.as_deref(),
                nofly: Some(&nofly),
            };
            let result = smooth_path_chain(&Path::new(pts), &chain, &opts, &ctx, None);
            pts = result.path.points;
            let vw = result.verify.warnings.clone();
            warnings = vw;
            for w in &result.verify.warnings {
                if w.contains("smoothing_failed") {
                    degradations.push(w.clone());
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

/// 圆形 zone → CircleIndex（smooth 复验禁飞包含用）。
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

/// Theta* 去锯齿段检查：直连 (a)→(b) 不穿任何 Zone（等距 16 点采样；水平几何）。
fn make_segment_check<'a>(
    _nofly: &'a CircleIndex,
    zones: &'a [&'a Zone],
) -> impl Fn(f64, f64, f64, f64, f64, f64) -> bool + 'a {
    move |lon1, lat1, _alt1, lon2, lat2, _alt2| {
        const N: usize = 16;
        for i in 0..=N {
            let t = i as f64 / N as f64;
            let lon = lon1 + (lon2 - lon1) * t;
            let lat = lat1 + (lat2 - lat1) * t;
            if let Ok(g) = Geo::new(lon, lat) {
                if zones.iter().any(|z| zone_contains(z, &g)) {
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


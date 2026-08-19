//! 崩溃/退化输入测试套件（B9 显式验收项：十四轮终裁落 B9 验收项）。
//!
//! 覆盖：config / coord / terrain / spatial / costfield 的退化与边界输入，
//! 断言不 panic、不段错误、不 OOM（fail-fast 返回错误或安全空结果）。
//! 每轮回归必跑（CI 一票否决），新增模块必须同步补用例。

use aircraft_router_planner_cli::config::{self, Input};
use aircraft_router_planner_cli::coord::{Ellipsoid, Geo, TransverseMercator, WebMercator};
use aircraft_router_planner_cli::costfield::{CostField, backtrack_path, fmm_propagate};
use aircraft_router_planner_cli::error::{AppError, InputInvalidReason};
use aircraft_router_planner_cli::solver;
use aircraft_router_planner_cli::spatial::{CircleIndex, RadarEntry, RadarIndex};
use aircraft_router_planner_cli::terrain::builtin::{BuiltinSource, write_pack_raw};
use aircraft_router_planner_cli::terrain::mask::{GeoMask, MaskedSource};
use aircraft_router_planner_cli::terrain::{
    TerrainSource, los_blocked, semantic_degradation_ratios,
};

// ==================== config ====================

#[test]
fn empty_input_is_malformed() {
    let r = Input::from_json_str("");
    assert!(r.is_err());
}

#[test]
fn null_json_is_malformed() {
    let r = Input::from_json_str("null");
    assert!(r.is_err());
}

#[test]
fn wrong_types_are_malformed() {
    // aircraft 应为数组
    let s = r#"{"aircraft": 123}"#;
    assert!(Input::from_json_str(s).is_err());
}

#[test]
fn missing_required_fields_are_malformed() {
    let s = r#"{}"#; // 缺 aircraft
    assert!(Input::from_json_str(s).is_err());
}

#[test]
fn nan_coordinates_rejected() {
    let s = r#"{"aircraft":[{"id":"a1","start":{"lon":NaN,"lat":39.0,"alt_m":0},"target":{"lon":3,"lat":4,"alt_m":0}}]}"#;
    // serde 对 NaN 默认拒绝（非有限数不合法 JSON number 语义？serde_json 允许 NaN）
    // 无论 parse 或 validate 层拦截，最终都应 input_invalid
    match Input::from_json_str(&s) {
        Ok(i) => match config::validate(&i) {
            Err(AppError::InputInvalid(InputInvalidReason::IllegalCoordinate)) => {}
            other => panic!("expected illegal_coordinate, got {other:?}"),
        },
        Err(_) => {} // parse 层拦截也算
    }
}

#[test]
fn huge_lat_rejected() {
    // MIN_AIRCRAFT 风格：单机显式 start/target，start.lat=91 越界
    let s = r#"{"aircraft":[{"id":"a1","start":{"lon":116.0,"lat":91.0,"alt_m":0},"target":{"lon":117.10,"lat":40.20,"alt_m":1000}}]}"#;
    let i = Input::from_json_str(&s).unwrap();
    match config::validate(&i) {
        Err(AppError::InputInvalid(InputInvalidReason::IllegalCoordinate)) => {}
        other => panic!("expected illegal_coordinate, got {other:?}"),
    }
}

#[test]
fn negative_radar_radius_rejected() {
    // MIN_AIRCRAFT 合法骨架 + 负半径雷达 → out_of_bounds
    let s = format!(
        r#"{{"aircraft":[{ac}],"red_forces":{{"radars":[{{"id":"r1","lon":116.0,"lat":39.5,"radius_km":-5}}]}}}}"#,
        ac = MIN_AIRCRAFT
    );
    let i = Input::from_json_str(&s).unwrap();
    match config::validate(&i) {
        Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds)) => {}
        other => panic!("expected out_of_bounds, got {other:?}"),
    }
}

/// v0.21 契约精简护栏：旧契约字段一律 malformed_json 拒绝（不 panic）。
const MIN_MISSION: &str = r#"{"start":{"lon":116.30,"lat":39.90,"alt_m":500},"target":{"lon":117.10,"lat":40.20,"alt_m":1000}}"#;

#[test]
fn legacy_schema_version_rejected() {
    let s = format!(r#"{{"schema_version":"0.20","mission":{}}}"#, MIN_MISSION);
    assert!(Input::from_json_str(&s).is_err());
}

#[test]
fn legacy_contract_fields_rejected() {
    let cases: &[&str] = &[
        &format!(r#"{{"crs":{{"datum":"WGS84","vertical":"MSL","input_projection":"lonlat"}},"mission":{}}}"#, MIN_MISSION),
        &format!(r#"{{"output_crs":{{"projection":"lonlat"}},"mission":{}}}"#, MIN_MISSION),
        &format!(r#"{{"mission":{{"start":{{"lon":116.30,"lat":39.90,"alt_m":500}},"target":{{"lon":117.10,"lat":40.20,"alt_m":1000}},"red_forces":{{"sams":[{{"id":"s1","lon":116.0,"lat":39.5,"radius_km":30}}]}}}}}}"#),
        &format!(r#"{{"mission":{{"start":{{"lon":116.30,"lat":39.90,"alt_m":500}},"target":{{"lon":117.10,"lat":40.20,"alt_m":1000}},"red_forces":{{"radars":[{{"id":"r1","lon":116.0,"lat":39.5,"radar_type":"tracking","radius_km":50}}]}}}}}}"#),
        &format!(r#"{{"mission":{{"start":{{"lon":116.30,"lat":39.90,"alt_m":500}},"target":{{"lon":117.10,"lat":40.20,"alt_m":1000}},"no_fly_zones":[{{"id":"z1","zone_type":"no_fly","shape":"circle","geometry":{{"center":[116.5,39.9],"radius_km":50}},"height_semantics":"msl"}}]}}}}"#),
        &format!(r#"{{"mission":{{"start":{{"lon":116.30,"lat":39.90,"alt_m":500}},"target":{{"lon":117.10,"lat":40.20,"alt_m":1000}},"vehicles":[{{"id":"v1","profile":{{"aircraft_type":"FIXED_WING"}},"start_pose":{{"lon":116.30,"lat":39.90,"alt_m":500,"heading_deg":45}}}}]}}}}"#),
        &format!(r#"{{"mission":{{"start":{{"lon":116.30,"lat":39.90,"alt_m":500}},"target":{{"lon":117.10,"lat":40.20,"alt_m":1000}},"vehicles":[{{"id":"v1","profile":{{"aircraft_type":"FIXED_WING","detection_probability":0.3}},"start_pose":{{"lon":116.30,"lat":39.90,"alt_m":500}}}}]}}}}"#),
        &format!(r#"{{"mission":{{"start":{{"lon":116.30,"lat":39.90,"alt_m":500}},"target":{{"lon":117.10,"lat":40.20,"alt_m":1000}},"parameters":{{"coarse_cell_m":2000}}}}}}"#),
    ];
    for s in cases {
        assert!(Input::from_json_str(s).is_err(), "旧契约字段应被拒: {s}");
    }
}

/// v0.21 最小合法 aircraft 数组元素骨架（id a1 + 合法 start/target；
/// 供坐标/雷达退化用例按新契约组合输入）。
const MIN_AIRCRAFT: &str = r#"{"id":"a1","start":{"lon":116.30,"lat":39.90,"alt_m":500},"target":{"lon":117.10,"lat":40.20,"alt_m":1000}}"#;

#[test]
fn legacy_mission_wrapper_rejected() {
    let s = r#"{"mission":{"start":{"lon":116.30,"lat":39.90,"alt_m":500},"target":{"lon":117.10,"lat":40.20,"alt_m":1000}}}"#;
    assert!(Input::from_json_str(s).is_err(), "mission 包裹层应被拒");
}

#[test]
fn legacy_top_level_start_target_rejected() {
    let cases: &[&str] = &[
        r#"{"start":{"lon":116.30,"lat":39.90,"alt_m":500},"aircraft":[]}"#,
        r#"{"target":{"lon":117.10,"lat":40.20,"alt_m":1000},"aircraft":[]}"#,
    ];
    for s in cases {
        assert!(Input::from_json_str(s).is_err(), "顶层 start/target 应被拒: {s}");
    }
}

#[test]
fn legacy_vehicles_key_rejected() {
    let s = r#"{"mission":{"start":{"lon":116.30,"lat":39.90,"alt_m":500},"target":{"lon":117.10,"lat":40.20,"alt_m":1000}},"vehicles":[{"id":"v1","profile":{"aircraft_type":"FIXED_WING"},"start_pose":{"lon":116.30,"lat":39.90,"alt_m":500}}]}"#;
    assert!(Input::from_json_str(s).is_err(), "vehicles 键应被拒");
}

#[test]
fn legacy_weapons_array_rejected() {
    let s = r#"{"mission":{"start":{"lon":116.30,"lat":39.90,"alt_m":500},"target":{"lon":117.10,"lat":40.20,"alt_m":1000}},"weapons":[{"weapon_id":"v1_w1","weapon_type":"agm"}]}"#;
    assert!(Input::from_json_str(s).is_err(), "顶层 weapons 键应被拒");
}

#[test]
fn legacy_zone_type_key_rejected() {
    let s = r#"{"mission":{"start":{"lon":116.30,"lat":39.90,"alt_m":500},"target":{"lon":117.10,"lat":40.20,"alt_m":1000}},"no_fly_zones":[{"id":"z1","zone_type":"no_fly","shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":50}}]}"#;
    assert!(Input::from_json_str(s).is_err(), "zone_type 键应被拒");
}

// `InputInvalidReason::MissingAircraft` 已由 Task 2 引入，恢复本护栏测试。
#[test]
fn aircraft_empty_rejected() {
    let s = r#"{"aircraft":[]}"#;
    let i = Input::from_json_str(s).unwrap();
    match config::validate(&i) {
        Err(AppError::InputInvalid(InputInvalidReason::MissingAircraft)) => {}
        other => panic!("expected missing_aircraft, got {other:?}"),
    }
}

// ==================== coord ====================

#[test]
fn coord_inf_nan_safe() {
    assert!(Geo::new(f64::NAN, 0.0).is_err());
    assert!(Geo::new(0.0, f64::NEG_INFINITY).is_err());
    assert!(Geo::new(-181.0, 0.0).is_err());
    assert!(Geo::new(0.0, 90.0001).is_err());
}

#[test]
fn tm_extreme_inverse_no_panic() {
    // 反算极端平面坐标（远超出有效区）不 panic
    let tm = TransverseMercator::utm(Ellipsoid::WGS84, 116.0);
    let _ = tm.inverse(1e15, 1e15);
    let _ = tm.inverse(-1e15, 0.0);
    let _ = tm.inverse(f64::NAN, 0.0);
}

#[test]
fn web_mercator_pole_safe() {
    // 纬度钳制，不 panic
    let _ = WebMercator::forward(0.0, 90.0);
    let _ = WebMercator::forward(0.0, -90.0);
    let _ = WebMercator::forward(f64::INFINITY, 0.0);
}

// ==================== terrain ====================

#[test]
fn builtin_garbage_bytes_no_panic() {
    let bytes = [0u8; 4096];
    assert!(BuiltinSource::parse(&bytes).is_err());
}

#[test]
fn builtin_empty_bytes_no_panic() {
    let bytes: [u8; 0] = [];
    assert!(BuiltinSource::parse(&bytes).is_err());
}

#[test]
fn builtin_tiny_valid_header_truncated() {
    // magic + 版本正确但头部截断
    let mut b = vec![0u8; 300];
    b[0..8].copy_from_slice(b"ARPACK1\0");
    b[8..12].copy_from_slice(&1u32.to_le_bytes());
    assert!(BuiltinSource::parse(&b).is_err());
}

#[test]
fn builtin_zero_dimension_rejected() {
    // 合法 header 但 rows=0（通过 write 构造不可能，直接改字节）
    let h = vec![10i16; 16 * 16];
    let bytes = write_pack_raw(16, 16, 0.0, 0.0, 0.001, 0.001, 50.0, true, -32768, "x", &h);
    let mut corrupted = bytes.clone();
    corrupted[16..20].copy_from_slice(&0u32.to_le_bytes()); // rows=0
    assert!(BuiltinSource::parse(&corrupted).is_err());
}

#[test]
fn srtm_bad_filename_no_panic() {
    let dir = std::env::temp_dir();
    let p = dir.join("NOPE123.hgt");
    // 打开不存在文件 → io error（安全）
    let _ = crate_terrain_open(&p);
}

// helper 编译期引用 open_source
fn crate_terrain_open(
    p: &std::path::Path,
) -> Result<Box<dyn aircraft_router_planner_cli::terrain::TerrainSource>, AppError> {
    aircraft_router_planner_cli::terrain::open_source(p)
}

#[test]
fn open_unsupported_extension_no_panic() {
    let dir = std::env::temp_dir();
    let p = dir.join("terrain.xyz");
    let _ = crate_terrain_open(&p);
}

// ==================== mask（Phase 2 掩膜集成） ====================

#[test]
fn mask_garbage_bytes_no_panic() {
    let bytes = [0u8; 4096];
    assert!(GeoMask::parse(&bytes).is_err());
}

#[test]
fn mask_empty_bytes_no_panic() {
    let bytes: [u8; 0] = [];
    assert!(GeoMask::parse(&bytes).is_err());
}

#[test]
fn mask_truncated_index_no_panic() {
    // magic 正确 + version 2，但索引区截断
    let mut b = vec![0u8; 100];
    b[0..16].copy_from_slice(b"ARPACK_MASK_V2__");
    b[16..20].copy_from_slice(&2u32.to_be_bytes());
    b[24..28].copy_from_slice(&1u32.to_be_bytes()); // rows=1
    b[28..32].copy_from_slice(&4u32.to_be_bytes()); // cols=4
    assert!(GeoMask::parse(&b).is_err());
}

#[test]
fn mask_absurd_dimensions_no_panic() {
    // rows/cols 巨大但文件小 → 校验失败（不 OOM）
    let mut b = vec![0u8; 64 + 2000 * 8 + 16];
    b[0..16].copy_from_slice(b"ARPACK_MASK_V2__");
    b[16..20].copy_from_slice(&2u32.to_be_bytes());
    b[24..28].copy_from_slice(&2000u32.to_be_bytes()); // rows=2000
    b[28..32].copy_from_slice(&4u32.to_be_bytes());
    assert!(GeoMask::parse(&b).is_err());
}

/// 极小合法掩膜（1×4：行 0 = 陆地 [1,3)）。
fn tiny_mask_bytes() -> Vec<u8> {
    let mut out = Vec::new();
    out.extend_from_slice(b"ARPACK_MASK_V2__");
    out.extend_from_slice(&2u32.to_be_bytes()); // version
    out.extend_from_slice(&1u32.to_be_bytes()); // arcsec
    out.extend_from_slice(&1u32.to_be_bytes()); // rows
    out.extend_from_slice(&4u32.to_be_bytes()); // cols
    out.extend_from_slice(&0.0f64.to_be_bytes()); // lon0
    out.extend_from_slice(&(-90.0f64).to_be_bytes()); // lat0
    out.extend_from_slice(&1.0f64.to_be_bytes()); // res
    out.extend_from_slice(&[0u8; 8]); // padding
    let seg_base = 64 + (1 + 1) * 8;
    out.extend_from_slice(
        &[seg_base as u64, (seg_base + 4 + 9) as u64]
            .iter()
            .flat_map(|o| o.to_be_bytes())
            .collect::<Vec<_>>()
            .as_slice(),
    );
    out.extend_from_slice(&1u32.to_be_bytes()); // nseg
    out.push(1);
    out.extend_from_slice(&1u32.to_be_bytes());
    out.extend_from_slice(&3u32.to_be_bytes());
    out
}

#[test]
fn mask_class_at_extreme_coords_no_panic() {
    let m = GeoMask::parse(&tiny_mask_bytes()).unwrap();
    // 极端/非有限坐标不 panic
    assert!(
        m.class_at(f64::NAN, 0.0) == aircraft_router_planner_cli::terrain::mask::MaskClass::Sea
    );
    assert!(
        m.class_at(f64::INFINITY, -90.0)
            == aircraft_router_planner_cli::terrain::mask::MaskClass::Sea
    );
    assert!(
        m.class_at(-f64::INFINITY, -90.0)
            == aircraft_router_planner_cli::terrain::mask::MaskClass::Sea
    );
    assert!(m.class_at(0.0, 90.0) == aircraft_router_planner_cli::terrain::mask::MaskClass::Sea);
    assert!(m.class_at(0.0, -90.0) == aircraft_router_planner_cli::terrain::mask::MaskClass::Sea);
    assert!(m.class_at(180.0, 0.0) == aircraft_router_planner_cli::terrain::mask::MaskClass::Sea);
    assert!(m.class_at(-180.0, 0.0) == aircraft_router_planner_cli::terrain::mask::MaskClass::Sea);
}

#[test]
fn masked_source_empty_inner_no_panic() {
    // 空 TerrainSource（0 尺寸）包装掩膜：采样不 panic（返回 OOB/NoData）
    let m = GeoMask::parse(&tiny_mask_bytes()).unwrap();
    let empty = aircraft_router_planner_cli::terrain::memory::Terrain {
        rows: 0,
        cols: 0,
        origin_lon: 0.0,
        origin_lat: 0.0,
        cell_lon_deg: 1.0,
        cell_lat_deg: 1.0,
        h: vec![],
    };
    let masked = MaskedSource::new(empty, m);
    let _ = masked.sample_at(1.5, -89.5);
    let _ = masked.sample_at(f64::NAN, f64::NAN);
}

#[test]
fn los_extreme_and_nodata_no_panic() {
    use aircraft_router_planner_cli::terrain::memory::Terrain;
    // 全 NaN 地形（全空洞）→ NoData → LOS 不遮挡（保守端），不 panic
    let t = Terrain {
        rows: 4,
        cols: 4,
        origin_lon: 0.0,
        origin_lat: 0.0,
        cell_lon_deg: 1.0,
        cell_lat_deg: 1.0,
        h: vec![f32::NAN; 16],
    };
    let _ = los_blocked(&t, 0.0, 0.0, 0.0, 1.0, 1.0, 0.0, 1.0, 100);
    let _ = los_blocked(&t, f64::NAN, 0.0, f64::INFINITY, 1.0, 1.0, 0.0, 1.0, 0);
    // 退化统计：n=0 / 全空洞
    let (nd, oob) = semantic_degradation_ratios(&t, 0);
    assert_eq!((nd, oob), (0.0, 0.0));
    let _ = semantic_degradation_ratios(&t, 16);
}

// ==================== spatial ====================

#[test]
fn spatial_empty_and_extremes_no_panic() {
    let idx = RadarIndex::build(vec![]);
    let _ = idx.within(0.0, 0.0, 1e9);
    let _ = idx.nearest(0.0, 0.0);
    let ci = CircleIndex::build(vec![]);
    let _ = ci.containing(0.0, 0.0);
    // 极端输入
    let mut entries = Vec::new();
    for i in 0..100 {
        entries.push(RadarEntry {
            id: format!("r{i}"),
            lon: (i as f64 - 50.0) * 10.0,
            lat: -60.0 + (i % 12) as f64 * 10.0,
            radius_m: 1e8,
        });
    }
    let big = RadarIndex::build(entries);
    let _ = big.within(181.0, 91.0, 1e12);
    let _ = big.nearest(f64::NAN, 0.0);
}

// ==================== costfield ====================

#[test]
fn fmm_degenerate_grids_no_panic() {
    // 1x1 / 单列 / 单行
    for (r, c) in [(1usize, 1usize), (1, 64), (64, 1)] {
        let f = CostField::new(r, c);
        let res = fmm_propagate(&f, 0, 0);
        let _ = res.accepted;
    }
    // 空网格
    let f = CostField::new(0, 0);
    let _ = fmm_propagate(&f, 0, 0);
    // 全高代价（不可达）
    let mut f = CostField::new(8, 8);
    for v in f.cost.iter_mut() {
        *v = f32::MAX;
    }
    let res = fmm_propagate(&f, 0, 0);
    let _ = backtrack_path(&f, &res, 7, 7, 0, 0);
    // 源=终点
    let res = fmm_propagate(&CostField::new(8, 8), 3, 3);
    let path = backtrack_path(&CostField::new(8, 8), &res, 3, 3, 3, 3);
    assert!(path.is_some());
    assert_eq!(path.unwrap().len(), 1);
}

#[test]
fn fmm_backtrack_unreachable_no_panic() {
    let f = CostField::new(4, 4);
    let res = fmm_propagate(&f, 0, 0);
    // 终点越界
    assert!(backtrack_path(&f, &res, 99, 99, 0, 0).is_none());
}

// ==================== output 序列化（防 panic） ====================

#[test]
fn output_serialize_with_extremes_no_panic() {
    use aircraft_router_planner_cli::config::{AircraftOutput, Output, Stats};
    use aircraft_router_planner_cli::error::ErrorBody;
    let out = Output::failure(
        "input_invalid",
        ErrorBody::input_invalid(InputInvalidReason::TargetInNoFly, "x"),
        1,
    );
    let s = serde_json::to_string(&out).expect("serialize ok");
    assert!(s.contains("input_invalid"));
    let out2 = Output {
        status: "success".into(),
        error: None,
        elapsed_ms: Some(0),
        aircraft: vec![AircraftOutput {
            id: "v1".into(),
            status: "planned".into(),
            path: vec![],
            distance_m: f64::NAN, // 序列化允许（Phase 2 保证路径有效）
            warnings: vec![],
        }],
        stats: Stats::default(),
    };
    let _ = serde_json::to_string(&out2).expect("serialize ok");
}

// ==================== Phase 3 平滑链 ====================

use aircraft_router_planner_cli::path::{Path, PathPoint};
use aircraft_router_planner_cli::smooth::{
    SmoothOptions, SmoothResult, Smoother, ThetaStarSmoother, VerifyContext, catmull_rom_spline,
    chaikin_smooth, dubins_fit, greedy_simplify, smooth_path_chain, theta_star_smooth, verify_path,
};

fn always_true() -> impl Fn(f64, f64, f64, f64, f64, f64) -> bool {
    |_, _, _, _, _, _| true
}

#[test]
fn smooth_empty_path_no_panic() {
    let p = Path::new(vec![]);
    let check = always_true();
    assert_eq!(
        theta_star_smooth(&p, &check, None, None, 0.0, 95.0).len(),
        0
    );
    assert_eq!(greedy_simplify(&p, 100.0).len(), 0);
    assert_eq!(chaikin_smooth(&p, 2).len(), 0);
    assert_eq!(catmull_rom_spline(&p, 4).len(), 0);
    assert!(dubins_fit(&p, 1000.0, 32).is_none());
}

#[test]
fn smooth_single_point_no_panic() {
    let p = Path::new(vec![PathPoint::new(0.0, 0.0, 100.0)]);
    let check = always_true();
    assert_eq!(
        theta_star_smooth(&p, &check, None, None, 0.0, 95.0).len(),
        1
    );
    assert_eq!(greedy_simplify(&p, 100.0).len(), 1);
    assert_eq!(chaikin_smooth(&p, 2).len(), 1);
    assert_eq!(catmull_rom_spline(&p, 4).len(), 1);
    assert!(dubins_fit(&p, 1000.0, 32).is_none());
}

#[test]
fn smooth_nan_inf_no_panic() {
    let bad = Path::new(vec![
        PathPoint::new(f64::NAN, 0.0, 100.0),
        PathPoint::new(1.0, 0.0, f64::INFINITY),
        PathPoint::new(2.0, 0.0, -f64::INFINITY),
    ]);
    let check = always_true();
    let _ = theta_star_smooth(&bad, &check, None, None, 0.0, 95.0);
    let _ = greedy_simplify(&bad, 100.0);
    let _ = chaikin_smooth(&bad, 2);
    let _ = catmull_rom_spline(&bad, 4);
    assert!(dubins_fit(&bad, 1000.0, 32).is_none());
    // 直接 dubins 入口
    let _ = aircraft_router_planner_cli::dubins::dubins_path(
        (f64::NAN, 0.0),
        0.0,
        (1.0, 0.0),
        0.0,
        1000.0,
    );
    let _ = aircraft_router_planner_cli::dubins::dubins_path(
        (0.0, 0.0),
        0.0,
        (1.0, 0.0),
        0.0,
        -5.0, // 负半径
    );
    let _ = aircraft_router_planner_cli::dubins::dubins_path(
        (0.0, 0.0),
        0.0,
        (1e9, 1e9), // 极端坐标
        0.0,
        1e12,
    );
}

#[test]
fn smooth_extreme_geometry_no_panic() {
    // 极端经纬度/高差
    let p = Path::new(vec![
        PathPoint::new(-180.0, -90.0, -1000.0),
        PathPoint::new(180.0, 90.0, 100_000.0),
        PathPoint::new(0.0, 0.0, 0.0),
    ]);
    let check = always_true();
    let _ = theta_star_smooth(&p, &check, None, None, 0.0, 95.0);
    let _ = greedy_simplify(&p, 1e-9);
    let _ = chaikin_smooth(&p, 10);
    let _ = catmull_rom_spline(&p, 100);
    let _ = dubins_fit(&p, 1e-9, 4);
    // verify 极端
    let opts = SmoothOptions::default();
    let ctx = VerifyContext {
        terrain: None,
        nofly: None,
        zones: None,
        threat: None,
        zone_inflation_m: 0.0,
    };
    let _ = verify_path(&p, None, &opts, &ctx, None);
}

#[test]
fn smooth_chain_degenerate_no_panic() {
    let opts = SmoothOptions::default();
    let ctx = VerifyContext {
        terrain: None,
        nofly: None,
        zones: None,
        threat: None,
        zone_inflation_m: 0.0,
    };
    // 空链 + 空路径
    let p = Path::new(vec![]);
    let r: SmoothResult = smooth_path_chain(&p, &[], &opts, &ctx, None);
    assert!(!r.verify.ok);
    // NaN 输入走链
    let bad = Path::new(vec![PathPoint::new(f64::NAN, 0.0, 100.0)]);
    let check = always_true();
    let chain: Vec<Box<dyn Smoother>> = vec![Box::new(ThetaStarSmoother {
        check: &check,
        max_turn_deg: None,
        entry_heading: None,
        min_r_m: 0.0,
        entry_max_deg: 95.0,
    })];
    let _ = smooth_path_chain(&bad, &chain, &opts, &ctx, None);
    // 退化的 verify 参数（零采样/负容差）不 panic
    let opts2 = SmoothOptions {
        verify_seg_samples: 0,
        chord_tol_m: -1.0,
        ..Default::default()
    };
    let _ = verify_path(&bad, None, &opts2, &ctx, None);
}

// ==================== 无效参数回落默认（主管决策 2026-08-05） ====================

#[test]
fn invalid_radar_params_recorded_as_degradations() {
    // 无外部参数或参数无效 → 使用默认值，且回落事实记入 stats.degradations。
    let s = r#"{
        "aircraft":[
            {"id":"a1","start":{"lon":115.0,"lat":39.0,"alt_m":3000},"target":{"lon":116.5,"lat":39.9,"alt_m":3000}}
        ],
        "parameters":{
            "radar_inflation":-1.0,
            "p_cross":5.0,
            "suppression_delta":9.0,
            "detection_curve":"weird"
        }
    }"#;
    let input = Input::from_json_str(s).unwrap();
    config::validate(&input).unwrap();
    let out = solver::solve(&input, &solver::SolveParams::default(), 0).unwrap();
    let degs = &out.stats.degradations;
    assert!(
        degs.iter()
            .any(|d| d.contains("radar_inflation=-1 invalid -> default 1.2")),
        "missing radar_inflation degradation: {degs:?}"
    );
    assert!(
        degs.iter()
            .any(|d| d.contains("p_cross=5 invalid -> default 0.1")),
        "missing p_cross degradation: {degs:?}"
    );
    assert!(
        degs.iter()
            .any(|d| d.contains("suppression_delta=9 invalid -> default 0.5")),
        "missing suppression_delta degradation: {degs:?}"
    );
    assert!(
        degs.iter()
            .any(|d| d.contains("detection_curve=weird invalid -> default exponential")),
        "missing detection_curve degradation: {degs:?}"
    );
}

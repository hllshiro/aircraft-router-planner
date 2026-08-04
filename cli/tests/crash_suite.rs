//! 崩溃/退化输入测试套件（B9 显式验收项：十四轮终裁落 B9 验收项）。
//!
//! 覆盖：config / coord / terrain / spatial / costfield 的退化与边界输入，
//! 断言不 panic、不段错误、不 OOM（fail-fast 返回错误或安全空结果）。
//! 每轮回归必跑（CI 一票否决），新增模块必须同步补用例。

use aircraft_router_planner_cli::config::{self, Input};
use aircraft_router_planner_cli::coord::{Ellipsoid, Geo, TransverseMercator, WebMercator};
use aircraft_router_planner_cli::costfield::{backtrack_path, fmm_propagate, CostField};
use aircraft_router_planner_cli::error::{AppError, InputInvalidReason};
use aircraft_router_planner_cli::spatial::{CircleIndex, RadarEntry, RadarIndex};
use aircraft_router_planner_cli::terrain::builtin::{write_pack_raw, BuiltinSource};

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
    // schema_version 应为字符串
    let s = r#"{"schema_version": 123, "mission": {"start": {"lon": 1, "lat": 2, "alt_m": 0}, "target": {"lon": 3, "lat": 4, "alt_m": 0}}}"#;
    assert!(Input::from_json_str(s).is_err());
}

#[test]
fn missing_required_fields_are_malformed() {
    let s = r#"{"schema_version": "0.20"}"#; // 缺 mission
    assert!(Input::from_json_str(s).is_err());
}

#[test]
fn nan_coordinates_rejected() {
    let s = r#"{"schema_version":"0.20","mission":{"start":{"lon":NaN,"lat":39.0,"alt_m":0},"target":{"lon":3,"lat":4,"alt_m":0}}}"#;
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
    let s = r#"{"schema_version":"0.20","mission":{"start":{"lon":116.0,"lat":91.0,"alt_m":0},"target":{"lon":3,"lat":4,"alt_m":0}}}"#;
    let i = Input::from_json_str(&s).unwrap();
    match config::validate(&i) {
        Err(AppError::InputInvalid(InputInvalidReason::IllegalCoordinate)) => {}
        other => panic!("expected illegal_coordinate, got {other:?}"),
    }
}

#[test]
fn negative_radar_radius_rejected() {
    let s = r#"{"schema_version":"0.20","mission":{"start":{"lon":115.0,"lat":39.0,"alt_m":0},"target":{"lon":117.0,"lat":40.0,"alt_m":0},"red_forces":{"radars":[{"id":"r1","lon":116.0,"lat":39.5,"radar_type":"tracking","radius_km":-5}]}}}"#;
    let i = Input::from_json_str(&s).unwrap();
    match config::validate(&i) {
        Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds)) => {}
        other => panic!("expected out_of_bounds, got {other:?}"),
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
fn crate_terrain_open(p: &std::path::Path) -> Result<Box<dyn aircraft_router_planner_cli::terrain::TerrainSource>, AppError> {
    aircraft_router_planner_cli::terrain::open_source(p)
}

#[test]
fn open_unsupported_extension_no_panic() {
    let dir = std::env::temp_dir();
    let p = dir.join("terrain.xyz");
    let _ = crate_terrain_open(&p);
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
    use aircraft_router_planner_cli::config::{Output, Stats, VehicleOutput};
    use aircraft_router_planner_cli::error::ErrorBody;
    let out = Output::failure("input_invalid", ErrorBody::input_invalid(InputInvalidReason::TargetInNoFly, "x"), 1);
    let s = serde_json::to_string(&out).expect("serialize ok");
    assert!(s.contains("input_invalid"));
    let out2 = Output {
        schema_version: "0.20".into(),
        status: "success".into(),
        error: None,
        elapsed_ms: Some(0),
        vehicles: vec![VehicleOutput {
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

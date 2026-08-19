//! P4-M5：确定性门禁（docs/12 §8 验收 4 + §12.1 共识 4）——同输入双跑，
//! 输出（stdout JSON 契约）逐字节一致。确定性构建约束（-fma 禁用 + 固定
//! target-cpu）保证跨平台一致；本测试锁同一进程内双跑（运行时无随机性）。
//!
//! 输入刻意不带 terrain/mask（默认掩膜自动探测找不到时纯地形，FMM 代价场
//! 确定性来源：BTreeMap/固定序 + 整数 tie-break）。

use aircraft_router_planner_cli::config::Input;
use aircraft_router_planner_cli::solver::{self, SolveParams};

const INPUT_NO_TERRAIN: &str = r#"{
  "mission": {
    "start": {"lon": 115.0, "lat": 39.0, "alt_m": 3000},
    "target": {"lon": 116.5, "lat": 39.9, "alt_m": 3000}
  }
}"#;

const INPUT_ZONE_NO_TERRAIN: &str = r#"{
  "mission": {
    "start": {"lon": 115.0, "lat": 39.0, "alt_m": 3000},
    "target": {"lon": 116.5, "lat": 39.9, "alt_m": 3000},
    "no_fly_zones": [
      {"id": "mid", "zone_type": "no_fly", "shape": "circle",
       "geometry": {"center": [115.75, 39.45], "radius_km": 15},
       "alt_min_m": 0, "alt_max_m": 10000}
    ]
  }
}"#;

/// 逐字节门禁：JSON 输出除运行时时间戳（elapsed_ms / stats.fmm_ms 天然受缓存
/// 热影响）外必须完全一致。时间字段清零后比较（其余字段仍要求字节级相等）。
fn assert_identical(
    out1: &aircraft_router_planner_cli::config::Output,
    out2: &aircraft_router_planner_cli::config::Output,
) {
    let j1 = serde_json::to_string_pretty(out1).unwrap();
    let j2 = serde_json::to_string_pretty(out2).unwrap();
    let mut v1: serde_json::Value = serde_json::from_str(&j1).unwrap();
    let mut v2: serde_json::Value = serde_json::from_str(&j2).unwrap();
    v1["elapsed_ms"] = 0.into();
    v2["elapsed_ms"] = 0.into();
    v1["stats"]["fmm_ms"] = 0.into();
    v2["stats"]["fmm_ms"] = 0.into();
    assert_eq!(
        v1, v2,
        "determinism gate: double run must be identical (except timestamps)"
    );
}

#[test]
fn double_run_byte_identical_no_terrain() {
    let input = Input::from_json_str(INPUT_NO_TERRAIN).unwrap();
    let params = SolveParams::default();
    let out1 = solver::solve(&input, &params, 0).unwrap();
    let out2 = solver::solve(&input, &params, 0).unwrap();
    assert_identical(&out1, &out2);
}

#[test]
fn double_run_byte_identical_with_zone() {
    // 带圆形硬墙输入：FMM 绕行 + （若触发）可见图路径确定性
    let input = Input::from_json_str(INPUT_ZONE_NO_TERRAIN).unwrap();
    let params = SolveParams::default();
    let out1 = solver::solve(&input, &params, 0).unwrap();
    let out2 = solver::solve(&input, &params, 0).unwrap();
    assert_identical(&out1, &out2);
}

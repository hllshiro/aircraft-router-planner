//! 崩溃测试套件（Phase 0 S8 / B9 验收项——CI 一票否决）。
//!
//! 退化/边界输入断言：不 panic、不段错误、不 OOM，返回 None 或合理值。
//! 覆盖：FMM 传播、Dubins 基元、地形采样/射线、rstar 空树查询。

use phase0::dubins::dubins_path;
use phase0::fmm::{self, CostField};
use phase0::terrain::{self, Terrain};
use rstar::{AABB, PointDistance, RTree, RTreeObject};

// ---------- FMM ----------

#[test]
fn fmm_empty_grid_no_panic() {
    let f = CostField::new(0, 0);
    let res = fmm::fmm_propagate(&f, 0, 0); // 源点对空网格越界
    assert_eq!(res.times.len(), 0);
}

#[test]
fn fmm_tiny_grid_ok() {
    let f = CostField::new(1, 1);
    let res = fmm::fmm_propagate(&f, 0, 0);
    assert_eq!(res.times.len(), 1);
    assert!(res.accepted[0]);
    assert_eq!(res.times[0], 0.0);
}

#[test]
fn fmm_source_out_of_bounds_no_panic() {
    let f = CostField::new(16, 16);
    let res = fmm::fmm_propagate(&f, 999, 999);
    assert!(res.times.iter().all(|t| t.is_infinite()));
    assert!(res.accepted.iter().all(|a| !a));
}

#[test]
fn fmm_inf_cost_field_no_panic() {
    // 全 Inf 代价：solve_t 中 Inf 算术不应 panic（返回 Inf 场即可）
    let mut f = CostField::new(8, 8);
    for v in f.cost.iter_mut() {
        *v = f32::INFINITY;
    }
    let res = fmm::fmm_propagate(&f, 4, 4);
    assert_eq!(res.times.len(), 64);
}

// ---------- Dubins ----------

#[test]
fn dubins_zero_radius_none() {
    assert!(dubins_path((0.0, 0.0), 0.0, (10.0, 0.0), 0.0, 0.0).is_none());
    assert!(dubins_path((0.0, 0.0), 0.0, (10.0, 0.0), 0.0, -1.0).is_none());
}

#[test]
fn dubins_nan_input_no_panic() {
    assert!(dubins_path((f64::NAN, 0.0), 0.0, (10.0, 0.0), 0.0, 1.0).is_none());
    assert!(dubins_path((0.0, 0.0), f64::INFINITY, (10.0, 0.0), 0.0, 1.0).is_none());
    assert!(dubins_path((0.0, 0.0), 0.0, (10.0, f64::NAN), 0.0, 1.0).is_none());
    assert!(dubins_path((0.0, 0.0), 0.0, (10.0, 0.0), 0.0, f64::NAN).is_none());
}

#[test]
fn dubins_extreme_coords_no_panic() {
    let len = dubins_path((1e12, 1e12), 1.0, (1e12 + 1e5, 1e12 + 1e5), 1.0, 1e3);
    assert!(len.is_some()); // 有限且可行
}

#[test]
fn dubins_same_point_opposite_heading() {
    // 同点反向：有解（绕 180°+）
    let p = dubins_path((0.0, 0.0), 0.0, (0.0, 0.0), std::f64::consts::PI, 1.0);
    assert!(p.is_some());
    assert!(p.unwrap().len() > 0.0);
}

// ---------- Terrain ----------

#[test]
fn terrain_empty_no_panic() {
    let t = Terrain {
        rows: 0,
        cols: 0,
        cell_mx: 1000.0,
        cell_my: 1000.0,
        h: vec![],
    };
    assert!(t.height_at(0.0, 0.0).is_none());
    assert!(!t.ray_blocked(0.0, 0.0, 100.0, 1.0, 1.0, 0.0, 1000.0, 10));
}

#[test]
fn terrain_ray_out_of_bounds_no_panic() {
    let t = terrain::synthetic_terrain(16, 16, 1000.0, 7);
    // 射线穿过网格外部：height_at 返回 None → ray_blocked 返回 false（不 panic）
    assert!(!t.ray_blocked(-5000.0, -5000.0, 100.0, 0.0, 1.0, 0.0, 20000.0, 1000));
}

#[test]
fn terrain_extreme_ray_no_panic() {
    let t = terrain::synthetic_terrain(16, 16, 1000.0, 7);
    assert!(!t.ray_blocked(f64::NAN, 0.0, 100.0, 1.0, 0.0, 0.0, 1000.0, 10)); // NaN → 无解
}

// ---------- rstar 空树/退化 ----------

#[derive(Clone, Copy)]
struct AabbItem {
    lo: [f64; 3],
    hi: [f64; 3],
}

impl RTreeObject for AabbItem {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        AABB::from_corners(self.lo, self.hi)
    }
}

impl PointDistance for AabbItem {
    fn distance_2(&self, point: &[f64; 3]) -> f64 {
        let c = [
            0.5 * (self.lo[0] + self.hi[0]),
            0.5 * (self.lo[1] + self.hi[1]),
            0.5 * (self.lo[2] + self.hi[2]),
        ];
        (point[0] - c[0]).powi(2) + (point[1] - c[1]).powi(2) + (point[2] - c[2]).powi(2)
    }
}

#[test]
fn rstar_empty_tree_no_panic() {
    let tree: RTree<AabbItem> = RTree::new();
    let q = AABB::from_corners([0.0, 0.0, 0.0], [10.0, 10.0, 10.0]);
    assert_eq!(tree.locate_in_envelope_intersecting(&q).count(), 0);
    assert!(tree.nearest_neighbor(&[5.0, 5.0, 5.0]).is_none());
}

#[test]
fn rstar_single_item_query() {
    let item = AabbItem {
        lo: [0.0, 0.0, 0.0],
        hi: [5.0, 5.0, 5.0],
    };
    let tree = RTree::bulk_load(vec![item]);
    let q = AABB::from_corners([1.0, 1.0, 1.0], [2.0, 2.0, 2.0]);
    assert_eq!(tree.locate_in_envelope_intersecting(&q).count(), 1);
}

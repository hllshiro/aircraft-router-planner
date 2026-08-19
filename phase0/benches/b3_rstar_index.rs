//! B3 基准：rstar 雷达/禁飞区索引（S4）。
//!
//! - N=100 雷达（3D 球体，AABB 入树）+ 50 禁飞区棱柱（矩形 AABB）；
//! - 索引构建耗时（bulk_load）；
//! - 1000 条随机线段：线段 AABB 范围查询（碰撞候选）→ 单查询耗时 + 平均候选数；
//! - 1000 随机点最近邻查询；
//! - 暴力遍历对比（100 球体逐一遍历距离检查）。
//!
//! 场景：100km 立方体（坐标单位 km），雷达半径 8–15km。

use criterion::{Criterion, criterion_group, criterion_main};
use rand::{RngExt, SeedableRng};
use rstar::{AABB, Envelope, PointDistance, RTree, RTreeObject};
use std::hint::black_box;

const SCENE: f64 = 100.0; // km
const N_RADAR: usize = 100;
const N_NOFLY: usize = 50;
const N_SEG: usize = 1000;

#[derive(Clone, Copy, Debug)]
struct Radar {
    center: [f64; 3],
    radius: f64,
}

impl RTreeObject for Radar {
    type Envelope = AABB<[f64; 3]>;
    fn envelope(&self) -> Self::Envelope {
        let r = self.radius;
        AABB::from_corners(
            [self.center[0] - r, self.center[1] - r, self.center[2] - r],
            [self.center[0] + r, self.center[1] + r, self.center[2] + r],
        )
    }
}

impl PointDistance for Radar {
    fn distance_2(&self, point: &[f64; 3]) -> f64 {
        let d2 = (point[0] - self.center[0]).powi(2)
            + (point[1] - self.center[1]).powi(2)
            + (point[2] - self.center[2]).powi(2);
        let d = d2.sqrt() - self.radius;
        d.max(0.0).powi(2)
    }
}

#[derive(Clone, Copy, Debug)]
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

fn gen_radars(seed: u64) -> Vec<Radar> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    (0..N_RADAR)
        .map(|_| Radar {
            center: [
                rng.random_range(0.0..SCENE),
                rng.random_range(0.0..SCENE),
                rng.random_range(0.0..12.0),
            ],
            radius: rng.random_range(8.0..15.0),
        })
        .collect()
}

fn gen_nofly(seed: u64) -> Vec<AabbItem> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    (0..N_NOFLY)
        .map(|_| {
            let x0 = rng.random_range(0.0..SCENE - 20.0);
            let y0 = rng.random_range(0.0..SCENE - 20.0);
            AabbItem {
                lo: [x0, y0, 0.0],
                hi: [
                    x0 + rng.random_range(5.0..20.0),
                    y0 + rng.random_range(5.0..20.0),
                    12.0,
                ],
            }
        })
        .collect()
}

/// 随机线段（起止点随机，用于碰撞候选查询）
fn gen_segments(seed: u64) -> Vec<([f64; 3], [f64; 3])> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    (0..N_SEG)
        .map(|_| {
            (
                [
                    rng.random_range(0.0..SCENE),
                    rng.random_range(0.0..SCENE),
                    rng.random_range(0.0..12.0),
                ],
                [
                    rng.random_range(0.0..SCENE),
                    rng.random_range(0.0..SCENE),
                    rng.random_range(0.0..12.0),
                ],
            )
        })
        .collect()
}

/// 线段包围盒（+扩展 margin 的查询范围）
fn seg_aabb(a: &[f64; 3], b: &[f64; 3], margin: f64) -> AABB<[f64; 3]> {
    AABB::from_corners(
        [
            a[0].min(b[0]) - margin,
            a[1].min(b[1]) - margin,
            a[2].min(b[2]) - margin,
        ],
        [
            a[0].max(b[0]) + margin,
            a[1].max(b[1]) + margin,
            a[2].max(b[2]) + margin,
        ],
    )
}

fn criterion_benchmark(c: &mut Criterion) {
    let radars = gen_radars(0x51);
    let nofly = gen_nofly(0x52);
    let segments = gen_segments(0x53);

    // 构建索引
    c.bench_function("b3_rstar/build_radar_100", |b| {
        b.iter(|| RTree::bulk_load(black_box(radars.clone())))
    });
    c.bench_function("b3_rstar/build_nofly_50", |b| {
        b.iter(|| RTree::bulk_load(black_box(nofly.clone())))
    });

    // 线段范围查询（碰撞候选）
    {
        let tree = RTree::bulk_load(radars.clone());
        c.bench_function("b3_rstar/range_query_1000_seg_radar", |b| {
            b.iter(|| {
                let mut cnt = 0usize;
                for (a, bb) in &segments {
                    let q = seg_aabb(a, bb, 0.5);
                    cnt += tree.locate_in_envelope_intersecting(&q).count();
                }
                black_box(cnt)
            })
        });
        println!(
            "\n[avg candidates] radar range query: {:.2} per segment (N={})",
            {
                let mut total = 0usize;
                for (a, bb) in &segments {
                    let q = seg_aabb(a, bb, 0.5);
                    total += tree.locate_in_envelope_intersecting(&q).count();
                }
                total as f64 / N_SEG as f64
            },
            N_RADAR
        );
    }

    // 禁飞区棱柱查询
    {
        let tree = RTree::bulk_load(nofly.clone());
        c.bench_function("b3_rstar/range_query_1000_seg_nofly", |b| {
            b.iter(|| {
                let mut cnt = 0usize;
                for (a, bb) in &segments {
                    let q = seg_aabb(a, bb, 0.5);
                    cnt += tree.locate_in_envelope_intersecting(&q).count();
                }
                black_box(cnt)
            })
        });
    }

    // 最近邻
    {
        let tree = RTree::bulk_load(radars.clone());
        let mut rng = rand::rngs::StdRng::seed_from_u64(0x54);
        let pts: Vec<[f64; 3]> = (0..N_SEG)
            .map(|_| {
                [
                    rng.random_range(0.0..SCENE),
                    rng.random_range(0.0..SCENE),
                    rng.random_range(0.0..12.0),
                ]
            })
            .collect();
        c.bench_function("b3_rstar/nearest_1000_pts", |b| {
            b.iter(|| {
                let mut d = 0f64;
                for p in &pts {
                    if let Some(n) = tree.nearest_neighbor(p) {
                        d += n.distance_2(p);
                    }
                }
                black_box(d)
            })
        });
    }

    // 暴力遍历对比（100 球体逐一遍历，判定线段 AABB 与球 AABB 相交）
    {
        c.bench_function("b3_rstar/brute_force_100_radar", |b| {
            b.iter(|| {
                let mut cnt = 0usize;
                for (a, bb) in &segments {
                    let q = seg_aabb(a, bb, 0.5);
                    for r in &radars {
                        let env = r.envelope();
                        if env.intersects(&q) {
                            cnt += 1;
                        }
                    }
                }
                black_box(cnt)
            })
        });
    }
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

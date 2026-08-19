//! B1 基准：FMM 粗层传播（S2）。
//!
//! - 多档网格（64/128/256/512²，100km 场景对应 1.56km/781m/390m/195m 格距）
//!   测单次传播耗时 → O(NlogN) 实际常数；
//! - 100 组随机起止（128²）：每次迭代跑 100 个随机源传播（统计样本）；
//! - 走廊质量统计（一次性，100 组随机起止）：可达率 / 平均路径步数 / 绕行比。

use criterion::{Criterion, criterion_group, criterion_main};
use phase0::fmm::{self, CostField};
use rand::{RngExt, SeedableRng};
use std::hint::black_box;

const SCENE_M: f64 = 100_000.0; // 100km 场景

fn make_field(grid: usize) -> CostField {
    fmm::synthetic_cost_field(grid, grid, SCENE_M / grid as f64, 20260804)
}

fn bench_grid(c: &mut Criterion, grid: usize) {
    let field = make_field(grid);
    let src = (grid / 2, grid / 2);
    let id = format!("fmm_propagation/grid_{}", grid);
    c.bench_function(&id, |b| {
        b.iter(|| {
            let res = fmm::fmm_propagate(black_box(&field), black_box(src.0), black_box(src.1));
            black_box(res.accepted[field.idx(grid - 1, grid - 1)])
        })
    });
}

fn corridor_quality_stats(grid: usize) {
    let field = make_field(grid);
    let mut rng = rand::rngs::StdRng::seed_from_u64(0xC0FFEE);
    let mut reachable = 0usize;
    let mut total_steps = 0f64;
    let mut total_ratio = 0f64;
    let mut worst_ratio = 0f64;
    for _ in 0..100 {
        let (sr, sc) = (rng.random_range(0..grid), rng.random_range(0..grid));
        let (dr, dc) = (rng.random_range(0..grid), rng.random_range(0..grid));
        let res = fmm::fmm_propagate(&field, sr, sc);
        if let Some(path) = fmm::backtrack_path(&field, &res, dr, dc, sr, sc) {
            reachable += 1;
            let steps = path.len() as f64;
            let direct =
                (((dr as i64 - sr as i64).pow(2) + (dc as i64 - sc as i64).pow(2)) as f64).sqrt();
            let ratio = steps / direct.max(1.0);
            total_steps += steps;
            total_ratio += ratio;
            worst_ratio = worst_ratio.max(ratio);
        }
    }
    println!(
        "\n[corridor-quality] grid={} 100 random start->dst pairs",
        grid
    );
    println!("  reachable: {}/100", reachable);
    println!("  avg path steps: {:.1}", total_steps / reachable as f64);
    println!("  avg detour ratio: {:.3}", total_ratio / reachable as f64);
    println!("  worst detour ratio: {:.3}", worst_ratio);
}

fn criterion_benchmark(c: &mut Criterion) {
    corridor_quality_stats(128);

    for grid in [64usize, 128, 256, 512] {
        bench_grid(c, grid);
    }

    // 100 组随机起止：每次迭代内串行跑 100 个随机源传播（128²）
    {
        let grid = 128;
        let field = make_field(grid);
        let mut rng = rand::rngs::StdRng::seed_from_u64(0xABCDEF);
        let sources: Vec<(usize, usize)> = (0..100)
            .map(|_| (rng.random_range(0..grid), rng.random_range(0..grid)))
            .collect();
        c.bench_function("fmm_propagation/100_random_sources_grid128", |b| {
            b.iter(|| {
                let mut acc = 0u64;
                for &(sr, sc) in &sources {
                    let res = fmm::fmm_propagate(black_box(&field), sr, sc);
                    acc = acc.wrapping_add(
                        res.times[field.idx((sr + 1) % grid, (sc + 1) % grid)] as u64,
                    );
                }
                black_box(acc)
            })
        });
    }
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

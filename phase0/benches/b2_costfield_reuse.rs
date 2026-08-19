//! B2 基准：代价场复用（S3）——多机摊薄。
//!
//! - 同源场景：10 机同起点（编队/同基地），独立=10 次传播 vs 共享=1 次传播+10 次回溯；
//! - 异源场景：独立=N×(构建+传播) vs 共享=1 次代价场构建+N 次传播；
//! - 内存口径：1 场（times f32 + accepted bool）vs 10 场。
//!
//! 产出：摊薄系数（独立总耗时 / 共享总耗时）、内存对比 → 落多机契约。

use criterion::{Criterion, criterion_group, criterion_main};
use phase0::fmm::{self, CostField};
use rand::{RngExt, SeedableRng};
use std::hint::black_box;

const SCENE_M: f64 = 100_000.0;
const GRID: usize = 128;
const N: usize = 10;

fn make_field() -> CostField {
    fmm::synthetic_cost_field(GRID, GRID, SCENE_M / GRID as f64, 20260804)
}

fn gen_pairs(seed: u64) -> Vec<((usize, usize), (usize, usize))> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    (0..N)
        .map(|_| {
            (
                (rng.random_range(0..GRID), rng.random_range(0..GRID)),
                (rng.random_range(0..GRID), rng.random_range(0..GRID)),
            )
        })
        .collect()
}

fn criterion_benchmark(c: &mut Criterion) {
    let field = make_field();
    let pairs = gen_pairs(0x51);
    let src = pairs[0].0;
    let dsts: Vec<(usize, usize)> = pairs.iter().map(|p| p.1).collect();

    // --- 场景 A：同源（10 机同起点，不同终点）---
    c.bench_function("b2_costfield_reuse/same_src_independent_10", |b| {
        b.iter(|| {
            let mut acc = 0u64;
            for &d in &dsts {
                let res = fmm::fmm_propagate(black_box(&field), src.0, src.1);
                let path = fmm::backtrack_path(&field, &res, d.0, d.1, src.0, src.1);
                acc = acc.wrapping_add(path.map(|p| p.len() as u64).unwrap_or(0));
            }
            black_box(acc)
        })
    });

    c.bench_function("b2_costfield_reuse/same_src_shared_1prop_10back", |b| {
        b.iter(|| {
            let res = fmm::fmm_propagate(black_box(&field), src.0, src.1);
            let mut acc = 0u64;
            for &d in &dsts {
                let path = fmm::backtrack_path(&field, &res, d.0, d.1, src.0, src.1);
                acc = acc.wrapping_add(path.map(|p| p.len() as u64).unwrap_or(0));
            }
            black_box(acc)
        })
    });

    // --- 场景 B：异源（10 机各自起止）---
    c.bench_function("b2_costfield_reuse/diff_src_independent_10", |b| {
        b.iter(|| {
            let mut acc = 0u64;
            for &(s, d) in &pairs {
                let f = make_field();
                let res = fmm::fmm_propagate(&f, s.0, s.1);
                let path = fmm::backtrack_path(&f, &res, d.0, d.1, s.0, s.1);
                acc = acc.wrapping_add(path.map(|p| p.len() as u64).unwrap_or(0));
            }
            black_box(acc)
        })
    });

    c.bench_function("b2_costfield_reuse/diff_src_shared_1build_10prop", |b| {
        b.iter(|| {
            let f = make_field();
            let mut acc = 0u64;
            for &(s, d) in &pairs {
                let res = fmm::fmm_propagate(&f, s.0, s.1);
                let path = fmm::backtrack_path(&f, &res, d.0, d.1, s.0, s.1);
                acc = acc.wrapping_add(path.map(|p| p.len() as u64).unwrap_or(0));
            }
            black_box(acc)
        })
    });

    // --- 内存口径 ---
    let n = GRID * GRID;
    let per = n * 5; // times 4B + accepted 1B
    println!(
        "\n[memory] grid {} per field = {:.1} KiB ; 10 fields = {:.1} MiB ; shared 1 field = {:.1} KiB",
        GRID,
        per as f64 / 1024.0,
        per as f64 * 10.0 / 1048576.0,
        per as f64 / 1024.0
    );
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

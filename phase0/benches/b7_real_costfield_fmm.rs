//! B7 基准：真实地形（Beijing_DEM）→ 粗层代价场 → FMM 传播（B1 真实双跑）。
//!
//! - Beijing_DEM 全量加载 → 块平均降采样到 ~1.5km 粗层（≈120x119，对应 180km 场景）；
//! - 代价语义与 synthetic_cost_field 一致：海拔 >1500m 线性升高（≤20x，山区绕飞）；
//!   空洞块（有效占比 < 25%，Beijing 边缘 0=NoData 环带）→ 高代价墙 100x（保守禁行，
//!   避免穿越未知区域——方案保守侧优先）；
//! - 产出：真实代价场下 FMM 单次传播耗时（对比合成 128² 2.62ms）、100 随机源耗时、
//!   走廊质量（可达率/绕行比，验证空洞墙是否阻断连通）。

use criterion::{criterion_group, criterion_main, Criterion};
use phase0::fmm::{self, CostField};
use phase0::terrain::Terrain;
use rand::{RngExt, SeedableRng};
use std::hint::black_box;

const CELL_M: f64 = 1500.0; // 粗层格距（米）
const HOLE_THRESH: f64 = 0.25; // 块有效占比低于此 → 空洞墙
const HOLE_COST: f32 = 100.0; // 空洞墙代价
const MTN_THRESH: f64 = 1500.0; // 山区代价阈值（Beijing 最高 2273m）
const MTN_MAX_MULT: f64 = 20.0; // 山区最高代价倍数（同 synthetic 语义）

fn real_cost_field() -> CostField {
    let d = concat!(env!("CARGO_MANIFEST_DIR"), "/data/");
    let (t, load_s, load_mib) = Terrain::from_raw(
        &format!("{}beijing_dem_f32.raw", d),
        &format!("{}beijing_dem.meta", d),
    )
    .unwrap();
    println!(
        "\n[real-costfield] loaded full {}x{} ({:.2}s {:.1}MiB)",
        t.rows, t.cols, load_s, load_mib
    );
    let rows = (t.rows as f64 * t.cell_mx / CELL_M) as usize;
    let cols = (t.cols as f64 * t.cell_my / CELL_M) as usize;
    let bx = (CELL_M / t.cell_mx).round().max(1.0) as usize; // 行向块
    let by = (CELL_M / t.cell_my).round().max(1.0) as usize; // 列向块
    println!(
        "[real-costfield] coarse grid {}x{} (cell {:.0}m, block {}x{})",
        rows, cols, CELL_M, bx, by
    );
    let mut f = CostField::new(rows, cols);
    for r in 0..rows {
        for c in 0..cols {
            let r0 = r * bx;
            let c0 = c * by;
            let r1 = ((r + 1) * bx).min(t.rows);
            let c1 = ((c + 1) * by).min(t.cols);
            let mut sum = 0.0f64;
            let mut n = 0u32;
            for br in r0..r1 {
                for bc in c0..c1 {
                    let v = t.h[br * t.cols + bc] as f64;
                    if !v.is_nan() {
                        sum += v;
                        n += 1;
                    }
                }
            }
            let frac = n as f64 / ((r1 - r0) * (c1 - c0)) as f64;
            let cost: f32 = if frac < HOLE_THRESH {
                HOLE_COST // 空洞墙（保守禁行）
            } else {
                let h = sum / n as f64;
                let v = if h > MTN_THRESH {
                    1.0 + (h - MTN_THRESH) / 1000.0 * (MTN_MAX_MULT - 1.0)
                } else {
                    1.0
                };
                v as f32
            };
            let idx = f.idx(r, c);
            f.cost[idx] = cost;
        }
    }
    // 统计
    let (mut hole, mut mtn, mut flat) = (0usize, 0usize, 0usize);
    for &v in &f.cost {
        if v >= HOLE_COST {
            hole += 1;
        } else if v > 1.0 {
            mtn += 1;
        } else {
            flat += 1;
        }
    }
    println!(
        "[real-costfield] hole={} ({:.1}%) mtn={} ({:.1}%) flat={} ({:.1}%)",
        hole,
        100.0 * hole as f64 / f.cost.len() as f64,
        mtn,
        100.0 * mtn as f64 / f.cost.len() as f64,
        flat,
        100.0 * flat as f64 / f.cost.len() as f64
    );
    f
}

fn corridor_quality_stats(field: &CostField) {
    let mut rng = rand::rngs::StdRng::seed_from_u64(0xC0FFEE);
    let mut reachable = 0usize;
    let mut total_steps = 0f64;
    let mut total_ratio = 0f64;
    let mut worst_ratio = 0f64;
    for _ in 0..100 {
        let (sr, sc) = (rng.random_range(0..field.rows), rng.random_range(0..field.cols));
        let (dr, dc) = (rng.random_range(0..field.rows), rng.random_range(0..field.cols));
        // 避开空洞墙起止点
        if field.get(sr, sc) >= HOLE_COST || field.get(dr, dc) >= HOLE_COST {
            continue;
        }
        let res = fmm::fmm_propagate(field, sr, sc);
        if let Some(path) = fmm::backtrack_path(field, &res, dr, dc, sr, sc) {
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
        "\n[corridor-quality real] 100 random start->dst pairs (valid cells only)\n  reachable: {}/100\n  avg path steps: {:.1}\n  avg detour ratio: {:.3}\n  worst detour ratio: {:.3}",
        reachable,
        total_steps / reachable as f64,
        total_ratio / reachable as f64,
        worst_ratio
    );
}

fn criterion_benchmark(c: &mut Criterion) {
    // 2026-08-08 主管清理 phase0/data：beijing_dem* 已删除（数据可经
    // phase0/scripts/beijing_prep.py 再生成）→ 数据缺失时基准跳过，不 panic。
    if !std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/data/beijing_dem_f32.raw"))
        .exists()
    {
        eprintln!("[b7] SKIP: beijing_dem_f32.raw not found (cleaned 2026-08-08; regenerate via scripts/beijing_prep.py)");
        return;
    }
    let field = real_cost_field();
    corridor_quality_stats(&field);

    let (src_r, src_c) = (field.rows / 2, field.cols / 2);
    c.bench_function("fmm_real_terrain/single_propagation", |b| {
        b.iter(|| {
            let res =
                fmm::fmm_propagate(black_box(&field), black_box(src_r), black_box(src_c));
            black_box(res.accepted[field.idx(field.rows - 1, field.cols - 1)])
        })
    });

    let mut rng = rand::rngs::StdRng::seed_from_u64(0xABCDEF);
    let sources: Vec<(usize, usize)> = (0..100)
        .map(|_| (rng.random_range(0..field.rows), rng.random_range(0..field.cols)))
        .collect();
    c.bench_function("fmm_real_terrain/100_random_sources", |b| {
        b.iter(|| {
            let mut acc = 0u64;
            for &(sr, sc) in &sources {
                let res = fmm::fmm_propagate(black_box(&field), sr, sc);
                acc = acc.wrapping_add(
                    res.times[field.idx((sr + 1) % field.rows, (sc + 1) % field.cols)] as u64,
                );
            }
            black_box(acc)
        })
    });
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

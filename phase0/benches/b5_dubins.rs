//! B5 基准：细层运动基元拟合（S6）——Dubins CSC 基元。
//!
//! - 100km 走廊场景，100 组随机起止（位置+航向）；
//! - 转弯半径 R=5km（巡航 ~200m/s、坡度 45° 量级，A6 自洽参考）；
//! - 产出：单段求解耗时、拟合成功率（≥90% 判据）、平均单段长度。

use criterion::{criterion_group, criterion_main, Criterion};
use phase0::dubins::{dubins_path, dubins_shortest_len, dubins_success_rate};
use rand::{RngExt, SeedableRng};
use std::hint::black_box;

const SCENE_M: f64 = 100_000.0;
const TURN_R_M: f64 = 5_000.0;
const N_SAMPLES: usize = 100;

fn gen_samples(seed: u64) -> Vec<((f64, f64), f64, (f64, f64), f64)> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    (0..N_SAMPLES)
        .map(|_| {
            // 走廊内随机起止：间隔 ≥ 10km（保证 CSC 可解性有一定压力但不全失败）
            let (x0, y0) = (
                rng.random_range(5_000.0..SCENE_M - 5_000.0),
                rng.random_range(5_000.0..SCENE_M - 5_000.0),
            );
            let (x1, y1) = (
                rng.random_range(5_000.0..SCENE_M - 5_000.0),
                rng.random_range(5_000.0..SCENE_M - 5_000.0),
            );
            let (th0, th1) = (
                rng.random_range(0.0..std::f64::consts::TAU),
                rng.random_range(0.0..std::f64::consts::TAU),
            );
            ((x0, y0), th0, (x1, y1), th1)
        })
        .collect()
}

fn criterion_benchmark(c: &mut Criterion) {
    let samples = gen_samples(0x71);
    let r = TURN_R_M;

    // 单段求解耗时
    c.bench_function("b5_dubins/single_solve", |b| {
        let s = samples[0];
        b.iter(|| {
            let len = dubins_shortest_len(
                black_box(s.0),
                black_box(s.1),
                black_box(s.2),
                black_box(s.3),
                black_box(r),
            );
            black_box(len)
        })
    });
    c.bench_function("b5_dubins/100_solves", |b| {
        b.iter(|| {
            let mut acc = 0u64;
            for &((x0, y0), th0, (x1, y1), th1) in &samples {
                if let Some(len) = dubins_shortest_len((x0, y0), th0, (x1, y1), th1, r) {
                    acc = acc.wrapping_add(len as u64);
                }
            }
            black_box(acc)
        })
    });

    // 成功率统计（一次性打印）
    let (ok, total, avg_len) = dubins_success_rate(&samples, r);
    println!(
        "\n[fit-success] {}/{} ({:.1}%)  avg segment len {:.1} km  (R={} km)",
        ok,
        total,
        100.0 * ok as f64 / total as f64,
        avg_len / 1000.0,
        r / 1000.0
    );

    // 细分失败原因（d < 2R 距离不足 vs 其他）
    let mut close = 0usize;
    for &((x0, y0), th0, (x1, y1), th1) in &samples {
        let d = ((x1 - x0).powi(2) + (y1 - y0).powi(2)).sqrt();
        if d < 2.0 * r {
            close += 1;
        }
        let _ = dubins_path((x0, y0), th0, (x1, y1), th1, r);
    }
    println!("[fit-fail-analysis] samples with d<2R: {}/{}", close, N_SAMPLES);
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

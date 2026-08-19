//! B4 基准：射线-地形求交 / LOS 遮挡（S5）。
//!
//! - 100km 场景，合成地形全分辨率 152.87m 语义（655²）；
//! - 1000 条随机射线（飞行高度 3–10km，长度 60km），单射线 1000 采样点；
//! - LOD 对比：2x（305.7m）/ 4x（611.5m）网格 → 单射线耗时 + 遮挡判定一致性；
//! - 产出：单射线耗时、LOD/降采样策略取舍依据。

use criterion::{Criterion, criterion_group, criterion_main};
use phase0::terrain::{self, Terrain};
use rand::{RngExt, SeedableRng};
use std::hint::black_box;

const SCENE_M: f64 = 100_000.0;
const RAY_LEN_M: f64 = 60_000.0;
const N_SAMPLES: usize = 1000;
const N_RAYS: usize = 1000;

fn gen_rays(seed: u64) -> Vec<([f64; 3], [f64; 3])> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    (0..N_RAYS)
        .map(|_| {
            let ox = rng.random_range(5_000.0..SCENE_M - 5_000.0);
            let oy = rng.random_range(5_000.0..SCENE_M - 5_000.0);
            let oz = rng.random_range(3_000.0..10_000.0);
            let theta = rng.random_range(0.0..std::f64::consts::TAU);
            let (dx, dy) = (theta.cos(), theta.sin());
            ([ox, oy, oz], [dx, dy, 0.0])
        })
        .collect()
}

fn count_blocked(t: &Terrain, rays: &[([f64; 3], [f64; 3])]) -> usize {
    rays.iter()
        .filter(|(o, d)| t.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES))
        .count()
}

fn criterion_benchmark(c: &mut Criterion) {
    let rays = gen_rays(0x61);
    let cell_full = 152.87;
    let t_full = terrain::synthetic_terrain(
        (SCENE_M / cell_full) as usize,
        (SCENE_M / cell_full) as usize,
        cell_full,
        7,
    );
    let t_2x = terrain::synthetic_terrain(
        (SCENE_M / (cell_full * 2.0)) as usize,
        (SCENE_M / (cell_full * 2.0)) as usize,
        cell_full * 2.0,
        7,
    );
    let t_4x = terrain::synthetic_terrain(
        (SCENE_M / (cell_full * 4.0)) as usize,
        (SCENE_M / (cell_full * 4.0)) as usize,
        cell_full * 4.0,
        7,
    );

    println!(
        "\n[terrain] full {}x{} @{:.2}m ; 2x {}x{} ; 4x {}x{}",
        t_full.rows, t_full.cols, cell_full, t_2x.rows, t_2x.cols, t_4x.rows, t_4x.cols
    );

    c.bench_function("b4_ray_terrain/1000_rays_full_res_1000samples", |b| {
        b.iter(|| black_box(count_blocked(&t_full, &rays)))
    });
    c.bench_function("b4_ray_terrain/1000_rays_lod2x", |b| {
        b.iter(|| black_box(count_blocked(&t_2x, &rays)))
    });
    c.bench_function("b4_ray_terrain/1000_rays_lod4x", |b| {
        b.iter(|| black_box(count_blocked(&t_4x, &rays)))
    });

    // 遮挡一致性（全分辨率为基准）
    let full = count_blocked(&t_full, &rays);
    let lod2 = count_blocked(&t_2x, &rays);
    let lod4 = count_blocked(&t_4x, &rays);
    let agree2 = rays
        .iter()
        .filter(|(o, d)| {
            t_full.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
                == t_2x.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
        })
        .count();
    let agree4 = rays
        .iter()
        .filter(|(o, d)| {
            t_full.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
                == t_4x.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
        })
        .count();
    println!(
        "\n[LOS consistency] full blocked={} ; lod2x blocked={} agree={:.1}% ; lod4x blocked={} agree={:.1}%",
        full,
        lod2,
        100.0 * agree2 as f64 / N_RAYS as f64,
        lod4,
        100.0 * agree4 as f64 / N_RAYS as f64
    );
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

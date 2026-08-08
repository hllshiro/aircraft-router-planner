//! B6 基准：真实 DEM（Beijing_DEM 测试数据）射线-地形求交 / LOS（S5 真实双跑）。
//!
//! - 数据：`phase0/data/beijing_dem_f32{.raw|.2x.raw|.4x.raw}`（Float32，0 空洞 → NaN，
//!   全量 4712x6091 @ 38.09x29.28m，有效 51.3%），元数据 meta 文本；
//! - 加载性能：三档 raw 的加载耗时 + 内存占用（from_raw）；
//! - 射线：仅有效区域起点（height_at 非 None），长度 60km，1000 采样点，
//!   飞行高度 3–10km，1000 条；
//! - 产出：真实数据下单射线耗时（对比合成 655² ≈ 13.4µs）、LOD 2x/4x 耗时、
//!   full vs LOD 遮挡一致性、空洞（NaN → 不遮挡）对 blocked 率的影响。

use criterion::{criterion_group, criterion_main, Criterion};
use phase0::terrain::{self, Terrain};
use rand::{RngExt, SeedableRng};
use std::hint::black_box;

const RAY_LEN_M: f64 = 60_000.0;
const N_SAMPLES: usize = 1000;
const N_RAYS: usize = 1000;

fn load() -> (Terrain, Terrain, Terrain, (f64, f64), (f64, f64), (f64, f64)) {
    let d = concat!(env!("CARGO_MANIFEST_DIR"), "/data/");
    let (tf, lf, mf) =
        Terrain::from_raw(&format!("{}beijing_dem_f32.raw", d), &format!("{}beijing_dem.meta", d))
            .unwrap();
    let (t2, l2, m2) = Terrain::from_raw(
        &format!("{}beijing_dem_f32.2x.raw", d),
        &format!("{}beijing_dem.2x.meta", d),
    )
    .unwrap();
    let (t4, l4, m4) = Terrain::from_raw(
        &format!("{}beijing_dem_f32.4x.raw", d),
        &format!("{}beijing_dem.4x.meta", d),
    )
    .unwrap();
    (tf, t2, t4, (lf, mf), (l2, m2), (l4, m4))
}

/// 有效区域随机起点（height_at 非 None，重试最多 200 次）。
/// 高度：相对地面 rel_h ∈ [500, 3000]m 的低空水平射线（贴合 LOS 语义——
/// Beijing 最高 2273m，绝对高度 3-10km 永不被挡，故必须相对地形）。
fn gen_rays(seed: u64, t: &Terrain) -> Vec<([f64; 3], [f64; 3])> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let mut rays = Vec::with_capacity(N_RAYS);
    let (xmax, ymax) = (t.rows as f64 * t.cell_mx, t.cols as f64 * t.cell_my);
    let mut attempts = 0u32;
    while rays.len() < N_RAYS && attempts < 200 * N_RAYS as u32 {
        attempts += 1;
        let ox = rng.random_range(1_000.0..xmax - 1_000.0);
        let oy = rng.random_range(1_000.0..ymax - 1_000.0);
        let Some(ground) = t.height_at(ox, oy) else {
            continue; // 空洞/无效区域，重试
        };
        let oz = ground + rng.random_range(500.0..3_000.0);
        let theta = rng.random_range(0.0..std::f64::consts::TAU);
        rays.push(([ox, oy, oz], [theta.cos(), theta.sin(), 0.0]));
    }
    assert_eq!(rays.len(), N_RAYS, "有效起点采样失败（空洞过多？）");
    rays
}

fn count_blocked(t: &Terrain, rays: &[([f64; 3], [f64; 3])]) -> usize {
    rays.iter()
        .filter(|(o, d)| {
            t.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
        })
        .count()
}

fn criterion_benchmark(c: &mut Criterion) {
    // 2026-08-08 主管清理 phase0/data：beijing_dem* 已删除（数据可经
    // phase0/scripts/beijing_prep.py 再生成）→ 数据缺失时基准跳过，不 panic。
    if !std::path::Path::new(concat!(env!("CARGO_MANIFEST_DIR"), "/data/beijing_dem_f32.raw"))
        .exists()
    {
        eprintln!("[b6] SKIP: beijing_dem_f32.raw not found (cleaned 2026-08-08; regenerate via scripts/beijing_prep.py)");
        return;
    }
    let (tf, t2, t4, (lf, mf), (l2, m2), (l4, m4)) = load();
    let rays = gen_rays(0x61, &tf);
    println!(
        "\n[load] full {}x{} @{:.2}x{:.2}m {:.1}s {:.1}MiB ; 2x {}x{} {:.1}s {:.1}MiB ; 4x {}x{} {:.1}s {:.1}MiB",
        tf.rows, tf.cols, tf.cell_mx, tf.cell_my, lf, mf, t2.rows, t2.cols, l2, m2, t4.rows,
        t4.cols, l4, m4
    );

    c.bench_function("b6_real_dem/1000_rays_full_res", |b| {
        b.iter(|| black_box(count_blocked(&tf, &rays)))
    });
    c.bench_function("b6_real_dem/1000_rays_lod2x", |b| {
        b.iter(|| black_box(count_blocked(&t2, &rays)))
    });
    c.bench_function("b6_real_dem/1000_rays_lod4x", |b| {
        b.iter(|| black_box(count_blocked(&t4, &rays)))
    });

    // 遮挡一致性（全分辨率为基准）
    let full = count_blocked(&tf, &rays);
    let lod2 = count_blocked(&t2, &rays);
    let lod4 = count_blocked(&t4, &rays);
    let agree2 = rays
        .iter()
        .filter(|(o, d)| {
            tf.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
                == t2.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
        })
        .count();
    let agree4 = rays
        .iter()
        .filter(|(o, d)| {
            tf.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
                == t4.ray_blocked(o[0], o[1], o[2], d[0], d[1], d[2], RAY_LEN_M, N_SAMPLES)
        })
        .count();
    // 无空洞合成对照：全有效 655² @152.87m
    let ts = terrain::synthetic_terrain(655, 655, 152.87, 7);
    let syn = count_blocked(&ts, &rays);
    println!(
        "\n[LOS real vs synthetic] full blocked={} ; lod2x={} agree={:.1}% ; lod4x={} agree={:.1}% ; synthetic655 full-valid={}",
        full, lod2, 100.0 * agree2 as f64 / N_RAYS as f64, lod4,
        100.0 * agree4 as f64 / N_RAYS as f64, syn
    );
}

criterion_group!(benches, criterion_benchmark);
criterion_main!(benches);

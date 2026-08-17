//! t_load_decompress 基准（技术方案 5.1：内置格式加载 ≤300ms 目标，落账 docs/10 §8 标定值）。
//!
//! 场景：ARPK1 raw 块包 1024×1024（≈4 块）→ parse（magic/版本/SHA-256/索引校验）
//! + 热采样 1000 次（按需解压缓存）。目标：parse+校验+首次采样 < 300ms。

use std::hint::black_box;

use aircraft_router_planner_cli::terrain::builtin::{write_pack_raw, BuiltinSource};
use aircraft_router_planner_cli::terrain::TerrainSource;
use criterion::{criterion_group, criterion_main, Criterion};

fn bench_load_and_sample(c: &mut Criterion) {
    let rows = 1024usize;
    let cols = 1024usize;
    let mut h = vec![500i16; rows * cols];
    // 少量起伏 + 一个空洞
    for r in 0..rows {
        for cc in 0..cols {
            if (r + cc) % 97 == 0 {
                h[r * cols + cc] = 1200;
            }
        }
    }
    h[rows * cols - 1] = -32768;
    let bytes = write_pack_raw(
        rows,
        cols,
        116.0,
        39.0,
        0.001,
        0.001,
        50.0,
        true,
        -32768,
        "bench fixture",
        &h,
    );

    c.bench_function("t_load_decompress_parse_1024", |b| {
        b.iter(|| {
            let s = BuiltinSource::parse(black_box(&bytes)).expect("parse ok");
            black_box(s.data_version());
        })
    });

    c.bench_function("t_load_decompress_sample_1000", |b| {
        let s = BuiltinSource::parse(&bytes).unwrap();
        b.iter(|| {
            let mut sum = 0.0f64;
            for i in 0..1000 {
                let lon = 116.0 + (i % 100) as f64 * 0.005;
                let lat = 39.0 + (i / 100) as f64 * 0.005;
                sum += s.height_at(lon, lat).unwrap_or(0.0);
            }
            black_box(sum);
        })
    });
}

criterion_group!(benches, bench_load_and_sample);
criterion_main!(benches);

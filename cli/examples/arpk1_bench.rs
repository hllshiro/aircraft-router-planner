//! 开发期基准：真实 ARPK1 数据（GMTED2010 全球 / China_Dem_L12）加载与采样性能
//! （Phase 0-2 待数据项：加载预算 ≤300ms、LOS 预计算耗时、空洞分布统计）。
//! 用法: cargo run --release --example arpk1_bench -- <gmted.arpack> <china.arpack> [mask.mask]

use aircraft_router_planner_cli::terrain::builtin::BuiltinSource;
use aircraft_router_planner_cli::terrain::mask::{GeoMask, MaskedSource};
use aircraft_router_planner_cli::terrain::{Sample, TerrainSource};
use rand::{RngExt, SeedableRng};
use std::hint::black_box;
use std::time::Instant;

fn bench_source(src: &BuiltinSource, n_sample: usize, seed: u64, label: &str) {
    // 打开计时（含 read + SHA-256 全量校验）
    let t0 = Instant::now();
    let open_ms = t0.elapsed().as_secs_f64() * 1000.0;
    let b = src.bounds().unwrap();
    println!(
        "\n[{label}] open {open_ms:.1}ms | {} | bounds lon {:.2}..{:.2} lat {:.2}..{:.2}",
        src.resolution_desc(),
        b.min_lon,
        b.max_lon,
        b.min_lat,
        b.max_lat
    );

    // 随机采样（bounds 内均匀；height_at）
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let pts: Vec<(f64, f64)> = (0..n_sample)
        .map(|_| {
            let lon = rng.random_range(b.min_lon..b.max_lon);
            let lat = rng.random_range(b.min_lat..b.max_lat);
            (lon, lat)
        })
        .collect();
    let t1 = Instant::now();
    let mut hits = 0usize;
    let mut sum = 0.0f64;
    for &(lon, lat) in &pts {
        if let Some(h) = src.height_at(lon, lat) {
            hits += 1;
            sum += h;
        }
    }
    let h_ms = t1.elapsed().as_secs_f64() * 1000.0;
    println!(
        "[{label}] height_at x{n_sample}: {h_ms:.2}ms ({:.2}us/op) valid {:.1}% mean {:.0}m",
        h_ms * 1000.0 / n_sample as f64,
        100.0 * hits as f64 / n_sample as f64,
        sum / hits.max(1) as f64
    );

    // 语义采样分布
    let t2 = Instant::now();
    let mut land = 0usize;
    let mut water = 0usize;
    let mut nodata = 0usize;
    let mut oob = 0usize;
    for &(lon, lat) in &pts {
        match src.sample_at(lon, lat) {
            Sample::Land(_) => land += 1,
            Sample::Water | Sample::Lake(_) => water += 1,
            Sample::NoData => nodata += 1,
            Sample::OutOfBounds => oob += 1,
            Sample::Forbidden => {} // 地形源不产生硬墙（示例不统计）
        }
    }
    let s_ms = t2.elapsed().as_secs_f64() * 1000.0;
    println!(
        "[{label}] sample_at x{n_sample}: {s_ms:.2}ms | Land {:.1}% Water/Lake {:.1}% NoData {:.1}% OOB {:.1}%",
        100.0 * land as f64 / n_sample as f64,
        100.0 * water as f64 / n_sample as f64,
        100.0 * nodata as f64 / n_sample as f64,
        100.0 * oob as f64 / n_sample as f64
    );
    black_box(sum);
}

fn bench_masked(gmted: &str, mask: &str, n_sample: usize, seed: u64) {
    let t0 = Instant::now();
    let inner = BuiltinSource::open(std::path::Path::new(gmted)).expect("gmted open failed");
    let gm = GeoMask::open(std::path::Path::new(mask)).expect("mask open failed");
    let mask_desc = format!("mask {} {}", gm.version(), gm.resolution_desc());
    let msrc = MaskedSource::new(inner, gm);
    let open_ms = t0.elapsed().as_secs_f64() * 1000.0;
    println!("\n[masked] open(inner+mask) {open_ms:.1}ms | {mask_desc}");

    // 语义采样分布（掩膜 3 态 → Land/Water/Lake）
    let b = msrc.bounds().unwrap();
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let mut land = 0usize;
    let mut water = 0usize;
    let mut lake = 0usize;
    let mut nodata = 0usize;
    let mut oob = 0usize;
    let t1 = Instant::now();
    for _ in 0..n_sample {
        let lon = rng.random_range(b.min_lon..b.max_lon);
        let lat = rng.random_range(b.min_lat..b.max_lat);
        match msrc.sample_at(lon, lat) {
            Sample::Land(_) => land += 1,
            Sample::Water => water += 1,
            Sample::Lake(_) => lake += 1,
            Sample::NoData => nodata += 1,
            Sample::OutOfBounds => oob += 1,
            Sample::Forbidden => {} // 地形源不产生硬墙（示例不统计）
        }
    }
    let s_ms = t1.elapsed().as_secs_f64() * 1000.0;
    println!(
        "[masked] sample_at x{n_sample}: {s_ms:.2}ms | Land {:.1}% Water {:.1}% Lake {:.1}% NoData {:.1}% OOB {:.1}%",
        100.0 * land as f64 / n_sample as f64,
        100.0 * water as f64 / n_sample as f64,
        100.0 * lake as f64 / n_sample as f64,
        100.0 * nodata as f64 / n_sample as f64,
        100.0 * oob as f64 / n_sample as f64
    );
    // 定点语义抽查
    for (lon, lat, name) in [
        (116.4, 39.9, "Beijing"),
        (150.0, 0.0, "Pacific"),
        (100.0, 37.0, "Qinghai Lake"),
        (-70.0, -55.0, "S. Ocean"),
    ] {
        println!(
            "[masked] {name} ({lon},{lat}) = {:?}",
            msrc.sample_at(lon, lat)
        );
    }
}

/// 本地等距近似前进（几何口径 4.2.3）：起点 + 方位角 + 距离 → 经纬度。
fn destination(lon0: f64, lat0: f64, bearing_deg: f64, dist_m: f64) -> (f64, f64) {
    let k = lat0.to_radians().cos().max(1e-6);
    let dx = dist_m * bearing_deg.to_radians().sin();
    let dy = dist_m * bearing_deg.to_radians().cos();
    (lon0 + dx / (111_320.0 * k), lat0 + dy / 111_320.0)
}

/// 真实路径局部采样（模拟路径规划访问模式：沿线每 60m 一点，强局部性）。
/// 从起点沿大圆方向前进 len_m，每 step_m 采样 height_at；统计单点耗时与缓存命中收益。
fn bench_path_sampling(
    src: &BuiltinSource,
    lon0: f64,
    lat0: f64,
    bearing_deg: f64,
    len_m: f64,
    step_m: f64,
) {
    let n = (len_m / step_m) as usize;
    let t0 = Instant::now();
    let mut hits = 0usize;
    for i in 0..n {
        let (lon, lat) = destination(lon0, lat0, bearing_deg, i as f64 * step_m);
        if src.height_at(lon, lat).is_some() {
            hits += 1;
        }
    }
    let ms = t0.elapsed().as_secs_f64() * 1000.0;
    println!(
        "[path] {len_m:.0}m @{step_m:.0}m step: {n} pts in {ms:.2}ms ({:.2}us/pt) valid {:.1}%",
        ms * 1000.0 / n as f64,
        100.0 * hits as f64 / n as f64
    );
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let (gmted, china) = (args[0].as_str(), args[1].as_str());
    let mask = args.get(2).map(|s| s.as_str());
    let g = BuiltinSource::open(std::path::Path::new(gmted)).unwrap();
    let c = BuiltinSource::open(std::path::Path::new(china)).unwrap();
    bench_source(&g, 100_000, 0x1234, "GMTED2010");
    bench_source(&c, 100_000, 0x5678, "China");
    // 真实路径：北京出发 300km 多方向（跨块连续采样）
    for b in [0.0f64, 45.0, 90.0, 135.0, 180.0, 225.0, 270.0, 315.0] {
        bench_path_sampling(&g, 116.4, 39.9, b, 300_000.0, 60.0);
    }
    for b in [0.0f64, 90.0, 180.0, 270.0] {
        bench_path_sampling(&c, 116.4, 39.9, b, 300_000.0, 60.0);
    }
    if let Some(m) = mask {
        bench_masked(gmted, m, 100_000, 0x9abc);
    }
}

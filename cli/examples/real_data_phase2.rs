//! 开发期基准：Phase 0-2 待数据项真实数据标定（Stage B）。
//! - 语义退化统计（semantic_degradation_ratios）：GMTED2010 / China / 北京区域；
//! - LOS 遮挡统计（los_blocked，低空相对射线）：GMTED2010 7.5as vs China 9.888as
//!   分辨率对 blocked 率的影响（LOS mask 系数 0.05-0.1 定值依据）；
//! - 代价场 FMM（build_semantic_cost_field + fmm_propagate）：
//!   北京全有效区 / China 空洞区（NODATA 5x）→ 可达率 + 耗时。
//! 用法: cargo run --release --example real_data_phase2 -- <gmted.arpack> <china.arpack>

use aircraft_router_planner_cli::costfield::{backtrack_path, build_semantic_cost_field, fmm_propagate};
use aircraft_router_planner_cli::terrain::builtin::BuiltinSource;
use aircraft_router_planner_cli::terrain::{los_blocked, semantic_degradation_ratios, Sample, TerrainSource};
use rand::{RngExt, SeedableRng};
use std::hint::black_box;
use std::time::Instant;

const RAY_LEN_KM: f64 = 60.0;
const N_RAYS: usize = 500;
const N_SAMPLES: usize = 1000;

/// 低空相对射线集合（北京有效区起点，ground+[500,3000]m，水平方向随机）。
fn gen_low_rays<T: TerrainSource + ?Sized>(src: &T, lon_c: f64, lat_c: f64, half_deg: f64, seed: u64) -> Vec<([f64; 3], [f64; 3])> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let mut rays = Vec::with_capacity(N_RAYS);
    let mut attempts = 0u32;
    while rays.len() < N_RAYS && attempts < 200 * N_RAYS as u32 {
        attempts += 1;
        let lon = lon_c + rng.random_range(-half_deg..half_deg);
        let lat = lat_c + rng.random_range(-half_deg..half_deg);
        let Some(ground) = src.height_at(lon, lat) else { continue };
        let oz = ground + rng.random_range(500.0..3_000.0);
        let theta = rng.random_range(0.0..std::f64::consts::TAU);
        rays.push(([lon, lat, oz], [theta.cos(), theta.sin(), 0.0]));
    }
    assert_eq!(rays.len(), N_RAYS, "有效起点不足（空洞过多？）");
    rays
}

fn count_blocked<T: TerrainSource + ?Sized>(src: &T, rays: &[([f64; 3], [f64; 3])]) -> usize {
    rays.iter()
        .filter(|(o, d)| {
            // 60km ≈ 0.54 度（纬度向）；方向为经度单位，长度用经度折算（近似）
            let len_deg = RAY_LEN_KM * 1000.0 / 111_320.0;
            los_blocked(src, o[0], o[1], o[2], d[0], d[1], d[2], len_deg, N_SAMPLES)
        })
        .count()
}

/// 区域代价场 + FMM 可达率。
fn bench_region<T: TerrainSource>(src: &T, label: &str, lon_c: f64, lat_c: f64, half_deg: f64, grid: usize, nodata_mult: f32) {
    let mut land = 0usize;
    let mut water = 0usize;
    let mut nodata = 0usize;
    let mut oob = 0usize;
    let t0 = Instant::now();
    let field = build_semantic_cost_field(grid, grid, |r, c| {
        let u = (c as f64 + 0.5) / grid as f64;
        let v = (r as f64 + 0.5) / grid as f64;
        let lon = lon_c - half_deg + 2.0 * half_deg * u;
        let lat = lat_c - half_deg + 2.0 * half_deg * v;
        let s = src.sample_at(lon, lat);
        match s {
            Sample::Land(_) => land += 1,
            Sample::Water | Sample::Lake(_) => water += 1,
            Sample::NoData => nodata += 1,
            Sample::OutOfBounds => oob += 1,
        }
        s
    }, nodata_mult);
    let build_ms = t0.elapsed().as_secs_f64() * 1000.0;
    // 单源 FMM（中心出发）
    let t1 = Instant::now();
    let res = fmm_propagate(&field, grid / 2, grid / 2);
    let fmm_ms = t1.elapsed().as_secs_f64() * 1000.0;
    let reachable = res.times.iter().filter(|t| t.is_finite()).count();
    // 回溯（中心 → 角点）可达性
    let t2 = Instant::now();
    let mut bt = 0usize;
    let (sr, sc) = (grid / 2, grid / 2);
    for &(dr, dc) in &[(0usize, 0usize), (0, grid - 1), (grid - 1, 0), (grid - 1, grid - 1)] {
        if backtrack_path(&field, &res, dr, dc, sr, sc).is_some() {
            bt += 1;
        }
    }
    let bt_ms = t2.elapsed().as_secs_f64() * 1000.0;
    let (land_pct, nodata_pct, oob_pct, reach_pct) = (
        100.0 * land as f64 / (grid * grid) as f64,
        100.0 * nodata as f64 / (grid * grid) as f64,
        100.0 * oob as f64 / (grid * grid) as f64,
        100.0 * reachable as f64 / (grid * grid) as f64,
    );
    println!(
        "[{label}] {grid}x{grid} build {build_ms:.1}ms | Land {land_pct:.0}% NoData {nodata_pct:.0}% OOB {oob_pct:.0}% | FMM {fmm_ms:.1}ms reach {reach_pct:.0}% | backtrack 4角 {bt}/4 ({bt_ms:.1}ms)"
    );
    black_box(&field);
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let g = BuiltinSource::open(std::path::Path::new(&args[0])).unwrap();
    let c = BuiltinSource::open(std::path::Path::new(&args[1])).unwrap();

    // 1) 语义退化统计
    for (src, name) in [(&g as &dyn TerrainSource, "GMTED2010"), (&c, "China")] {
        let (nd, oob) = semantic_degradation_ratios(src, 256);
        println!("[degrad] {name}: NoData {:.1}% OOB {:.1}%", nd * 100.0, oob * 100.0);
    }
    // 北京 1°x1° 区域（两数据都应全 Land）
    let (nd, _) = semantic_degradation_ratios(&BeijingSrc(&g), 256);
    println!("[degrad] Beijing@GMTED2010 1deg: NoData {:.1}%", nd * 100.0);
    let (nd, _) = semantic_degradation_ratios(&BeijingSrc(&c), 256);
    println!("[degrad] Beijing@China 1deg: NoData {:.1}%", nd * 100.0);

    // 2) LOS 遮挡统计（低空相对射线，北京区域）
    for (src, name) in [(&g as &dyn TerrainSource, "GMTED2010"), (&c, "China")] {
        let rays = gen_low_rays(src, 116.4, 39.9, 0.5, 0xbeef);
        let t0 = Instant::now();
        let blocked = count_blocked(src, &rays);
        let ms = t0.elapsed().as_secs_f64() * 1000.0;
        println!(
            "[LOS] {name} Beijing: blocked {:.1}% ({ms:.1}ms for {N_RAYS} rays, {:.1}us/ray)",
            blocked as f64 * 100.0 / N_RAYS as f64,
            ms * 1000.0 / N_RAYS as f64
        );
    }

    // 3) 代价场 + FMM
    // 北京全有效区（1°x1°，256² ≈ 430m/格）
    bench_region(&g, "Beijing@GMTED2010", 116.4, 39.9, 0.5, 256, 5.0);
    bench_region(&c, "Beijing@China", 116.4, 39.9, 0.5, 256, 5.0);
    // China 空洞区（境外/海洋：南海、云南边境、藏西境外）
    bench_region(&c, "Hole@SouthChinaSea", 113.0, 17.0, 0.5, 256, 5.0);
    bench_region(&c, "Hole@YunnanBorder", 97.0, 22.0, 0.5, 256, 5.0);
    bench_region(&c, "Hole@W.Tibet", 80.0, 32.0, 0.5, 256, 5.0);
    // NODATA 5x 敏感性：南海空洞区 1x vs 5x vs INF（禁行）——影响路径可用性
    for mult in [1.0f32, 5.0, f32::INFINITY] {
        bench_region(&c, &format!("SouthChinaSea mult={mult}"), 113.0, 17.0, 0.5, 256, mult);
    }
}

/// 把源裁剪到北京 1°x1°（bounds 包装，供 degradation 统计用）。
struct BeijingSrc<'a, T: TerrainSource>(&'a T);
impl<T: TerrainSource> TerrainSource for BeijingSrc<'_, T> {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        if (115.9..=116.9).contains(&lon) && (39.4..=40.4).contains(&lat) {
            self.0.height_at(lon, lat)
        } else {
            None
        }
    }
    fn bounds(&self) -> Option<aircraft_router_planner_cli::terrain::GeoBounds> {
        Some(aircraft_router_planner_cli::terrain::GeoBounds {
            min_lon: 115.9,
            min_lat: 39.4,
            max_lon: 116.9,
            max_lat: 40.4,
        })
    }
    fn resolution_desc(&self) -> String {
        format!("beijing-window({})", self.0.resolution_desc())
    }
    fn sample_at(&self, lon: f64, lat: f64) -> Sample {
        if !self.bounds().unwrap().contains(lon, lat) {
            return Sample::OutOfBounds;
        }
        self.0.sample_at(lon, lat)
    }
}

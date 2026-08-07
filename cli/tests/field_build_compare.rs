//! 对比验证：粗层代价场构建——串行（现状） vs 并行（候选优化，2026-08-07）。
//!
//! 背景：zigzag11 实测 field_build 1266ms 占单次规划 80%（FMM 247ms / 平滑 65ms）。
//! 主成本是 ARPACK zstd 块解压（400 块 × 1-3ms 串行）。并行构建让不同线程并发
//! 解压不同块（解压在锁外，BuiltinSource 块缓存 Mutex 天然线程安全）。
//!
//! 验证目标：
//! 1. **正确性（硬断言）**：并行构建产物与串行逐位一致（bit-exact，f32 Vec 全等）
//! 2. **性能（打印对比）**：真实 china_dem_l12.arpack 上 1024² 网格串行 vs 并行耗时
//!    （多次取 min；不硬断言——CI 单核/波动容忍）
//!
//! 数据：优先真实 china_dem_l12.arpack（zigzag11 同款），缺失则合成 ARPK1 数据
//! （write_pack_raw）只验正确性。

use std::path::Path;
use std::time::Instant;

use aircraft_router_planner_cli::costfield::{build_semantic_cost_field, build_semantic_cost_field_par, CostField};
use aircraft_router_planner_cli::terrain::builtin::{write_pack_raw, BuiltinSource};
use aircraft_router_planner_cli::terrain::{Sample, TerrainSource};

/// zigzag11 区域（同 solver::region_of：start/target 包围盒 + 0.15° 缓冲，方形 span）。
fn zigzag11_region() -> (f64, f64, f64) {
    let start_lon: f64 = 124.08488491093874;
    let start_lat: f64 = 34.66470606380233;
    let target_lon: f64 = 111.25990351543102;
    let target_lat: f64 = 43.82628474871892;
    let min_lon = start_lon.min(target_lon) - 0.15;
    let min_lat = start_lat.min(target_lat) - 0.15;
    let span = (start_lon.max(target_lon) - min_lon).max(start_lat.max(target_lat) - min_lat) + 0.15;
    (min_lon, min_lat, span)
}

/// 网格点采样闭包（同 solver::cell_lonlat + build_semantic_cost_field 闭包）。
fn make_sampler<'a>(
    t: Option<&'a dyn TerrainSource>,
    min_lon: f64,
    min_lat: f64,
    span: f64,
    grid: usize,
) -> impl Fn(usize, usize) -> Sample + Sync + Send + 'a {
    move |r, c| {
        let u = (c as f64 + 0.5) / grid as f64;
        let v = (r as f64 + 0.5) / grid as f64;
        let lon = min_lon + u * span;
        let lat = min_lat + v * span;
        match t {
            Some(t) => t.sample_at(lon, lat),
            None => Sample::Land(0.0),
        }
    }
}

/// 真实数据（存在则返回，否则 None）。
fn load_real() -> Option<BuiltinSource> {
    let p = Path::new(env!("CARGO_MANIFEST_DIR"))
        .join("..")
        .join("phase0/data/pending/china_dem_l12.arpack");
    if p.exists() {
        BuiltinSource::open(&p).ok()
    } else {
        None
    }
}

/// 合成 ARPK1 数据（512×512，cell 0.01°，正弦地形 + NoData 空洞；单块 raw）。
fn synth_source() -> BuiltinSource {
    let (rows, cols) = (512usize, 512usize);
    let mut h = vec![0i16; rows * cols];
    for r in 0..rows {
        for c in 0..cols {
            let i = r * cols + c;
            let x = r as f64 * 0.01;
            let y = c as f64 * 0.01;
            // 空洞：以 (0.5, 0.5) 为中心 0.08° 半径圆内 = NoData
            let dr = x - 0.5;
            let dc = y - 0.5;
            if dr * dr + dc * dc < 0.08 * 0.08 {
                h[i] = -32768; // no_data 哨兵
            } else {
                h[i] = (1000.0 + 300.0 * (x * 6.0).sin() * (y * 5.0).cos()) as i16;
            }
        }
    }
    let bytes = write_pack_raw(
        rows,
        cols,
        0.0,
        0.0,
        0.01,
        0.01,
        1.0,
        true,
        -32768,
        "synthetic-bench",
        &h,
    );
    BuiltinSource::parse(&bytes).expect("synth arpack parse")
}

fn build_serial<F: Fn(usize, usize) -> Sample>(s: &F, grid: usize) -> CostField {
    let t = Instant::now();
    let f = build_semantic_cost_field(grid, grid, s, 5.0);
    eprintln!("[compare] serial {grid}x{grid}: {:.1}ms", t.elapsed().as_secs_f64() * 1000.0);
    f
}

fn build_parallel<F: Fn(usize, usize) -> Sample + Sync + Send>(s: &F, grid: usize) -> CostField {
    let t = Instant::now();
    let f = build_semantic_cost_field_par(grid, grid, s, 5.0);
    eprintln!("[compare] parallel {grid}x{grid}: {:.1}ms", t.elapsed().as_secs_f64() * 1000.0);
    f
}

/// 核心对比：同闭包、同区域，串行/并行产物逐位一致。
fn assert_bit_exact(grid: usize, t: Option<&dyn TerrainSource>) {
    let (min_lon, min_lat, span) = zigzag11_region();
    let s = make_sampler(t, min_lon, min_lat, span, grid);
    let serial = build_serial(&s, grid);
    let parallel = build_parallel(&s, grid);
    assert_eq!(
        serial.cost.len(),
        parallel.cost.len(),
        "size mismatch: {} vs {}",
        serial.cost.len(),
        parallel.cost.len()
    );
    for i in 0..serial.cost.len() {
        assert_eq!(
            serial.cost[i].to_bits(),
            parallel.cost[i].to_bits(),
            "bit-exact mismatch at idx {i}: serial={} parallel={}",
            serial.cost[i],
            parallel.cost[i]
        );
    }
    eprintln!(
        "[compare] OK: {grid}x{grid} = {} cells bit-exact identical",
        serial.cost.len()
    );
}

#[test]
fn field_build_parallel_bit_exact_synthetic() {
    // 合成数据（CI 无真实文件也可跑）：正确性硬断言，性能仅打印。
    let src = synth_source();
    assert_bit_exact(128, Some(&src));
}

#[test]
fn field_build_parallel_real_data() {
    // 真实 china_dem_l12.arpack：1024² 正确性 + 性能对比（文件缺失则跳过）。
    let Some(src) = load_real() else {
        eprintln!("[compare] SKIP: china_dem_l12.arpack not found (CI 环境跳过性能对比)");
        return;
    };
    eprintln!(
        "[compare] terrain: {}",
        src.resolution_desc()
    );
    let t: &dyn TerrainSource = &src;
    assert_bit_exact(1024, Some(t));
    // 性能公平对比：真实规划每次新实例（冷缓存，zstd 解压主导）。
    // 每轮交替 A/B 各用新解析实例（cache 空），取各自 min——消除预热/顺序偏差。
    let (min_lon, min_lat, span) = zigzag11_region();
    let bytes = std::fs::read(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("phase0/data/pending/china_dem_l12.arpack"),
    )
    .expect("re-read real arpack");
    let mut p_best = f64::INFINITY;
    let mut s_best = f64::INFINITY;
    for round in 0..3 {
        if round % 2 == 0 {
            // A: parallel（新实例冷缓存）
            let src_p = BuiltinSource::parse(&bytes).expect("parse");
            let s = make_sampler(Some(&src_p), min_lon, min_lat, span, 1024);
            let tt = Instant::now();
            let _ = build_semantic_cost_field_par(1024, 1024, &s, 5.0);
            p_best = p_best.min(tt.elapsed().as_secs_f64() * 1000.0);
            // B: serial（新实例冷缓存）
            let src_s = BuiltinSource::parse(&bytes).expect("parse");
            let s = make_sampler(Some(&src_s), min_lon, min_lat, span, 1024);
            let tt = Instant::now();
            let _ = build_semantic_cost_field(1024, 1024, &s, 5.0);
            s_best = s_best.min(tt.elapsed().as_secs_f64() * 1000.0);
        } else {
            let src_s = BuiltinSource::parse(&bytes).expect("parse");
            let s = make_sampler(Some(&src_s), min_lon, min_lat, span, 1024);
            let tt = Instant::now();
            let _ = build_semantic_cost_field(1024, 1024, &s, 5.0);
            s_best = s_best.min(tt.elapsed().as_secs_f64() * 1000.0);
            let src_p = BuiltinSource::parse(&bytes).expect("parse");
            let s = make_sampler(Some(&src_p), min_lon, min_lat, span, 1024);
            let tt = Instant::now();
            let _ = build_semantic_cost_field_par(1024, 1024, &s, 5.0);
            p_best = p_best.min(tt.elapsed().as_secs_f64() * 1000.0);
        }
    }
    let speedup = s_best / p_best;
    eprintln!(
        "[compare] real 1024x1024 cold-cache best: serial={:.1}ms parallel={:.1}ms speedup={:.2}x",
        s_best, p_best, speedup
    );
    // 不硬断言性能（CI 单核/负载波动）；但提示明显退化
    assert!(
        p_best <= s_best * 1.5,
        "parallel suspiciously slower: serial {s_best:.1}ms vs parallel {p_best:.1}ms"
    );
}

//! 对比验证：粗层代价场构建——串行（现状） vs 并行（候选①） vs 无锁预取（候选②）。
//!
//! 背景：zigzag11 实测 field_build 1266ms 占单次规划 80%（FMM 247ms / 平滑 65ms）。
//! 主成本 = zstd 块解压（~354 块）+ 4M 次 Mutex 锁内 cell 查表。
//! - 候选① 并行化（build_semantic_cost_field_par）：bit-exact ✓ 但性能 0.97× 无收益
//!   （锁竞争抵消解压并行，commit 9504381 结论）
//! - 候选② 无锁批量预取（BulkPrefetch::prefetch_lonlat + sample_local）：一次锁外解压
//!   区域全部块到局部表，4M 次查表无锁；本文件验证 bit-exact + 性能。
//!
//! 数据：优先真实 china_dem_l12.arpack（zigzag11 同款），缺失则合成 ARPK1 数据
//! （write_pack_raw）只验正确性。

use std::path::Path;
use std::time::Instant;

use aircraft_router_planner_cli::costfield::{build_semantic_cost_field, build_semantic_cost_field_par, CostField};
use aircraft_router_planner_cli::terrain::builtin::{write_pack_raw, BuiltinSource};
use aircraft_router_planner_cli::terrain::mask::{GeoMask, MaskedSource, MAGIC as MASK_MAGIC};
use aircraft_router_planner_cli::terrain::{BulkPrefetch, Sample, TerrainSource};

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

/// 无锁批量预取构建（候选②）：prefetch 一次（锁外解压）+ sample_local 无锁查表。
fn build_local<B: BulkPrefetch + ?Sized>(
    src: &B,
    min_lon: f64,
    min_lat: f64,
    span: f64,
    grid: usize,
) -> CostField {
    let half = 0.5 / grid as f64 * span;
    let slack = span / grid as f64; // 一网格单元余量（覆盖双线性邻域）
    let local = src.prefetch_lonlat(
        min_lon + half - slack,
        min_lat + half - slack,
        min_lon + span - half + slack,
        min_lat + span - half + slack,
    );
    let mut f = CostField::new(grid, grid);
    for r in 0..grid {
        for c in 0..grid {
            let u = (c as f64 + 0.5) / grid as f64;
            let v = (r as f64 + 0.5) / grid as f64;
            let lon = min_lon + u * span;
            let lat = min_lat + v * span;
            f.cost[r * grid + c] = src.sample_local(&local, lon, lat).base_cost(5.0);
        }
    }
    f
}

/// 并行 + 无锁（候选③）：行分块，每线程预取自己行范围的块（并行锁外解压）
/// + 局部无锁查表——消除并行版的锁竞争，同时兑现解压并行收益。
fn build_par_local<B: BulkPrefetch + Sync + Send>(
    src: &B,
    min_lon: f64,
    min_lat: f64,
    span: f64,
    grid: usize,
) -> CostField {
    let nthreads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .clamp(1, 16);
    let mut f = CostField::new(grid, grid);
    if nthreads <= 1 || grid < 64 {
        return build_local(src, min_lon, min_lat, span, grid);
    }
    let chunk = grid.div_ceil(nthreads);
    let half = 0.5 / grid as f64 * span;
    let slack = span / grid as f64;
    let col_lon0 = min_lon + half - slack;
    let col_lon1 = min_lon + span - half + slack;
    std::thread::scope(|scope| {
        let mut ranges: Vec<(usize, usize)> = Vec::new();
        let mut handles = Vec::new();
        for t in 0..nthreads {
            let r0 = t * chunk;
            if r0 >= grid {
                break;
            }
            let r1 = (r0 + chunk).min(grid);
            ranges.push((r0, r1));
            let src = src;
            handles.push(scope.spawn(move || {
                // 本线程行范围 [r0, r1) 的网格点 lat 跨度 → 预取
                let lat0 = min_lat + (r0 as f64 + 0.5) / grid as f64 * span;
                let lat1 = min_lat + (r1 as f64 + 0.5) / grid as f64 * span;
                let local = src.prefetch_lonlat(col_lon0, lat0 - slack, col_lon1, lat1 + slack);
                let n = (r1 - r0) * grid;
                let mut sub = vec![0f32; n];
                for r in r0..r1 {
                    let v = (r as f64 + 0.5) / grid as f64;
                    let lat = min_lat + v * span;
                    let base = (r - r0) * grid;
                    for c in 0..grid {
                        let u = (c as f64 + 0.5) / grid as f64;
                        let lon = min_lon + u * span;
                        sub[base + c] = src.sample_local(&local, lon, lat).base_cost(5.0);
                    }
                }
                sub
            }));
        }
        let mut offset = 0;
        for (h, (_r0, r1)) in handles.into_iter().zip(&ranges) {
            let sub = h.join().unwrap_or_else(|_| vec![0.0; (r1 - _r0) * grid]);
            f.cost[offset..offset + sub.len()].copy_from_slice(&sub);
            offset += sub.len();
        }
    });
    f
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

/// 全 Land 掩膜（20×30，1° 网格，lon0=100 lat0=30，覆盖 zigzag11 区域 [111,124)×[34,43)；
/// 仅验证 MaskedSource 转发语义：全 Land 下 sample_at == sample_local）。
fn all_land_mask() -> GeoMask {
    let rows = 20u32;
    let cols = 30u32;
    let seg_base = 64u64 + (rows as u64 + 1) * 8;
    let mut out = Vec::new();
    out.extend_from_slice(&MASK_MAGIC);
    out.extend_from_slice(&2u32.to_be_bytes()); // version
    out.extend_from_slice(&1u32.to_be_bytes()); // arcsec
    out.extend_from_slice(&rows.to_be_bytes());
    out.extend_from_slice(&cols.to_be_bytes());
    out.extend_from_slice(&100.0f64.to_be_bytes()); // lon0
    out.extend_from_slice(&30.0f64.to_be_bytes()); // lat0
    out.extend_from_slice(&1.0f64.to_be_bytes()); // res_deg
    out.extend_from_slice(&[0u8; 8]); // padding 56..64
    for i in 0..=rows {
        // 每行 = nseg 4B + 1 段 9B → 13B
        out.extend_from_slice(&(seg_base + i as u64 * 13).to_be_bytes());
    }
    for _ in 0..rows {
        out.extend_from_slice(&1u32.to_be_bytes()); // nseg=1
        out.push(1); // class = Land
        out.extend_from_slice(&0u32.to_be_bytes()); // c0
        out.extend_from_slice(&cols.to_be_bytes()); // c1 = 30
    }
    GeoMask::parse(&out).expect("mask parse")
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

/// 核心对比：同闭包、同区域，串行/并行产物逐位一致（&dyn TerrainSource 路径）。
fn assert_bit_exact(grid: usize, t: Option<&dyn TerrainSource>) {
    let (min_lon, min_lat, span) = zigzag11_region();
    let s = make_sampler(t, min_lon, min_lat, span, grid);
    let serial = build_serial(&s, grid);
    let parallel = build_parallel(&s, grid);
    assert_eq!(serial.cost.len(), parallel.cost.len());
    for i in 0..serial.cost.len() {
        assert_eq!(
            serial.cost[i].to_bits(),
            parallel.cost[i].to_bits(),
            "parallel bit-exact mismatch at idx {i}: serial={} parallel={}",
            serial.cost[i],
            parallel.cost[i]
        );
    }
    eprintln!(
        "[compare] OK: {grid}x{grid} = {} cells bit-exact identical (serial==parallel)",
        serial.cost.len()
    );
}

/// 无锁路径对比：串行（sample_at trait） vs 无锁（BulkPrefetch，具体源）。
fn assert_bit_exact_local<B: BulkPrefetch>(grid: usize, b: &B) {
    let t: &dyn TerrainSource = b;
    let (min_lon, min_lat, span) = zigzag11_region();
    let s = make_sampler(Some(t), min_lon, min_lat, span, grid);
    let serial = build_serial(&s, grid);
    let local = build_local(b, min_lon, min_lat, span, grid);
    assert_eq!(serial.cost.len(), local.cost.len());
    for i in 0..serial.cost.len() {
        assert_eq!(
            serial.cost[i].to_bits(),
            local.cost[i].to_bits(),
            "local bit-exact mismatch at idx {i}: serial={} local={}",
            serial.cost[i],
            local.cost[i]
        );
    }
    eprintln!(
        "[compare] OK: {grid}x{grid} = {} cells bit-exact identical (serial==local)",
        serial.cost.len()
    );
}

#[test]
fn field_build_bit_exact_synthetic() {
    let src = synth_source();
    let t: &dyn TerrainSource = &src;
    assert_bit_exact(128, Some(t));
    assert_bit_exact_local(128, &src);
    // MaskedSource 转发：全 Land 掩膜下 sample_at == sample_local（无锁穿透）
    let mask = all_land_mask();
    let msrc = MaskedSource::new(src, mask);
    let m_t: &dyn TerrainSource = &msrc;
    let (min_lon, min_lat, span) = zigzag11_region();
    let s = make_sampler(Some(m_t), min_lon, min_lat, span, 128);
    let serial = build_serial(&s, 128);
    let local = build_local(&msrc, min_lon, min_lat, span, 128);
    for i in 0..serial.cost.len() {
        assert_eq!(
            serial.cost[i].to_bits(),
            local.cost[i].to_bits(),
            "masked local mismatch at idx {i}"
        );
    }
    eprintln!("[compare] OK: MaskedSource sample_at == sample_local ({})", serial.cost.len());
}

#[test]
fn field_build_real_data() {
    // 真实 china_dem_l12.arpack：1024² 正确性 + 性能对比（文件缺失则跳过）。
    let Some(src) = load_real() else {
        eprintln!("[compare] SKIP: china_dem_l12.arpack not found (CI 环境跳过性能对比)");
        return;
    };
    eprintln!("[compare] terrain: {}", src.resolution_desc());
    let t: &dyn TerrainSource = &src;
    assert_bit_exact(1024, Some(t));
    assert_bit_exact_local(1024, &src);
    // 性能公平对比：真实规划每次新实例（冷缓存，zstd 解压主导）。
    // 每轮交替 A/B 各用新解析实例（cache 空），取各自 min——消除预热/顺序偏差。
    let (min_lon, min_lat, span) = zigzag11_region();
    let bytes = std::fs::read(
        Path::new(env!("CARGO_MANIFEST_DIR")).join("..").join("phase0/data/pending/china_dem_l12.arpack"),
    )
    .expect("re-read real arpack");
    let mut s_best = f64::INFINITY;
    let mut p_best = f64::INFINITY;
    let mut l_best = f64::INFINITY;
    let mut pl_best = f64::INFINITY;
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
            // C: local 无锁（新实例冷缓存）
            let src_l = BuiltinSource::parse(&bytes).expect("parse");
            let tt = Instant::now();
            let _ = build_local(&src_l, min_lon, min_lat, span, 1024);
            l_best = l_best.min(tt.elapsed().as_secs_f64() * 1000.0);
            // D: 并行 + 无锁（新实例冷缓存）
            let src_pl = BuiltinSource::parse(&bytes).expect("parse");
            let tt = Instant::now();
            let _ = build_par_local(&src_pl, min_lon, min_lat, span, 1024);
            pl_best = pl_best.min(tt.elapsed().as_secs_f64() * 1000.0);
        } else {
            let src_pl = BuiltinSource::parse(&bytes).expect("parse");
            let tt = Instant::now();
            let _ = build_par_local(&src_pl, min_lon, min_lat, span, 1024);
            pl_best = pl_best.min(tt.elapsed().as_secs_f64() * 1000.0);
            let src_l = BuiltinSource::parse(&bytes).expect("parse");
            let tt = Instant::now();
            let _ = build_local(&src_l, min_lon, min_lat, span, 1024);
            l_best = l_best.min(tt.elapsed().as_secs_f64() * 1000.0);
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
    eprintln!(
        "[compare] real 1024x1024 cold-cache best: serial={:.1}ms parallel={:.1}ms local={:.1}ms par+local={:.1}ms",
        s_best, p_best, l_best, pl_best
    );
    eprintln!(
        "[compare] speedup: parallel {:.2}x, local {:.2}x, par+local {:.2}x (vs serial)",
        s_best / p_best,
        s_best / l_best,
        s_best / pl_best
    );
    // 不硬断言性能（CI 单核/负载波动）；但提示明显退化
    assert!(
        pl_best <= s_best * 1.5,
        "par+local suspiciously slower: serial {s_best:.1}ms vs par+local {pl_best:.1}ms"
    );
}

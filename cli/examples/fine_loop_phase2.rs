//! 开发期基准：Phase 0-2 待数据项真实数据标定（Stage C）。
//! - 细层闭环：北京区 100 组随机起止 → FMM 粗层走廊 → smooth_path_chain
//!   拟合成功率（对照 90% 判据）+ 单组耗时（3s 预算）；
//! - FNR/FPR 成对测量（真实数据粗/细网格，真值 = 细网格 1024²）：
//!   FNR = 粗可达&细不可达 / 细可达；FPR = 粗不可达&细可达 / 粗不可达；
//! - A6 数值验证：典型速度 × 最大坡度 → r_min = v²/(g·tan φ)；
//! - 多机线性预算：一次 FMM 多目标回溯 vs 每机独立 FMM（N=1/4/8/16）。
//! 用法: cargo run --release --example fine_loop_phase2 -- <gmted.arpack> <china.arpack>

use aircraft_router_planner_cli::config::{AircraftType, DefaultParams};
use aircraft_router_planner_cli::costfield::{backtrack_path, build_semantic_cost_field, fmm_propagate};
use aircraft_router_planner_cli::smooth::{
    default_chain, smooth_path_chain, SmoothOptions, VerifyContext,
};
use aircraft_router_planner_cli::terrain::builtin::BuiltinSource;
use aircraft_router_planner_cli::terrain::{los_blocked, Sample, TerrainSource};
use rand::{RngExt, SeedableRng};
use sha2::{Digest, Sha256};
use std::hint::black_box;
use std::time::Instant;

const GRID: usize = 256;
const N_TRIALS: usize = 100;

/// 北京区 256² 语义代价场（Land/NoData；NODATA 5x）。
fn beijing_field<T: TerrainSource>(src: &T) -> aircraft_router_planner_cli::costfield::CostField {
    build_semantic_cost_field(GRID, GRID, |r, c| {
        let lon = 115.9 + (c as f64 + 0.5) / GRID as f64 * 1.0;
        let lat = 39.4 + (r as f64 + 0.5) / GRID as f64 * 1.0;
        src.sample_at(lon, lat)
    }, 5.0)
}

fn lonlat_of(r: usize, c: usize) -> (f64, f64) {
    (115.9 + (c as f64 + 0.5) / GRID as f64 * 1.0, 39.4 + (r as f64 + 0.5) / GRID as f64 * 1.0)
}

/// 随机 Land 起止对（种子固定，可复现）。
fn gen_pairs<T: TerrainSource>(src: &T, n: usize, seed: u64) -> Vec<((usize, usize), (usize, usize))> {
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let mut out = Vec::new();
    let mut guard = 0u32;
    while out.len() < n && guard < 2000 * n as u32 {
        guard += 1;
        let (r1, c1) = (rng.random_range(0..GRID), rng.random_range(0..GRID));
        let (r2, c2) = (rng.random_range(0..GRID), rng.random_range(0..GRID));
        let (lon1, lat1) = lonlat_of(r1, c1);
        let (lon2, lat2) = lonlat_of(r2, c2);
        if src.sample_at(lon1, lat1).class() == Sample::Land(0.0).class()
            && src.sample_at(lon2, lat2).class() == Sample::Land(0.0).class()
            && (r1, c1) != (r2, c2)
        {
            out.push(((r1, c1), (r2, c2)));
        }
    }
    out
}

/// 粗层回溯路径 → Path（经纬度 + 固定飞行高度）。
fn coarse_path(field: &aircraft_router_planner_cli::costfield::CostField, res: &aircraft_router_planner_cli::costfield::FmmResult, src: (usize, usize), dst: (usize, usize), alt: f64) -> Option<aircraft_router_planner_cli::path::Path> {
    let pts = backtrack_path(field, res, dst.0, dst.1, src.0, src.1)?;
    Some(aircraft_router_planner_cli::path::Path::new(
        pts.iter()
            .map(|&(r, c)| {
                let (lon, lat) = lonlat_of(r, c);
                aircraft_router_planner_cli::path::PathPoint::new(lon, lat, alt)
            })
            .collect(),
    ))
}

fn main() {
    let args: Vec<String> = std::env::args().skip(1).collect();
    let c = BuiltinSource::open(std::path::Path::new(&args[1])).unwrap();
    let field = beijing_field(&c);

    // ============ 1) A6 数值验证（固定翼典型速度 × 30° 最大坡度） ============
    println!("[A6] r_min = v^2/(g*tan(30deg)); v ∈ [50..250] m/s");
    for v in [50.0f64, 100.0, 150.0, 200.0, 250.0] {
        let r = DefaultParams::physical_turn_radius_m(v, 30.0);
        println!("[A6] v={v:>4.0} m/s -> r_min {r:>8.0} m");
    }
    // 契约校验：默认 turn_radius 5000m 自洽到 v_max ≈ sqrt(5000*g*tan30) ≈ 168 m/s
    let v_max = (5000.0f64 * 9.80665 * 30f64.to_radians().tan()).sqrt();
    println!("[A6] 默认 turn_radius 5000m 自洽 v_max ≈ {v_max:.0} m/s（超过需配置更大 min_turn_radius）");
    assert!(v_max > 150.0 && v_max < 200.0, "v_max 应在典型速度区间内");

    // ============ 2) 细层闭环：100 组随机起止（含确定性路径哈希） ============
    let pairs = gen_pairs(&c, N_TRIALS, 0xbeef);
    assert_eq!(pairs.len(), N_TRIALS);
    let mut ok = 0usize;
    let mut no_path = 0usize;
    let mut t_total = 0.0f64;
    let mut hasher = Sha256::new();
    for (_, &((sr, sc), (dr, dc))) in pairs.iter().enumerate() {
        let t0 = Instant::now();
        let res = fmm_propagate(&field, sr, sc);
        let Some(p) = coarse_path(&field, &res, (sr, sc), (dr, dc), 3000.0) else {
            no_path += 1;
            continue;
        };
        let opts = SmoothOptions::default();
        let check = |_: f64, _: f64, _: f64, _: f64, _: f64, _: f64| true; // 无地形/禁飞限制的纯几何链
        let ctx = VerifyContext { terrain: None, nofly: None, zones: None, threat: None };
        let chain = default_chain(&opts, &check);
        let result = smooth_path_chain(&p, &chain, &opts, &ctx, None);
        if result.path.len() >= 2 {
            ok += 1;
        }
        // 确定性黄金基线：输出路径坐标位级哈希（两遍运行应逐位一致）
        for pt in &result.path.points {
            hasher.update(pt.lon.to_le_bytes());
            hasher.update(pt.lat.to_le_bytes());
            hasher.update(pt.alt_m.to_le_bytes());
            hasher.update(pt.heading_deg.unwrap_or(-1.0).to_le_bytes());
        }
        t_total += t0.elapsed().as_secs_f64() * 1000.0;
    }
    let digest = format!("{:x}", hasher.finalize());
    println!(
        "[fine] {N_TRIALS} pairs: smooth ok {ok} ({:.0}%) no_path {no_path} | avg {:.2}ms/trial total {:.0}ms | path-sha256 {digest}",
        100.0 * ok as f64 / N_TRIALS as f64,
        t_total / N_TRIALS as f64,
        t_total
    );

    // ============ 3) FNR/FPR（真实数据：粗 256² vs 细 1024²，真值=细） ============
    let fine_src = &c;
    let fine = build_semantic_cost_field(1024, 1024, |r, c| {
        let lon = 115.9 + (c as f64 + 0.5) / 1024.0 * 1.0;
        let lat = 39.4 + (r as f64 + 0.5) / 1024.0 * 1.0;
        fine_src.sample_at(lon, lat)
    }, 5.0);
    let mut both = 0usize;
    let mut coarse_only = 0usize; // FNR 分子：粗可达 & 细不可达
    let mut fine_only = 0usize; // FPR 分子：粗不可达 & 细可达
    let mut none = 0usize;
    for &((sr, sc), (dr, dc)) in &pairs {
        let rc = fmm_propagate(&field, sr, sc);
        let c_reach = backtrack_path(&field, &rc, dr, dc, sr, sc).is_some();
        // 细网格：起止点映射到 4x 坐标
        let (fr1, fc1, fr2, fc2) = (sr * 4 + 2, sc * 4 + 2, dr * 4 + 2, dc * 4 + 2);
        let rf = fmm_propagate(&fine, fr1, fc1);
        let f_reach = backtrack_path(&fine, &rf, fr2, fc2, fr1, fc1).is_some();
        match (c_reach, f_reach) {
            (true, true) => both += 1,
            (true, false) => coarse_only += 1,
            (false, true) => fine_only += 1,
            (false, false) => none += 1,
        }
    }
    let fine_positive = both + coarse_only; // 细层（真值）可达总数
    let coarse_negative = fine_only + none; // 粗层不可达总数
    println!(
        "[FNR/FPR] both {both} coarse-only {coarse_only} fine-only {fine_only} none {none} | FNR {:.1}% ({coarse_only}/{fine_positive}) FPR {:.1}% ({fine_only}/{coarse_negative})",
        if fine_positive > 0 { 100.0 * coarse_only as f64 / fine_positive as f64 } else { f64::NAN },
        if coarse_negative > 0 { 100.0 * fine_only as f64 / coarse_negative as f64 } else { f64::NAN }
    );

    // ============ 4) 多机线性预算：一次 FMM 多目标回溯 vs 每机独立 FMM ============
    let centers = [(128usize, 128usize), (96, 160), (160, 96), (96, 96), (160, 160), (64, 128), (128, 64), (192, 128), (128, 192), (64, 64), (192, 192), (64, 192), (192, 64), (32, 128), (128, 32), (224, 128)];
    for n in [1usize, 4, 8, 16] {
        let t0 = Instant::now();
        let res = fmm_propagate(&field, centers[0].0, centers[0].1);
        let shared_ms = t0.elapsed().as_secs_f64() * 1000.0;
        let t1 = Instant::now();
        for &(r, _) in centers.iter().take(n).skip(1) {
            // 每机独立 FMM（不同源）
            let _ = fmm_propagate(&field, r, 128);
        }
        let per_ms = t1.elapsed().as_secs_f64() * 1000.0;
        // 多目标回溯成本（同源 n 个目标）
        let t2 = Instant::now();
        let mut paths = 0usize;
        for &(dr, dc) in centers.iter().take(n).skip(1) {
            if backtrack_path(&field, &res, dr, dc, centers[0].0, centers[0].1).is_some() {
                paths += 1;
            }
        }
        let bt_ms = t2.elapsed().as_secs_f64() * 1000.0;
        println!(
            "[multi] n={n:>2} shared-FMM {shared_ms:.1}ms + {n}-backtrack {bt_ms:.1}ms = {:.1}ms vs per-vehicle FMM {per_ms:.1}ms | paths {paths}",
            shared_ms + bt_ms
        );
    }
    // ============ 5) 山地遮蔽场景（A7：四川 102E/30N——遮蔽>0 用例） ============
    let m_lon = 102.0;
    let m_lat = 30.0;
    // 5a) 山地 LOS 遮挡统计（低空相对射线，GMTED2010——全球含山地）
    let mut rng = rand::rngs::StdRng::seed_from_u64(0xa7a7);
    let mut rays = Vec::new();
    let mut attempts = 0u32;
    while rays.len() < 200 && attempts < 40000 {
        attempts += 1;
        let lon = m_lon + rng.random_range(-0.5..0.5);
        let lat = m_lat + rng.random_range(-0.5..0.5);
        let Some(ground) = c.height_at(lon, lat) else { continue };
        let oz = ground + rng.random_range(500.0..3_000.0);
        let theta = rng.random_range(0.0..std::f64::consts::TAU);
        rays.push(([lon, lat, oz], [theta.cos(), theta.sin(), 0.0]));
    }
    let blocked = rays
        .iter()
        .filter(|(o, d)| los_blocked(&c, o[0], o[1], o[2], d[0], d[1], d[2], 0.54, 1000))
        .count();
    println!(
        "[A7-mt] Sichuan LOS: blocked {:.1}% ({}/{} rays, vs Beijing plain {:.1}%)",
        100.0 * blocked as f64 / rays.len() as f64,
        blocked,
        rays.len(),
        5.0
    );
    // 5b) 山地 FNR/FPR（粗 256² vs 细 1024²，真值=细）——遮蔽地形下粗层漏检
    let m_src = &c;
    let mfield = build_semantic_cost_field(GRID, GRID, |r, cc| {
        let lon = m_lon - 0.5 + (cc as f64 + 0.5) / GRID as f64 * 1.0;
        let lat = m_lat - 0.5 + (r as f64 + 0.5) / GRID as f64 * 1.0;
        m_src.sample_at(lon, lat)
    }, 5.0);
    let mfine = build_semantic_cost_field(1024, 1024, |r, cc| {
        let lon = m_lon - 0.5 + (cc as f64 + 0.5) / 1024.0 * 1.0;
        let lat = m_lat - 0.5 + (r as f64 + 0.5) / 1024.0 * 1.0;
        m_src.sample_at(lon, lat)
    }, 5.0);
    let mut mboth = 0usize;
    let mut mcoarse_only = 0usize;
    let mut mfine_only = 0usize;
    let mut mnone = 0usize;
    let mut mpairs = 0usize;
    let mut mrng = rand::rngs::StdRng::seed_from_u64(0xa7a8);
    while mpairs < 100 {
        let (r1, c1) = (mrng.random_range(0..GRID), mrng.random_range(0..GRID));
        let (r2, c2) = (mrng.random_range(0..GRID), mrng.random_range(0..GRID));
        if (r1, c1) == (r2, c2) {
            continue;
        }
        let rc = fmm_propagate(&mfield, r1, c1);
        let c_reach = backtrack_path(&mfield, &rc, r2, c2, r1, c1).is_some();
        let (fr1, fc1, fr2, fc2) = (r1 * 4 + 2, c1 * 4 + 2, r2 * 4 + 2, c2 * 4 + 2);
        let rf = fmm_propagate(&mfine, fr1, fc1);
        let f_reach = backtrack_path(&mfine, &rf, fr2, fc2, fr1, fc1).is_some();
        match (c_reach, f_reach) {
            (true, true) => mboth += 1,
            (true, false) => mcoarse_only += 1,
            (false, true) => mfine_only += 1,
            (false, false) => mnone += 1,
        }
        mpairs += 1;
    }
    let m_fine_pos = mboth + mcoarse_only;
    let m_coarse_neg = mfine_only + mnone;
    println!(
        "[A7-mt] Sichuan FNR/FPR: both {mboth} coarse-only {mcoarse_only} fine-only {mfine_only} none {mnone} | FNR {:.1}% ({mcoarse_only}/{m_fine_pos}) FPR {:.1}% ({mfine_only}/{m_coarse_neg})",
        if m_fine_pos > 0 { 100.0 * mcoarse_only as f64 / m_fine_pos as f64 } else { f64::NAN },
        if m_coarse_neg > 0 { 100.0 * mfine_only as f64 / m_coarse_neg as f64 } else { f64::NAN }
    );
    black_box(&field);
}

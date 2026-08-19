//! 开发期工具：真实 GSHHG 掩膜验证（Phase 2 掩膜集成）。
//!
//! 用法（workspace 根目录）：
//!   cargo run -p aircraft-router-planner-cli --example mask_probe -- data/mask_10as.mask
//!
//! 输出：解析耗时 / 关键点分类 / 陆地·湖泊占比（与 Python 侧对照）。

use std::time::Instant;

use aircraft_router_planner_cli::terrain::mask::{GeoMask, MaskClass};

fn main() {
    let path = std::env::args()
        .nth(1)
        .unwrap_or_else(|| "data/mask_10as.mask".into());
    let t0 = Instant::now();
    let m = match GeoMask::open(std::path::Path::new(&path)) {
        Ok(m) => m,
        Err(e) => {
            eprintln!("open failed: {e}");
            std::process::exit(1);
        }
    };
    println!(
        "mask: {}  version={} arcsec={} rows={} cols={}  parse={:.1}s",
        path,
        m.version(),
        m.arcsec(),
        m.rows(),
        m.cols(),
        t0.elapsed().as_secs_f64()
    );

    let points: &[(&str, f64, f64, MaskClass)] = &[
        ("北京(116.4,39.9)", 116.4, 39.9, MaskClass::Land),
        ("上海(121.5,31.2)", 121.5, 31.2, MaskClass::Land),
        ("太平洋中部(180,0)", 180.0, 0.0, MaskClass::Sea),
        ("死海(35.5,31.5)", 35.5, 31.5, MaskClass::Lake),
        ("地中海(15,35)", 15.0, 35.0, MaskClass::Sea),
        ("东太平洋(-150,10)", -150.0, 10.0, MaskClass::Sea),
        ("格陵兰(-40,72)", -40.0, 72.0, MaskClass::Land),
        ("南极半岛(-60,-75)", -60.0, -75.0, MaskClass::Sea),
        ("南极内陆(0,-85)", 0.0, -85.0, MaskClass::Land),
        ("南极极点(0,-89)", 0.0, -89.0, MaskClass::Land),
        ("大西洋中部(-30,10)", -30.0, 10.0, MaskClass::Sea),
        ("撒哈拉(15,25)", 15.0, 25.0, MaskClass::Land),
        ("青藏高原(90,33)", 90.0, 33.0, MaskClass::Land),
        ("里海(50,42)", 50.0, 42.0, MaskClass::Lake),
        ("贝加尔湖(108,53.5)", 108.0, 53.5, MaskClass::Lake),
        ("吐鲁番盆地(89.2,42.9)", 89.2, 42.9, MaskClass::Land),
        ("罗斯海(170,-77)", 170.0, -77.0, MaskClass::Sea),
        ("威德尔海(-50,-76)", -50.0, -76.0, MaskClass::Sea),
        ("东南极内陆(80,-80)", 80.0, -80.0, MaskClass::Land),
        ("杭州湾(121.5,30.5)", 121.5, 30.5, MaskClass::Sea),
    ];
    let mut bad = 0;
    let tq = Instant::now();
    for (name, lon, lat, exp) in points {
        let got = m.class_at(*lon, *lat);
        let ok = got == *exp;
        if !ok {
            bad += 1;
            println!("  MISMATCH {name}: got {got:?}, expect {exp:?}");
        }
    }
    println!(
        "关键点 {} 项, {} OK, {} MISMATCH (query 共 {:.1}ms)",
        points.len(),
        points.len() - bad,
        bad,
        tq.elapsed().as_secs_f64() * 1000.0
    );

    let tr = Instant::now();
    let (land, lake) = m.land_lake_ratio();
    println!(
        "陆地占比 {:.2}%  湖泊占比 {:.4}%  (统计 {:.1}s)",
        land * 100.0,
        lake * 100.0,
        tr.elapsed().as_secs_f64()
    );
    if bad > 0 {
        std::process::exit(2);
    }
}

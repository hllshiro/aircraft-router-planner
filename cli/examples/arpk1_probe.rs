//! 开发期工具：读 ARPK1 文件并采样打印（验证转换工具输出兼容性）。
//! 用法: cargo run --example arpk1_probe -- <path.arpk1> [lon lat ...]

use aircraft_router_planner_cli::terrain::TerrainSource;

fn main() {
    let mut args = std::env::args().skip(1);
    let path = args
        .next()
        .expect("usage: arpk1_probe <file.arpk1> [lon lat ...]");
    let src = match aircraft_router_planner_cli::terrain::open_source(std::path::Path::new(&path)) {
        Ok(s) => s,
        Err(e) => {
            eprintln!("open failed: {e}");
            std::process::exit(2);
        }
    };
    println!("res: {}", src.resolution_desc());
    if let Some(b) = src.bounds() {
        println!(
            "bounds: lon {:.4}..{:.4} lat {:.4}..{:.4}",
            b.min_lon, b.max_lon, b.min_lat, b.max_lat
        );
    }
    let pts: Vec<(f64, f64)> = args
        .collect::<Vec<_>>()
        .chunks(2)
        .map(|c| (c[0].parse().unwrap(), c[1].parse().unwrap()))
        .collect();
    if pts.is_empty() {
        pts_default(&*src);
    } else {
        for (lon, lat) in pts {
            println!("sample({lon}, {lat}) = {:?}", src.height_at(lon, lat));
        }
    }
}

fn pts_default(src: &dyn TerrainSource) {
    for (lon, lat) in [
        (116.000, 39.000),
        (116.001, 39.001),
        (116.080, 39.120),
        (116.199, 39.299),
        (116.050, 39.300),
    ] {
        println!("sample({lon}, {lat}) = {:?}", src.height_at(lon, lat));
    }
}

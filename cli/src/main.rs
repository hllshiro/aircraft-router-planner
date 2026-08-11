//! AircraftRouterPlanner CLI 入口。
//!
//! 管道形态（技术方案 8 章）：stdin 读任务 JSON → stdout 输出结果 JSON；
//! 可选参数：--input / --output / --seed / --config。
//! Phase 1 骨架：解析 + InputValidator 校验 + status 契约输出（success 占位，
//! 无解算路径——Phase 2 接入 solver 后填充）。

use std::io::{Read, Write};
use std::path::PathBuf;

use aircraft_router_planner_cli::config::{self, Input, Output};
use aircraft_router_planner_cli::error::{AppError, ErrorBody, InputInvalidReason};
use aircraft_router_planner_cli::solver::{self, SolveParams};
use aircraft_router_planner_cli::terrain::convert::{self, ConvertOptions};
use clap::{Parser, Subcommand};

#[derive(Parser, Debug)]
#[command(
    name = "aircraft-router-planner",
    version,
    about = "AircraftRouterPlanner 核心 CLI：JSON 契约解析 / 校验 / 航路规划 / 地形转换",
    long_about = "管道形态：stdin 读任务 JSON → stdout 输出结果 JSON。\
                  \nstatus 契约：success / degraded_timeout / no_solution / input_invalid。\
                  \n子命令 convert：外部地形（GeoTIFF/DTED/SRTM .hgt）→ ARPK1（自有格式）。"
)]
struct Args {
    /// 任务 JSON 文件（缺省读 stdin）
    #[arg(short, long)]
    input: Option<PathBuf>,
    /// 结果 JSON 文件（缺省写 stdout）
    #[arg(short, long)]
    output: Option<PathBuf>,
    /// 随机种子（确定性：相同种子逐位一致）
    #[arg(long)]
    seed: Option<u64>,
    /// 默认参数表覆盖文件（JSON）
    #[arg(short, long)]
    config: Option<PathBuf>,
    /// 地形文件（ARPK1；缺省用输入 terrain.path；none 源不加载）
    #[arg(long)]
    terrain: Option<PathBuf>,
    /// 海岸掩膜文件（GSHHG 3 态；缺省自动探测默认掩膜 east_asia_7p5as.mask）
    #[arg(long)]
    mask: Option<PathBuf>,
    /// 粗网格分辨率（缺省 256；任务区域自适应）
    #[arg(long, default_value_t = 256)]
    grid: usize,
    /// 子命令：地形转换（外部格式 → ARPK1）
    #[command(subcommand)]
    cmd: Option<Command>,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// 地形转换：外部格式（GeoTIFF/DTED/SRTM .hgt）→ ARPK1（自有格式）
    Convert {
        /// 输入地形文件（.tif/.tiff/.dt0/.dt1/.dt2/.hgt）
        input: PathBuf,
        /// 输出 ARPK1 文件
        output: PathBuf,
        /// 输出 source 描述（缺省用输入格式+尺寸）
        #[arg(long)]
        source: Option<String>,
        /// 空洞值（缺省 -32768）
        #[arg(long, default_value_t = -32768)]
        no_data: i16,
        /// 垂直基准：ellipsoid（椭球高，缺省 egm96）
        #[arg(long)]
        ellipsoid: bool,
        /// 实验性：zstd 块压缩（ruzstd 0.9 encoding Fastest ≈ zstd level 1；
        /// 压缩率低于 deflate，默认不启用）
        #[arg(long)]
        experimental_zstd: bool,
    },
    /// ARPK1 重压缩：任意块压缩（raw/zstd/deflate）→ deflate（默认，内置纯 Rust 编码器）；
    /// `--experimental-zstd` 输出 zstd 块压缩（实验性）
    Recompress {
        /// 输入 ARPK1 文件
        input: PathBuf,
        /// 输出 ARPK1 文件
        output: PathBuf,
        /// 实验性：zstd 块压缩（ruzstd 0.9 encoding Fastest ≈ zstd level 1；
        /// 压缩率低于 deflate，默认不启用）
        #[arg(long)]
        experimental_zstd: bool,
    },
}

fn main() {
    let args = Args::parse();
    // 子命令优先（转换是独立工具形态，不走任务 JSON 契约）
    if let Some(Command::Convert { input, output, source, no_data, ellipsoid, experimental_zstd }) = &args.cmd {
        match run_convert(input, output, source.as_deref(), *no_data, *ellipsoid, *experimental_zstd) {
            Ok(()) => {}
            Err(e) => {
                eprintln!("aircraft-router-planner convert: hard failure: {e}");
                std::process::exit(2);
            }
        }
        return;
    }
    if let Some(Command::Recompress { input, output, experimental_zstd }) = &args.cmd {
        match run_recompress(input, output, *experimental_zstd) {
            Ok(()) => {}
            Err(e) => {
                eprintln!("aircraft-router-planner recompress: hard failure: {e}");
                std::process::exit(2);
            }
        }
        return;
    }
    match run(&args) {
        Ok(()) => {}
        // 硬故障（IO/内部）：stderr + 非零退出（不静默）
        Err(e) => {
            eprintln!("aircraft-router-planner: hard failure: {e}");
            std::process::exit(2);
        }
    }
}

/// convert 子命令实现：转换 + 校验（mmap 打开 + verify_sha）+ 统计输出。
fn run_convert(
    input: &std::path::Path,
    output: &std::path::Path,
    source: Option<&str>,
    no_data: i16,
    ellipsoid: bool,
    experimental_zstd: bool,
) -> Result<(), AppError> {
    let started = std::time::Instant::now();
    let opts = ConvertOptions {
        source: source.unwrap_or("").to_string(),
        no_data,
        datum_ellipsoid: ellipsoid,
        experimental_zstd,
    };
    let stats = convert::convert_file(input, output, &opts)?;
    // 转换产物自校验：mmap 打开（结构校验）+ verify_sha（全量）
    let s = aircraft_router_planner_cli::terrain::builtin::BuiltinSource::open(output)?;
    s.verify_sha()
        .map_err(|e| AppError::Data(format!("convert output sha mismatch: {e}")))?;
    println!(
        "converted: {} -> {}\n  grid {}x{} origin ({:.6}, {:.6}) cell {:.6}deg x {:.6}deg\n  blocks {} data {} bytes ({} MB) in {:.1}s\n  source: {}",
        input.display(),
        output.display(),
        stats.rows,
        stats.cols,
        stats.origin_lon,
        stats.origin_lat,
        stats.cell_lon_deg,
        stats.cell_lat_deg,
        stats.n_blocks,
        stats.bytes_written,
        stats.bytes_written as f64 / 1e6,
        started.elapsed().as_secs_f64(),
        stats.source_desc
    );
    Ok(())
}

/// recompress 子命令实现：块压缩 → 默认 deflate（实验性 `--experimental-zstd` → zstd）
/// + 产物自校验（open + verify_sha）。
fn run_recompress(
    input: &std::path::Path,
    output: &std::path::Path,
    experimental_zstd: bool,
) -> Result<(), AppError> {
    let started = std::time::Instant::now();
    let bytes = convert::recompress_arpk1(input, output, experimental_zstd)?;
    let s = aircraft_router_planner_cli::terrain::builtin::BuiltinSource::open(output)?;
    s.verify_sha()
        .map_err(|e| AppError::Data(format!("recompress output sha mismatch: {e}")))?;
    println!(
        "recompressed: {} -> {}\n  {} bytes ({} MB) in {:.1}s\n  compression: {}",
        input.display(),
        output.display(),
        bytes,
        bytes as f64 / 1e6,
        started.elapsed().as_secs_f64(),
        if experimental_zstd { "zstd (experimental, ruzstd Fastest)" } else { "deflate (miniz_oxide)" },
    );
    Ok(())
}

fn run(args: &Args) -> Result<(), AppError> {
    let started = std::time::Instant::now();

    // 1. 读输入
    let input_str = read_input(args.input.as_deref())?;

    // 2. 解析（畸形 JSON → input_invalid: malformed_json，合法输出不崩溃）
    let input: Input = match Input::from_json_str(&input_str) {
        Ok(i) => i,
        Err(e) => {
            eprintln!("[debug] serde error: {e}");
            let out = Output::failure(
                "input_invalid",
                ErrorBody::input_invalid(InputInvalidReason::MalformedJson, "malformed JSON input"),
                started.elapsed().as_millis() as u64,
            );
            write_output(args, &out)?;
            return Ok(());
        }
    };

    // 3. InputValidator（退化输入 → input_invalid + 原因码，不进入算法）
    if let Err(e) = config::validate(&input) {
        let body: ErrorBody = (&e).into();
        let out = Output::failure("input_invalid", body, started.elapsed().as_millis() as u64);
        write_output(args, &out)?;
        return Ok(());
    }

    // 4. 解算前：外部地形（GeoTIFF/DTED/SRTM .hgt）自动 convert 为 ARPK1。
    //    solver 只吃 ARPK1（架构决策）；demo 用户直接选外部 tiff 时由 CLI 运行时
    //    路径补齐「外部文件 → convert → ARPK1 → solver」（2026-08-11 主管
    //    Beijing_DEM.tif 被 arpack magic mismatch 拒绝）。
    //    转换产物按 输入路径+mtime 缓存于系统临时目录 arp_auto_convert/，
    //    重复规划不重转（57MB tiff ≈ 2-4s）。
    let terrain_path = args
        .terrain
        .clone()
        .or_else(|| input.mission.terrain.path.clone().map(PathBuf::from));
    let terrain_path = match terrain_path {
        Some(p) if is_external_terrain(&p) => Some(auto_convert_to_arpk1(&p)?),
        other => other,
    };
    let params = SolveParams {
        terrain_path,
        mask_path: args.mask.clone(),
        grid: args.grid,
    };
    let out = match solver::solve(&input, &params, started.elapsed().as_millis() as u64) {
        Ok(out) => out,
        // 解算层错误（如地形文件缺失）→ input_invalid（可预期输入问题，不 exit 2）
        Err(e) => {
            let body: ErrorBody = (&e).into();
            Output::failure("input_invalid", body, started.elapsed().as_millis() as u64)
        }
    };
    write_output(args, &out)?;
    Ok(())
}

fn read_input(path: Option<&std::path::Path>) -> Result<String, AppError> {
    let mut buf = String::new();
    match path {
        Some(p) => {
            std::fs::File::open(p)?.read_to_string(&mut buf)?;
        }
        None => {
            std::io::stdin().read_to_string(&mut buf)?;
        }
    }
    Ok(buf)
}

fn write_output(args: &Args, out: &Output) -> Result<(), AppError> {
    let json = serde_json::to_string_pretty(out)?;
    match &args.output {
        Some(p) => {
            let mut f = std::fs::File::create(p)?;
            f.write_all(json.as_bytes())?;
        }
        None => {
            let mut stdout = std::io::stdout();
            stdout.write_all(json.as_bytes())?;
            stdout.write_all(b"\n")?;
        }
    }
    Ok(())
}

/// 外部地形扩展名（solver 只吃 ARPK1，这些格式需先 convert）。
fn is_external_terrain(p: &std::path::Path) -> bool {
    matches!(
        p.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase().as_str(),
        "tif" | "tiff" | "hgt" | "dt0" | "dt1" | "dt2"
    )
}

/// FNV-1a 64（稳定跨进程，用于缓存文件名——DefaultHasher 每次随机）。
fn fnv1a(s: &str) -> u64 {
    let mut h: u64 = 0xcbf2_9ce4_8422_2325;
    for b in s.as_bytes() {
        h ^= *b as u64;
        h = h.wrapping_mul(0x0000_0100_0000_01b3);
    }
    h
}

/// 转换缓存版本：auto-convert 缓存 key 的一部分。外部格式解析/转换逻辑变更时 bump
/// （旧缓存强制失效——2026-08-11 Beijing_DEM north-up 方向修复后，旧错误方向缓存
/// 因 path+mtime 不变被复用，导致 OOB → FMM no path）。
const CONVERT_CACHE_VERSION: u64 = 2;

/// 外部地形 → ARPK1（缓存到系统临时目录 arp_auto_convert/，按输入路径+mtime 命中；
/// convert 失败清理半成品，避免缓存损坏文件被复用）。
fn auto_convert_to_arpk1(input: &std::path::Path) -> Result<PathBuf, AppError> {
    let dir = std::env::temp_dir().join("arp_auto_convert");
    std::fs::create_dir_all(&dir)?;
    let mtime = std::fs::metadata(input)
        .and_then(|m| m.modified())
        .ok()
        .and_then(|t| t.duration_since(std::time::UNIX_EPOCH).ok())
        .map(|d| d.as_secs())
        .unwrap_or(0);
    let out = dir.join(format!(
        "arp_v{}_{:016x}_{}.arpack",
        CONVERT_CACHE_VERSION,
        fnv1a(&input.to_string_lossy()),
        mtime
    ));
    if !out.exists() {
        if let Err(e) = convert::convert_file(input, &out, &ConvertOptions::default()) {
            let _ = std::fs::remove_file(&out);
            return Err(e);
        }
    }
    Ok(out)
}

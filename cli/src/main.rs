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
use clap::Parser;

#[derive(Parser, Debug)]
#[command(
    name = "aircraft-router-planner",
    version,
    about = "AircraftRouterPlanner 核心 CLI：JSON 契约解析 / 校验 / 航路规划",
    long_about = "管道形态：stdin 读任务 JSON → stdout 输出结果 JSON。\
                  \nstatus 契约：success / degraded_timeout / no_solution / input_invalid。"
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
    /// 粗网格分辨率（缺省 256；任务区域自适应）
    #[arg(long, default_value_t = 256)]
    grid: usize,
}

fn main() {
    let args = Args::parse();
    match run(&args) {
        Ok(()) => {}
        // 硬故障（IO/内部）：stderr + 非零退出（不静默）
        Err(e) => {
            eprintln!("aircraft-router-planner: hard failure: {e}");
            std::process::exit(2);
        }
    }
}

fn run(args: &Args) -> Result<(), AppError> {
    let started = std::time::Instant::now();

    // 1. 读输入
    let input_str = read_input(args.input.as_deref())?;

    // 2. 解析（畸形 JSON → input_invalid: malformed_json，合法输出不崩溃）
    let input: Input = match Input::from_json_str(&input_str) {
        Ok(i) => i,
        Err(_) => {
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

    // 4. 解算（Phase 4 M1 接入：代价场 → FMM → 回溯 → 平滑链 → 输出契约）
    let params = SolveParams {
        terrain_path: args.terrain.clone(),
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

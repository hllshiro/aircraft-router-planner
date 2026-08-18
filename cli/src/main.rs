//! AircraftRouterPlanner 核心 CLI 入口。
//!
//! 核心功能只有一条：**路径规划**（`plan` 子命令）——stdin/`--input` 读任务 JSON →
//! stdout/`--output` 写结果 JSON（status 四态契约）。
//!
//! help 风格（2026 起约定）：不使用 `--help`/`-h` 标志；直接 `arp-cli` 或 `arp-cli help`
//! 显示顶层帮助，`arp-cli help <command>` 显示子命令帮助。
//!
//! 子命令：
//!   - `plan`   路径规划（核心）
//!   - `schema` 输出输入/输出 JSON Schema（schemars 动态生成，代码即事实）

use std::io::{Read, Write};
use std::path::PathBuf;

use aircraft_router_planner_cli::config::{self, Input, Output};
use aircraft_router_planner_cli::error::{AppError, ErrorBody, InputInvalidReason};
use aircraft_router_planner_cli::solver::{self, SolveParams};
use clap::{Parser, Subcommand, ValueEnum};

#[derive(Parser, Debug)]
#[command(
    name = "arp-cli",
    version,
    about = "AircraftRouterPlanner 核心 CLI：航路路径规划",
    long_about = "核心功能只有一条：路径规划（plan）。\
                  \n任务 JSON → 结果 JSON（status 四态契约：success / degraded_timeout / \
                  \nno_solution / input_invalid）。",
    after_help = "JSON Schema：`arp-cli schema` 查看输入/输出 schema（schemars 动态生成，代码即事实）。\n子命令帮助：`arp-cli help <command>`。",
    // 不使用 --help/-h 标志：用 `arp-cli` / `arp-cli help` / `arp-cli help <command>`
    disable_help_flag = true,
    // 无任何参数 → 显示顶层 help（即 `arp-cli` == `arp-cli help`）
    arg_required_else_help = true
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// 路径规划（核心）：stdin 或 --input 读任务 JSON → stdout 或 --output 写结果 JSON
    Plan {
        /// 任务 JSON 文件（缺省读 stdin）
        #[arg(short, long)]
        input: Option<PathBuf>,
        /// 结果 JSON 文件（缺省写 stdout）
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// 随机种子（确定性：相同种子逐位一致；当前为保留字段）
        #[arg(long)]
        seed: Option<u64>,
        /// 默认参数表覆盖文件（JSON；当前为保留字段）
        #[arg(short, long)]
        config: Option<PathBuf>,
        /// 地形文件（ARPK1；缺省用输入 terrain.path；none 源不加载）
        #[arg(long)]
        terrain: Option<PathBuf>,
        /// 海岸掩膜文件（GSHHG 3 态；缺省自动探测默认掩膜）
        #[arg(long)]
        mask: Option<PathBuf>,
        /// 粗网格分辨率（缺省 256；任务区域自适应）
        #[arg(long, default_value_t = 256)]
        grid: usize,
    },
    /// 输出输入/输出 JSON Schema（schemars 动态生成，代码即事实）
    Schema {
        /// 输出哪个 schema（缺省 all）
        #[arg(value_enum, default_value_t = SchemaTarget::All)]
        target: SchemaTarget,
    },
}

#[derive(ValueEnum, Debug, Clone, Copy, PartialEq, Eq)]
enum SchemaTarget {
    /// 仅输入 schema（任务 JSON）
    Input,
    /// 仅输出 schema（结果 JSON）
    Output,
    /// 输入 + 输出（默认）
    All,
}

/// plan 子命令参数集合（与 clap 结构解耦，便于传参）。
struct PlanArgs {
    input: Option<PathBuf>,
    output: Option<PathBuf>,
    seed: Option<u64>,
    config: Option<PathBuf>,
    terrain: Option<PathBuf>,
    mask: Option<PathBuf>,
    grid: usize,
}

fn main() {
    let cli = Cli::parse();
    match cli.command {
        Command::Plan {
            input,
            output,
            seed,
            config,
            terrain,
            mask,
            grid,
        } => {
            let args = PlanArgs {
                input,
                output,
                seed,
                config,
                terrain,
                mask,
                grid,
            };
            if let Err(e) = run_plan(&args) {
                // 硬故障（IO/内部）：stderr + 非零退出（不静默）
                eprintln!("arp-cli plan: hard failure: {e}");
                std::process::exit(2);
            }
        }
        Command::Schema { target } => run_schema(target),
    }
}

/// `schema` 子命令：用 schemars 动态生成输入/输出 JSON Schema（代码即事实，零漂移）。
fn run_schema(target: SchemaTarget) {
    let input_schema = schemars::schema_for!(Input);
    let output_schema = schemars::schema_for!(Output);
    match target {
        SchemaTarget::Input => {
            println!(
                "{}",
                serde_json::to_string_pretty(&input_schema).expect("serialize input schema")
            );
        }
        SchemaTarget::Output => {
            println!(
                "{}",
                serde_json::to_string_pretty(&output_schema).expect("serialize output schema")
            );
        }
        SchemaTarget::All => {
            let combined = serde_json::json!({
                "input": input_schema,
                "output": output_schema,
            });
            println!(
                "{}",
                serde_json::to_string_pretty(&combined).expect("serialize combined schema")
            );
        }
    }
}

fn run_plan(args: &PlanArgs) -> Result<(), AppError> {
    // seed/config 为保留字段（当前解算路径暂未消费），显式忽略避免未使用告警。
    let _ = (args.seed, args.config.as_deref());

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

    // 4. 解算（代价场 → FMM → 回溯 → 平滑链 → 输出契约）。
    let params = SolveParams {
        terrain_path: args.terrain.clone(),
        mask_path: args.mask.clone(),
        grid: args.grid,
        // P6-B：3s 预算硬护栏（docs/07 §5）。测试/CI 用 ARP_BUDGET_MS=0 关闭
        //（默认 CLI 3000ms；超预算 → degraded_timeout 返回 warm best-so-far）。
        time_budget_ms: std::env::var("ARP_BUDGET_MS")
            .ok()
            .and_then(|s| s.parse::<u64>().ok())
            .unwrap_or(3000),
    };
    let out = match solver::solve(&input, &params, started.elapsed().as_millis() as u64) {
        Ok(out) => out,
        // 解算层错误 → 按类型映射 status（P6-B：DegradedTimeout → degraded_timeout；
        // NoSolution → no_solution；InputInvalid（如地形文件缺失）→ input_invalid；
        // 其余（Io/Data/Internal）走上层硬故障）。
        Err(e) => {
            let body: ErrorBody = (&e).into();
            let status = match &e {
                AppError::NoSolution(_) => "no_solution",
                AppError::DegradedTimeout(_) => "degraded_timeout",
                _ => "input_invalid",
            };
            Output::failure(status, body, started.elapsed().as_millis() as u64)
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

fn write_output(args: &PlanArgs, out: &Output) -> Result<(), AppError> {
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

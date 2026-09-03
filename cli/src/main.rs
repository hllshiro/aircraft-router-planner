//! AircraftRouterPlanner 核心 CLI 入口。
//!
//! 核心功能只有一条：**路径规划**（`plan` 子命令）——stdin/`--input` 读任务 JSON →
//! stdout/`--output` 写结果 JSON（status 四态契约）。
//!
//! help 风格（2026 起约定）：不使用 `--help`/`-h`/`--version` 标志；直接执行文件或
//! `执行文件 help` 显示顶层帮助（首行为 `执行文件名 v版本 - Aircraft Route Planner`），
//! `执行文件 help <command>` 显示子命令帮助。可执行文件引用一律使用当前执行文件名。
//!
//! 子命令：
//!   - `plan`   路径规划
//!   - `schema` 输出输入/输出 JSON Schema

use std::io::{Read, Write};
use std::path::PathBuf;

use aircraft_router_planner_cli::config::{self, Input, Output};
use aircraft_router_planner_cli::error::{AppError, ErrorBody, InputInvalidReason};
use aircraft_router_planner_cli::solver::{self, SolveParams};
use clap::{CommandFactory, FromArgMatches, Parser, Subcommand, ValueEnum};

#[derive(Parser, Debug)]
#[command(
    name = "arp-cli",
    // 版本信息直接显示在 help 首行，不提供 --version/-V 命令
    disable_version_flag = true,
    // 不使用 --help/-h 标志：用 `{bin}` / `{bin} help` / `{bin} help <command>`
    disable_help_flag = true,
    // 无任何参数 → 显示顶层 help（即 `{bin}` == `{bin} help`）
    arg_required_else_help = true,
    // Usage 行子命令占位符用小写：<command>
    subcommand_value_name = "command"
)]
struct Cli {
    #[command(subcommand)]
    command: Command,
}

#[derive(Subcommand, Debug)]
enum Command {
    /// Path planning
    Plan {
        /// Task JSON file (default: read from stdin)
        #[arg(short, long)]
        input: Option<PathBuf>,
        /// Result JSON file (default: write to stdout)
        #[arg(short, long)]
        output: Option<PathBuf>,
        /// Terrain file (ARPK1; default: input terrain.path)
        #[arg(long)]
        terrain: Option<PathBuf>,
        /// Coastline mask file (GSHHG; default: auto-detect)
        #[arg(long)]
        mask: Option<PathBuf>,
        /// Coarse grid resolution
        #[arg(long, default_value_t = 256)]
        grid: usize,
    },
    /// Output input/output JSON Schema
    Schema {
        /// Which schema to output (default: all)
        #[arg(value_enum, default_value_t = SchemaTarget::All)]
        target: SchemaTarget,
    },
}

#[derive(ValueEnum, Debug, Clone, Copy, PartialEq, Eq)]
enum SchemaTarget {
    /// Input schema only (task JSON)
    Input,
    /// Output schema only (result JSON)
    Output,
    /// Input and output (default)
    All,
}

/// plan 子命令参数集合（与 clap 结构解耦，便于传参）。
struct PlanArgs {
    input: Option<PathBuf>,
    output: Option<PathBuf>,
    terrain: Option<PathBuf>,
    mask: Option<PathBuf>,
    grid: usize,
}

fn main() {
    // Windows 控制台默认代码页为 GBK(936)/OEM(437)，而本程序所有输出（帮助/告警/错误）
    // 均为 UTF-8 字节，直接打印会在控制台显示为中文乱码。启动时把控制台切到 UTF-8
    //（输出/输入代码页 65001）；重定向到文件/管道时调用返回 0（无控制台），静默忽略——
    // 此时字节仍为 UTF-8，落盘/管道内容不受影响。
    #[cfg(windows)]
    win_console::enable_utf8();

    let bin = executable_name();
    let version = env!("CARGO_PKG_VERSION");

    // 动态注入可执行文件名与版本：help 首行 / Usage / 示例统一使用当前执行文件名。
    let matches = Cli::command()
        .about(format!("{bin} v{version} - Aircraft Route Planner"))
        .after_help(format!(
            "Examples:\n  {bin} plan < task.json\n  {bin} plan --input task.json --output result.json\n  {bin} schema"
        ))
        .get_matches();
    let cli = Cli::from_arg_matches(&matches).expect("参数已由 clap 校验");

    match cli.command {
        Command::Plan {
            input,
            output,
            terrain,
            mask,
            grid,
        } => {
            let args = PlanArgs {
                input,
                output,
                terrain,
                mask,
                grid,
            };
            if let Err(e) = run_plan(&args) {
                // 硬故障（IO/内部）：stderr + 非零退出（不静默）
                eprintln!("{bin} plan: hard failure: {e}");
                std::process::exit(2);
            }
        }
        Command::Schema { target } => run_schema(target),
    }
}

/// 当前执行文件名（argv[0] 的 basename），help/错误信息中引用可执行文件时使用。
fn executable_name() -> String {
    std::env::args()
        .next()
        .as_deref()
        .map(std::path::Path::new)
        .and_then(std::path::Path::file_name)
        .and_then(std::ffi::OsStr::to_str)
        .unwrap_or("arp-cli")
        .to_string()
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

/// Windows 控制台 UTF-8 适配：程序输出为 UTF-8，而 Windows 控制台默认 GBK/OEM 代码页
/// 会把中文显示为乱码。此处通过 kernel32 的 `SetConsoleOutputCP`/`SetConsoleCP` 把控制台
/// 输出/输入代码页切到 UTF-8（65001）。
///
/// 实现说明：
///   - 直接 FFI 链接 kernel32（系统 DLL，静态 CRT 红线不受影响，`check_pe_deps.py` 白名单内），
///     不引入 windows-sys 等额外依赖，保持零 C 依赖红线。
///   - 调用失败（输出被重定向到文件/管道，无关联控制台）返回 0，静默忽略——重定向场景下
///     字节本身就是 UTF-8，下游按 UTF-8 解码即可，无需改动代码页。
#[cfg(windows)]
mod win_console {
    /// UTF-8 代码页（CP_UTF8）。
    const CP_UTF8: u32 = 65001;

    #[link(name = "kernel32")]
    unsafe extern "system" {
        fn SetConsoleOutputCP(code_page: u32) -> i32;
        fn SetConsoleCP(code_page: u32) -> i32;
    }

    pub fn enable_utf8() {
        unsafe {
            let _ = SetConsoleOutputCP(CP_UTF8);
            let _ = SetConsoleCP(CP_UTF8);
        }
    }
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

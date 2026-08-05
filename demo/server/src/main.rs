//! AircraftRouterPlanner Demo 后端（开发期工具，不随发布版分发）。
//!
//! POST /api/plan —— 接收完整 Input JSON（技术方案 4.2.1，schema 0.20），
//! 以 stdin/stdout 管道调用核心 CLI 二进制（aircraft-router-planner-cli），
//! 透传其 Output JSON。CLI 二进制默认位于 workspace 根 `target/release/`，
//! 可用环境变量 `ARP_CLI` 覆盖路径。

use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};

use axum::{Json, Router, routing::post};
use serde_json::Value;

/// POST /api/plan — accepts full Input JSON, runs CLI binary, returns result
async fn plan_route(Json(payload): Json<Value>) -> Json<Value> {
    let input_json = serde_json::to_string(&payload).unwrap_or_else(|_| "{}".into());

    match run_cli(&input_json) {
        Ok(json) => Json(json),
        Err(msg) => Json(serde_json::json!({
            "schema_version": "0.20",
            "status": "input_invalid",
            "error": {
                "code": "INTERNAL_SERVER_ERROR",
                "message": msg
            },
            "elapsed_ms": 0,
            "vehicles": [],
            "stats": { "fmm_ms": 0.0, "los_checks": 0, "degradations": [] }
        })),
    }
}

/// 定位 CLI 二进制：ARP_CLI 环境变量优先；否则尝试常见 workspace 相对路径。
fn cli_bin() -> PathBuf {
    if let Ok(p) = std::env::var("ARP_CLI") {
        return PathBuf::from(p);
    }
    const CANDIDATES: &[&str] = &[
        "./target/release/aircraft-router-planner-cli",
        "../target/release/aircraft-router-planner-cli",
        "target/release/aircraft-router-planner-cli",
    ];
    for c in CANDIDATES {
        let p = PathBuf::from(c);
        if p.exists() {
            return p;
        }
    }
    PathBuf::from(CANDIDATES[0])
}

fn run_cli(input_json: &str) -> Result<Value, String> {
    let mut child = Command::new(cli_bin())
        .stdin(Stdio::piped())
        .stdout(Stdio::piped())
        .stderr(Stdio::piped())
        .spawn()
        .map_err(|e| format!("Failed to spawn CLI: {}", e))?;

    {
        let stdin = child
            .stdin
            .as_mut()
            .ok_or("Failed to open CLI stdin")?;
        stdin
            .write_all(input_json.as_bytes())
            .map_err(|e| format!("Failed to write to CLI stdin: {}", e))?;
    }

    let output = child
        .wait_with_output()
        .map_err(|e| format!("CLI process error: {}", e))?;

    if !output.status.success() {
        let stderr = String::from_utf8_lossy(&output.stderr);
        return Err(format!("CLI exited with error: {}", stderr));
    }

    let stdout = String::from_utf8_lossy(&output.stdout).trim().to_string();
    if stdout.is_empty() {
        return Err("CLI produced empty output".into());
    }

    serde_json::from_str(&stdout).map_err(|e| format!("Failed to parse CLI output: {}", e))
}

#[tokio::main]
async fn main() {
    let app = Router::new()
        .route("/api/plan", post(plan_route))
        .layer(tower_http::cors::CorsLayer::permissive());

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3001")
        .await
        .expect("Failed to bind port 3001");

    println!("Server listening on http://0.0.0.0:3001");
    axum::serve(listener, app).await.expect("Server error");
}

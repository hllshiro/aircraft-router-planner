//! AircraftRouterPlanner Demo 后端（开发期工具，不随发布版分发）。
//!
//! 端点：
//! - `POST /api/plan` —— 接收完整 Input JSON（技术方案 4.2.1，schema 0.20），
//!   以 stdin/stdout 管道调用核心 CLI 二进制（aircraft-router-planner-cli），透传其 Output JSON。
//! - `POST /api/terrain` —— 直接复用核心 lib 地形源，对指定 ARPK1 文件在 bbox 内
//!   采样 nx×ny 高度网格，供前端 3D 地形可视化。
//!
//! CLI 二进制默认位于 workspace 根 `target/release/`，可用环境变量 `ARP_CLI` 覆盖路径。

use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex, OnceLock};

use axum::{Json, Router, routing::post};
use serde::Deserialize;
use serde_json::Value;

use aircraft_router_planner_cli::terrain::{TerrainSource, open_source};

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

// ===================== /api/terrain =====================

type TerrainCache = OnceLock<Mutex<HashMap<String, Arc<dyn TerrainSource>>>>;

fn terrain_cache() -> &'static Mutex<HashMap<String, Arc<dyn TerrainSource>>> {
    static CACHE: TerrainCache = OnceLock::new();
    CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

/// 按路径取地形源（进程内缓存；ARPK1 首次打开可能较慢——China 76MB 约 1s，全球 2.3GB 约 13s）。
fn get_source(path: &str) -> Result<Arc<dyn TerrainSource>, String> {
    let key = path.to_string();
    {
        let guard = terrain_cache().lock().unwrap();
        if let Some(src) = guard.get(&key) {
            return Ok(src.clone());
        }
    }
    let src = open_source(std::path::Path::new(&key)).map_err(|e| e.to_string())?;
    let arc: Arc<dyn TerrainSource> = Arc::from(src);
    let mut guard = terrain_cache().lock().unwrap();
    guard.insert(key.clone(), arc.clone());
    Ok(arc)
}

#[derive(Deserialize)]
struct TerrainReq {
    path: String,
    /// [min_lon, min_lat, max_lon, max_lat]
    bbox: [f64; 4],
    /// 网格尺寸 [nx, ny]，默认 [64, 64]，上限 128
    grid: Option<[usize; 2]>,
}

async fn terrain_route(Json(payload): Json<TerrainReq>) -> Json<Value> {
    let nx = payload.grid.map_or(64, |g| g[0].clamp(2, 128));
    let ny = payload.grid.map_or(64, |g| g[1].clamp(2, 128));
    let [min_lon, min_lat, max_lon, max_lat] = payload.bbox;

    if !(min_lon.is_finite()
        && min_lat.is_finite()
        && max_lon.is_finite()
        && max_lat.is_finite())
        || min_lon >= max_lon
        || min_lat >= max_lat
    {
        return Json(serde_json::json!({ "error": "invalid bbox" }));
    }

    let src = match get_source(&payload.path) {
        Ok(s) => s,
        Err(e) => return Json(serde_json::json!({ "error": format!("open terrain: {e}") })),
    };

    let mut heights: Vec<Option<f64>> = Vec::with_capacity(nx * ny);
    let step_lon = (max_lon - min_lon) / (nx as f64 - 1.0);
    let step_lat = (max_lat - min_lat) / (ny as f64 - 1.0);
    for j in 0..ny {
        let lat = max_lat - j as f64 * step_lat;
        for i in 0..nx {
            let lon = min_lon + i as f64 * step_lon;
            // height_at 返回 Option<f64>；NaN 视为无数据 → null（serde_json 不支持 NaN 序列化）
            heights.push(src.height_at(lon, lat).filter(|v| v.is_finite()));
        }
    }

    let bounds = src.bounds();
    Json(serde_json::json!({
        "nx": nx,
        "ny": ny,
        "min_lon": min_lon,
        "min_lat": min_lat,
        "max_lon": max_lon,
        "max_lat": max_lat,
        "resolution": src.resolution_desc(),
        "source_bounds": bounds.map(|b| [b.min_lon, b.min_lat, b.max_lon, b.max_lat]),
        "heights": heights,
    }))
}

#[tokio::main]
async fn main() {
    let app = Router::new()
        .route("/api/plan", post(plan_route))
        .route("/api/terrain", post(terrain_route))
        .layer(tower_http::cors::CorsLayer::permissive());

    let listener = tokio::net::TcpListener::bind("0.0.0.0:3001")
        .await
        .expect("Failed to bind port 3001");

    println!("Server listening on http://0.0.0.0:3001");
    axum::serve(listener, app).await.expect("Server error");
}

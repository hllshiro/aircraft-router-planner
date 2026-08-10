//! AircraftRouterPlanner Demo 后端（开发期工具 + 独立可运行 Demo）。
//!
//! 端点：
//! - `POST /api/plan` —— 接收完整 Input JSON（技术方案 4.2.1，schema 0.20），
//!   以 stdin/stdout 管道调用核心 CLI 二进制（aircraft-router-planner-cli），透传其 Output JSON。
//! - `POST /api/terrain` —— 直接复用核心 lib 地形源，对指定 ARPK1 文件在 bbox 内
//!   采样 nx×ny 高度网格，供前端 3D 地形可视化。
//! - 其余路径 —— 静态文件（前端生产构建产物），默认 exe 同目录 `web-dist/`，
//!   环境变量 `DEMO_WEB_DIR` 可覆盖 → **单进程 :3001 同时提供 API 与页面**，
//!   浏览器打开 http://localhost:3001 即用（无需 Node/Vite）。
//!
//! CLI 二进制定位：环境变量 `ARP_CLI` 优先；否则 exe 同目录 / 上级目录 /
//! 常见 workspace 相对路径（含 .exe 后缀，Windows 与独立包形态兼容）。
//! 端口：环境变量 `DEMO_PORT`，默认 3001。

use std::collections::HashMap;
use std::io::Write;
use std::path::PathBuf;
use std::process::{Command, Stdio};
use std::sync::{Arc, Mutex, OnceLock};

use axum::{Json, Router, routing::post};
use serde::Deserialize;
use serde_json::Value;
use tower_http::services::{ServeDir, ServeFile};

use aircraft_router_planner_cli::terrain::{TerrainSource, open_source};

/// POST /api/plan — accepts full Input JSON, runs CLI binary, returns result
async fn plan_route(Json(mut payload): Json<Value>) -> Json<Value> {
    // 可移植地形路径（2026-08-10）：前端默认 terrain.path 为相对路径
    // `data/east_asia_7p5as.arpack`（开发模式），独立包地形在 install/ 根——转发 CLI
    // 前把相对路径解析为绝对路径（resolve_terrain_path 多候选），CLI 一定可打开。
    // 解析失败（用户显式填了不存在的路径）则保留原样，由 CLI 报错。
    if let Some(mission) = payload.get_mut("mission") {
        if let Some(terrain) = mission.get_mut("terrain") {
            if let Some(Value::String(path)) = terrain.get("path") {
                if let Ok(abs) = resolve_terrain_path(path) {
                    terrain["path"] = Value::String(abs.to_string_lossy().into_owned());
                }
            }
        }
    }
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

/// 定位 CLI 二进制：ARP_CLI 环境变量优先（存在才用，误设/指向不存在则回退候选）；
/// 否则 exe 同目录（独立包形态：demo-server.exe 与 aircraft-router-planner-cli.exe
/// 同目录或上级目录）→ 常见 workspace 相对路径（Windows 下带 .exe 后缀才能被
/// Command 找到）。
fn cli_bin() -> PathBuf {
    if let Ok(p) = std::env::var("ARP_CLI") {
        let pb = PathBuf::from(p);
        if pb.exists() {
            return pb;
        }
        eprintln!("[warn] ARP_CLI set but not found: {} — falling back to candidates", pb.display());
    }
    let mut cands: Vec<PathBuf> = Vec::new();
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            cands.push(dir.join("aircraft-router-planner-cli.exe"));
            cands.push(dir.join("aircraft-router-planner-cli"));
            cands.push(dir.join("..").join("aircraft-router-planner-cli.exe"));
            cands.push(dir.join("..").join("aircraft-router-planner-cli"));
        }
    }
    cands.extend([
        PathBuf::from("aircraft-router-planner-cli.exe"),
        PathBuf::from("./target/release/aircraft-router-planner-cli.exe"),
        PathBuf::from("./target/release/aircraft-router-planner-cli"),
        PathBuf::from("../target/release/aircraft-router-planner-cli"),
        PathBuf::from("target/release/aircraft-router-planner-cli"),
    ]);
    for c in cands {
        if c.exists() {
            return c;
        }
    }
    PathBuf::from("aircraft-router-planner-cli.exe")
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
    let key = resolve_terrain_path(path).map(|p| p.to_string_lossy().into_owned())?;
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

/// 可移植地形路径解析（独立包形态兼容，2026-08-10）：
/// 前端默认地形路径为 `data/east_asia_7p5as.arpack`（开发模式 cwd=workspace 根命中）；
/// 独立包把地形放在 install/ 根（`east_asia_7p5as.arpack`），demo-server.exe 位于
/// install/demo/。依次尝试：
///   1) 绝对路径原样；
///   2) cwd 原样 / cwd 去首段（data/x → x）；
///   3) exe 同目录原样 / exe 上级原样；
///   4) exe 同目录去首段 / exe 上级去首段（install/demo → install/east_asia_7p5as.arpack）。
/// 首段非目录（如 `foo.arpack`）时去首段视为整体（仅用于 data/x 形态）。
fn resolve_terrain_path(path: &str) -> Result<PathBuf, String> {
    let p = PathBuf::from(path);
    if p.is_absolute() {
        if p.exists() {
            return Ok(p);
        }
        return Err(format!("terrain file not found: {}", p.display()));
    }
    let stripped = strip_first_dir(&p);
    let mut cands: Vec<PathBuf> = Vec::new();
    if let Ok(cwd) = std::env::current_dir() {
        cands.push(cwd.join(&p));
        cands.push(cwd.join(&stripped));
    }
    if let Ok(exe) = std::env::current_exe() {
        if let Some(dir) = exe.parent() {
            cands.push(dir.join(&p));
            cands.push(dir.join("..").join(&p));
            cands.push(dir.join(&stripped));
            cands.push(dir.join("..").join(&stripped));
        }
    }
    if let Some(c) = cands.iter().find(|c| c.exists()) {
        return Ok(c.clone());
    }
    Err(format!("terrain file not found: {}（候选: {:?}）", path, cands))
}

/// `data/x.arpack` → `x.arpack`；`x.arpack` → `x.arpack`（无目录则原样）。
fn strip_first_dir(p: &PathBuf) -> PathBuf {
    let mut comps = p.components();
    let first = comps.next();
    match first {
        Some(std::path::Component::Normal(_)) if comps.clone().next().is_some() => {
            comps.as_path().to_path_buf()
        }
        _ => p.clone(),
    }
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
    // 静态前端目录：环境变量 DEMO_WEB_DIR 覆盖，默认 exe 同目录 web-dist/
    // （独立包形态：install/demo/demo-server.exe + install/demo/web-dist/）。
    let web_dir: PathBuf = std::env::var("DEMO_WEB_DIR")
        .unwrap_or_else(|_| "web-dist".into())
        .into();
    let web_dir = std::env::current_exe()
        .ok()
        .and_then(|e| e.parent().map(|d| d.to_path_buf()))
        .map(|d| d.join(&web_dir))
        .unwrap_or(web_dir);
    let index = web_dir.join("index.html");
    let serve = ServeDir::new(&web_dir).not_found_service(ServeFile::new(index));

    let app = Router::new()
        .route("/api/plan", post(plan_route))
        .route("/api/terrain", post(terrain_route))
        .fallback_service(serve)
        .layer(tower_http::cors::CorsLayer::permissive());

    let port = std::env::var("DEMO_PORT").unwrap_or_else(|_| "3001".into());
    let addr = format!("0.0.0.0:{port}");
    let listener = tokio::net::TcpListener::bind(&addr)
        .await
        .unwrap_or_else(|e| panic!("Failed to bind port {port}: {e}"));

    println!("Server listening on http://0.0.0.0:{port} (web: {})", web_dir.display());
    axum::serve(listener, app).await.expect("Server error");
}

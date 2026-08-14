//! 底图层数据源（2026-08-13 主管定稿：掩膜 / GeoTIFF / WMS 三选一）。
//!
//! 统一契约：所有源都归一化为「经纬度 bbox 对齐的 RGBA 网格」（nx×ny×4，
//! 上限 256×256），前端按 bbox 直接贴图，**不感知投影差异**：
//! - `POST /api/basemap` `source="mask"`：GSHHG 海陆掩膜（0=海 / 1=陆 / 2=湖）→ 三色填充；
//! - `POST /api/basemap` `source="tiff"`：GeoTIFF（EPSG:4326 线性 / EPSG:3857 Web Mercator
//!   重采样；LZW/Deflate/无压缩/JPEG 压缩均支持）；存在 `.ovr`（GDAL 外部金字塔）时
//!   优先读最适分辨率层（大图快速载入），无合适层回退主文件；
//! - `GET /api/wms`：局域网 GeoServer 代理（固定 WMS 1.1.1 + SRS，规避 1.3.0 轴序陷阱）。

use std::collections::HashMap;
use std::fs::File;
use std::io::BufReader;
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex, OnceLock};
use std::time::Duration;

use axum::Json;
use axum::extract::Query;
use axum::http::header;
use axum::response::Response;
use axum::body::Body;
use base64::Engine;
use base64::engine::general_purpose::STANDARD as BASE64;
use serde::Deserialize;
use serde_json::Value;
use tiff::decoder::{Decoder, DecodingResult};
use tiff::tags::Tag;
use tiff::ColorType;

use aircraft_router_planner_cli::terrain::mask::{GeoMask, MaskClass};

const MAX_GRID: usize = 256;
/// 无 .ovr 时主文件「全量解码」的像素上限（≤ 此值全读 + 双线性；> 此值走抽稀降级）
const MAX_MAIN_PIXELS: usize = 30_000_000;
/// 抽稀降级的原始像素字节上限（8bit RGB 233M 像素 ≈ 700MB 可过；16bit RGB 1.4GB 拒）
const MAX_MAIN_BYTES: u64 = 800_000_000;

// ===================== TIFF 解码缓存 =====================
// 2026-08-13 瓦片化后每瓦片都调用 tiff_basemap：若每次重新解码主文件（HYP 700MB
// 抽稀 ~0.7s），视口 24 瓦片 → 十几秒。改为按路径缓存「抽稀/ovr 全图」，首次解码后
// 所有瓦片从缓存采样（~10ms）；瓦片 grid 固定 128×128 → 抽稀全图恒定可复用。

struct CachedTiff {
    img_w: u32,
    img_h: u32,
    img_rgba: Vec<u8>,
    px_eff: f64,
    py_eff: f64,
    origin_x: f64,
    origin_y: f64,
    projection: Projection,
    georef: &'static str,
    warning: Option<String>,
}

static TIFF_CACHE: OnceLock<Mutex<HashMap<PathBuf, Arc<CachedTiff>>>> = OnceLock::new();

fn tiff_cache() -> &'static Mutex<HashMap<PathBuf, Arc<CachedTiff>>> {
    TIFF_CACHE.get_or_init(|| Mutex::new(HashMap::new()))
}

fn tiff_cache_get(key: &Path) -> Option<Arc<CachedTiff>> {
    tiff_cache().lock().unwrap().get(key).cloned()
}

fn tiff_cache_put(key: PathBuf, c: Arc<CachedTiff>) {
    let mut m = tiff_cache().lock().unwrap();
    // 简单上限：超过 4 个 tiff 文件清空（实际一般只配 1 个）
    if m.len() >= 4 {
        m.clear();
    }
    m.insert(key, c);
}

// ===================== /api/basemap =====================

#[derive(Deserialize)]
pub struct BasemapReq {
    pub source: String, // "mask" | "tiff"
    pub path: String,
    /// [min_lon, min_lat, max_lon, max_lat]
    pub bbox: Option<[f64; 4]>,
    /// [nx, ny]
    pub grid: Option<[usize; 2]>,
    /// tiff 投影：auto（读 GeoKey）/ "4326" / "3857"
    pub projection: Option<String>,
    /// rgba 传输编码：缺省 → JSON 数组（兼容旧前端）；"base64" → 字符串（省体积）
    pub rgba_encoding: Option<String>,
}

/// rgba 输出字段（按请求编码：数组 / base64）
fn rgba_field(rgba: &[u8], req: &BasemapReq) -> Value {
    if req.rgba_encoding.as_deref() == Some("base64") {
        serde_json::json!({ "rgba_b64": BASE64.encode(rgba) })
    } else {
        serde_json::json!({ "rgba": rgba })
    }
}

pub async fn basemap_route(Json(payload): Json<BasemapReq>) -> Json<Value> {
    // TIFF 解码/掩膜采样可能耗时（大图全读），挪到阻塞线程池
    let result = tokio::task::spawn_blocking(move || build_basemap(&payload))
        .await
        .unwrap_or_else(|e| serde_json::json!({ "error": format!("basemap task error: {e}") }));
    Json(result)
}

pub(crate) fn build_basemap(req: &BasemapReq) -> Value {
    let (nx, ny) = match req.grid {
        Some(g) => (g[0].clamp(2, MAX_GRID), g[1].clamp(2, MAX_GRID)),
        None => (128, 128),
    };
    match req.source.as_str() {
        "mask" => mask_basemap(req, nx, ny),
        "tiff" => tiff_basemap(req, nx, ny),
        other => serde_json::json!({ "error": format!("unknown basemap source: {other}") }),
    }
}

/// 掩膜三色（海深蓝 / 陆浅灰绿 / 湖浅蓝，2026-08-13）
fn mask_color(c: MaskClass) -> [u8; 4] {
    match c {
        MaskClass::Sea => [26, 58, 92, 255],
        MaskClass::Land => [138, 154, 106, 255],
        MaskClass::Lake => [58, 106, 138, 255],
    }
}

fn mask_basemap(req: &BasemapReq, nx: usize, ny: usize) -> Value {
    let key = match crate::resolve_terrain_path(&req.path) {
        Ok(p) => p,
        Err(e) => return serde_json::json!({ "error": e }),
    };
    let mask = match GeoMask::open(&key) {
        Ok(m) => m,
        Err(e) => return serde_json::json!({ "error": format!("open mask: {e}") }),
    };
    // 掩膜为全球 7.5as（GSHHG）；bbox 缺省 → 全球范围
    let [min_lon, min_lat, max_lon, max_lat] = req
        .bbox
        .unwrap_or([-180.0, -90.0, 180.0, 90.0]);
    if !(min_lon.is_finite()
        && min_lat.is_finite()
        && max_lon.is_finite()
        && max_lat.is_finite())
        || min_lon >= max_lon
        || min_lat >= max_lat
    {
        return serde_json::json!({ "error": "invalid bbox" });
    }
    let mut rgba: Vec<u8> = Vec::with_capacity(nx * ny * 4);
    let step_lon = (max_lon - min_lon) / (nx as f64 - 1.0);
    let step_lat = (max_lat - min_lat) / (ny as f64 - 1.0);
    for j in 0..ny {
        let lat = max_lat - j as f64 * step_lat;
        for i in 0..nx {
            let lon = min_lon + i as f64 * step_lon;
            rgba.extend_from_slice(&mask_color(mask.class_at(lon, lat)));
        }
    }
    let mut out = serde_json::json!({
        "nx": nx,
        "ny": ny,
        "min_lon": min_lon,
        "min_lat": min_lat,
        "max_lon": max_lon,
        "max_lat": max_lat,
        "resolution": mask.resolution_desc(),
        "source": "mask",
        "projection": "mask",
    });
    if let Value::Object(map) = &mut out {
        if let Value::Object(rgba_obj) = rgba_field(&rgba, req) {
            for (k, v) in rgba_obj {
                map.insert(k, v);
            }
        }
    }
    out
}

// ===================== GeoTIFF / OVR =====================

/// 打开 TIFF 解码器：放宽 tiff crate 默认解码限制（max_alloc 默认 512MiB，
/// 无法容纳 HYP_HR 等 8bit RGB 700MB 全量解码；2GiB 内可容纳 16bit RGBA 上限）。
fn open_decoder(file: File) -> Result<Decoder<BufReader<File>>, String> {
    let mut limits = tiff::decoder::Limits::default();
    limits.decoding_buffer_size = 2 * 1024 * 1024 * 1024;
    Decoder::new(BufReader::new(file))
        .map_err(|e| format!("tiff header: {e}"))
        .map(|d| d.with_limits(limits))
}

#[derive(Clone, Copy, PartialEq, Debug)]
enum Projection {
    P4326,
    P3857,
}

/// GeoTIFF 地理变换（GDAL 约定）：
/// model_x = origin_x + col * px；model_y = origin_y - row * py（py 为正，row 向下）。
struct TiffMeta {
    width: u32,
    height: u32,
    px: f64,
    py: f64,
    origin_x: f64,
    origin_y: f64,
    projection: Projection,
    /// 地理参考来源："embedded"（内嵌 GeoTIFF 标签）| "tfw"（同目录 .tfw 世界文件）
    georef: &'static str,
    /// 非致命提示（如 .tfw 无 .prj 时默认按 4326）
    warning: Option<String>,
}

fn parse_projection(s: &str) -> Option<Projection> {
    match s {
        "4326" => Some(Projection::P4326),
        "3857" => Some(Projection::P3857),
        _ => None,
    }
}

/// GeoKeyDirectory → 投影（只处理 loc==0 的标量 key）：
/// 1024 GTModelType、2048 GeographicType（4326）、3072 ProjectedCSType（3857）
fn geokey_projection(dec: &mut Decoder<BufReader<File>>) -> Result<Option<Projection>, String> {
    let shorts: Vec<u16> = match dec.get_tag_u16_vec(Tag::GeoKeyDirectoryTag) {
        Ok(v) => v,
        Err(_) => return Ok(None), // 无 GeoKey
    };
    if shorts.len() < 4 {
        return Ok(None);
    }
    let n = shorts[3] as usize;
    let mut proj: Option<u16> = None;
    let mut geo: Option<u16> = None;
    for k in 0..n {
        let base = 4 + k * 4;
        if base + 3 >= shorts.len() {
            break;
        }
        let (key_id, loc, count, val) =
            (shorts[base], shorts[base + 1], shorts[base + 2], shorts[base + 3]);
        if loc != 0 || count != 1 {
            continue;
        }
        match key_id {
            3072 => proj = Some(val),
            2048 => geo = Some(val),
            _ => {}
        }
    }
    if let Some(p) = proj {
        if p == 3857 {
            return Ok(Some(Projection::P3857));
        }
        if p == 4326 {
            return Ok(Some(Projection::P4326));
        }
    }
    if let Some(g) = geo {
        if g == 4326 {
            return Ok(Some(Projection::P4326));
        }
    }
    Ok(None)
}

fn read_main_meta(path: &Path, fallback: Option<Projection>) -> Result<TiffMeta, String> {
    let file = File::open(path).map_err(|e| format!("open tiff: {e}"))?;
    let mut dec = open_decoder(file)?;
    let (width, height) = dec.dimensions().map_err(|e| format!("tiff dims: {e}"))?;

    // 地理参考：优先内嵌 GeoTIFF 标签；缺失 → 回退同目录 .tfw 世界文件
    // （NOAA 海图等常见：HYP_HR_SR_OB_DR 提供 .tfw/.prj，内嵌标签为空）
    let mut georef = "embedded";
    let mut warning: Option<String> = None;
    let (px, py, origin_x, origin_y) = match read_embedded_georef(&mut dec) {
        Ok(Some(g)) => g,
        Ok(None) => match read_tfw_georef(path)? {
            Some((px, py, ox, oy)) => {
                georef = "tfw";
                (px, py, ox, oy)
            }
            None => {
                return Err(
                    "GeoTIFF 缺少内嵌 ModelPixelScale/ModelTiepoint 且无同目录 .tfw 世界文件"
                        .into(),
                )
            }
        },
        Err(e) => return Err(e),
    };
    if !(px.is_finite() && py.is_finite() && px > 0.0 && py > 0.0) {
        return Err(format!("像素尺寸非法: px={px} py={py}"));
    }

    let projection = match fallback {
        Some(p) => p,
        None => match geokey_projection(&mut dec)? {
            Some(p) => p,
            None => {
                if let Some(p) = proj_from_prj(path) {
                    p
                } else if georef == "tfw" {
                    warning = Some(
                        "未找到 .prj 投影文件，按 EPSG:4326（经纬度）处理；如坐标异常请在前端显式指定投影"
                            .into(),
                    );
                    Projection::P4326
                } else {
                    return Err(
                        "GeoTIFF 投影未知（无 GeoKey 或非 4326/3857），请在前端显式指定投影"
                            .into(),
                    );
                }
            }
        },
    };
    Ok(TiffMeta {
        width,
        height,
        px,
        py,
        origin_x,
        origin_y,
        projection,
        georef,
        warning,
    })
}

/// 读内嵌 ModelPixelScale/ModelTiepoint → (px, py, origin_x, origin_y)。
/// 标签缺失/值非法 → Ok(None)（调用方回退 .tfw）；IO 错误 → Err。
fn read_embedded_georef(
    dec: &mut Decoder<BufReader<File>>,
) -> Result<Option<(f64, f64, f64, f64)>, String> {
    let scale = match dec.get_tag_f64_vec(Tag::ModelPixelScaleTag) {
        Ok(v) => v,
        Err(_) => return Ok(None),
    };
    let tie = match dec.get_tag_f64_vec(Tag::ModelTiepointTag) {
        Ok(v) => v,
        Err(_) => return Ok(None),
    };
    if scale.len() < 2 || tie.len() < 6 {
        return Ok(None);
    }
    let (px, py) = (scale[0], scale[1]);
    if !(px.is_finite() && py.is_finite() && px > 0.0 && py > 0.0) {
        return Ok(None);
    }
    let (ti, tj, tx, ty) = (tie[0], tie[1], tie[3], tie[4]);
    Ok(Some((px, py, tx - ti * px, ty + tj * py)))
}

/// 读同目录 `<stem>.tfw` 世界文件 → (px, py, origin_x, origin_y)（角点坐标）。
/// 世界文件 6 行：A D B E C F（x = A·col + B·row + C，y = D·col + E·row + F），
/// 坐标是像素**中心**，角点 = 中心 − 半像素；含旋转（B/D ≠ 0）暂不支持。
fn read_tfw_georef(path: &Path) -> Result<Option<(f64, f64, f64, f64)>, String> {
    let tfw = path.with_extension("tfw");
    if !tfw.exists() {
        return Ok(None);
    }
    let text = std::fs::read_to_string(&tfw).map_err(|e| format!("read tfw: {e}"))?;
    let vals: Vec<f64> = text
        .split_whitespace()
        .filter_map(|t| t.parse::<f64>().ok())
        .collect();
    if vals.len() < 6 {
        return Err(format!(
            "世界文件 {} 格式非法（需 6 个数字，实际 {}）",
            tfw.display(),
            vals.len()
        ));
    }
    let (a, d, b, e, c, f) = (vals[0], vals[1], vals[2], vals[3], vals[4], vals[5]);
    if b.abs() > 1e-9 || d.abs() > 1e-9 {
        return Err(format!(
            "世界文件 {} 含旋转（B={b} D={d}），暂不支持",
            tfw.display()
        ));
    }
    let (px, py) = (a, -e);
    if !(px.is_finite() && py.is_finite() && px > 0.0 && py > 0.0) {
        return Err(format!(
            "世界文件 {} 像素尺寸非法: px={px} py={py}",
            tfw.display()
        ));
    }
    Ok(Some((px, py, c - a / 2.0, f - e / 2.0)))
}

/// 粗解析同目录 .prj（WKT）：GEOGCS → 4326；Web Mercator / 3857 → 3857；其他 → None。
fn proj_from_prj(path: &Path) -> Option<Projection> {
    let prj = path.with_extension("prj");
    let text = std::fs::read_to_string(&prj).ok()?;
    if text.len() > 64 * 1024 {
        return None;
    }
    let s = text.to_ascii_lowercase();
    if s.contains("web_mercator") || s.contains("3857") {
        return Some(Projection::P3857);
    }
    if s.contains("geogcs") && !s.contains("projcs") {
        return Some(Projection::P4326);
    }
    None
}

/// 解码当前 IFD 为 RGBA8（tiff 0.10 read_image 返回交错标量数组，按 colortype 展开；
/// 支持 Gray/RGB/RGBA 8/16 位，其余报错）
fn decode_current_rgba(dec: &mut Decoder<BufReader<File>>) -> Result<Vec<u8>, String> {
    let ct = dec.colortype().map_err(|e| format!("colortype: {e}"))?;
    let result = dec.read_image().map_err(|e| format!("tiff decode: {e}"))?;
    let to8 = |v: u16| (v >> 8) as u8;
    let out: Vec<u8> = match (ct, result) {
        (ColorType::Gray(8), DecodingResult::U8(v)) => v
            .iter()
            .flat_map(|x| [*x, *x, *x, 255])
            .collect(),
        (ColorType::Gray(16), DecodingResult::U16(v)) => v
            .iter()
            .flat_map(|x| {
                let b = to8(*x);
                [b, b, b, 255]
            })
            .collect(),
        (ColorType::RGB(8), DecodingResult::U8(v)) => v
            .chunks_exact(3)
            .flat_map(|p| [p[0], p[1], p[2], 255])
            .collect(),
        (ColorType::RGB(16), DecodingResult::U16(v)) => v
            .chunks_exact(3)
            .flat_map(|p| [to8(p[0]), to8(p[1]), to8(p[2]), 255])
            .collect(),
        (ColorType::RGBA(8), DecodingResult::U8(v)) => v
            .chunks_exact(4)
            .flat_map(|p| [p[0], p[1], p[2], p[3]])
            .collect(),
        (ColorType::RGBA(16), DecodingResult::U16(v)) => v
            .chunks_exact(4)
            .flat_map(|p| [to8(p[0]), to8(p[1]), to8(p[2]), to8(p[3])])
            .collect(),
        other => return Err(format!("不支持的 TIFF 像素类型: {other:?}")),
    };
    Ok(out)
}

/// 探测 .ovr：`<path>.ovr`（GDAL 默认）→ `<basename>.ovr`
fn pick_ovr_path(tiff_path: &Path) -> Option<PathBuf> {
    let c1 = PathBuf::from(format!("{}.ovr", tiff_path.to_string_lossy()));
    if c1.exists() {
        return Some(c1);
    }
    let c2 = tiff_path.with_extension("ovr");
    if c2.exists() {
        return Some(c2);
    }
    None
}

/// 读 .ovr 全部分层，选最贴近目标像素数的一层（优先 ≥ 目标的最近层，否则最大层）。
/// 返回 (ov_width, ov_height, rgba, 有效像素宽 px_eff, 有效像素高 py_eff)。
fn read_ovr_best(
    ov_path: &Path,
    target_pixels: usize,
    main: &TiffMeta,
) -> Result<Option<(u32, u32, Vec<u8>, f64, f64)>, String> {
    let file = File::open(ov_path).map_err(|e| format!("open ovr: {e}"))?;
    let mut dec = open_decoder(file)?;
    let mut best_ge: Option<(usize, u32, u32, Vec<u8>)> = None;
    let mut best_max: Option<(usize, u32, u32, Vec<u8>)> = None;
    loop {
        let (w, h) = dec.dimensions().map_err(|e| format!("ovr dims: {e}"))?;
        let pixels = w as usize * h as usize;
        let rgba = decode_current_rgba(&mut dec)?;
        // 分层选择：IFD0 是 .ovr 最大层（先遍历），else-if 保证同一层只进一个分支——
        // 语义等价于「≥target 取最小」+「全 <target 取最大」（最大层恒先见）。
        if pixels >= target_pixels
            && (best_ge.is_none() || pixels < best_ge.as_ref().unwrap().0)
        {
            best_ge = Some((pixels, w, h, rgba));
        } else if best_max.is_none() || pixels > best_max.as_ref().unwrap().0 {
            best_max = Some((pixels, w, h, rgba));
        }
        if !dec.more_images() {
            break;
        }
        dec.next_image().map_err(|e| format!("ovr next: {e}"))?;
    }
    let chosen = best_ge.or(best_max);
    match chosen {
        Some((_, w, h, rgba)) => {
            // .ovr 分层覆盖同一地理范围 → 像素有效尺寸按主文件尺寸比例缩放
            let px_eff = main.px * (main.width as f64 / w as f64);
            let py_eff = main.py * (main.height as f64 / h as f64);
            Ok(Some((w, h, rgba, px_eff, py_eff)))
        }
        None => Ok(None),
    }
}

/// Web Mercator（EPSG:3857，R=6378137）。lat 越界 clamp 到 ±85.05112878°。
/// y = R·asinh(tan φ)（2026-08-13 修正：此前误写 asinh(sin φ) 导致 3857 全透明）
fn lonlat_to_merc(lon: f64, lat: f64) -> (f64, f64) {
    const R: f64 = 6378137.0;
    let lat_c = lat.clamp(-85.05112878, 85.05112878);
    let x = lon.to_radians() * R;
    let y = lat_c.to_radians().tan().asinh() * R;
    (x, y)
}

/// 双线性采样；越界 → 透明 [0,0,0,0]
fn sample_bilinear(img: &[u8], width: u32, height: u32, col: f64, row: f64) -> [u8; 4] {
    let w = width as f64;
    let h = height as f64;
    if !(col >= 0.0 && row >= 0.0 && col <= w - 1.0 && row <= h - 1.0) {
        return [0, 0, 0, 0];
    }
    let c0 = col.floor() as usize;
    let r0 = row.floor() as usize;
    let c1 = (c0 + 1).min(width as usize - 1);
    let r1 = (r0 + 1).min(height as usize - 1);
    let fx = (col - c0 as f64) as f32;
    let fy = (row - r0 as f64) as f32;
    let idx = |r: usize, c: usize| (r * width as usize + c) * 4;
    let p00 = &img[idx(r0, c0)..idx(r0, c0) + 4];
    let p10 = &img[idx(r0, c1)..idx(r0, c1) + 4];
    let p01 = &img[idx(r1, c0)..idx(r1, c0) + 4];
    let p11 = &img[idx(r1, c1)..idx(r1, c1) + 4];
    let mut out = [0u8; 4];
    for k in 0..4 {
        let v = (p00[k] as f32) * (1.0 - fx) * (1.0 - fy)
            + (p10[k] as f32) * fx * (1.0 - fy)
            + (p01[k] as f32) * (1.0 - fx) * fy
            + (p11[k] as f32) * fx * fy;
        out[k] = v.round().clamp(0.0, 255.0) as u8;
    }
    out
}

fn tiff_basemap(req: &BasemapReq, nx: usize, ny: usize) -> Value {
    eprintln!(
        "[tiff] path={:?} len={} first_chars={:?}",
        req.path,
        req.path.len(),
        req.path.chars().take(8).map(|c| c as u32).collect::<Vec<_>>()
    );
    let key = match crate::resolve_terrain_path(&req.path) {
        Ok(p) => p,
        Err(e) => return serde_json::json!({ "error": e }),
    };
    eprintln!("[tiff] resolved={:?} exists={}", key, key.exists());
    let fallback = req.projection.as_deref().and_then(parse_projection);
    let main = match read_main_meta(&key, fallback) {
        Ok(m) => m,
        Err(e) => {
            return serde_json::json!({ "error": format!(
                "GeoTIFF 打开失败: {e}（确认文件是 TIFF 且数据源选择正确）"
            ) })
        }
    };
    let [min_lon, min_lat, max_lon, max_lat] = match req.bbox {
        Some(b) => b,
        None => {
            // bbox 缺省 → TIFF 覆盖范围（4326 直接算；3857 反算近似矩形）
            match main.projection {
                Projection::P4326 => [
                    main.origin_x,
                    main.origin_y - main.height as f64 * main.py,
                    main.origin_x + main.width as f64 * main.px,
                    main.origin_y,
                ],
                Projection::P3857 => {
                    let (x0, y1) = (main.origin_x, main.origin_y);
                    let (x1, y0) = (
                        main.origin_x + main.width as f64 * main.px,
                        main.origin_y - main.height as f64 * main.py,
                    );
                    [
                        merc_x_to_lon(x0),
                        merc_y_to_lat(y0),
                        merc_x_to_lon(x1),
                        merc_y_to_lat(y1),
                    ]
                }
            }
        }
    };
    if !(min_lon.is_finite()
        && min_lat.is_finite()
        && max_lon.is_finite()
        && max_lat.is_finite())
        || min_lon >= max_lon
        || min_lat >= max_lat
    {
        return serde_json::json!({ "error": "invalid bbox" });
    }

    // 目标像素数 ×4：保证所选 overview 分辨率 ≥ 目标网格 2 倍（双线性超采样余量）
    let target_pixels = nx * ny * 4;
    // 解码图像缓存命中 → 直接采样（瓦片 grid 固定时抽稀全图恒定，可复用）
    if let Some(c) = tiff_cache_get(&key) {
        return sample_tiff_from_img(&c, req, nx, ny);
    }
    // 未命中：图像源 .ovr 优先；失败/无 .ovr → 主文件（>30M 像素自动抽稀降级）
    let img = match pick_ovr_path(&key) {
        Some(ov) => match read_ovr_best(&ov, target_pixels, &main) {
            Ok(Some((w, h, rgba, px, py))) => CachedTiff {
                img_w: w,
                img_h: h,
                img_rgba: rgba,
                px_eff: px,
                py_eff: py,
                origin_x: main.origin_x,
                origin_y: main.origin_y,
                projection: main.projection,
                georef: main.georef,
                warning: main.warning.clone(),
            },
            Ok(None) => {
                return serde_json::json!({ "error": ".ovr 无可用分层" });
            }
            Err(e) => {
                eprintln!("[warn] ovr read failed ({e}) — fallback to main tiff");
                match read_main_pixels(&key, &main, target_pixels) {
                    Ok((w, h, rgba, px, py)) => CachedTiff {
                        img_w: w,
                        img_h: h,
                        img_rgba: rgba,
                        px_eff: px,
                        py_eff: py,
                        origin_x: main.origin_x,
                        origin_y: main.origin_y,
                        projection: main.projection,
                        georef: main.georef,
                        warning: main.warning.clone(),
                    },
                    Err(e2) => return serde_json::json!({ "error": e2 }),
                }
            }
        },
        None => match read_main_pixels(&key, &main, target_pixels) {
            Ok((w, h, rgba, px, py)) => CachedTiff {
                img_w: w,
                img_h: h,
                img_rgba: rgba,
                px_eff: px,
                py_eff: py,
                origin_x: main.origin_x,
                origin_y: main.origin_y,
                projection: main.projection,
                georef: main.georef,
                warning: main.warning.clone(),
            },
            Err(e) => return serde_json::json!({ "error": e }),
        },
    };
    let cached = Arc::new(img);
    tiff_cache_put(key, cached.clone());
    sample_tiff_from_img(&cached, req, nx, ny)
}

/// 从解码缓存图采样 RGBA 网格并组装响应。
fn sample_tiff_from_img(c: &CachedTiff, req: &BasemapReq, nx: usize, ny: usize) -> Value {
    let [min_lon, min_lat, max_lon, max_lat] = req.bbox.unwrap_or([-180.0, -90.0, 180.0, 90.0]);
    let img_w = c.img_w;
    let img_h = c.img_h;
    let px_eff = c.px_eff;
    let py_eff = c.py_eff;
    let mut rgba: Vec<u8> = Vec::with_capacity(nx * ny * 4);
    let step_lon = (max_lon - min_lon) / (nx as f64 - 1.0);
    let step_lat = (max_lat - min_lat) / (ny as f64 - 1.0);
    for j in 0..ny {
        let lat = max_lat - j as f64 * step_lat;
        for i in 0..nx {
            let lon = min_lon + i as f64 * step_lon;
            let (mx, my) = match c.projection {
                Projection::P4326 => (lon, lat),
                Projection::P3857 => lonlat_to_merc(lon, lat),
            };
            let col = (mx - c.origin_x) / px_eff;
            let row = (c.origin_y - my) / py_eff;
            rgba.extend_from_slice(&sample_bilinear(&c.img_rgba, img_w, img_h, col, row));
        }
    }

    let mut out = serde_json::json!({
        "nx": nx,
        "ny": ny,
        "min_lon": min_lon,
        "min_lat": min_lat,
        "max_lon": max_lon,
        "max_lat": max_lat,
        "resolution": format!("geotiff {}x{} px {:.4}°/px", img_w, img_h, px_eff / 111320.0),
        "source": "tiff",
        "projection": match c.projection {
            Projection::P4326 => "4326",
            Projection::P3857 => "3857",
        },
        "georef": c.georef,
        "warning": c.warning,
    });
    if let Value::Object(map) = &mut out {
        if let Value::Object(rgba_obj) = rgba_field(&rgba, req) {
            for (k, v) in rgba_obj {
                map.insert(k, v);
            }
        }
    }
    out
}

fn merc_x_to_lon(x: f64) -> f64 {
    x / 6378137.0 * 180.0 / std::f64::consts::PI
}
fn merc_y_to_lat(y: f64) -> f64 {
    (y / 6378137.0).sinh().atan() * 180.0 / std::f64::consts::PI
}

fn read_main_pixels(
    path: &Path,
    main: &TiffMeta,
    target_pixels: usize,
) -> Result<(u32, u32, Vec<u8>, f64, f64), String> {
    let pixels = main.width as usize * main.height as usize;
    if pixels > MAX_MAIN_PIXELS {
        // 大图且无 .ovr：边解码边抽稀（最近邻步长），RGBA 只保留采样网格，内存可控
        return read_main_pixels_downsampled(path, main, target_pixels);
    }
    let file = File::open(path).map_err(|e| format!("open tiff: {e}"))?;
    let mut dec = open_decoder(file)?;
    let rgba = decode_current_rgba(&mut dec)?;
    Ok((main.width, main.height, rgba, main.px, main.py))
}

/// 大图抽稀降级：全量解码原始像素（按 colortype 控制字节上限），只保留
/// `step` 步长的最近邻样本 → RGBA。抽样后尺寸 ≈ target_pixels（目标网格 ×4 超采样）。
fn read_main_pixels_downsampled(
    path: &Path,
    main: &TiffMeta,
    _target_pixels: usize,
) -> Result<(u32, u32, Vec<u8>, f64, f64), String> {
    let pixels = main.width as u64 * main.height as u64;
    let file = File::open(path).map_err(|e| format!("open tiff: {e}"))?;
    let mut dec = open_decoder(file)?;
    let ct = dec.colortype().map_err(|e| format!("colortype: {e}"))?;
    let bpp = match ct {
        ColorType::Gray(8) => 1u64,
        ColorType::Gray(16) => 2,
        ColorType::RGB(8) => 3,
        ColorType::RGB(16) => 6,
        ColorType::RGBA(8) => 4,
        ColorType::RGBA(16) => 8,
        other => return Err(format!("不支持的 TIFF 像素类型: {other:?}")),
    };
    let bytes = pixels * bpp;
    if bytes > MAX_MAIN_BYTES {
        return Err(format!(
            "GeoTIFF 过大（{pixels} 像素 × {bpp}B ≈ {}MB > {}MB）且无可用 .ovr——请生成 GDAL overview（gdaladdo -ro）",
            bytes / 1024 / 1024,
            MAX_MAIN_BYTES / 1024 / 1024
        ));
    }
    // 抽稀步长：抽样后像素数 ≈ 固定高分辨率（≈4096×2048，0.1° 级）。
    // 2026-08-13 修复：原 target_pixels(=nx*ny*4) 在 700MB 大图上算出 step≈60
    // → 抽稀仅 360×180（1°/px），双线性采样把采样网格上的水体像素混入，
    // 华北平原整体偏蓝（北京视口看起来像海洋）；提高到 0.1° 级后偏色 ≤2。
    let ds_target = 4096u64 * 2048u64;
    let step = ((pixels as f64 / ds_target as f64).sqrt().ceil() as usize).max(1);
    let nw = ((main.width as usize + step - 1) / step).max(2);
    let nh = ((main.height as usize + step - 1) / step).max(2);
    let result = dec.read_image().map_err(|e| format!("tiff decode: {e}"))?;
    let w = main.width as usize;
    let to8 = |v: u16| (v >> 8) as u8;
    let mut out = Vec::with_capacity(nw * nh * 4);
    match (ct, result) {
        (ColorType::Gray(8), DecodingResult::U8(v)) => {
            for j in 0..nh {
                let row = j * step;
                for i in 0..nw {
                    let col = i * step;
                    let x = v[row * w + col];
                    out.extend_from_slice(&[x, x, x, 255]);
                }
            }
        }
        (ColorType::Gray(16), DecodingResult::U16(v)) => {
            for j in 0..nh {
                let row = j * step;
                for i in 0..nw {
                    let col = i * step;
                    let b = to8(v[row * w + col]);
                    out.extend_from_slice(&[b, b, b, 255]);
                }
            }
        }
        (ColorType::RGB(8), DecodingResult::U8(v)) => {
            for j in 0..nh {
                let row = j * step;
                for i in 0..nw {
                    let col = i * step;
                    let p = &v[(row * w + col) * 3..];
                    out.extend_from_slice(&[p[0], p[1], p[2], 255]);
                }
            }
        }
        (ColorType::RGB(16), DecodingResult::U16(v)) => {
            for j in 0..nh {
                let row = j * step;
                for i in 0..nw {
                    let col = i * step;
                    let p = &v[(row * w + col) * 3..];
                    out.extend_from_slice(&[to8(p[0]), to8(p[1]), to8(p[2]), 255]);
                }
            }
        }
        (ColorType::RGBA(8), DecodingResult::U8(v)) => {
            for j in 0..nh {
                let row = j * step;
                for i in 0..nw {
                    let col = i * step;
                    let p = &v[(row * w + col) * 4..];
                    out.extend_from_slice(&[p[0], p[1], p[2], p[3]]);
                }
            }
        }
        (ColorType::RGBA(16), DecodingResult::U16(v)) => {
            for j in 0..nh {
                let row = j * step;
                for i in 0..nw {
                    let col = i * step;
                    let p = &v[(row * w + col) * 4..];
                    out.extend_from_slice(&[to8(p[0]), to8(p[1]), to8(p[2]), to8(p[3])]);
                }
            }
        }
        other => return Err(format!("不支持的 TIFF 像素类型: {other:?}")),
    }
    // 有效像素尺寸按主文件尺寸比例缩放（与 .ovr 分层一致）
    let px_eff = main.px * (main.width as f64 / nw as f64);
    let py_eff = main.py * (main.height as f64 / nh as f64);
    Ok((nw as u32, nh as u32, out, px_eff, py_eff))
}

// ===================== /api/wms（GeoServer 代理） =====================

#[derive(Deserialize)]
pub struct WmsReq {
    /// GeoServer WMS 端点，如 http://192.168.1.10:8080/geoserver/wms
    base_url: String,
    /// "minx,miny,maxx,maxy"（经纬度/3857 米，前端按 crs 算好）
    bbox: String,
    width: u32,
    height: u32,
    layers: String,
    /// "EPSG:4326" | "EPSG:3857"
    crs: String,
    format: Option<String>,
}

fn wms_client() -> &'static reqwest::Client {
    static CLIENT: OnceLock<reqwest::Client> = OnceLock::new();
    CLIENT.get_or_init(|| {
        reqwest::Client::builder()
            .timeout(Duration::from_secs(30))
            .build()
            .expect("wms client")
    })
}

/// GET /api/wms?base_url=...&bbox=...&width=...&height=...&layers=...&crs=...
/// 内部固定 WMS 1.1.1 + SRS（BBOX 恒为 x,y 顺序），GeoServer 兼容，规避 1.3.0 EPSG:4326
/// 轴序反转陷阱；原样返回 GeoServer 的图片二进制（image/png 等）。
pub async fn wms_proxy(Query(q): Query<WmsReq>) -> Response {
    if !(q.base_url.starts_with("http://") || q.base_url.starts_with("https://")) {
        return json_error("base_url 必须是 http(s):// 地址");
    }
    if q.width > 4096 || q.height > 4096 || q.width == 0 || q.height == 0 {
        return json_error("width/height 必须在 1..=4096");
    }
    let format = q.format.clone().unwrap_or_else(|| "image/png".into());
    let params: Vec<(&str, String)> = vec![
        ("SERVICE", "WMS".into()),
        ("VERSION", "1.1.1".into()),
        ("REQUEST", "GetMap".into()),
        ("LAYERS", q.layers.clone()),
        ("STYLES", String::new()),
        ("SRS", q.crs.clone()),
        ("BBOX", q.bbox.clone()),
        ("WIDTH", q.width.to_string()),
        ("HEIGHT", q.height.to_string()),
        ("FORMAT", format),
        ("TRANSPARENT", "true".into()),
    ];
    // reqwest .query 自动做 URL 编码（layer 名 workspace:layer 等）
    match wms_client().get(&q.base_url).query(&params).send().await {
        Ok(resp) => {
            let status = resp.status();
            let content_type = resp
                .headers()
                .get(header::CONTENT_TYPE)
                .and_then(|v| v.to_str().ok())
                .unwrap_or("image/png")
                .to_string();
            match resp.bytes().await {
                Ok(bytes) => {
                    if !status.is_success() {
                        return json_error(&format!("GeoServer HTTP {status}: {}", String::from_utf8_lossy(&bytes).chars().take(300).collect::<String>()));
                    }
                    Response::builder()
                        .status(status)
                        .header(header::CONTENT_TYPE, content_type)
                        .header(header::CACHE_CONTROL, "no-store")
                        .body(Body::from(bytes))
                        .unwrap_or_else(|e| json_error(&format!("response: {e}")))
                }
                Err(e) => json_error(&format!("read GeoServer body: {e}")),
            }
        }
        Err(e) => json_error(&format!("GeoServer 请求失败: {e}")),
    }
}

fn json_error(msg: &str) -> Response {
    Response::builder()
        .status(500)
        .header(header::CONTENT_TYPE, "application/json")
        .body(Body::from(serde_json::json!({ "error": msg }).to_string()))
        .unwrap()
}

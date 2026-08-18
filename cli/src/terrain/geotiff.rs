//! GeoTIFF 数据源（技术方案 4.3：tiff crate（image-rs）tile/strip 按需读取，纯 Rust）。
//!
//! 2026-08-07 主管拍板：外部格式按需读取，GeoTIFF 优先级最高。
//! 两级策略：chunk（tile/strip）数 ≤ 阈值 → open 全量读入内存网格（现状语义，
//! 快速路径）；超过阈值 → chunk 按需解压 + LRU 缓存（内存有界，打开 O(IFD)）。
//! 地理参考：ModelTiepoint + ModelPixelScale（无旋转；ModelTransformation 对角兼容）。
//! 采样语义与旧 geotiff crate 实现一致：双线性插值、NaN/NoData → None、
//! 无地理参考拒绝（Data 错误）。

use std::path::Path;
use std::sync::Mutex;

use tiff::decoder::{ChunkType, Decoder, DecodingResult};
use tiff::tags::Tag;

use super::{GeoBounds, Sample, TerrainSource};
use crate::error::AppError;

/// 全量路径 chunk 数阈值（≤ 此值 open 全量；超过 → LRU 按需）。
const FULL_LOAD_CHUNK_THRESHOLD: u32 = 64;
/// LRU 缓存 chunk 上限（256² f32 ≈ 256KB → 512 chunk ≈ 128MB；strip 更小）。
const LRU_MAX_CHUNKS: usize = 512;
/// Overview 全量层内存上限（所有层合计；超过 → 停止收集，避免大文件 open 击穿内存）。
const OVERVIEW_MAX_BYTES: usize = 128 * 1024 * 1024;
/// GDAL_NODATA tag（42113）。
const TAG_GDAL_NODATA: Tag = Tag::Unknown(42113);

/// 地理参考（tiepoint + pixelscale，无旋转）。
#[derive(Debug, Clone, Copy)]
pub struct GeoRef {
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
    pub cell_lon_deg: f64,
    pub cell_lat_deg: f64,
    /// 源像素行 0 = 北（scale_y < 0）→ 采样行翻转。
    pub row_flip: bool,
    /// 源像素列 0 = 东（scale_x < 0，罕见）→ 采样列翻转。
    pub col_flip: bool,
}

/// GeoTIFF 源（两级：全量内存网格 / chunk 按需 + LRU）。
pub struct GeoTiffSource {
    geo: GeoRef,
    width: u32,
    height: u32,
    /// 全量网格（行优先，NaN = 空洞）。
    full: Option<Vec<f32>>,
    /// chunk 按需状态（大文件）。
    lazy: Option<LazyState>,
    /// Overview 降采样层（兄弟 IFD，2026-08-13 P9 T4）：与主图同地理参考原点、
    /// 尺寸更小 → open 时全量读入内存。采样按请求分辨率选层（`sample_at_res`）。
    /// 空 = 文件无可用 overview（回退主层，行为与旧版一致）。
    overviews: Vec<OverviewLevel>,
}

/// Overview 降采样层（兄弟 IFD 全量内存）。
struct OverviewLevel {
    geo: GeoRef,
    width: u32,
    height: u32,
    /// 行优先（NaN = 空洞）。
    data: Vec<f32>,
}

/// chunk 按需状态（大文件路径）。
struct LazyState {
    decoder: Mutex<Decoder<std::fs::File>>,
    chunk_w: u32,
    chunk_h: u32,
    chunks_x: u32,
    #[allow(dead_code)] // 未来 BulkPrefetch 预取范围计算用
    chunks_y: u32,
    cache: Mutex<ChunkCache>,
    samples: u32,
    no_data: Option<f32>,
}

/// chunk 解压缓存（FIFO 有界）。
#[derive(Debug)]
struct ChunkCache {
    map: std::collections::HashMap<u32, Vec<f32>>,
    order: std::collections::VecDeque<u32>,
    max_chunks: usize,
}

impl ChunkCache {
    fn with_max(max_chunks: usize) -> Self {
        Self {
            map: std::collections::HashMap::new(),
            order: std::collections::VecDeque::new(),
            max_chunks: max_chunks.max(1),
        }
    }
    fn insert(&mut self, idx: u32, chunk: Vec<f32>) {
        if self.map.contains_key(&idx) {
            return;
        }
        if self.map.len() >= self.max_chunks {
            if let Some(old) = self.order.pop_front() {
                self.map.remove(&old);
            }
        }
        self.map.insert(idx, chunk);
        self.order.push_back(idx);
    }
}

fn lock_cache(m: &Mutex<ChunkCache>) -> std::sync::MutexGuard<'_, ChunkCache> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

fn lock_decoder(
    m: &Mutex<Decoder<std::fs::File>>,
) -> std::sync::MutexGuard<'_, Decoder<std::fs::File>> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

/// 解析地理参考（ModelTiepoint + ModelPixelScale，无旋转）。
/// 拒绝：无 tiepoint/scale（无地理参考）、退化 cell、非仿射 Transformation。
pub fn parse_georef(
    dec: &mut Decoder<std::fs::File>,
    width: u32,
    height: u32,
) -> Result<GeoRef, AppError> {
    // tiepoint：[(raster_i, raster_j, raster_k, model_x, model_y, model_z) ...]（doubles）
    let tie = dec
        .get_tag_f64_vec(Tag::ModelTiepointTag)
        .map_err(|_| AppError::Data("geotiff missing ModelTiepoint".into()))?;
    if tie.len() < 6 {
        return Err(AppError::Data("geotiff ModelTiepoint truncated".into()));
    }
    // 标准 GeoTIFF 语义：tie[0..3] = 栅格坐标 (raster_i, raster_j, raster_k)，
    // tie[3..6] = 模型坐标 (model_x, model_y, model_z)。旧实现把 tie[2]/tie[3]
    // 当 (tx, ty) 是错位（raster_k 恒 0，model_x 被吞）——2026-08-08 主管测试
    // gdal 裁剪东亚 GeoTIFF 暴露（convert 输出 origin (0, 69.999861) 应为
    // (70, 60)）。
    let (ti, tj, tx, ty) = (tie[0], tie[1], tie[3], tie[4]);
    // pixelscale：[sx, sy, sz]（doubles）
    let scale = dec
        .get_tag_f64_vec(Tag::ModelPixelScaleTag)
        .map_err(|_| AppError::Data("geotiff missing ModelPixelScale".into()))?;
    if scale.len() < 2 {
        return Err(AppError::Data("geotiff ModelPixelScale truncated".into()));
    }
    let (sx, sy0) = (scale[0], scale[1]);
    if sx == 0.0 || sy0 == 0.0 {
        return Err(AppError::Data("geotiff degenerate pixelscale".into()));
    }
    // 行向判定（2026-08-11 主管 Beijing_DEM.tif 暴露）：tiepoint (0,0) + ModelPixelScale
    // sy>0 时 GDAL north-up 与 south-up 布局无法靠 tj 区分（都 tj=0），旧逻辑仅用
    // 「south-up 假设下北界 |lat|>90 越界」检测 north-up——北京数据北界 42.68° 不越界
    // → 误判 south-up → origin 取左上角 41.06 当南界 → 全区域采样 OutOfBounds。
    // 新判定：分别算 south-up 假设的北界（y0+(h-1)*sy）与 north-up 假设的南界
    // （y0-(h-1)*sy），越界者淘汰；两者都合理（或都越界）时默认 north-up（GDAL /
    // ArcGIS / Global Mapper 等主流工具默认 north-up，tiepoint (0,0) = 左上角）；
    // tiepoint 不在行 0（tj>0，如左下角布局）保守 south-up。
    let y0_tie = ty - tj * sy0;
    let sy = resolve_sy(sy0, tj, height as usize, y0_tie);
    // 像素 (0,0) 的模型坐标（仿射：x = tx + (col - ti)*sx, y = ty + (row - tj)*sy）
    let x0 = tx - ti * sx;
    let y0 = ty - tj * sy;
    let x1 = x0 + (width as f64 - 1.0) * sx;
    let y1 = y0 + (height as f64 - 1.0) * sy;
    // 无地理参考启发（与旧实现一致：extent 即像素范围 → 拒绝）
    if x0.abs() < 1e-9 && y0.abs() < 1e-9 && (x1 - width as f64).abs() < 1e-9 && (y1 - height as f64).abs() < 1e-9
    {
        return Err(AppError::Data(
            "geotiff has no georeferencing (model extent == pixel range)".into(),
        ));
    }
    let min_lon = x0.min(x1);
    let max_lon = x0.max(x1);
    let min_lat = y0.min(y1);
    let max_lat = y0.max(y1);
    let cell_lon_deg = sx.abs();
    let cell_lat_deg = sy.abs();
    if cell_lon_deg <= 0.0 || cell_lat_deg <= 0.0 || min_lon >= max_lon || min_lat >= max_lat {
        return Err(AppError::Data("geotiff degenerate geo bounds".into()));
    }
    Ok(GeoRef {
        min_lon,
        min_lat,
        max_lon,
        max_lat,
        cell_lon_deg,
        cell_lat_deg,
        row_flip: sy < 0.0,
        col_flip: sx < 0.0,
    })
}

/// 行向解析：ModelPixelScale sy0（>0 时 GDAL 写正）、tiepoint 行 tj、行数 height、
/// tiepoint 的 model_y（y0_tie = ty - tj*sy0）→ 实际 sy（正 = south-up，行 0 = 南；
/// 负 = north-up，行 0 = 北，需 row_flip）。判定逻辑见 parse_georef 注释。
fn resolve_sy(sy0: f64, tj: f64, height: usize, y0_tie: f64) -> f64 {
    if sy0 > 0.0 {
        let h = height as f64 - 1.0;
        let south_max = y0_tie + h * sy0;
        let north_min = y0_tie - h * sy0;
        let south_ok = (-90.0..=90.0).contains(&south_max);
        let north_ok = (-90.0..=90.0).contains(&north_min);
        if south_ok && !north_ok {
            sy0
        } else if north_ok && !south_ok {
            -sy0
        } else if tj > 0.0 {
            sy0
        } else {
            -sy0
        }
    } else {
        sy0
    }
}

/// GDAL_NODATA tag → 空洞值（None = 无标记；F32/F64 的 NaN 仍按空洞）。
pub fn read_gdal_nodata(dec: &mut Decoder<std::fs::File>) -> Option<f32> {
    dec.get_tag_ascii_string(TAG_GDAL_NODATA)
        .ok()
        .and_then(|s| s.trim().parse::<f64>().ok())
        .map(|v| v as f32)
}

/// DecodingResult → f32 展平（chunky 多波段只取 band 0）。
fn chunk_to_f32(result: DecodingResult, samples: u32) -> Vec<f32> {
    let band0 = |d: Vec<f32>| -> Vec<f32> {
        if samples <= 1 {
            d
        } else {
            d.chunks_exact(samples as usize).map(|ch| ch[0]).collect()
        }
    };
    match result {
        DecodingResult::U8(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::U16(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::U32(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::U64(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::I8(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::I16(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::I32(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::I64(d) => d.into_iter().map(|v| v as f32).collect(),
        DecodingResult::F32(d) => band0(d),
        DecodingResult::F64(d) => band0(d.into_iter().map(|v| v as f32).collect()),
    }
}

/// 应用空洞标记（NoData 值 → NaN）。
fn apply_nodata(data: &mut [f32], no_data: Option<f32>) {
    if let Some(nd) = no_data {
        for v in data.iter_mut() {
            if *v == nd {
                *v = f32::NAN;
            }
        }
    }
}

impl GeoTiffSource {
    pub fn open(path: &Path) -> Result<Self, AppError> {
        let file = std::fs::File::open(path)?;
        let mut dec = Decoder::new(file)
            .map_err(|e| AppError::Data(format!("tiff decode failed: {e}")))?;
        let (width, height) = dec
            .dimensions()
            .map_err(|e| AppError::Data(format!("tiff dimensions failed: {e}")))?;
        if width == 0 || height == 0 {
            return Err(AppError::Data("geotiff zero dimensions".into()));
        }
        let geo = parse_georef(&mut dec, width, height)?;
        let samples = dec
            .find_tag_unsigned::<u16>(Tag::SamplesPerPixel)
            .ok()
            .flatten()
            .unwrap_or(1) as u32;
        if samples == 0 {
            return Err(AppError::Data("geotiff zero samples".into()));
        }
        let no_data = read_gdal_nodata(&mut dec);
        let chunk_type = dec.get_chunk_type();
        let chunk_count = match chunk_type {
            ChunkType::Tile => dec
                .tile_count()
                .map_err(|e| AppError::Data(format!("tiff tile_count failed: {e}")))?,
            ChunkType::Strip => dec
                .strip_count()
                .map_err(|e| AppError::Data(format!("tiff strip_count failed: {e}")))?,
        };
        let (chunk_w, chunk_h) = match chunk_type {
            ChunkType::Tile => {
                let tw = dec
                    .get_tag_u32(Tag::TileWidth)
                    .map_err(|e| AppError::Data(format!("tiff TileWidth failed: {e}")))?;
                let th = dec
                    .get_tag_u32(Tag::TileLength)
                    .map_err(|e| AppError::Data(format!("tiff TileLength failed: {e}")))?;
                (tw.max(1), th.max(1))
            }
            ChunkType::Strip => {
                let rps = dec
                    .get_tag_u32(Tag::RowsPerStrip)
                    .map_err(|e| AppError::Data(format!("tiff RowsPerStrip failed: {e}")))?;
                (width.max(1), rps.max(1))
            }
        };

        // 两级策略：小文件全量（现状语义）；大文件 chunk 按需 + LRU
        if chunk_count <= FULL_LOAD_CHUNK_THRESHOLD {
            let result = dec
                .read_image()
                .map_err(|e| AppError::Data(format!("tiff read_image failed: {e}")))?;
            let mut data = chunk_to_f32(result, samples);
            apply_nodata(&mut data, no_data);
            Ok(Self {
                geo,
                width,
                height,
                full: Some(data),
                lazy: None,
                overviews: Vec::new(),
            })
        } else {
            // Overview 收集（P9 T4）：遍历兄弟 IFD，识别与主图同地理参考原点、
            // 尺寸更小的降采样层 → 全量读入内存（tiff crate 支持兄弟 IFD 链，
            // SubIFD 内嵌 overview 不支持——文档说明）。遍历后 seek 回主图。
            let overviews = collect_overviews(&mut dec, &geo, width, height);
            let chunks_x = width.div_ceil(chunk_w);
            let chunks_y = height.div_ceil(chunk_h);
            Ok(Self {
                geo,
                width,
                height,
                full: None,
                lazy: Some(LazyState {
                    decoder: Mutex::new(dec),
                    chunk_w,
                    chunk_h,
                    chunks_x,
                    chunks_y,
                    cache: Mutex::new(ChunkCache::with_max(LRU_MAX_CHUNKS)),
                    samples,
                    no_data,
                }),
                overviews,
            })
        }
    }

    /// 双线性插值采样（经纬度，度）。NaN/NoData → None；出界 → None。
    fn sample(&self, lon: f64, lat: f64) -> Option<f64> {
        let w = self.width as f64;
        let h = self.height as f64;
        let (col, row) = self.geo.to_pixel(lon, lat, w, h)?;
        let c0 = col.floor() as isize;
        let r0 = row.floor() as isize;
        if c0 < 0 || r0 < 0 || c0 + 1 >= self.width as isize || r0 + 1 >= self.height as isize {
            return None;
        }
        let w_c = col - c0 as f64;
        let w_r = row - r0 as f64;
        let (r0, c0) = (r0 as usize, c0 as usize);
        let v00 = self.pixel(r0, c0)?;
        let v01 = self.pixel(r0, c0 + 1)?;
        let v10 = self.pixel(r0 + 1, c0)?;
        let v11 = self.pixel(r0 + 1, c0 + 1)?;
        Some(interp2(v00, v01, v10, v11, w_c, w_r))
    }

    /// 像素值（NaN = 空洞 → None）。源像素行序（row_flip 已由 to_pixel 处理）。
    fn pixel(&self, r: usize, c: usize) -> Option<f32> {
        if r >= self.height as usize || c >= self.width as usize {
            return None;
        }
        if let Some(full) = &self.full {
            let v = full[r * self.width as usize + c];
            return if v.is_nan() { None } else { Some(v) };
        }
        let lazy = self.lazy.as_ref()?;
        let cx = (c as u32) / lazy.chunk_w;
        let cy = (r as u32) / lazy.chunk_h;
        let idx = cy * lazy.chunks_x + cx;
        let chunk = self.lazy_chunk(idx)?;
        let lc = (c as u32) % lazy.chunk_w;
        let lr = (r as u32) % lazy.chunk_h;
        let l = (lr as usize) * (lazy.chunk_w as usize) + lc as usize;
        let v = *chunk.get(l)?;
        if v.is_nan() {
            None
        } else {
            Some(v)
        }
    }

    /// 加载 chunk（缓存命中直取；未命中锁内解压 + 双检插入）。
    fn lazy_chunk(&self, idx: u32) -> Option<Vec<f32>> {        let lazy = self.lazy.as_ref()?;
        {
            let cache = lock_cache(&lazy.cache);
            if let Some(ch) = cache.map.get(&idx) {
                return Some(ch.clone());
            }
        }
        let (dims, mut data) = {
            let mut dec = lock_decoder(&lazy.decoder);
            let dims = dec.chunk_data_dimensions(idx);
            let result = dec.read_chunk(idx).ok()?;
            let mut data = chunk_to_f32(result, lazy.samples);
            apply_nodata(&mut data, lazy.no_data);
            (dims, data)
        };
        // 边界 tile 尺寸 < chunk 标称：填充 NaN（访问防御）
        let nominal = (lazy.chunk_w as usize) * (lazy.chunk_h as usize);
        if data.len() < nominal {
            let (cw, chh) = (lazy.chunk_w as usize, lazy.chunk_h as usize);
            let mut buf = vec![f32::NAN; cw * chh];
            for rr in 0..chh.min(dims.1 as usize) {
                for cc in 0..cw.min(dims.0 as usize) {
                    let src = rr * (dims.0 as usize) + cc;
                    buf[rr * cw + cc] = data[src];
                }
            }
            data = buf;
        }
        let mut cache = lock_cache(&lazy.cache);
        if !cache.map.contains_key(&idx) {
            cache.insert(idx, data.clone());
        }
        Some(data)
    }

    /// overview 层采样（与主层 bounds 语义一致：出界 → OutOfBounds；层内空洞 → NoData）。
    fn overview_sample(&self, ov: &OverviewLevel, lon: f64, lat: f64) -> Sample {
        if let Some(b) = self.bounds() {
            if !b.contains(lon, lat) {
                return Sample::OutOfBounds;
            }
        }
        let w = ov.width as f64;
        let h = ov.height as f64;
        let (col, row) = match ov.geo.to_pixel(lon, lat, w, h) {
            Some(p) => p,
            None => return Sample::NoData, // 主层内但 overview 边缘外（浮点）→ 空洞语义
        };
        match bilinear_at(&ov.data, ov.width, ov.height, col, row) {
            Some(hh) => Sample::Land(hh),
            None => Sample::NoData,
        }
    }
}

impl GeoRef {
    /// 经纬度 → 源像素浮点 (col, row)（含行/列翻转；出界 → None）。
    fn to_pixel(&self, lon: f64, lat: f64, width: f64, height: f64) -> Option<(f64, f64)> {
        let fc = (lon - self.min_lon) / self.cell_lon_deg;
        let fr = (lat - self.min_lat) / self.cell_lat_deg;
        let col = if self.col_flip { width - 1.0 - fc } else { fc };
        let row = if self.row_flip { height - 1.0 - fr } else { fr };
        if col < 0.0 || row < 0.0 || col >= width || row >= height {
            None
        } else {
            Some((col, row))
        }
    }
}

/// 遍历兄弟 IFD 收集 Overview 层（P9 T4）。
/// 判定：后续 IFD 尺寸更小（≤ 主图对应维）且地理参考原点与主图对齐
/// （原点差 ≤ 两方 cell 半宽，容 tile 边缘舍入）→ 全量读入。
/// 内存护栏：合计超 OVERVIEW_MAX_BYTES → 停止收集。
/// 失败防御：任一步 IO/格式错误 → 停止遍历（保留已收集层，不报错回退主层）。
fn collect_overviews(
    dec: &mut Decoder<std::fs::File>,
    main_geo: &GeoRef,
    main_w: u32,
    main_h: u32,
) -> Vec<OverviewLevel> {
    let mut out = Vec::new();
    let mut total_bytes = 0usize;
    while dec.more_images() {
        if dec.next_image().is_err() {
            break;
        }
        let Ok((ow, oh)) = dec.dimensions() else { break };
        if ow == 0 || oh == 0 || ow > main_w || oh > main_h {
            continue; // 非降采样（更大/异常）→ 跳过，继续下一个 IFD
        }
        // 忽略重复 IFD（tiff 链上可能出现尺寸相同的页）
        if ow == main_w && oh == main_h {
            continue;
        }
        let Ok(ogeo) = parse_georef(dec, ow, oh) else { continue };
        // 原点对齐判定（两方 cell 半宽容差）
        let lon_ok = (ogeo.min_lon - main_geo.min_lon).abs()
            <= main_geo.cell_lon_deg * 0.5 + ogeo.cell_lon_deg * 0.5;
        let lat_ok = (ogeo.min_lat - main_geo.min_lat).abs()
            <= main_geo.cell_lat_deg * 0.5 + ogeo.cell_lat_deg * 0.5;
        if !lon_ok || !lat_ok {
            continue;
        }
        let osamples = dec
            .find_tag_unsigned::<u16>(Tag::SamplesPerPixel)
            .ok()
            .flatten()
            .unwrap_or(1) as u32;
        let onodata = read_gdal_nodata(dec);
        let Ok(oresult) = dec.read_image() else { break };
        let mut odata = chunk_to_f32(oresult, osamples);
        apply_nodata(&mut odata, onodata);
        total_bytes += odata.len() * 4;
        if total_bytes > OVERVIEW_MAX_BYTES {
            break; // 内存护栏：丢弃最后（已超限）层并停止
        }
        out.push(OverviewLevel {
            geo: ogeo,
            width: ow,
            height: oh,
            data: odata,
        });
    }
    // 回主图（collect 遍历移动了 decoder 的当前 IFD）
    let _ = dec.seek_to_image(0);
    out
}

impl TerrainSource for GeoTiffSource {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        self.sample(lon, lat)
    }

    /// Overview 感知采样（P9 T4）：请求分辨率 `max_cell_deg` 下的低分辨率层选层。
    /// 选层原则：cell ≤ 请求分辨率（宁细勿粗）的最粗层——精度不低于请求、解压/查询
    /// 成本最低。无合适层（含文件无 overview）→ 回退主层（行为与旧版一致）。
    /// 精度语义：overview 为降采样近似（GDAL/tifffile 写多页时的实际降采样方式），
    /// 供 FMM 粗层等低分辨率采样使用；verify/逐段精查仍走全分辨率（主层），
    /// 粗层误差由回退链兜底（与 docs/01 三层架构一致）。
    fn sample_at_res(&self, lon: f64, lat: f64, max_cell_deg: f64) -> Sample {
        if max_cell_deg > 0.0 {
            let best = self
                .overviews
                .iter()
                .filter(|o| o.geo.cell_lon_deg <= max_cell_deg + 1e-12)
                .max_by(|a, b| a.geo.cell_lon_deg.total_cmp(&b.geo.cell_lon_deg));
            if let Some(ov) = best {
                return self.overview_sample(ov, lon, lat);
            }
        }
        self.sample_at(lon, lat)
    }

    fn bounds(&self) -> Option<GeoBounds> {
        Some(GeoBounds {
            min_lon: self.geo.min_lon,
            min_lat: self.geo.min_lat,
            max_lon: self.geo.max_lon,
            max_lat: self.geo.max_lat,
        })
    }

    fn resolution_desc(&self) -> String {
        let mut s = format!(
            "geotiff {}x{} cell {:.6}deg x {:.6}deg",
            self.width, self.height, self.geo.cell_lon_deg, self.geo.cell_lat_deg
        );
        if !self.overviews.is_empty() {
            s.push_str(&format!(" + {} overview", self.overviews.len()));
        }
        s
    }
}

/// 双线性插值（数据行优先，NaN = 空洞；越界 → None）。
fn bilinear_at(data: &[f32], width: u32, height: u32, col: f64, row: f64) -> Option<f64> {
    let c0 = col.floor() as isize;
    let r0 = row.floor() as isize;
    if c0 < 0 || r0 < 0 || c0 + 1 >= width as isize || r0 + 1 >= height as isize {
        return None;
    }
    let w_c = col - c0 as f64;
    let w_r = row - r0 as f64;
    let (r0, c0) = (r0 as usize, c0 as usize);
    let idx = |r: usize, c: usize| r * width as usize + c;
    let v00 = data[idx(r0, c0)];
    let v01 = data[idx(r0, c0 + 1)];
    let v10 = data[idx(r0 + 1, c0)];
    let v11 = data[idx(r0 + 1, c0 + 1)];
    if v00.is_nan() || v01.is_nan() || v10.is_nan() || v11.is_nan() {
        return None;
    }
    Some(interp2(v00, v01, v10, v11, w_c, w_r))
}

/// 双线性插值（与 BuiltinSource 同式，保证数值一致）。
#[inline]
fn interp2(v00: f32, v01: f32, v10: f32, v11: f32, w_c: f64, w_r: f64) -> f64 {
    let h00 = v00 as f64;
    let h01 = v01 as f64;
    let h10 = v10 as f64;
    let h11 = v11 as f64;
    let top = h00 + (h01 - h00) * w_c;
    let bot = h10 + (h11 - h10) * w_c;
    top + (bot - top) * w_r
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::TerrainSource;

    /// 项目内 GeoTIFF（_test_small.tif，300×200，cell 0.001°）：路径定位。
    fn test_tif() -> Option<std::path::PathBuf> {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/_test_small.tif");
        p.exists().then_some(p)
    }

    /// 小文件（chunk ≤ 阈值）：全量路径采样正常。
    #[test]
    fn small_full_load_sampling() {
        let Some(p) = test_tif() else {
            eprintln!("skip: _test_small.tif not found");
            return;
        };
        let s = GeoTiffSource::open(&p).unwrap();
        assert!(s.full.is_some(), "small file should take full-load path");
        assert_eq!(s.width, 200);
        assert_eq!(s.height, 300);
        let b = s.bounds().unwrap();
        assert!((b.min_lon - 116.0).abs() < 1e-9);
        assert!((b.min_lat - 38.7).abs() < 1e-9);
        // 中心采样有值
        assert!(s.height_at(116.1, 38.85).is_some());
        // 出界 → None
        assert!(s.height_at(116.5, 38.85).is_none());
        assert!(s.height_at(116.1, 38.5).is_none());
    }

    /// 全量路径与旧 geotiff crate 采样一致（同源数值口径）。
    #[test]
    fn full_matches_old_impl() {
        let Some(p) = test_tif() else {
            eprintln!("skip");
            return;
        };
        let s = GeoTiffSource::open(&p).unwrap();
        let f = std::fs::File::open(&p).unwrap();
        let old = geotiff::GeoTiff::read(f).unwrap();
        let b = s.bounds().unwrap();
        for i in 0..8 {
            let t = (i as f64 + 0.5) / 8.0;
            let lon = b.min_lon + (b.max_lon - b.min_lon) * t;
            let lat = b.min_lat + (b.max_lat - b.min_lat) * t;
            let got = s.height_at(lon, lat);
            let want = old
                .get_value_at::<f64>(&geo_types::Coord { x: lon, y: lat }, 0)
                .filter(|v| v.is_finite());
            match (got, want) {
                (Some(a), Some(b)) => assert!((a - b).abs() < 1e-6, "mismatch at ({lon},{lat}): {a} vs {b}"),
                (None, None) => {}
                (a, b) => panic!("mismatch at ({lon},{lat}): {a:?} vs {b:?}"),
            }
        }
    }

    /// 两级策略：大 chunk 数文件走 LRU 路径（构造伪 tiled GeoTIFF 不可行——用 strip 大文件
    /// 防御：这里直接验证 LRU 缓存 FIFO 行为）。
    #[test]
    fn chunk_cache_fifo() {
        let mut c = ChunkCache::with_max(2);
        c.insert(1, vec![1.0]);
        c.insert(2, vec![2.0]);
        assert!(c.map.contains_key(&1) && c.map.contains_key(&2));
        c.insert(3, vec![3.0]);
        assert!(!c.map.contains_key(&1), "oldest chunk 1 evicted");
        assert!(c.map.contains_key(&2) && c.map.contains_key(&3));
        assert_eq!(c.map.len(), 2);
    }

    /// LRU 路径（大文件 >64 chunks，tifffile 生成）：chunk 按需采样 = 生成公式原值。
    /// 文件 `data/lazy_test_4096.tif` 由开发期手动生成（2026-08-08 清理删除，
    /// 可按需重生成——tifffile 写 4096² 分块 tif 即可）——
    /// 不存在时跳过（逻辑由其余测试 + 代码审查覆盖）。
    #[test]
    fn lazy_large_file_sampling() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/lazy_test_4096.tif");
        if !p.exists() {
            eprintln!("skip: lazy_test_4096.tif not found");
            return;
        }
        let s = GeoTiffSource::open(&p).unwrap();
        assert!(s.lazy.is_some(), "large file should take lazy LRU path");
        assert!(s.full.is_none());
        // 中心格点 (2048, 2048)：lon = min_lon + 2048*cell；源像素行 = 4095-2048（row_flip）
        let b = s.bounds().unwrap();
        let lon = b.min_lon + 2048.0 * s.geo.cell_lon_deg;
        let lat = b.min_lat + (s.height as f64 - 1.0 - 2048.0) * s.geo.cell_lat_deg;
        let got = s.height_at(lon, lat).unwrap();
        let want = (2048.0 * 4096.0 + 2048.0) % 2000.0;
        assert!((got - want).abs() < 1.0, "got {got} want {want}");
        // NW / SE 角区域有值
        assert!(s.height_at(b.min_lon + 0.0005, b.max_lat - 0.0005).is_some());
        assert!(s.height_at(b.min_lon + 0.0005, b.min_lat + 0.0005).is_some());
        // 出界 → None
        assert!(s.height_at(b.max_lon + 0.01, b.min_lat).is_none());
    }

    /// 行向判定（2026-08-11 Beijing_DEM north-up 暴露）：
    /// 1) Beijing_DEM.tif（tiepoint (0,0)=(115.42,41.06) 左上角，sy0>0，北界 42.68 不越界）
    ///    → north-up（sy 反号，row_flip=true）→ 南界 = 41.06-(4711)*cell ≈ 39.44；
    /// 2) 南极 south-up（y0=-89，north-up 假设南界越界）→ south-up；
    /// 3) gdal 裁剪东亚（tiepoint 左上角 69.999861，高度 10°）→ north-up → 南界 60
    ///    （2026-08-08 主管"origin (70,60)"修复回归）；
    /// 4) tj>0（tiepoint 非行 0，左下角布局）保守 south-up。
    #[test]
    fn resolve_sy_direction() {
        // Beijing_DEM.tif
        let sy = resolve_sy(0.0003433228, 0.0, 4712, 41.05923271);
        assert!(sy < 0.0, "Beijing north-up should flip, sy={sy}");
        // 南极 south-up：north-up 假设南界 -89.9999-0.0999 < -90 越界 → south-up
        assert_eq!(resolve_sy(0.0001, 0.0, 1000, -89.9999), 0.0001);
        // gdal 裁剪东亚：都合理 → tj=0 → north-up（南界 = 69.999861-9999*0.001 = 60）
        let sy = resolve_sy(0.001, 0.0, 10000, 69.999861);
        assert!(sy < 0.0, "gdal north-up should flip, sy={sy}");
        // tj>0 保守 south-up
        assert_eq!(resolve_sy(0.001, 500.0, 1000, 60.0), 0.001);
        // sy0<0 显式（north-up 直写）保持负
        assert_eq!(resolve_sy(-0.0003, 0.0, 100, 40.0), -0.0003);
        // 北极 north-up：south-up 假设北界 85+10=95 越界 → north-up
        let sy = resolve_sy(0.01, 0.0, 1000, 85.0);
        assert!(sy < 0.0, "arctic north-up should flip, sy={sy}");
    }

    // ---------- P9 T4：GeoTIFF Overview（兄弟 IFD 降采样层） ----------

    /// 无 overview 文件（_test_small.tif）：sample_at_res 回退主层，行为与 sample_at 一致。
    #[test]
    fn sample_at_res_falls_back_without_overview() {
        let Some(p) = test_tif() else {
            eprintln!("skip: _test_small.tif not found");
            return;
        };
        let s = GeoTiffSource::open(&p).unwrap();
        assert!(s.overviews.is_empty());
        let b = s.bounds().unwrap();
        let lon = 116.1;
        let lat = 38.85;
        assert_eq!(s.sample_at_res(lon, lat, 0.0), s.sample_at(lon, lat));
        assert_eq!(s.sample_at_res(lon, lat, 0.5), s.sample_at(lon, lat));
        // 出界同语义
        assert_eq!(
            s.sample_at_res(b.max_lon + 0.1, b.min_lat, 0.5),
            s.sample_at(b.max_lon + 0.1, b.min_lat)
        );
    }

    /// 多 IFD overview 文件（data/overview_test_multi.tif，开发期 tifffile 生成）：
    /// 主图 512×512 cell 0.001°（1024 chunks → lazy）+ 兄弟 IFD overview 128×128 cell 0.004°。
    /// 验证：overview 收集、低分辨率请求选层（值 = 4×4 块均值双线性）、
    /// 高分辨率请求回退主层（值 = 源公式原值）。
    #[test]
    fn overview_multi_ifd_used() {
        let p = std::path::Path::new(env!("CARGO_MANIFEST_DIR"))
            .join("../../data/overview_test_multi.tif");
        if !p.exists() {
            eprintln!("skip: overview_test_multi.tif not found (regenerate with scripts/gen_overview_test.py)");
            return;
        }
        let s = GeoTiffSource::open(&p).unwrap();
        assert!(s.lazy.is_some(), "512x512 tile16 → lazy path");
        assert_eq!(s.overviews.len(), 1, "one sibling-IFD overview");
        assert!(s.resolution_desc().contains("+ 1 overview"));
        // 源公式：v(r,c) = (r*512 + c) % 5000（north-up，row 0 = 北）
        let v = |r: usize, c: usize| ((r * 512 + c) % 5000) as f64;
        // 采样点 (lon=116.256, lat=38.45)：主图 col=256, row=250（整数 → 无插值）
        let lon = 116.256;
        let lat = 38.45;
        // 高分辨率请求（max_cell_deg=0.001：overview 0.004 不满足 → 回退主层）
        let high = s.sample_at_res(lon, lat, 0.001);
        assert_eq!(high, Sample::Land(v(250, 256)), "main layer exact value");
        // 低分辨率请求（max_cell_deg=0.005：overview 0.004 ≤ 0.005 → 选 overview 层）
        let low = s.sample_at_res(lon, lat, 0.005);
        let Sample::Land(low_h) = low else {
            panic!("overview should return Land, got {low:?}");
        };
        // overview 期望：col=64（整数），row=61.75 → 双线性（w_c=0, w_r=0.75）
        let ov_at = |r: usize, c: usize| -> f64 {
            // 4×4 块均值：base[(r*4)..(r*4+4), (c*4)..(c*4+4)]
            let mut sum = 0.0;
            for dr in 0..4 {
                for dc in 0..4 {
                    sum += v(r * 4 + dr, c * 4 + dc);
                }
            }
            sum / 16.0
        };
        let top = ov_at(61, 64);
        let bot = ov_at(62, 64);
        let want = top + (bot - top) * 0.75;
        assert!((low_h - want).abs() < 1e-6, "overview {low_h} vs {want}");
        // overview 层出主图范围（lat 边缘）→ 空洞/OutOfBounds 语义
        assert_eq!(
            s.sample_at_res(b_lat_max(&s) + 0.05, lon, 0.005).class(),
            crate::terrain::SurfaceClass::OutOfBounds
        );
        // 主层 height_at 不受影响（全分辨率精查路径）
        assert!((s.height_at(lon, lat).unwrap() - v(250, 256)).abs() < 1e-6);
    }

    fn b_lat_max(s: &GeoTiffSource) -> f64 {
        s.bounds().unwrap().max_lat
    }
}

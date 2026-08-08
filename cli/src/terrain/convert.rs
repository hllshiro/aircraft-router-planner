//! 地形转换：外部格式（GeoTIFF / DTED / SRTM .hgt）→ ARPK1（自有格式）。
//!
//! 2026-08-07 主管拍板：提供转换命令（用户任意大型地形文件 → ARPK1）。
//! 2026-08-08 主管拍板：内置纯 Rust 压缩编码器直接输出压缩 ARPK1 ——
//! COMPRESSION_DEFLATE（miniz_oxide，flate2 官方 rust 后端，零 C 红线；成熟纯 Rust
//! zstd 编码器不存在，zstd-pure-rs 为 immature LLM 翻译有数据损坏风险；实测真实地形
//! 差分块 deflate 压缩比 4.06:1 ≥ zstd 3.98:1）。压缩块变长 → 索引动态记录 + finish
//! 回填 + 流式重读算 SHA（与开发期 Python convert 语义一致）。
//! 输入侧：DTED/GeoTIFF 由 dted2/geotiff crate 全量持有（单片尺寸小，可接受）；
//! 大 GeoTIFF tile 流式读取随 M3（tile LRU）一并替换（TODO）。

use std::io::{Read, Seek, SeekFrom, Write};
use std::path::{Path, PathBuf};

use sha2::{Digest, Sha256};

use super::builtin::{
    BLOCK_SIZE, COMPRESSION_DEFLATE, FORMAT_VERSION, HEADER_SIZE, MAGIC, SEMANTICS_EQUIANGULAR,
    VDATUM_EGM96, VDATUM_ELLIPSOID,
};
use crate::error::AppError;

/// 转换选项。
#[derive(Debug, Clone)]
pub struct ConvertOptions {
    /// 输出 source 描述（缺省用输入格式+文件名）。
    pub source: String,
    /// 空洞值（缺省 -32768）。
    pub no_data: i16,
    /// 垂直基准：true = ellipsoid（椭球高），false = egm96（默认）。
    pub datum_ellipsoid: bool,
}

impl Default for ConvertOptions {
    fn default() -> Self {
        Self { source: String::new(), no_data: -32768, datum_ellipsoid: false }
    }
}

/// 转换统计。
#[derive(Debug)]
pub struct ConvertStats {
    pub rows: usize,
    pub cols: usize,
    pub origin_lon: f64,
    pub origin_lat: f64,
    pub cell_lon_deg: f64,
    pub cell_lat_deg: f64,
    pub n_blocks: usize,
    pub bytes_written: u64,
    pub source_desc: String,
}

/// 网格源统一视图（转换输入：按需提供 (r,c) 高度值）。
/// 行 = 纬向（行 0 = 南）、列 = 经向（列 0 = 西）——ARPK1 语义（origin = 左下角）。
pub trait GridSource {
    /// (rows, cols)。
    fn dims(&self) -> (usize, usize);
    /// (origin_lon, origin_lat)（左下角，度）。
    fn origin(&self) -> (f64, f64);
    /// (cell_lon_deg, cell_lat_deg)。
    fn cell(&self) -> (f64, f64);
    /// 高度值（米，i16）；空洞/越界 → None。
    fn height(&self, r: usize, c: usize) -> Option<i16>;
    /// 源描述（写入 ARPK1 source 字段）。
    fn source_desc(&self) -> String;
}

/// 输入格式探测。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum InputFormat {
    GeoTiff,
    Dted,
    SrtmHgt,
}

pub fn detect_format(path: &Path) -> Result<InputFormat, AppError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    match ext.as_str() {
        "tif" | "tiff" => Ok(InputFormat::GeoTiff),
        "dt0" | "dt1" | "dt2" => Ok(InputFormat::Dted),
        "hgt" => Ok(InputFormat::SrtmHgt),
        other => Err(AppError::Data(format!(
            "unsupported convert input: .{other} (expect .tif/.tiff/.dt0-2/.hgt)"
        ))),
    }
}

/// 转换入口：外部格式文件 → ARPK1（自有格式）。输出块级流式（内存 O(块)）。
pub fn convert_file(
    input: &Path,
    output: &Path,
    opts: &ConvertOptions,
) -> Result<ConvertStats, AppError> {
    let grid: Box<dyn GridSource> = match detect_format(input)? {
        InputFormat::GeoTiff => Box::new(GeoTiffGrid::open(input)?),
        InputFormat::Dted => Box::new(DtedGrid::open(input)?),
        InputFormat::SrtmHgt => Box::new(SrtmGrid::open(input)?),
    };
    write_arpk(grid.as_ref(), output, opts)
}

// ==================== 输入网格 ====================

/// GeoTIFF 输入（tiff crate 直读 + 大 Limits，支持 >512MB 大文件；内存 O(网格)）。
///
/// 2026-08-08 主管测试：gdal_translate 裁剪的东亚 GeoTIFF（38400×28800 int16，
/// 2.2GB）走 convert 报 "Decoder limits are exceeded"——geotiff crate 内部 tiff
/// 默认 Limits（max_alloc 512MB）拒绝大图。改用 tiff crate 直读并调大 Limits
/// （max_alloc/max_bytes 16GB）；行翻转按 GeoRef.row_flip（GeoTIFF 北朝上 →
/// ARPK1 行 0 = 南）。
struct GeoTiffGrid {
    data: Vec<i16>,
    rows: usize,
    cols: usize,
    origin_lon: f64,
    origin_lat: f64,
    cell_lon_deg: f64,
    cell_lat_deg: f64,
    no_data: Option<i16>,
}

impl GeoTiffGrid {
    fn open(path: &Path) -> Result<Self, AppError> {
        use super::geotiff;
        use tiff::decoder::{DecodingResult, Decoder};
        use tiff::tags::Tag;

        let f = std::fs::File::open(path)?;
        let mut dec = Decoder::new(f)
            .map_err(|e| AppError::Data(format!("geotiff decode failed: {e}")))?
            // 大文件支持：tiff 默认 Limits.decoding_buffer_size=256MiB，2GB+ GeoTIFF
            // read_image 全图超限 → "Decoder limits are exceeded"（2026-08-08 主管
            // 测试 gdal 裁剪东亚 GeoTIFF 38400×28800 int16 复现）；unlimited 放开，
            // 内存由调用方承诺（机器 16GB 可扛 2.2GB 网格 + 流式块写）。
            .with_limits(tiff::decoder::Limits::unlimited());
        let (width, height) = dec
            .dimensions()
            .map_err(|e| AppError::Data(format!("geotiff dimensions failed: {e}")))?;
        if width == 0 || height == 0 {
            return Err(AppError::Data("geotiff zero dimensions".into()));
        }
        let geo = geotiff::parse_georef(&mut dec, width, height)?;
        let samples = dec
            .find_tag_unsigned::<u16>(Tag::SamplesPerPixel)
            .ok()
            .flatten()
            .unwrap_or(1) as usize;
        let no_data = geotiff::read_gdal_nodata(&mut dec).map(|v| v.round() as i16);
        let result = dec
            .read_image()
            .map_err(|e| AppError::Data(format!("geotiff read_image failed: {e}")))?;
        // DecodingResult → i16 网格（多波段取 band 0；NaN/非有限 → i16::MIN 空洞）
        let raw: Vec<i16> = match result {
            DecodingResult::U8(v) => v.into_iter().map(|x| x as i16).collect(),
            DecodingResult::U16(v) => v.into_iter().map(|x| x as i16).collect(),
            DecodingResult::U32(v) => v
                .into_iter()
                .map(|x| x.min(i16::MAX as u32) as i16)
                .collect(),
            DecodingResult::I8(v) => v.into_iter().map(|x| x as i16).collect(),
            DecodingResult::I16(v) => v,
            DecodingResult::I32(v) => v
                .into_iter()
                .map(|x| x.clamp(i16::MIN as i32, i16::MAX as i32) as i16)
                .collect(),
            DecodingResult::F32(v) => v
                .into_iter()
                .map(|x| if x.is_finite() { x.round() as i16 } else { i16::MIN })
                .collect(),
            DecodingResult::F64(v) => v
                .into_iter()
                .map(|x| if x.is_finite() { x.round() as i16 } else { i16::MIN })
                .collect(),
            _ => {
                return Err(AppError::Data(
                    "geotiff unsupported sample type (u64/i64)".into(),
                ))
            }
        };
        let n = (width as usize) * (height as usize);
        if raw.len() < n {
            return Err(AppError::Data("geotiff image data truncated".into()));
        }
        let take = |i: usize| -> i16 {
            // samples>1 时 read_image 返回 chunky interleaved（每像素 samples 个值）
            let idx = i * samples;
            if idx < raw.len() {
                raw[idx]
            } else {
                i16::MIN
            }
        };
        let (rows, cols) = (height as usize, width as usize);
        let mut data = Vec::with_capacity(n);
        if geo.row_flip {
            // GeoTIFF 行 0 = 北 → ARPK1 行 0 = 南
            for r in 0..rows {
                let src_row = rows - 1 - r;
                for c in 0..cols {
                    let src_c = if geo.col_flip { cols - 1 - c } else { c };
                    data.push(take(src_row * cols + src_c));
                }
            }
        } else {
            for r in 0..rows {
                for c in 0..cols {
                    let src_c = if geo.col_flip { cols - 1 - c } else { c };
                    data.push(take(r * cols + src_c));
                }
            }
        }
        Ok(Self {
            data,
            rows,
            cols,
            origin_lon: geo.min_lon,
            origin_lat: geo.min_lat,
            cell_lon_deg: geo.cell_lon_deg,
            cell_lat_deg: geo.cell_lat_deg,
            no_data,
        })
    }
}

impl GridSource for GeoTiffGrid {
    fn dims(&self) -> (usize, usize) {
        (self.rows, self.cols)
    }
    fn origin(&self) -> (f64, f64) {
        (self.origin_lon, self.origin_lat)
    }
    fn cell(&self) -> (f64, f64) {
        (self.cell_lon_deg, self.cell_lat_deg)
    }
    fn height(&self, r: usize, c: usize) -> Option<i16> {
        if r >= self.rows || c >= self.cols {
            return None;
        }
        let v = self.data[r * self.cols + c];
        if v == i16::MIN {
            return None; // NaN/非有限 → 空洞
        }
        if let Some(nd) = self.no_data {
            if v == nd {
                return None;
            }
        }
        Some(v)
    }
    fn source_desc(&self) -> String {
        format!(
            "GeoTIFF {}x{} cell {:.6}deg x {:.6}deg",
            self.cols, self.rows, self.cell_lon_deg, self.cell_lat_deg
        )
    }
}

/// DTED 输入（dted2 crate 全量；`data[c].elevations[r]` 网格直读）。
struct DtedGrid {
    dted: dted2::DTEDData,
}

impl DtedGrid {
    fn open(path: &Path) -> Result<Self, AppError> {
        let p = path
            .to_str()
            .ok_or_else(|| AppError::Data("non-UTF8 dted path".into()))?;
        let dted = dted2::DTEDData::read(p)
            .map_err(|e| AppError::Data(format!("dted2 read failed: {e:?}")))?;
        Ok(Self { dted })
    }
}

impl GridSource for DtedGrid {
    fn dims(&self) -> (usize, usize) {
        let m = &self.dted.metadata;
        (m.count.lat as usize, m.count.lon as usize)
    }
    fn origin(&self) -> (f64, f64) {
        let m = &self.dted.metadata;
        (m.origin.lon, m.origin.lat)
    }
    fn cell(&self) -> (f64, f64) {
        let m = &self.dted.metadata;
        (m.interval.lon, m.interval.lat)
    }
    fn height(&self, r: usize, c: usize) -> Option<i16> {
        // data 按 lon 列，elevations 按 lat 行（行 0 = 南，与 ARPK1 一致）
        let rec = self.dted.data.get(c)?;
        rec.elevations.get(r).copied()
    }
    fn source_desc(&self) -> String {
        let m = &self.dted.metadata;
        format!(
            "DTED {}x{} interval {:.6}deg x {:.6}deg",
            m.count.lon, m.count.lat, m.interval.lon, m.interval.lat
        )
    }
}

/// SRTM .hgt 输入（自研解析：大端 i16 行优先，行 0 = 北 → 翻转为南行序）。
struct SrtmGrid {
    data: Vec<i16>,
    rows: usize,
    cols: usize,
    origin_lon: f64,
    origin_lat: f64,
    cell_deg: f64,
}

impl SrtmGrid {
    fn open(path: &Path) -> Result<Self, AppError> {
        let bytes = std::fs::read(path)?;
        let name = path
            .file_stem()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_ascii_uppercase();
        let (lat_origin, lon_origin) = parse_hgt_name(&name)?;
        // 尺寸按文件大小推断（SRTM1 = 3601²×2B ≈ 26MB；SRTM3 = 1201²×2B ≈ 2.9MB）
        let n = bytes.len() / 2;
        if bytes.len() % 2 != 0 || n == 0 {
            return Err(AppError::Data(format!("hgt size not even: {} bytes", bytes.len())));
        }
        let side = (n as f64).sqrt().round() as usize;
        if side == 0 || side * side != n {
            return Err(AppError::Data(format!(
                "hgt size {} samples is not a square grid",
                n
            )));
        }
        // 大端 → i16；行 0 = 北 → 翻转为行 0 = 南（ARPK1 语义）
        let mut data = vec![0i16; n];
        for r in 0..side {
            for c in 0..side {
                let src = (side - 1 - r) * side + c; // 源行（北=0）→ 目标行 side-1-r
                let i = r * side + c;
                data[i] = i16::from_be_bytes([bytes[src * 2], bytes[src * 2 + 1]]);
            }
        }
        let cell_deg = 1.0 / (side as f64 - 1.0); // 1°×1° 片惯例（SRTM1=1as，SRTM3=3as）
        Ok(Self {
            data,
            rows: side,
            cols: side,
            origin_lon: lon_origin,
            origin_lat: lat_origin,
            cell_deg,
        })
    }
}

impl GridSource for SrtmGrid {
    fn dims(&self) -> (usize, usize) {
        (self.rows, self.cols)
    }
    fn origin(&self) -> (f64, f64) {
        (self.origin_lon, self.origin_lat)
    }
    fn cell(&self) -> (f64, f64) {
        (self.cell_deg, self.cell_deg)
    }
    fn height(&self, r: usize, c: usize) -> Option<i16> {
        if r >= self.rows || c >= self.cols {
            return None;
        }
        let v = self.data[r * self.cols + c];
        if v == -32768 {
            None // SRTM 空洞惯例
        } else {
            Some(v)
        }
    }
    fn source_desc(&self) -> String {
        format!("SRTM .hgt {}x{} cell {:.6}deg", self.rows, self.cols, self.cell_deg)
    }
}

/// 解析 .hgt 片名（N39E116 → (lat, lon) 左下角；支持 N/S/E/W 前缀，大小写不敏感）。
fn parse_hgt_name(name: &str) -> Result<(f64, f64), AppError> {
    let bytes = name.as_bytes();
    if bytes.len() < 3 {
        return Err(AppError::Data(format!("hgt name too short: {name:?}")));
    }
    let (lat_sign, mut i) = match bytes[0] {
        b'N' | b'n' => (1.0, 1),
        b'S' | b's' => (-1.0, 1),
        _ => return Err(AppError::Data(format!("hgt name {name:?} missing N/S prefix"))),
    };
    let lat_start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    let lat_str = &name[lat_start..i];
    if i >= bytes.len() {
        return Err(AppError::Data(format!("hgt name {name:?} missing E/W separator")));
    }
    let lon_sign = match bytes[i] {
        b'E' | b'e' => 1.0,
        b'W' | b'w' => -1.0,
        _ => return Err(AppError::Data(format!("hgt name {name:?} bad E/W separator"))),
    };
    i += 1;
    let lon_start = i;
    while i < bytes.len() && bytes[i].is_ascii_digit() {
        i += 1;
    }
    let lon_str = &name[lon_start..i];
    if lat_str.is_empty() || lon_str.is_empty() {
        return Err(AppError::Data(format!("hgt name {name:?} missing coordinate digits")));
    }
    let lat = lat_str
        .parse::<f64>()
        .map_err(|_| AppError::Data(format!("hgt name {name:?} bad latitude")))?;
    let lon = lon_str
        .parse::<f64>()
        .map_err(|_| AppError::Data(format!("hgt name {name:?} bad longitude")))?;
    Ok((lat * lat_sign, lon * lon_sign))
}

// ==================== ARPK1 流式写入 ====================

/// ARPK1 写入器（块级流式：内存 O(块)，与 write_pack_raw 同格式/同 SHA 语义）。
/// 压缩块变长 → 索引动态记录；SHA-256 覆盖数据部分（索引区 + 数据流），
/// finish 时回填索引后流式重读算 sha（与开发期 Python convert 语义一致）。
struct ArpkWriter {
    out: std::fs::File,
    path: PathBuf,
    rows: usize,
    cols: usize,
    origin_lon: f64,
    origin_lat: f64,
    cell_lon_deg: f64,
    cell_lat_deg: f64,
    blocks_x: usize,
    blocks_y: usize,
    index: Vec<(u64, u32)>,
    written: u64,
    data_start: u64,
    no_data: i16,
    source: String,
    datum_ellipsoid: bool,
}

impl ArpkWriter {
    fn new(
        path: &Path,
        rows: usize,
        cols: usize,
        origin_lon: f64,
        origin_lat: f64,
        cell_lon_deg: f64,
        cell_lat_deg: f64,
        no_data: i16,
        source: String,
        datum_ellipsoid: bool,
    ) -> Result<Self, AppError> {
        let blocks_x = rows.div_ceil(BLOCK_SIZE as usize);
        let blocks_y = cols.div_ceil(BLOCK_SIZE as usize);
        let n = blocks_x
            .checked_mul(blocks_y)
            .ok_or_else(|| AppError::Data("convert: block count overflow".into()))?;
        let idx_start = HEADER_SIZE + 32;
        let data_start = (idx_start + n * 12) as u64;
        let mut out = std::fs::File::create(path)?;
        out.write_all(&vec![0u8; idx_start])?; // 头部 + sha 占位
        out.write_all(&vec![0u8; n * 12])?; // 索引区占位（压缩块变长 → finish 回填）
        Ok(Self {
            out,
            path: path.to_path_buf(),
            rows,
            cols,
            origin_lon,
            origin_lat,
            cell_lon_deg,
            cell_lat_deg,
            blocks_x,
            blocks_y,
            index: Vec::with_capacity(n),
            written: 0,
            data_start,
            no_data,
            source,
            datum_ellipsoid,
        })
    }

    /// 写入一个块（256×256 i16 展平，行优先；行内差分编码 + deflate 压缩）。
    /// 主管 2026-08-08：内置纯 Rust 压缩编码器直接输出压缩 ARPK1（COMPRESSION_DEFLATE，
    /// miniz_oxide 纯 Rust deflate；实测真实地形差分块压缩比 4.06:1 ≥ zstd 3.98:1）。
    fn write_block(&mut self, _bidx: usize, h: &[i16]) -> Result<(), AppError> {
        let mut diff = h.to_vec();
        for i in (1..diff.len()).rev() {
            diff[i] = diff[i].wrapping_sub(diff[i - 1]);
        }
        let mut bytes = Vec::with_capacity(diff.len() * 2);
        for v in &diff {
            bytes.extend_from_slice(&v.to_le_bytes());
        }
        debug_assert_eq!(bytes.len(), BLOCK_SIZE as usize * BLOCK_SIZE as usize * 2);
        // 差分后压缩：zlib 包装（miniz_oxide level 6 平衡速度/压缩比）。
        // 差分块高相关（地形平滑/NoData 全 0），deflate 压缩比与 zstd 持平（POC 实测）。
        let comp = miniz_oxide::deflate::compress_to_vec_zlib(&bytes, 6);
        self.index.push((self.data_start + self.written, comp.len() as u32));
        self.out.write_all(&comp)?;
        self.written += comp.len() as u64;
        Ok(())
    }

    /// 回写头部 + 索引区 + sha，返回数据流字节数。
    fn finish(mut self) -> Result<u64, AppError> {
        // 索引区字节（动态记录的实际 offset/len）
        let mut idx_bytes = Vec::with_capacity(self.index.len() * 12);
        for (o, l) in &self.index {
            idx_bytes.extend_from_slice(&o.to_le_bytes());
            idx_bytes.extend_from_slice(&l.to_le_bytes());
        }
        // 先回填索引区（sha 计算前文件须含真实索引字节）
        self.out.seek(SeekFrom::Start((HEADER_SIZE + 32) as u64))?;
        self.out.write_all(&idx_bytes)?;
        self.out.flush()?;
        let digest = {
            // SHA-256 覆盖数据部分（索引区 + 数据流），与开发期 Python convert 一致：
            // 从 idx_start 流式读整个数据部分（内存 O(1)）。
            let mut hasher = Sha256::new();
            let mut buf = [0u8; 1 << 16];
            let mut rd = std::fs::File::open(&self.path)?;
            rd.seek(SeekFrom::Start((HEADER_SIZE + 32) as u64))?;
            loop {
                let n = rd.read(&mut buf)?;
                if n == 0 {
                    break;
                }
                hasher.update(&buf[..n]);
            }
            hasher.finalize()
        };
        let mut hdr = [0u8; HEADER_SIZE + 32];
        hdr[0..8].copy_from_slice(&MAGIC);
        put_u32(&mut hdr, 8, FORMAT_VERSION);
        put_u32(&mut hdr, 12, 1); // data_version
        put_u32(&mut hdr, 16, self.rows as u32);
        put_u32(&mut hdr, 20, self.cols as u32);
        put_f64(&mut hdr, 24, self.origin_lon);
        put_f64(&mut hdr, 32, self.origin_lat);
        put_f64(&mut hdr, 40, self.cell_lon_deg);
        put_f64(&mut hdr, 48, self.cell_lat_deg);
        put_f64(&mut hdr, 56, 1.0); // z_resolution_m（i16 米）
        hdr[64] = if self.datum_ellipsoid { VDATUM_ELLIPSOID } else { VDATUM_EGM96 };
        hdr[65] = SEMANTICS_EQUIANGULAR;
        hdr[66] = COMPRESSION_DEFLATE;
        hdr[67] = 0;
        put_u32(&mut hdr, 68, BLOCK_SIZE);
        put_u32(&mut hdr, 72, self.blocks_x as u32);
        put_u32(&mut hdr, 76, self.blocks_y as u32);
        hdr[80..82].copy_from_slice(&self.no_data.to_le_bytes());
        let src = self.source.as_bytes();
        let n = src.len().min(174);
        hdr[82..82 + n].copy_from_slice(&src[..n]);
        hdr[HEADER_SIZE..HEADER_SIZE + 32].copy_from_slice(&digest);

        // 回写头部（含 sha）
        self.out.seek(SeekFrom::Start(0))?;
        self.out.write_all(&hdr)?;
        self.out.flush()?;
        Ok(self.written)
    }
}

fn put_u32(b: &mut [u8], i: usize, v: u32) {
    b[i..i + 4].copy_from_slice(&v.to_le_bytes());
}
fn put_f64(b: &mut [u8], i: usize, v: f64) {
    b[i..i + 8].copy_from_slice(&v.to_le_bytes());
}

/// 网格 → ARPK1（块级流式；块外区域填 no_data）。
fn write_arpk(
    grid: &dyn GridSource,
    output: &Path,
    opts: &ConvertOptions,
) -> Result<ConvertStats, AppError> {
    let (rows, cols) = grid.dims();
    let (origin_lon, origin_lat) = grid.origin();
    let (cell_lon, cell_lat) = grid.cell();
    if rows == 0 || cols == 0 {
        return Err(AppError::Data("convert: empty grid".into()));
    }
    if cell_lon <= 0.0 || cell_lat <= 0.0 {
        return Err(AppError::Data("convert: degenerate cell size".into()));
    }
    let source = if opts.source.is_empty() {
        grid.source_desc()
    } else {
        opts.source.clone()
    };
    let mut w = ArpkWriter::new(
        output,
        rows,
        cols,
        origin_lon,
        origin_lat,
        cell_lon,
        cell_lat,
        opts.no_data,
        source,
        opts.datum_ellipsoid,
    )?;
    let blocks_x = rows.div_ceil(BLOCK_SIZE as usize);
    let blocks_y = cols.div_ceil(BLOCK_SIZE as usize);
    for bx in 0..blocks_x {
        for by in 0..blocks_y {
            let mut block = vec![opts.no_data; BLOCK_SIZE as usize * BLOCK_SIZE as usize];
            for lr in 0..BLOCK_SIZE as usize {
                for lc in 0..BLOCK_SIZE as usize {
                    let r = bx * BLOCK_SIZE as usize + lr;
                    let c = by * BLOCK_SIZE as usize + lc;
                    if r < rows && c < cols {
                        if let Some(v) = grid.height(r, c) {
                            block[lr * BLOCK_SIZE as usize + lc] = v;
                        }
                    }
                }
            }
            w.write_block(bx * blocks_y + by, &block)?;
        }
    }
    let bytes_written = w.finish()?;
    Ok(ConvertStats {
        rows,
        cols,
        origin_lon,
        origin_lat,
        cell_lon_deg: cell_lon,
        cell_lat_deg: cell_lat,
        n_blocks: blocks_x * blocks_y,
        bytes_written,
        source_desc: grid.source_desc(),
    })
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::builtin::BuiltinSource;
    use crate::terrain::TerrainSource;

    /// 构造 SRTM .hgt 字节（大端 i16 行优先；行 0 = 北）。
    /// 网格：3×3，值 = 行号*10 + 列号（便于验证翻转：南行序应为 2*10+.. → 0*10+..）。
    fn hgt_bytes(rows: usize, cols: usize) -> Vec<u8> {
        let mut out = Vec::with_capacity(rows * cols * 2);
        for r in 0..rows {
            for c in 0..cols {
                let v: i16 = (r as i16) * 10 + c as i16;
                out.extend_from_slice(&v.to_be_bytes());
            }
        }
        out
    }

    #[test]
    fn parse_hgt_name_basic() {
        assert_eq!(parse_hgt_name("N39E116").unwrap(), (39.0, 116.0));
        assert_eq!(parse_hgt_name("S12W075").unwrap(), (-12.0, -75.0));
        assert_eq!(parse_hgt_name("n01e002").unwrap(), (1.0, 2.0));
        assert!(parse_hgt_name("E116N39").is_err());
        assert!(parse_hgt_name("ABC").is_err());
    }

    /// 定位 cargo registry 下 dted2-1.0.0/tests/test_data.dt2（找不到 → None，测试跳过）。
    fn find_dted_test_data() -> Option<std::path::PathBuf> {
        let home = std::env::var("CARGO_HOME")
            .ok()
            .or_else(|| std::env::var("USERPROFILE").ok().map(|p| format!("{p}\\.cargo")))
            .or_else(|| std::env::var("HOME").ok().map(|p| format!("{p}/.cargo")))?;
        let src_root = std::path::Path::new(&home).join("registry/src");
        let entries = std::fs::read_dir(&src_root).ok()?;
        for e in entries.flatten() {
            let candidate = e.path().join("dted2-1.0.0/tests/test_data.dt2");
            if candidate.exists() {
                return Some(candidate);
            }
        }
        None
    }

    /// SRTM 转换：行翻转正确（南行序）+ round-trip 采样一致。
    #[test]
    fn convert_srtm_roundtrip() {
        let dir = std::env::temp_dir();
        let id = std::process::id();
        let hgt = dir.join(format!("N39E116_{id}.hgt"));
        let arpk = dir.join(format!("N39E116_{id}.arpack"));
        // 3×3：源行 0（北）= [0,1,2] ... 行 2（南）= [20,21,22]
        std::fs::write(&hgt, hgt_bytes(3, 3)).unwrap();
        let opts = ConvertOptions::default();
        let stats = convert_file(&hgt, &arpk, &opts).unwrap();
        assert_eq!(stats.rows, 3);
        assert_eq!(stats.cols, 3);
        assert_eq!(stats.origin_lon, 116.0);
        assert_eq!(stats.origin_lat, 39.0);
        assert!((stats.cell_lon_deg - 0.5).abs() < 1e-12); // 3 点 → cell = 1/2

        // 打开转换产物（mmap 按需路径）→ 验证行翻转 + 采样
        let s = BuiltinSource::open(&arpk).unwrap();
        s.verify_sha().unwrap();
        // 格点采样（双线性：格点处权重 0 返回原值）
        // ARPK1 行 0 = 南 = 源行 2 = [20,21,22]
        assert_eq!(s.height_at(116.0, 39.0), Some(20.0)); // 南行首列
        assert_eq!(s.height_at(116.5, 39.0), Some(21.0));
        // 北行参与（插值中点 (116.25,39.75)：v=[20,21,10,11,0,1] → 南行 20/21、中行 10/11、北行 0/1
        // 结果 = top(10.5) + (0.5-10.5)*0.5 = 5.5）
        assert_eq!(s.height_at(116.25, 39.75), Some(5.5));
        // 越界
        assert!(s.height_at(117.5, 39.0).is_none());
        let _ = std::fs::remove_file(&hgt);
        let _ = std::fs::remove_file(&arpk);
    }

    /// DTED 转换（dted2 crate 自带测试数据）：round-trip 采样一致。
    #[test]
    fn convert_dted_roundtrip() {
        let Some(dted_src) = find_dted_test_data() else {
            eprintln!("skip: dted2 test data not found in cargo registry");
            return;
        };
        let dir = std::env::temp_dir();
        let id = std::process::id();
        let arpk = dir.join(format!("dted_conv_{id}.arpack"));
        let opts = ConvertOptions::default();
        let stats = convert_file(&dted_src, &arpk, &opts).unwrap();
        assert!(stats.rows > 0 && stats.cols > 0);
        let s = BuiltinSource::open(&arpk).unwrap();
        s.verify_sha().unwrap();
        // 与 dted2 采样对比（中心点）
        let d = dted2::DTEDData::read(dted_src.to_str().unwrap()).unwrap();
        let (r0, c0) = (stats.rows / 2, stats.cols / 2);
        let lon = stats.origin_lon + (c0 as f64 + 0.5) * stats.cell_lon_deg;
        let lat = stats.origin_lat + (r0 as f64 + 0.5) * stats.cell_lat_deg;
        let got = s.height_at(lon, lat);
        let want = d.get_elevation(lat, lon).map(|v| v.round());
        assert_eq!(got, want, "DTED roundtrip mismatch at ({lon},{lat})");
        let _ = std::fs::remove_file(&arpk);
    }

    /// 转换输出为合法 ARPK1（可 mmap 打开 + verify_sha）。
    #[test]
    fn convert_output_is_valid_arpack() {
        let dir = std::env::temp_dir();
        let id = std::process::id();
        let hgt = dir.join(format!("N00E000_{id}.hgt"));
        let arpk = dir.join(format!("N00E000_{id}.arpack"));
        std::fs::write(&hgt, hgt_bytes(4, 4)).unwrap();
        let opts = ConvertOptions { source: "conv-test".into(), ..Default::default() };
        let stats = convert_file(&hgt, &arpk, &opts).unwrap();
        assert_eq!(stats.n_blocks, 1); // 4×4 < 256
        let s = BuiltinSource::open(&arpk).unwrap();
        s.verify_sha().unwrap();
        assert!(s.source_desc().contains("conv-test"));
        let _ = std::fs::remove_file(&hgt);
        let _ = std::fs::remove_file(&arpk);
    }

    /// GeoTIFF 转换（项目内 `_test_small.tif`）：产物采样 = 原 GeoTIFF 源采样（±1m 取整容差）。
    #[test]
    fn convert_geotiff_roundtrip() {
        let tif = Path::new(env!("CARGO_MANIFEST_DIR")).join("../../phase0/data/_test_small.tif");
        if !tif.exists() {
            eprintln!("skip: _test_small.tif not found");
            return;
        }
        let dir = std::env::temp_dir();
        let id = std::process::id();
        let arpk = dir.join(format!("tif_conv_{id}.arpack"));
        let opts = ConvertOptions::default();
        let stats = convert_file(&tif, &arpk, &opts).unwrap();
        let s = BuiltinSource::open(&arpk).unwrap();
        s.verify_sha().unwrap();
        let g = super::super::geotiff::GeoTiffSource::open(&tif).unwrap();
        // 网格中心采样对比
        let lon = stats.origin_lon + (stats.cols as f64 / 2.0) * stats.cell_lon_deg;
        let lat = stats.origin_lat + (stats.rows as f64 / 2.0) * stats.cell_lat_deg;
        let got = s.height_at(lon, lat);
        let want = g.height_at(lon, lat);
        match (got, want) {
            (Some(a), Some(b)) => {
                assert!(
                    (a - b).abs() < 1.0,
                    "GeoTIFF roundtrip mismatch at ({lon},{lat}): {a} vs {b}"
                )
            }
            (None, None) => {}
            (a, b) => panic!("GeoTIFF roundtrip mismatch at ({lon},{lat}): {a:?} vs {b:?}"),
        }
        let _ = std::fs::remove_file(&arpk);
    }
}

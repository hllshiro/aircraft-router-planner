//! 自建紧凑格式数据源（技术方案 4.2.5：16-bit + 分块差分 + Zstd，运行时 ruzstd）。
//!
//! ARPK1 格式（定长二进制头 + SHA-256 + 块索引 + 压缩块）：
//! ```text
//! [0..8)    magic "ARPACK1\0"
//! [8..288)  定长头（LE）：format_version u32 / data_version u32 / rows u32 /
//!          cols u32 / origin_lon f64 / origin_lat f64 / cell_lon_deg f64 /
//!          cell_lat_deg f64 / z_resolution_m f64 / vertical_datum u8 /
//!          resolution_semantics u8 / block_compression u8 / reserved u8 /
//!          block_size u32 / blocks_x u32 / blocks_y u32 / no_data i16 / source[174B]
//! [288..320) sha256（数据部分 = 块索引 + 数据块流）
//! [320..)   块索引：blocks_x*blocks_y × (offset u64 LE, len u32 LE)
//! […)       数据块流：每块 = 行内差分 i16 展平（block_size×block_size），
//!            raw（block_compression=0）或 zstd（block_compression=1，ruzstd 解码）
//! ```
//!
//! fail-fast（八轮共识）：magic / format_version / data_version / sha256 /
//! 截断任一不符 → `AppError::Data` 硬故障（调用方非零退出），禁止静默降级。
//! 块内差分：块数据行优先展平，v[0] 原样，v[i] = v[i] + v[i-1]（行内差分还原）。

use std::collections::HashMap;
use std::path::Path;
use std::sync::Mutex;

use sha2::{Digest, Sha256};

use super::{BulkPrefetch, GeoBounds, Sample, TerrainSource};
use crate::error::AppError;

/// 文件字节存储：内存（`parse` 测试/转换校验路径）或 mmap（`open` 按需路径）。
/// 2026-08-07 按需读取（主管拍板）：大型 ARPK1 打开 O(1)（只读头部+块索引），
/// 块数据按需从映射读（虚拟内存，物理页按访问加载），不再全量读入内存。
#[derive(Debug)]
enum Store {
    Mem(Vec<u8>),
    Mmap(memmap2::Mmap),
}

impl Store {
    /// 取 [offset, offset+len) 字节切片（offset 为**文件绝对偏移**，含头部+索引+数据流）。
    #[inline]
    fn get(&self, offset: usize, len: usize) -> Option<&[u8]> {
        match self {
            Store::Mem(b) => b.get(offset..offset.checked_add(len)?),
            Store::Mmap(m) => m.get(offset..offset.checked_add(len)?),
        }
    }
}

/// 格式魔数与版本。
pub const MAGIC: [u8; 8] = *b"ARPACK1\0";
pub const FORMAT_VERSION: u32 = 1;
pub const HEADER_SIZE: usize = 288;
pub const BLOCK_SIZE: u32 = 256;

pub(crate) const VDATUM_ELLIPSOID: u8 = 0;
pub(crate) const VDATUM_EGM96: u8 = 1;
pub(crate) const SEMANTICS_EQUIANGULAR: u8 = 0;
pub(crate) const COMPRESSION_RAW: u8 = 0;
pub(crate) const COMPRESSION_ZSTD: u8 = 1;
/// 块压缩：zlib(deflate) 包装（miniz_oxide 纯 Rust）。主管 2026-08-08 内置编码器输出格式。
pub(crate) const COMPRESSION_DEFLATE: u8 = 2;

/// 已解压块缓存上限（FIFO 淘汰）。
/// 256² 块 ≈ 131KB → 2048 块 ≈ 256MB，防止无界缓存在大文件随机访问时膨胀内存
/// （GMTED2010 全球 17.8 万块，随机采样曾缓存全部 → 13GB+，实测发现）。
const CACHE_MAX_BLOCKS: usize = 2048;

/// 有界解压缓存（FIFO 淘汰，锁内仅做查表/淘汰，解压在锁外进行）。
#[derive(Debug)]
struct BlockCache {
    map: HashMap<usize, Vec<i16>>,
    order: std::collections::VecDeque<usize>,
    max_blocks: usize,
}

impl Default for BlockCache {
    fn default() -> Self {
        Self::with_max(CACHE_MAX_BLOCKS)
    }
}

impl BlockCache {
    fn with_max(max_blocks: usize) -> Self {
        Self {
            map: HashMap::new(),
            order: std::collections::VecDeque::new(),
            max_blocks: max_blocks.max(1),
        }
    }

    fn insert(&mut self, bidx: usize, block: Vec<i16>) {
        if self.map.contains_key(&bidx) {
            return;
        }
        if self.map.len() >= self.max_blocks {
            if let Some(old) = self.order.pop_front() {
                self.map.remove(&old);
            }
        }
        self.map.insert(bidx, block);
        self.order.push_back(bidx);
    }
}

/// 内置格式源。
#[derive(Debug)]
pub struct BuiltinSource {
    data_version: u32,
    rows: usize,
    cols: usize,
    origin_lon: f64,
    origin_lat: f64,
    cell_lon_deg: f64,
    cell_lat_deg: f64,
    z_resolution_m: f64,
    vertical_datum: u8,
    resolution_semantics: u8,
    block_compression: u8,
    blocks_x: usize,
    blocks_y: usize,
    no_data: i16,
    source: String,
    /// 块索引：[(offset, len)]（offset 为文件绝对偏移）
    index: Vec<(u64, u32)>,
    /// 预期 SHA-256（头部 [288..320]；`verify_sha` 可选全量校验用）
    sha_expected: [u8; 32],
    /// 文件字节（全文件；Mem=内存副本 / Mmap=只读映射，块按需读）
    store: Store,
    /// 已解压块缓存（block_idx → i16 展平，FIFO 有界）；Mutex 满足 TerrainSource: Send+Sync
    cache: Mutex<BlockCache>,
}

/// 头部 + 块索引解析结果（parse / open 共用）。
#[derive(Debug, Clone)]
struct HeaderFields {
    data_version: u32,
    rows: usize,
    cols: usize,
    origin_lon: f64,
    origin_lat: f64,
    cell_lon_deg: f64,
    cell_lat_deg: f64,
    z_resolution_m: f64,
    vertical_datum: u8,
    resolution_semantics: u8,
    block_compression: u8,
    blocks_x: usize,
    blocks_y: usize,
    no_data: i16,
    source: String,
    sha_expected: [u8; 32],
}

/// 解析头部 + 块索引（结构校验：magic/版本/字段/索引越界）。
/// `data_end` = 数据流终点（parse 用 bytes.len()，mmap open 用文件长度）——
/// 索引越界检查用它防止截断/越界（替代全量 SHA 的 fail-fast 语义，O(1)）。
fn parse_header_index(
    bytes: &[u8],
    data_end: usize,
) -> Result<(HeaderFields, Vec<(u64, u32)>), AppError> {
    if bytes.len() < HEADER_SIZE + 32 {
        return Err(AppError::Data(format!(
            "arpack file truncated: {} bytes < header+sha",
            bytes.len()
        )));
    }
    if bytes[0..8] != MAGIC {
        return Err(AppError::Data("arpack magic mismatch (not an ARPACK1 file)".into()));
    }
    let le = |i: usize| -> u32 {
        u32::from_le_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]])
    };
    let lef = |i: usize| -> f64 {
        f64::from_le_bytes([
            bytes[i],
            bytes[i + 1],
            bytes[i + 2],
            bytes[i + 3],
            bytes[i + 4],
            bytes[i + 5],
            bytes[i + 6],
            bytes[i + 7],
        ])
    };
    let format_version = le(8);
    if format_version != FORMAT_VERSION {
        return Err(AppError::Data(format!(
            "arpack format_version mismatch: got {format_version}, expect {FORMAT_VERSION}"
        )));
    }
    let data_version = le(12);
    let rows = le(16) as usize;
    let cols = le(20) as usize;
    let origin_lon = lef(24);
    let origin_lat = lef(32);
    let cell_lon_deg = lef(40);
    let cell_lat_deg = lef(48);
    let z_resolution_m = lef(56);
    let vertical_datum = bytes[64];
    let resolution_semantics = bytes[65];
    let block_compression = bytes[66];
    let _reserved = bytes[67];
    let block_size = le(68);
    if block_size != BLOCK_SIZE {
        return Err(AppError::Data(format!(
            "arpack block_size {block_size} unsupported (expect {BLOCK_SIZE})"
        )));
    }
    let blocks_x = le(72) as usize;
    let blocks_y = le(76) as usize;
    let no_data = i16::from_le_bytes([bytes[80], bytes[81]]);
    let source = String::from_utf8_lossy(&bytes[82..256]).trim_end_matches('\0').to_string();
    if rows == 0 || cols == 0 || blocks_x == 0 || blocks_y == 0 || cell_lon_deg <= 0.0 || cell_lat_deg <= 0.0
    {
        return Err(AppError::Data("arpack degenerate header".into()));
    }
    let mut sha_expected = [0u8; 32];
    sha_expected.copy_from_slice(&bytes[HEADER_SIZE..HEADER_SIZE + 32]);

    // 块索引
    let n_blocks = blocks_x * blocks_y;
    let idx_start = HEADER_SIZE + 32;
    let idx_bytes = n_blocks * 12;
    if bytes.len() < idx_start + idx_bytes {
        return Err(AppError::Data("arpack truncated: block index out of range".into()));
    }
    let mut index = Vec::with_capacity(n_blocks);
    for b in 0..n_blocks {
        let p = idx_start + b * 12;
        let offset = u64::from_le_bytes([
            bytes[p],
            bytes[p + 1],
            bytes[p + 2],
            bytes[p + 3],
            bytes[p + 4],
            bytes[p + 5],
            bytes[p + 6],
            bytes[p + 7],
        ]);
        let len = u32::from_le_bytes([bytes[p + 8], bytes[p + 9], bytes[p + 10], bytes[p + 11]]);
        if offset as usize + len as usize > data_end {
            return Err(AppError::Data("arpack truncated: block data out of range".into()));
        }
        index.push((offset, len));
    }
    Ok((
        HeaderFields {
            data_version,
            rows,
            cols,
            origin_lon,
            origin_lat,
            cell_lon_deg,
            cell_lat_deg,
            z_resolution_m,
            vertical_datum,
            resolution_semantics,
            block_compression,
            blocks_x,
            blocks_y,
            no_data,
            source,
            sha_expected,
        },
        index,
    ))
}

impl BuiltinSource {
    /// 打开 + 结构校验（O(1)：magic/版本/块索引越界；**不做全量 SHA**——按需读取）。
    /// 块数据按需从 mmap 读取；全量完整性用 [`Self::verify_sha`]（导入后/开发期）。
    pub fn open(path: &Path) -> Result<Self, AppError> {
        use std::io::{Read, Seek, SeekFrom};
        let mut file = std::fs::File::open(path)?;
        let file_len = file.metadata()?.len() as usize;
        if file_len < HEADER_SIZE + 32 {
            return Err(AppError::Data(format!(
                "arpack file truncated: {file_len} bytes < header+sha"
            )));
        }
        // 读头部（含 sha 字段）→ 解析 blocks_x/y → 读块索引区 → 结构校验（O(1)）
        let mut hdr = [0u8; HEADER_SIZE + 32];
        file.read_exact(&mut hdr)?;
        let le = |i: usize| -> u32 {
            u32::from_le_bytes([hdr[i], hdr[i + 1], hdr[i + 2], hdr[i + 3]])
        };
        if hdr[0..8] != MAGIC {
            return Err(AppError::Data("arpack magic mismatch (not an ARPACK1 file)".into()));
        }
        if le(8) != FORMAT_VERSION {
            return Err(AppError::Data(format!(
                "arpack format_version mismatch: got {}, expect {FORMAT_VERSION}",
                le(8)
            )));
        }
        let blocks_x = le(72) as usize;
        let blocks_y = le(76) as usize;
        let idx_bytes = blocks_x.checked_mul(blocks_y).unwrap_or(0).checked_mul(12).unwrap_or(0);
        let mut hdr_index = vec![0u8; HEADER_SIZE + 32 + idx_bytes];
        hdr_index[..HEADER_SIZE + 32].copy_from_slice(&hdr);
        if idx_bytes > 0 {
            file.seek(SeekFrom::Start((HEADER_SIZE + 32) as u64))?;
            file.read_exact(&mut hdr_index[HEADER_SIZE + 32..])?;
        }
        let (h, index) = parse_header_index(&hdr_index, file_len)?;
        // mmap 全文件（虚拟映射，物理页按访问加载；只读共享，多线程无锁读块）
        let mmap = unsafe { memmap2::Mmap::map(&file)? };
        Ok(Self {
            data_version: h.data_version,
            rows: h.rows,
            cols: h.cols,
            origin_lon: h.origin_lon,
            origin_lat: h.origin_lat,
            cell_lon_deg: h.cell_lon_deg,
            cell_lat_deg: h.cell_lat_deg,
            z_resolution_m: h.z_resolution_m,
            vertical_datum: h.vertical_datum,
            resolution_semantics: h.resolution_semantics,
            block_compression: h.block_compression,
            blocks_x: h.blocks_x,
            blocks_y: h.blocks_y,
            no_data: h.no_data,
            source: h.source,
            sha_expected: h.sha_expected,
            index,
            store: Store::Mmap(mmap),
            cache: Mutex::new(BlockCache::default()),
        })
    }

    /// 从字节解析（测试友好）。全部校验失败 → `AppError::Data`（含全量 SHA）。
    pub fn parse(bytes: &[u8]) -> Result<Self, AppError> {
        let (h, index) = parse_header_index(bytes, bytes.len())?;

        // SHA-256 校验（数据部分 = 从 idx_start 到文件尾）
        let idx_start = HEADER_SIZE + 32;
        let mut hasher = Sha256::new();
        hasher.update(&bytes[idx_start..]);
        let digest = hasher.finalize();
        if digest.as_slice() != &bytes[HEADER_SIZE..HEADER_SIZE + 32] {
            return Err(AppError::Data("arpack sha256 mismatch (corrupt data)".into()));
        }

        Ok(Self {
            data_version: h.data_version,
            rows: h.rows,
            cols: h.cols,
            origin_lon: h.origin_lon,
            origin_lat: h.origin_lat,
            cell_lon_deg: h.cell_lon_deg,
            cell_lat_deg: h.cell_lat_deg,
            z_resolution_m: h.z_resolution_m,
            vertical_datum: h.vertical_datum,
            resolution_semantics: h.resolution_semantics,
            block_compression: h.block_compression,
            blocks_x: h.blocks_x,
            blocks_y: h.blocks_y,
            no_data: h.no_data,
            source: h.source,
            sha_expected: h.sha_expected,
            index,
            store: Store::Mem(bytes.to_vec()),
            cache: Mutex::new(BlockCache::default()),
        })
    }

    /// 全量 SHA-256 校验（数据部分 = 块索引区起）。开发期/导入后用；发布默认不调
    /// （open 仅结构校验，O(1)）。数据损坏 → `AppError::Data`。
    pub fn verify_sha(&self) -> Result<(), AppError> {
        let idx_start = HEADER_SIZE + 32;
        let mut hasher = Sha256::new();
        match &self.store {
            Store::Mem(b) => hasher.update(&b[idx_start..]),
            Store::Mmap(m) => hasher.update(&m[idx_start..]),
        }
        let digest = hasher.finalize();
        if digest.as_slice() != self.sha_expected {
            return Err(AppError::Data("arpack sha256 mismatch (corrupt data)".into()));
        }
        Ok(())
    }

    pub fn data_version(&self) -> u32 {
        self.data_version
    }
    pub fn z_resolution_m(&self) -> f64 {
        self.z_resolution_m
    }
    pub fn vertical_datum(&self) -> u8 {
        self.vertical_datum
    }
    pub fn source_desc(&self) -> &str {
        &self.source
    }

    /// 取原始格值（空洞 → None）。跨块自动加载/缓存（FIFO 有界，解压锁外进行）。
    fn cell(&self, r: usize, c: usize) -> Option<i16> {
        if r >= self.rows || c >= self.cols {
            return None;
        }
        let bx = r / BLOCK_SIZE as usize;
        let by = c / BLOCK_SIZE as usize;
        let lr = r % BLOCK_SIZE as usize;
        let lc = c % BLOCK_SIZE as usize;
        let bidx = bx * self.blocks_y + by;

        // 快速路径：缓存命中（锁内仅查表，不解压）
        let v = {
            let cache = lock_cache(&self.cache);
            cache.map.get(&bidx).map(|b| b[lr * BLOCK_SIZE as usize + lc])
        };
        if let Some(v) = v {
            return if v == self.no_data { None } else { Some(v) };
        }

        // 未命中：锁外解压（多线程不互相阻塞）
        let block = self.load_block(bidx)?;
        let v = block[lr * BLOCK_SIZE as usize + lc];
        // 双检锁插入（并发下重复解压可接受，但避免重复驻留）
        let mut cache = lock_cache(&self.cache);
        if !cache.map.contains_key(&bidx) {
            cache.insert(bidx, block);
        }
        if v == self.no_data {
            None
        } else {
            Some(v)
        }
    }

    /// 加载并还原一个块（raw / zstd）。块数据按需从 store（mmap/内存）读取。
    fn load_block(&self, bidx: usize) -> Option<Vec<i16>> {
        let (offset, len) = *self.index.get(bidx)?;
        let raw = self.store.get(offset as usize, len as usize)?;
        let block_n = BLOCK_SIZE as usize * BLOCK_SIZE as usize;
        let mut out = match self.block_compression {
            COMPRESSION_RAW => {
                if raw.len() != block_n * 2 {
                    return None;
                }
                raw.chunks_exact(2)
                    .map(|b| i16::from_le_bytes([b[0], b[1]]))
                    .collect::<Vec<i16>>()
            }
            COMPRESSION_ZSTD => {
                let mut dec = ruzstd::decoding::StreamingDecoder::new(raw).ok()?;
                let mut buf = Vec::with_capacity(block_n * 2);
                std::io::Read::read_to_end(&mut dec, &mut buf).ok()?;
                if buf.len() != block_n * 2 {
                    return None;
                }
                buf.chunks_exact(2)
                    .map(|b| i16::from_le_bytes([b[0], b[1]]))
                    .collect::<Vec<i16>>()
            }
            COMPRESSION_DEFLATE => {
                let buf = miniz_oxide::inflate::decompress_to_vec_zlib(raw).ok()?;
                if buf.len() != block_n * 2 {
                    return None;
                }
                buf.chunks_exact(2)
                    .map(|b| i16::from_le_bytes([b[0], b[1]]))
                    .collect::<Vec<i16>>()
            }
            _ => return None,
        };
        // 行内差分还原：v[0] 原样，v[i] += v[i-1]
        for i in 1..out.len() {
            let (a, b) = (out[i - 1], out[i]);
            out[i] = a.wrapping_add(b);
        }
        Some(out)
    }

    /// 双线性插值采样（经纬度，度）。
    fn sample(&self, lon: f64, lat: f64) -> Option<f64> {
        let fc = (lon - self.origin_lon) / self.cell_lon_deg;
        let fr = (lat - self.origin_lat) / self.cell_lat_deg;
        let c0 = fc.floor() as isize;
        let r0 = fr.floor() as isize;
        if c0 < 0 || r0 < 0 || c0 + 1 >= self.cols as isize || r0 + 1 >= self.rows as isize {
            return None;
        }
        let w_c = fc - c0 as f64;
        let w_r = fr - r0 as f64;
        let (r0, c0) = (r0 as usize, c0 as usize);
        let v00 = self.cell(r0, c0)?;
        let v01 = self.cell(r0, c0 + 1)?;
        let v10 = self.cell(r0 + 1, c0)?;
        let v11 = self.cell(r0 + 1, c0 + 1)?;
        Some(interp2(v00, v01, v10, v11, w_c, w_r))
    }
}

/// 双线性插值（cell 值 → f64；供带锁/无锁两路径复用，保证数值一致）。
#[inline]
fn interp2(v00: i16, v01: i16, v10: i16, v11: i16, w_c: f64, w_r: f64) -> f64 {
    let h00 = v00 as f64;
    let h01 = v01 as f64;
    let h10 = v10 as f64;
    let h11 = v11 as f64;
    let top = h00 + (h01 - h00) * w_c;
    let bot = h10 + (h11 - h10) * w_c;
    top + (bot - top) * w_r
}

impl BulkPrefetch for BuiltinSource {
    /// 经纬度矩形 → 地形行列范围（±1 余量覆盖双线性邻域）→ 锁外加载全部块。
    fn prefetch_lonlat(
        &self,
        min_lon: f64,
        min_lat: f64,
        max_lon: f64,
        max_lat: f64,
    ) -> HashMap<usize, Vec<i16>> {
        let c0f = ((min_lon - self.origin_lon) / self.cell_lon_deg).floor() - 1.0;
        let c1f = ((max_lon - self.origin_lon) / self.cell_lon_deg).ceil() + 1.0;
        let r0f = ((min_lat - self.origin_lat) / self.cell_lat_deg).floor() - 1.0;
        let r1f = ((max_lat - self.origin_lat) / self.cell_lat_deg).ceil() + 1.0;
        let c0 = c0f.max(0.0) as usize;
        let c1 = (c1f.min(self.cols as f64)).max(c0 as f64) as usize;
        let r0 = r0f.max(0.0) as usize;
        let r1 = (r1f.min(self.rows as f64)).max(r0 as f64) as usize;
        self.prefetch_rect(r0, r1, c0, c1)
    }

    /// 无锁采样（与 `sample_at` 同语义：bounds 外 OOB，洞 NoData，否则 Land(h)）。
    fn sample_local(&self, local: &HashMap<usize, Vec<i16>>, lon: f64, lat: f64) -> Sample {
        if let Some(b) = self.bounds() {
            if !b.contains(lon, lat) {
                return Sample::OutOfBounds;
            }
        }
        match self.sample_local_height(local, lon, lat) {
            Some(h) => Sample::Land(h),
            None => Sample::NoData,
        }
    }
}

impl BuiltinSource {
    /// 锁外加载 [r0..=r1]×[c0..=c1]（地形行列）覆盖的全部块。
    /// 优先复用全局缓存（clone），未命中锁外解压并双检插入全局缓存（后续 FMM 阶段仍可用）。
    /// 返回局部块表（与 `cell()` 同数值；配合 `sample_local` 无锁访问）。
    fn prefetch_rect(&self, r0: usize, r1: usize, c0: usize, c1: usize) -> HashMap<usize, Vec<i16>> {
        let mut local = HashMap::new();
        if r0 > r1 || c0 > c1 {
            return local;
        }
        let bx0 = r0 / BLOCK_SIZE as usize;
        let bx1 = r1 / BLOCK_SIZE as usize;
        let by0 = c0 / BLOCK_SIZE as usize;
        let by1 = c1 / BLOCK_SIZE as usize;
        for bx in bx0..=bx1 {
            for by in by0..=by1 {
                let bidx = bx * self.blocks_y + by;
                if local.contains_key(&bidx) {
                    continue;
                }
                // 先查全局缓存（锁内仅查表+clone）
                let hit = {
                    let cache = lock_cache(&self.cache);
                    cache.map.get(&bidx).cloned()
                };
                let block = match hit {
                    Some(b) => b,
                    None => {
                        // 未命中：锁外解压，双检插入全局（后续 FMM 阶段仍可用）
                        let Some(b) = self.load_block(bidx) else { continue };
                        let mut cache = lock_cache(&self.cache);
                        if !cache.map.contains_key(&bidx) {
                            cache.insert(bidx, b.clone());
                        }
                        b
                    }
                };
                local.insert(bidx, block);
            }
        }
        local
    }

    /// 无锁高度采样（双线性插值，与 `sample` 数值一致；local 未命中回退带锁 `cell`）。
    fn sample_local_height(
        &self,
        local: &HashMap<usize, Vec<i16>>,
        lon: f64,
        lat: f64,
    ) -> Option<f64> {
        let fc = (lon - self.origin_lon) / self.cell_lon_deg;
        let fr = (lat - self.origin_lat) / self.cell_lat_deg;
        let c0 = fc.floor() as isize;
        let r0 = fr.floor() as isize;
        if c0 < 0 || r0 < 0 || c0 + 1 >= self.cols as isize || r0 + 1 >= self.rows as isize {
            return None;
        }
        let w_c = fc - c0 as f64;
        let w_r = fr - r0 as f64;
        let (r0, c0) = (r0 as usize, c0 as usize);
        let v00 = self.cell_local(local, r0, c0)?;
        let v01 = self.cell_local(local, r0, c0 + 1)?;
        let v10 = self.cell_local(local, r0 + 1, c0)?;
        let v11 = self.cell_local(local, r0 + 1, c0 + 1)?;
        Some(interp2(v00, v01, v10, v11, w_c, w_r))
    }

    /// 无锁格值读取：local 命中直读（无锁），未命中回退带锁 `cell`（语义与 cell 完全一致）。
    #[inline]
    fn cell_local(&self, local: &HashMap<usize, Vec<i16>>, r: usize, c: usize) -> Option<i16> {
        if r >= self.rows || c >= self.cols {
            return None;
        }
        let bx = r / BLOCK_SIZE as usize;
        let by = c / BLOCK_SIZE as usize;
        let bidx = bx * self.blocks_y + by;
        let lr = r % BLOCK_SIZE as usize;
        let lc = c % BLOCK_SIZE as usize;
        match local.get(&bidx) {
            Some(b) => {
                let v = b[lr * BLOCK_SIZE as usize + lc];
                if v == self.no_data {
                    None
                } else {
                    Some(v)
                }
            }
            None => self.cell(r, c),
        }
    }
}

/// Mutex 锁（防 poison panic：崩溃套件不允许任何 panic）。
fn lock_cache(m: &Mutex<BlockCache>) -> std::sync::MutexGuard<'_, BlockCache> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

impl TerrainSource for BuiltinSource {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        self.sample(lon, lat)
    }

    fn bounds(&self) -> Option<GeoBounds> {
        Some(GeoBounds {
            min_lon: self.origin_lon,
            min_lat: self.origin_lat,
            max_lon: self.origin_lon + self.cols as f64 * self.cell_lon_deg,
            max_lat: self.origin_lat + self.rows as f64 * self.cell_lat_deg,
        })
    }

    fn resolution_desc(&self) -> String {
        format!(
            "arpack1 {}x{} ({}x{} blocks) cell {:.6}deg x {:.6}deg z={}m vd={} sem={} src={}",
            self.rows,
            self.cols,
            self.blocks_x,
            self.blocks_y,
            self.cell_lon_deg,
            self.cell_lat_deg,
            self.z_resolution_m,
            if self.vertical_datum == VDATUM_ELLIPSOID { "ellipsoid" } else { "egm96" },
            if self.resolution_semantics == SEMANTICS_EQUIANGULAR { "equiangular" } else { "equidistant" },
            self.source
        )
    }
}

// ==================== Writer（测试/开发期工具；raw 块，zstd 块由开发期脚本产出） ====================

/// 生成 ARPK1 字节（raw 块，供测试/工具；压缩块由 phase0 脚本 pyzstd 产出）。
/// `h`：行优先 i16 高度（rows × cols）。
pub fn write_pack_raw(
    rows: usize,
    cols: usize,
    origin_lon: f64,
    origin_lat: f64,
    cell_lon_deg: f64,
    cell_lat_deg: f64,
    z_resolution_m: f64,
    vertical_datum_ellipsoid: bool,
    no_data: i16,
    source: &str,
    h: &[i16],
) -> Vec<u8> {
    debug_assert_eq!(h.len(), rows * cols);
    let blocks_x = rows.div_ceil(BLOCK_SIZE as usize);
    let blocks_y = cols.div_ceil(BLOCK_SIZE as usize);
    let n_blocks = blocks_x * blocks_y;
    let mut out = Vec::new();
    // header 288B + sha 32B 占位（sha 在 [288..320]，索引从 320 起——与 parse 的 idx_start 对齐）
    out.resize(HEADER_SIZE + 32, 0);
    out[0..8].copy_from_slice(&MAGIC);
    put_u32(&mut out, 8, FORMAT_VERSION);
    put_u32(&mut out, 12, 1); // data_version = 1
    put_u32(&mut out, 16, rows as u32);
    put_u32(&mut out, 20, cols as u32);
    put_f64(&mut out, 24, origin_lon);
    put_f64(&mut out, 32, origin_lat);
    put_f64(&mut out, 40, cell_lon_deg);
    put_f64(&mut out, 48, cell_lat_deg);
    put_f64(&mut out, 56, z_resolution_m);
    out[64] = if vertical_datum_ellipsoid { VDATUM_ELLIPSOID } else { VDATUM_EGM96 };
    out[65] = SEMANTICS_EQUIANGULAR;
    out[66] = COMPRESSION_RAW;
    out[67] = 0;
    put_u32(&mut out, 68, BLOCK_SIZE);
    put_u32(&mut out, 72, blocks_x as u32);
    put_u32(&mut out, 76, blocks_y as u32);
    out[80..82].copy_from_slice(&no_data.to_le_bytes());
    let src = source.as_bytes();
    let n = src.len().min(174);
    out[82..82 + n].copy_from_slice(&src[..n]);

    // 块数据（先算好偏移，再填索引）
    let idx_start = HEADER_SIZE + 32;
    let mut idx = vec![(0u64, 0u32); n_blocks];
    let mut blocks: Vec<Vec<u8>> = Vec::with_capacity(n_blocks);
    for bx in 0..blocks_x {
        for by in 0..blocks_y {
            let mut block = vec![0i16; BLOCK_SIZE as usize * BLOCK_SIZE as usize];
            for lr in 0..BLOCK_SIZE as usize {
                for lc in 0..BLOCK_SIZE as usize {
                    let r = bx * BLOCK_SIZE as usize + lr;
                    let c = by * BLOCK_SIZE as usize + lc;
                    block[lr * BLOCK_SIZE as usize + lc] = if r < rows && c < cols {
                        h[r * cols + c]
                    } else {
                        no_data
                    };
                }
            }
            // 行内差分编码
            for i in (1..block.len()).rev() {
                block[i] = block[i].wrapping_sub(block[i - 1]);
            }
            let mut bytes = Vec::with_capacity(block.len() * 2);
            for v in &block {
                bytes.extend_from_slice(&v.to_le_bytes());
            }
            blocks.push(bytes);
        }
    }
    // 计算偏移（相对文件开头）
    let mut off = (idx_start + n_blocks * 12) as u64;
    for (i, b) in blocks.iter().enumerate() {
        idx[i] = (off, b.len() as u32);
        off += b.len() as u64;
    }
    // 索引区
    for (offset, len) in &idx {
        out.extend_from_slice(&offset.to_le_bytes());
        out.extend_from_slice(&len.to_le_bytes());
    }
    // 块流
    for b in &blocks {
        out.extend_from_slice(b);
    }
    // SHA-256（数据部分 = idx_start..）
    let data_part = &out[idx_start..];
    let mut hasher = Sha256::new();
    hasher.update(data_part);
    let digest = hasher.finalize();
    out[HEADER_SIZE..HEADER_SIZE + 32].copy_from_slice(&digest);
    out
}

fn put_u32(b: &mut Vec<u8>, i: usize, v: u32) {
    b[i..i + 4].copy_from_slice(&v.to_le_bytes());
}
fn put_f64(b: &mut Vec<u8>, i: usize, v: f64) {
    b[i..i + 8].copy_from_slice(&v.to_le_bytes());
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 缓存 FIFO 淘汰：容量 2，插入 3 块 → 最旧被淘汰。
    #[test]
    fn cache_fifo_eviction() {
        let mut c = BlockCache::with_max(2);
        c.insert(1, vec![10]);
        c.insert(2, vec![20]);
        assert!(c.map.contains_key(&1) && c.map.contains_key(&2));
        c.insert(3, vec![30]);
        assert!(!c.map.contains_key(&1), "最旧块 1 应被淘汰");
        assert!(c.map.contains_key(&2) && c.map.contains_key(&3));
        assert_eq!(c.map.len(), 2, "缓存容量不得超限");
        // 重复插入不扩容
        c.insert(3, vec![99]);
        assert_eq!(c.map.len(), 2);
    }

    /// 缓存命中后值保持正确（不因淘汰/重复插入改变）。
    #[test]
    fn cache_reinsert_keeps_value() {
        let mut c = BlockCache::with_max(2);
        c.insert(1, vec![10]);
        c.insert(1, vec![11]); // 已存在 → 忽略
        assert_eq!(c.map[&1], vec![10]);
    }

    /// 构造 300×200 网格（block 覆盖 2x1 块），全 500m，右下角空洞。
    fn pack_fixture() -> (Vec<u8>, Vec<i16>) {
        let rows = 300;
        let cols = 200;
        let mut h = vec![500i16; rows * cols];
        h[rows * cols - 1] = -32768; // 空洞
        let bytes = write_pack_raw(rows, cols, 116.0, 39.0, 0.001, 0.001, 50.0, true, -32768, "test fixture", &h);
        (bytes, h)
    }

    #[test]
    fn open_and_sample() {
        let (bytes, _h) = pack_fixture();
        let s = BuiltinSource::parse(&bytes).unwrap();
        assert_eq!(s.data_version(), 1);
        assert_eq!(s.z_resolution_m(), 50.0);
        assert!(s.height_at(116.1, 39.1).is_some());
        assert!(s.height_at(116.2, 39.3).is_none()); // 出界
        // 空洞（右下角 299,199 → lon=116.0+199*0.001=116.199, lat=39.0+299*0.001=39.299）
        assert!(s.height_at(116.199, 39.299).is_none());
    }

    #[test]
    fn fail_fast_hash_mismatch() {
        let (mut bytes, _) = pack_fixture();
        let n = bytes.len();
        bytes[n - 1] ^= 0xFF; // 破坏数据（sha 不符）
        match BuiltinSource::parse(&bytes) {
            Err(AppError::Data(msg)) => assert!(msg.contains("sha256"), "msg={msg}"),
            other => panic!("expected sha256 error, got {other:?}"),
        }
    }

    #[test]
    fn fail_fast_version_mismatch() {
        let (mut bytes, _) = pack_fixture();
        put_u32(&mut bytes, 8, 99); // 破坏 format_version（不改 sha：校验顺序版本在前）
        match BuiltinSource::parse(&bytes) {
            Err(AppError::Data(msg)) => assert!(msg.contains("format_version"), "msg={msg}"),
            other => panic!("expected version error, got {other:?}"),
        }
    }

    #[test]
    fn fail_fast_truncated() {
        let (bytes, _) = pack_fixture();
        let cut = bytes.len() / 2;
        match BuiltinSource::parse(&bytes[..cut]) {
            Err(AppError::Data(_)) => {}
            other => panic!("expected truncation error, got {other:?}"),
        }
    }

    #[test]
    fn fail_fast_magic() {
        let (mut bytes, _) = pack_fixture();
        bytes[0] = b'X';
        match BuiltinSource::parse(&bytes) {
            Err(AppError::Data(msg)) => assert!(msg.contains("magic"), "msg={msg}"),
            other => panic!("expected magic error, got {other:?}"),
        }
    }

    #[test]
    fn multi_block_cross_boundary_sample() {
        // 跨块边界采样（250,150 附近 = 块(0,0) 与块(1,0) 交界）
        let (bytes, _) = pack_fixture();
        let s = BuiltinSource::parse(&bytes).unwrap();
        // lon=116.0+150*0.001=116.15, lat=39.0+255*0.001=39.255 → r=255 跨块边界（块0行 250..255）
        assert!(s.height_at(116.15, 39.255).is_some());
        // 空洞邻域正常采样（避开空洞 4 角）
        assert!(s.height_at(116.198, 39.297).is_some());
        // 空洞 → None
        assert!(s.height_at(116.199, 39.299).is_none());
    }

    // ---- mmap 按需打开（2026-08-07 主管拍板：大型 ARPK1 打开 O(1) + 块按需） ----

    /// mmap open 与 parse 内存路径 bit-exact 一致（全网格采样 + 越界 + 空洞）。
    #[test]
    fn open_mmap_matches_parse() {
        let (bytes, _h) = pack_fixture();
        let dir = std::env::temp_dir();
        let path = dir.join(format!("arpk_mmap_test_{}.arpack", std::process::id()));
        std::fs::write(&path, &bytes).unwrap();
        let result = (|| -> Result<(), AppError> {
            let s = BuiltinSource::open(&path)?;
            s.verify_sha()?;
            let p = BuiltinSource::parse(&bytes).unwrap();
            assert_eq!(s.rows, p.rows);
            assert_eq!(s.cols, p.cols);
            assert_eq!(s.origin_lon, p.origin_lon);
            assert_eq!(s.origin_lat, p.origin_lat);
            assert_eq!(s.cell_lon_deg, p.cell_lon_deg);
            assert_eq!(s.cell_lat_deg, p.cell_lat_deg);
            assert_eq!(s.blocks_x, p.blocks_x);
            assert_eq!(s.blocks_y, p.blocks_y);
            assert_eq!(s.no_data, p.no_data);
            assert_eq!(s.source, p.source);
            // 全网格采样 bit-exact
            for r in 0..s.rows {
                for c in 0..s.cols {
                    let lon = s.origin_lon + c as f64 * s.cell_lon_deg + s.cell_lon_deg / 2.0;
                    let lat = s.origin_lat + r as f64 * s.cell_lat_deg + s.cell_lat_deg / 2.0;
                    assert_eq!(s.sample(lon, lat), p.sample(lon, lat), "mismatch r={r} c={c}");
                }
            }
            // 越界 / 空洞语义一致
            assert!(s.height_at(116.2, 39.3).is_none());
            assert!(s.height_at(116.199, 39.299).is_none());
            Ok(())
        })();
        let _ = std::fs::remove_file(&path);
        result.unwrap();
    }

    /// mmap open 仅结构校验（O(1)）：块数据流内破坏 → open 成功 + 采样可用，
    /// 全量校验 `verify_sha` 才报错（发布默认不调；开发期/导入后用）。
    #[test]
    fn open_mmap_structural_ok_sha_fails() {
        let (mut bytes, _) = pack_fixture();
        let n = bytes.len();
        bytes[n - 1] ^= 0xFF; // 破坏最后一个字节（块数据流内，头部/索引结构完好）
        let dir = std::env::temp_dir();
        let path = dir.join(format!("arpk_mmap_corrupt_{}.arpack", std::process::id()));
        std::fs::write(&path, &bytes).unwrap();
        let result = (|| -> Result<(), AppError> {
            let s = BuiltinSource::open(&path)?; // 结构校验通过
            assert!(s.height_at(116.1, 39.1).is_some()); // 采样仍可用
            match s.verify_sha() {
                Err(AppError::Data(msg)) => assert!(msg.contains("sha256"), "msg={msg}"),
                other => panic!("expected sha256 error, got {other:?}"),
            }
            Ok(())
        })();
        let _ = std::fs::remove_file(&path);
        result.unwrap();
    }

    /// mmap open 截断：块数据流被截断 → open 结构校验失败（索引越界，O(1) 拦下）。
    #[test]
    fn open_mmap_truncated_fails() {
        let (bytes, _) = pack_fixture();
        let dir = std::env::temp_dir();
        let path = dir.join(format!("arpk_mmap_trunc_{}.arpack", std::process::id()));
        std::fs::write(&path, &bytes[..bytes.len() - 100]).unwrap(); // 截断 100B
        let result = (|| -> Result<(), AppError> {
            match BuiltinSource::open(&path) {
                Err(AppError::Data(_)) => Ok(()),
                other => panic!("expected truncated error, got {other:?}"),
            }
        })();
        let _ = std::fs::remove_file(&path);
        result.unwrap();
    }
}

//! 海陆掩膜（GSHHG 3 态，水体判定先验 B 档：岸线掩膜）。
//!
//! 格式（V2 3 态）：
//! ```text
//! [0..16)   magic "ARPACK_MASK_V2__"
//! [16..64)  header：version u32 BE / arcsec u32 BE / rows u32 BE / cols u32 BE /
//!           lon0 f64 BE / lat0 f64 BE / res_deg f64 BE
//! [64..)    行索引表：(rows+1) × offset u64 BE（段区绝对偏移，自文件头）
//! […)       行段区：每行 [nseg u32 BE, (class u8, start u32 BE, end u32 BE)×nseg]
//! ```
//!
//! 类别语义（主管 2026-08-04 拍板）：
//! - 0 = 海洋（隐式，段不存；海平面高程 0）
//! - 1 = 陆地（含南极内陆补全：-85.15°S 以南 + 东南极 -75..-85.15°S 0..160°E）
//! - 2 = 内陆湖（湖面高程由 DEM 提供，一般高于海平面）

use super::{BulkPrefetch, GeoBounds, Sample, TerrainSource};
use crate::error::AppError;

/// 掩膜类别。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum MaskClass {
    Sea = 0,
    Land = 1,
    Lake = 2,
}

pub const MAGIC: [u8; 16] = *b"ARPACK_MASK_V2__";
pub const HEADER_SIZE: usize = 64;

/// GSHHG 海陆掩膜（3 态，RLE 行段存储）。
#[derive(Debug)]
pub struct GeoMask {
    version: u32,
    arcsec: u32,
    rows: usize,
    cols: usize,
    lon0: f64,
    lat0: f64,
    res_deg: f64,
    /// 行段区绝对偏移（len = rows+1）
    row_offsets: Vec<u64>,
    /// 段区字节（自 64 + (rows+1)*8 起）
    data: Vec<u8>,
}

impl GeoMask {
    /// 打开 + fail-fast 校验（magic/版本/尺寸/索引一致性）。
    pub fn open(path: &std::path::Path) -> Result<Self, AppError> {
        let bytes = std::fs::read(path)?;
        Self::parse(&bytes)
    }

    /// 从字节解析（测试友好）。全部校验失败 → `AppError::Data`。
    pub fn parse(bytes: &[u8]) -> Result<Self, AppError> {
        if bytes.len() < HEADER_SIZE {
            return Err(AppError::Data("mask file truncated: < 64B header".into()));
        }
        if bytes[0..16] != MAGIC {
            return Err(AppError::Data("mask magic mismatch (not a V2 mask)".into()));
        }
        let be_u32 = |i: usize| -> u32 {
            u32::from_be_bytes([bytes[i], bytes[i + 1], bytes[i + 2], bytes[i + 3]])
        };
        let be_f64 = |i: usize| -> f64 {
            f64::from_be_bytes([
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
        let version = be_u32(16);
        if version != 2 {
            return Err(AppError::Data(format!(
                "mask version mismatch: got {version}, expect 2"
            )));
        }
        let arcsec = be_u32(20);
        let rows = be_u32(24) as usize;
        let cols = be_u32(28) as usize;
        let lon0 = be_f64(32);
        let lat0 = be_f64(40);
        let res_deg = be_f64(48);
        if rows == 0 || cols == 0 || !res_deg.is_finite() || res_deg <= 0.0 {
            return Err(AppError::Data("mask degenerate header".into()));
        }
        let idx_bytes = (rows + 1) * 8;
        let need = HEADER_SIZE + idx_bytes;
        if bytes.len() < need {
            return Err(AppError::Data(
                "mask truncated: row index out of range".into(),
            ));
        }
        // 行索引表（绝对偏移，应单调不减）
        let mut row_offsets = Vec::with_capacity(rows + 1);
        let mut prev = HEADER_SIZE as u64;
        for i in 0..=rows {
            let p = HEADER_SIZE + i * 8;
            let off = u64::from_be_bytes([
                bytes[p],
                bytes[p + 1],
                bytes[p + 2],
                bytes[p + 3],
                bytes[p + 4],
                bytes[p + 5],
                bytes[p + 6],
                bytes[p + 7],
            ]);
            if off < prev {
                return Err(AppError::Data("mask row index not monotonic".into()));
            }
            row_offsets.push(off);
            prev = off;
        }
        if row_offsets[rows] > bytes.len() as u64 {
            return Err(AppError::Data(
                "mask truncated: row data out of range".into(),
            ));
        }
        let data = bytes[need..].to_vec();
        Ok(Self {
            version,
            arcsec,
            rows,
            cols,
            lon0,
            lat0,
            res_deg,
            row_offsets,
            data,
        })
    }

    pub fn arcsec(&self) -> u32 {
        self.arcsec
    }
    pub fn version(&self) -> u32 {
        self.version
    }
    pub fn rows(&self) -> usize {
        self.rows
    }
    pub fn cols(&self) -> usize {
        self.cols
    }
    pub fn resolution_desc(&self) -> String {
        format!(
            "gshhg mask {}as {}x{} cell {:.6}deg",
            self.arcsec, self.rows, self.cols, self.res_deg
        )
    }

    /// 查询类别（经纬度，度；lon ∈ [-180, 180]）。
    /// 防御：非有限输入 / 越界（lat=90 等）→ Sea（掩膜外视为海洋，不 panic）。
    pub fn class_at(&self, lon: f64, lat: f64) -> MaskClass {
        if !lon.is_finite() || !lat.is_finite() {
            return MaskClass::Sea;
        }
        let lon = if lon < 0.0 {
            lon + 360.0
        } else if lon >= 360.0 {
            lon - 360.0
        } else {
            lon
        };
        let c = ((lon - self.lon0) / self.res_deg).floor() as i64;
        let r = ((lat - self.lat0) / self.res_deg).floor() as i64;
        if r < 0 || r >= self.rows as i64 || c < 0 || c >= self.cols as i64 {
            return MaskClass::Sea;
        }
        let (r, c) = (r as usize, c as usize);
        let start = self.row_offsets[r] as usize;
        let end = self.row_offsets[r + 1] as usize;
        // start 相对 data 区（data 从 need 起；row_offsets 是绝对偏移）
        let seg_base = HEADER_SIZE + (self.rows + 1) * 8;
        let start = start.saturating_sub(seg_base);
        let end = end.saturating_sub(seg_base);
        if start + 4 > end || end > self.data.len() {
            return MaskClass::Sea;
        }
        let nseg = u32::from_be_bytes([
            self.data[start],
            self.data[start + 1],
            self.data[start + 2],
            self.data[start + 3],
        ]);
        // 注意：段按 class 分组存储（class 1 全部在前、class 2 在后），非列序！
        // 因此不能按列序早停（`c < c0 → break` 会漏掉后置的湖泊段）——
        // 与 Python query 的 `if c > c1: continue` 语义一致，遍历全部段。
        let mut p = start + 4;
        for _ in 0..nseg {
            if p + 9 > end {
                break;
            }
            let cls = self.data[p];
            let c0 = u32::from_be_bytes([
                self.data[p + 1],
                self.data[p + 2],
                self.data[p + 3],
                self.data[p + 4],
            ]);
            let c1 = u32::from_be_bytes([
                self.data[p + 5],
                self.data[p + 6],
                self.data[p + 7],
                self.data[p + 8],
            ]);
            if (c0 as usize) <= c && c < c1 as usize {
                return match cls {
                    1 => MaskClass::Land,
                    2 => MaskClass::Lake,
                    _ => MaskClass::Sea,
                };
            }
            p += 9;
        }
        MaskClass::Sea
    }

    /// 调试：行偏移（example 用）。
    pub fn debug_row_offset(&self, r: usize) -> u64 {
        self.row_offsets.get(r).copied().unwrap_or(0)
    }
    /// 调试：段区长度。
    pub fn debug_data_len(&self) -> usize {
        self.data.len()
    }
    /// 调试：段区引用。
    pub fn debug_data(&self) -> &[u8] {
        &self.data
    }
    /// 调试：网格分辨率（度）。
    pub fn debug_res(&self) -> f64 {
        self.res_deg
    }

    /// 陆地/湖泊占比（格子口径，遍历全部段；验证用）。
    pub fn land_lake_ratio(&self) -> (f64, f64) {
        let seg_base = HEADER_SIZE + (self.rows + 1) * 8;
        let mut land: u64 = 0;
        let mut lake: u64 = 0;
        for r in 0..self.rows {
            let s = (self.row_offsets[r] as usize).saturating_sub(seg_base);
            let e = (self.row_offsets[r + 1] as usize).saturating_sub(seg_base);
            if s + 4 > e || e > self.data.len() {
                continue;
            }
            let nseg = u32::from_be_bytes([
                self.data[s],
                self.data[s + 1],
                self.data[s + 2],
                self.data[s + 3],
            ]);
            let mut p = s + 4;
            for _ in 0..nseg {
                if p + 9 > e {
                    break;
                }
                let cls = self.data[p];
                let c0 = u32::from_be_bytes([
                    self.data[p + 1],
                    self.data[p + 2],
                    self.data[p + 3],
                    self.data[p + 4],
                ]);
                let c1 = u32::from_be_bytes([
                    self.data[p + 5],
                    self.data[p + 6],
                    self.data[p + 7],
                    self.data[p + 8],
                ]);
                let n = (c1 - c0) as u64;
                if cls == 1 {
                    land += n;
                } else if cls == 2 {
                    lake += n;
                }
                p += 9;
            }
        }
        let total = (self.rows * self.cols) as f64;
        (land as f64 / total, lake as f64 / total)
    }
}

/// 掩膜包装数据源（水体判定 B 档：GSHHG 岸线掩膜分类）。
///
/// 结合 DEM 高度与掩膜 3 态：
/// - 掩膜 Sea → `Sample::Water`（海洋，海平面 0，不依赖 DEM）；
/// - 掩膜 Lake → `Sample::Lake(h)`（湖面高程 = DEM 值；DEM 缺失 → 保守 NoData，
///   湖面高度未知不得按海平面 0 飞）；
/// - 掩膜 Land → 委托 inner 采样（`Land(h)` / `NoData` / `OutOfBounds`）。
pub struct MaskedSource<T: TerrainSource> {
    inner: T,
    mask: GeoMask,
}

impl<T: TerrainSource> MaskedSource<T> {
    pub fn new(inner: T, mask: GeoMask) -> Self {
        Self { inner, mask }
    }

    pub fn inner(&self) -> &T {
        &self.inner
    }
    pub fn mask(&self) -> &GeoMask {
        &self.mask
    }
}

impl<T: TerrainSource> TerrainSource for MaskedSource<T> {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        self.sample_at(lon, lat).height()
    }

    fn sample_at(&self, lon: f64, lat: f64) -> Sample {
        match self.mask.class_at(lon, lat) {
            MaskClass::Sea => Sample::Water,
            MaskClass::Lake => match self.inner.sample_at(lon, lat) {
                Sample::Land(h) | Sample::Lake(h) => Sample::Lake(h),
                Sample::Water => Sample::Lake(0.0),
                Sample::NoData | Sample::OutOfBounds => Sample::NoData,
                Sample::Forbidden => Sample::Forbidden, // 防御透传
            },
            MaskClass::Land => self.inner.sample_at(lon, lat),
        }
    }

    fn bounds(&self) -> Option<GeoBounds> {
        self.inner.bounds()
    }

    fn resolution_desc(&self) -> String {
        format!(
            "{} + {}",
            self.inner.resolution_desc(),
            self.mask.resolution_desc()
        )
    }
}

/// 无锁批量预取转发：mask 分层逻辑与 `sample_at` 完全同构，内层高度走无锁路径。
impl<T: BulkPrefetch> BulkPrefetch for MaskedSource<T> {
    fn prefetch_lonlat(
        &self,
        min_lon: f64,
        min_lat: f64,
        max_lon: f64,
        max_lat: f64,
    ) -> std::collections::HashMap<usize, Vec<i16>> {
        self.inner
            .prefetch_lonlat(min_lon, min_lat, max_lon, max_lat)
    }

    fn sample_local(
        &self,
        local: &std::collections::HashMap<usize, Vec<i16>>,
        lon: f64,
        lat: f64,
    ) -> Sample {
        match self.mask.class_at(lon, lat) {
            MaskClass::Sea => Sample::Water,
            MaskClass::Lake => match self.inner.sample_local(local, lon, lat) {
                Sample::Land(h) | Sample::Lake(h) => Sample::Lake(h),
                Sample::Water => Sample::Lake(0.0),
                Sample::NoData | Sample::OutOfBounds => Sample::NoData,
                Sample::Forbidden => Sample::Forbidden, // 防御透传
            },
            MaskClass::Land => self.inner.sample_local(local, lon, lat),
        }
    }
}

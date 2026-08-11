//! 海陆掩膜（GSHHG 3 态，Phase 2 水体判定先验 B 档：岸线掩膜）。
//!
//! 格式（`phase0/scripts/gshhg_mask.py` 产出，V2 3 态）：
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

use crate::error::AppError;
use super::{BulkPrefetch, GeoBounds, Sample, TerrainSource};

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
            return Err(AppError::Data("mask truncated: row index out of range".into()));
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
            return Err(AppError::Data("mask truncated: row data out of range".into()));
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
        format!("gshhg mask {}as {}x{} cell {:.6}deg", self.arcsec, self.rows, self.cols, self.res_deg)
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
            let c0 = u32::from_be_bytes([self.data[p + 1], self.data[p + 2], self.data[p + 3], self.data[p + 4]]);
            let c1 = u32::from_be_bytes([self.data[p + 5], self.data[p + 6], self.data[p + 7], self.data[p + 8]]);
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
            let nseg = u32::from_be_bytes([self.data[s], self.data[s + 1], self.data[s + 2], self.data[s + 3]]);
            let mut p = s + 4;
            for _ in 0..nseg {
                if p + 9 > e {
                    break;
                }
                let cls = self.data[p];
                let c0 = u32::from_be_bytes([self.data[p + 1], self.data[p + 2], self.data[p + 3], self.data[p + 4]]);
                let c1 = u32::from_be_bytes([self.data[p + 5], self.data[p + 6], self.data[p + 7], self.data[p + 8]]);
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
        self.inner.prefetch_lonlat(min_lon, min_lat, max_lon, max_lat)
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

#[cfg(test)]
mod tests {
    use super::*;

    /// 构造 2×4 掩膜：行 0（lat -90..-88）= 陆地 [0,2)；行 1（lat -88..-86）= 湖泊 [1,3)。
    fn fixture() -> Vec<u8> {
        let mut out = Vec::new();
        out.extend_from_slice(&MAGIC);
        out.extend_from_slice(&2u32.to_be_bytes()); // version
        out.extend_from_slice(&1u32.to_be_bytes()); // arcsec
        out.extend_from_slice(&2u32.to_be_bytes()); // rows
        out.extend_from_slice(&4u32.to_be_bytes()); // cols
        out.extend_from_slice(&0.0f64.to_be_bytes()); // lon0
        out.extend_from_slice(&(-90.0f64).to_be_bytes()); // lat0
        out.extend_from_slice(&1.0f64.to_be_bytes()); // res_deg
        out.extend_from_slice(&[0u8; 8]); // padding 56..64（对齐 HEADER_SIZE）
        // 行索引：段区自 64 + 3*8 = 88
        let seg_base = 88u64;
        let rows_off = [
            seg_base,            // 行 0 起点
            seg_base + 4 + 9,    // 行 1 起点（行 0 = nseg 4B + 1 段 9B）
            seg_base + 4 + 9 + 4 + 9, // 行 2（结束）
        ];
        for o in rows_off {
            out.extend_from_slice(&o.to_be_bytes());
        }
        // 行 0：1 段 (class 1, 0, 2)
        out.extend_from_slice(&1u32.to_be_bytes());
        out.push(1);
        out.extend_from_slice(&0u32.to_be_bytes());
        out.extend_from_slice(&2u32.to_be_bytes());
        // 行 1：1 段 (class 2, 1, 3)
        out.extend_from_slice(&1u32.to_be_bytes());
        out.push(2);
        out.extend_from_slice(&1u32.to_be_bytes());
        out.extend_from_slice(&3u32.to_be_bytes());
        out
    }

    #[test]
    fn parse_and_query() {
        let m = GeoMask::parse(&fixture()).unwrap();
        assert_eq!(m.arcsec(), 1);
        assert_eq!(m.rows(), 2);
        assert_eq!(m.cols(), 4);
        // 行 0（lat -89.5）：陆地 [0,2)
        assert_eq!(m.class_at(0.5, -89.5), MaskClass::Land);
        assert_eq!(m.class_at(1.9, -89.5), MaskClass::Land);
        assert_eq!(m.class_at(2.5, -89.5), MaskClass::Sea);
        // 行 1（lat -88.5）：湖泊 [1,3)
        assert_eq!(m.class_at(1.5, -88.5), MaskClass::Lake);
        assert_eq!(m.class_at(0.5, -88.5), MaskClass::Sea);
        assert_eq!(m.class_at(2.9, -88.5), MaskClass::Lake);
        assert_eq!(m.class_at(3.1, -88.5), MaskClass::Sea);
        // 负经度归一化：lon -179.5 → 180.5 → 列 180（越界 4 列 → Sea）
        assert_eq!(m.class_at(-179.5, -89.5), MaskClass::Sea);
        // 行 1 内（lat -88.1）：湖泊
        assert_eq!(m.class_at(1.5, -88.1), MaskClass::Lake);
        // 边界：lat 恰好 -88（行 1 上边界，半开区间 → 行 2 越界 → Sea，不 panic）
        assert_eq!(m.class_at(1.5, -88.0), MaskClass::Sea);
        // 越界 lat 90 → Sea（不 panic）
        assert_eq!(m.class_at(0.5, 90.0), MaskClass::Sea);
        assert_eq!(m.class_at(0.5, -91.0), MaskClass::Sea);
        // NaN 防御（崩溃套件：不 panic）
        assert_eq!(m.class_at(f64::NAN, 0.0), MaskClass::Sea);
        assert_eq!(m.class_at(0.0, f64::INFINITY), MaskClass::Sea);
    }

    #[test]
    fn ratio() {
        let m = GeoMask::parse(&fixture()).unwrap();
        let (land, lake) = m.land_lake_ratio();
        assert!((land - 0.25).abs() < 1e-9); // 陆地 2/8 格
        assert!((lake - 0.25).abs() < 1e-9); // 湖泊 2/8 格
    }

    #[test]
    fn fail_fast_magic() {
        let mut b = fixture();
        b[0] = b'X';
        assert!(GeoMask::parse(&b).is_err());
    }

    #[test]
    fn fail_fast_version() {
        let mut b = fixture();
        b[16] = 1; // version 2 → 1（大端高位）
        assert!(GeoMask::parse(&b).is_err());
    }

    #[test]
    fn fail_fast_truncated() {
        let b = fixture();
        assert!(GeoMask::parse(&b[..30]).is_err());
    }

    #[test]
    fn fail_fast_non_monotonic_index() {
        let mut b = fixture();
        // 破坏行 1 偏移（88 → 40，破坏单调）
        b[64 + 8] = 0;
        b[64 + 9] = 0;
        b[64 + 10] = 0;
        b[64 + 11] = 40;
        assert!(GeoMask::parse(&b).is_err());
    }

    // ==================== MaskedSource（B 档掩膜分类） ====================

    /// 简易内存源：全高度 `h`；4×4 网格（lat -90..-86，res=1°），保证行 1 可双线性插值。
    fn dem_src(h: f64, hole: bool) -> super::super::memory::Terrain {
        let mut t = super::super::memory::Terrain {
            rows: 4,
            cols: 4,
            origin_lon: 0.0,
            origin_lat: -90.0,
            cell_lon_deg: 1.0,
            cell_lat_deg: 1.0,
            h: vec![h as f32; 16],
        };
        if hole {
            t.h[0] = f32::NAN; // (0.5, -89.5) 空洞
        }
        t
    }

    #[test]
    fn masked_sea_water_lake_land() {
        let mask = GeoMask::parse(&fixture()).unwrap();
        let src = dem_src(500.0, false);
        let m = MaskedSource::new(src, mask);
        // 行 0 陆地 [0,2)：Land(500)
        assert_eq!(m.sample_at(0.5, -89.5), Sample::Land(500.0));
        assert_eq!(m.sample_at(1.5, -89.5), Sample::Land(500.0));
        // 行 0 无段区：Sea → Water（高度 0）
        assert_eq!(m.sample_at(2.5, -89.5), Sample::Water);
        // 行 1 湖泊 [1,3)：Lake(DEM 高度)
        assert_eq!(m.sample_at(1.5, -88.5), Sample::Lake(500.0));
        assert_eq!(m.sample_at(2.5, -88.5), Sample::Lake(500.0));
        // 行 1 无段区：Sea → Water
        assert_eq!(m.sample_at(0.5, -88.5), Sample::Water);
        // height_at 统一走 sample_at
        assert_eq!(m.height_at(2.5, -89.5), Some(0.0));
        assert_eq!(m.height_at(1.5, -89.5), Some(500.0));
    }

    #[test]
    fn masked_lake_dem_hole_conservative_nodata() {
        // DEM 空洞在湖泊区域 → 湖面高程未知 → 保守 NoData（不得按海平面 0）
        let mask = GeoMask::parse(&fixture()).unwrap();
        let src = dem_src(500.0, true); // (0.5,-89.5) 空洞——但那是陆地；湖泊 (1.5,-88.5) 无空洞
        let m = MaskedSource::new(src, mask);
        // 湖泊 DEM 正常 → Lake
        assert_eq!(m.sample_at(1.5, -88.5), Sample::Lake(500.0));
        // 陆地空洞 → NoData（委托 inner）
        assert_eq!(m.sample_at(0.5, -89.5), Sample::NoData);
        // 海洋区域 DEM 空洞无影响（Water 固定 0）
        assert_eq!(m.sample_at(2.5, -89.5), Sample::Water);
    }
}

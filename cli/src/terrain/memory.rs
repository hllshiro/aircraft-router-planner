//! 内存地形网格（迁移自 phase0 terrain.rs，Phase 0 B4/B6 验证）。
//!
//! - 经纬度网格：行 = 纬度方向（北向），列 = 经度方向（东向）；
//! - 双线性插值采样；空洞（NaN）或出界 → `None`（Phase 0 验证：空洞不遮挡语义，
//!   非保守侧风险由 Phase 1 空洞策略裁决处理——见 PHASE1_TODO S6）；
//! - `from_raw`：Float32 行优先二进制 + 文本元数据
//!   `rows cols origin_lon origin_lat cell_lon_deg cell_lat_deg`。

use std::path::Path;

use super::{GeoBounds, TerrainSource};

/// 规则地形网格（经纬度坐标系，行优先：行 = 纬向、列 = 经向）。
/// 高度值 `NaN` = 空洞/无效（如地形数据中的 NoData 标记）。
pub struct Terrain {
    pub rows: usize,
    pub cols: usize,
    pub origin_lon: f64,
    pub origin_lat: f64,
    pub cell_lon_deg: f64,
    pub cell_lat_deg: f64,
    pub h: Vec<f32>,
}

impl Terrain {
    /// 从预处理 raw 加载真实 DEM。
    /// `raw_path`：Float32 行优先二进制（空洞应为 NaN）；`meta_path`：文本
    /// `rows cols origin_lon origin_lat cell_lon_deg cell_lat_deg`。
    /// 返回 `(Terrain, 加载耗时秒, 内存 MiB)`。
    pub fn from_raw(raw_path: &Path, meta_path: &Path) -> std::io::Result<(Terrain, f64, f64)> {
        let t0 = std::time::Instant::now();
        let meta = std::fs::read_to_string(meta_path)?;
        let mut it = meta.split_whitespace();
        let rows: usize = it.next().unwrap().parse().unwrap();
        let cols: usize = it.next().unwrap().parse().unwrap();
        let origin_lon: f64 = it.next().unwrap().parse().unwrap();
        let origin_lat: f64 = it.next().unwrap().parse().unwrap();
        let cell_lon_deg: f64 = it.next().unwrap().parse().unwrap();
        let cell_lat_deg: f64 = it.next().unwrap().parse().unwrap();
        let raw = std::fs::read(raw_path)?;
        let mem_mib = raw.len() as f64 / (1024.0 * 1024.0);
        let mut h: Vec<f32> = Vec::with_capacity(rows * cols);
        h.extend(
            raw.chunks_exact(4)
                .map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])),
        );
        debug_assert_eq!(h.len(), rows * cols, "raw 大小与 meta 不匹配");
        let t_load = t0.elapsed().as_secs_f64();
        Ok((
            Terrain {
                rows,
                cols,
                origin_lon,
                origin_lat,
                cell_lon_deg,
                cell_lat_deg,
                h,
            },
            t_load,
            mem_mib,
        ))
    }

    /// 双线性插值采样高度（米）。经纬度（度）。
    /// 超出网格边界、或插值邻域含空洞（NaN）返回 `None`。
    pub fn height_at_ll(&self, lon: f64, lat: f64) -> Option<f64> {
        let fr = (lat - self.origin_lat) / self.cell_lat_deg;
        let fc = (lon - self.origin_lon) / self.cell_lon_deg;
        let r0 = fr.floor() as isize;
        let c0 = fc.floor() as isize;
        if r0 < 0 || c0 < 0 || r0 + 1 >= self.rows as isize || c0 + 1 >= self.cols as isize {
            return None;
        }
        let (r0, c0) = (r0 as usize, c0 as usize);
        let w_r = fr - r0 as f64; // 纬向插值权重
        let w_c = fc - c0 as f64; // 经向插值权重
        let h00 = self.h[r0 * self.cols + c0] as f64;
        let h01 = self.h[r0 * self.cols + c0 + 1] as f64;
        let h10 = self.h[(r0 + 1) * self.cols + c0] as f64;
        let h11 = self.h[(r0 + 1) * self.cols + c0 + 1] as f64;
        // 空洞语义：任一邻域像素为 NaN → 该点无效
        if h00.is_nan() || h01.is_nan() || h10.is_nan() || h11.is_nan() {
            return None;
        }
        let top = h00 + (h01 - h00) * w_c;
        let bot = h10 + (h11 - h10) * w_c;
        Some(top + (bot - top) * w_r)
    }

    /// 射线遮挡判断（经纬度射线，4.2.2 语义）：起点 `(olon, olat, oz)` 沿方向
    /// `(dlon, dlat, dz)` 长度 `len_deg`（经度单位），等距采样 `n` 个点；
    /// 任一采样点高度 ≤ 地形高度则视为被遮挡。空洞 → 该点不遮挡（Phase 0 语义）。
    pub fn ray_blocked_ll(
        &self,
        olon: f64,
        olat: f64,
        oz: f64,
        dlon: f64,
        dlat: f64,
        dz: f64,
        len_deg: f64,
        n: usize,
    ) -> bool {
        for i in 1..=n {
            let t = len_deg * (i as f64 / n as f64);
            let lon = olon + dlon * t;
            let lat = olat + dlat * t;
            let z = oz + dz * t;
            match self.height_at_ll(lon, lat) {
                Some(ht) if z <= ht => return true,
                None => return false, // 出界/空洞视为不遮挡（Phase 0 语义）
                _ => {}
            }
        }
        false
    }
}

impl TerrainSource for Terrain {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        self.height_at_ll(lon, lat)
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
            "grid {}x{} cell {:.6}deg x {:.6}deg (lon x lat)",
            self.rows, self.cols, self.cell_lon_deg, self.cell_lat_deg
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::terrain::memory::Terrain;

    /// 构造平坦小网格（全 1000m，除指定空洞）。
    fn flat_grid(rows: usize, cols: usize, origin_lon: f64, origin_lat: f64, cell: f64) -> Terrain {
        Terrain {
            rows,
            cols,
            origin_lon,
            origin_lat,
            cell_lon_deg: cell,
            cell_lat_deg: cell,
            h: vec![1000f32; rows * cols],
        }
    }

    #[test]
    fn height_interp_and_bounds() {
        let t = flat_grid(64, 64, 115.0, 39.0, 0.05);
        // 网格内（120.0, 40.0 附近）可采样
        assert!(t.height_at_ll(117.0, 40.0).is_some());
        // 边界外返回 None
        assert!(t.height_at_ll(113.0, 40.0).is_none());
        assert!(t.height_at_ll(117.0, 44.0).is_none());
    }

    #[test]
    fn nan_hole_returns_none() {
        let mut t = flat_grid(8, 8, 115.0, 39.0, 0.05);
        t.h[3 * 8 + 3] = f32::NAN; // 制造空洞
        // 空洞邻域 → None
        assert!(
            t.height_at_ll(115.0 + 3.5 * 0.05, 39.0 + 3.5 * 0.05)
                .is_none()
        );
        // 正常区域可采样
        assert!(
            t.height_at_ll(115.0 + 1.5 * 0.05, 39.0 + 1.5 * 0.05)
                .is_some()
        );
    }

    #[test]
    fn ray_high_clear_low_blocked() {
        let t = flat_grid(128, 128, 115.0, 39.0, 0.02);
        // 高飞（3000m，网格 1000m）：不遮挡
        assert!(!t.ray_blocked_ll(115.0, 39.0, 3000.0, 1.0, 1.0, 0.0, 0.5, 1000));
        // 低飞（500m，网格 1000m）：遮挡
        assert!(t.ray_blocked_ll(115.0, 39.0, 500.0, 1.0, 1.0, 0.0, 0.5, 1000));
    }

    #[test]
    fn trait_bounds() {
        let t = flat_grid(64, 64, 115.0, 39.0, 0.05);
        let b = t.bounds().unwrap();
        assert!((b.max_lon - 118.2).abs() < 1e-9);
        assert!((b.max_lat - 42.2).abs() < 1e-9);
        assert!(b.contains(116.0, 40.0));
    }
}

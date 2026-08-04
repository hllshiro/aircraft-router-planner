//! GeoTIFF 数据源（技术方案 4.3：运行时用 `geotiff` crate，纯 Rust）。
//!
//! 两级策略（5.1）：小范围（少于阈值瓦片）全量解压为内存网格；大范围走
//! 瓦片 LRU + Overview——Phase 1 先实现全量加载（内存网格），LRU 留 Phase 2
//! 性能优化。`model_extent()` 提供地理边界；无地理参考的 tif 拒绝（Data 错误）。

use std::path::Path;

use geo_types::Coord;

use super::{GeoBounds, TerrainSource};
use crate::error::AppError;

/// GeoTIFF 源（全量加载为内存网格，行优先：行 = 纬向、列 = 经向）。
pub struct GeoTiffSource {
    tiff: geotiff::GeoTiff,
    /// model space 边界（x=lon, y=lat 时）
    bounds: GeoBounds,
    /// 每像素经纬度步长
    cell_lon_deg: f64,
    cell_lat_deg: f64,
}

impl GeoTiffSource {
    pub fn open(path: &Path) -> Result<Self, AppError> {
        let f = std::fs::File::open(path)?;
        let tiff = geotiff::GeoTiff::read(f)
            .map_err(|e| AppError::Data(format!("geotiff read failed: {e}")))?;
        if tiff.num_samples == 0 {
            return Err(AppError::Data("geotiff has zero samples".into()));
        }
        let ext = tiff.model_extent();
        let (min_x, min_y, max_x, max_y) = (ext.min().x, ext.min().y, ext.max().x, ext.max().y);
        // 无地理参考（extent 即像素范围 0..w, 0..h）→ 拒绝
        if min_x.abs() < 1e-9 && min_y.abs() < 1e-9 && (max_x - tiff.raster_width as f64).abs() < 1e-9
        {
            return Err(AppError::Data(
                "geotiff has no georeferencing (model extent == pixel range)".into(),
            ));
        }
        let cell_lon_deg = (max_x - min_x) / tiff.raster_width as f64;
        let cell_lat_deg = (max_y - min_y) / tiff.raster_height as f64;
        if cell_lon_deg <= 0.0 || cell_lat_deg <= 0.0 {
            return Err(AppError::Data("geotiff degenerate cell size".into()));
        }
        Ok(Self {
            tiff,
            bounds: GeoBounds {
                min_lon: min_x,
                min_lat: min_y,
                max_lon: max_x,
                max_lat: max_y,
            },
            cell_lon_deg,
            cell_lat_deg,
        })
    }

    /// 双线性插值采样（model space 坐标，经度 x / 纬度 y）。
    /// 四角最近邻查询（`get_value_at` 内部按像素取整）→ 双线性；NaN/非有限 → None。
    fn sample(&self, lon: f64, lat: f64) -> Option<f64> {
        let t = &self.tiff;
        let fc = (lon - self.bounds.min_lon) / self.cell_lon_deg;
        let fr = (lat - self.bounds.min_lat) / self.cell_lat_deg;
        let c0 = fc.floor() as isize;
        let r0 = fr.floor() as isize;
        if c0 < 0 || r0 < 0 || c0 + 1 >= t.raster_width as isize || r0 + 1 >= t.raster_height as isize
        {
            return None;
        }
        let w_c = fc - c0 as f64;
        let w_r = fr - r0 as f64;
        let lon0 = self.bounds.min_lon + c0 as f64 * self.cell_lon_deg;
        let lon1 = lon0 + self.cell_lon_deg;
        let lat0 = self.bounds.min_lat + r0 as f64 * self.cell_lat_deg;
        let lat1 = lat0 + self.cell_lat_deg;
        let h00 = t.get_value_at::<f64>(&Coord { x: lon0, y: lat0 }, 0)?;
        let h01 = t.get_value_at::<f64>(&Coord { x: lon1, y: lat0 }, 0)?;
        let h10 = t.get_value_at::<f64>(&Coord { x: lon0, y: lat1 }, 0)?;
        let h11 = t.get_value_at::<f64>(&Coord { x: lon1, y: lat1 }, 0)?;
        // NoData 语义：任一角 NaN/非有限 → 该点无效
        if !h00.is_finite() || !h01.is_finite() || !h10.is_finite() || !h11.is_finite() {
            return None;
        }
        let top = h00 + (h01 - h00) * w_c;
        let bot = h10 + (h11 - h10) * w_c;
        Some(top + (bot - top) * w_r)
    }
}

impl TerrainSource for GeoTiffSource {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        self.sample(lon, lat)
    }

    fn bounds(&self) -> Option<GeoBounds> {
        Some(self.bounds)
    }

    fn resolution_desc(&self) -> String {
        format!(
            "geotiff {}x{} cell {:.6}deg x {:.6}deg",
            self.tiff.raster_width, self.tiff.raster_height, self.cell_lon_deg, self.cell_lat_deg
        )
    }
}

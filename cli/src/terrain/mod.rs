//! 地形数据源（技术方案 4.3 TerrainSource trait）。
//!
//! 实现：SRTM/HGT（自研 .hgt）、GeoTIFF（geotiff）、DTED/DTED2（dted2）、
//! 自建紧凑格式（16-bit 分块差分 + ruzstd，S9）。
//! 几何口径（4.2.3）：一切距离计算在本地等距投影平面；`height_at` 接口以
//! 经纬度（度）为源级查询键，投影平面射线经反投影后逐点查询（调用方负责）。

pub mod builtin;
pub mod dted;
pub mod geotiff;
pub mod memory;
pub mod srtm;

use std::path::Path;

use crate::error::AppError;

/// 经纬度矩形覆盖（度）。用于缺瓦/出界判定（4.2.5：缺数/极区按 no_solution）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct GeoBounds {
    pub min_lon: f64,
    pub min_lat: f64,
    pub max_lon: f64,
    pub max_lat: f64,
}

impl GeoBounds {
    pub fn contains(&self, lon: f64, lat: f64) -> bool {
        lon >= self.min_lon && lon <= self.max_lon && lat >= self.min_lat && lat <= self.max_lat
    }
}

/// 地形数据源 trait（4.3）：源级高度查询 + 覆盖范围 + 分辨率语义。
pub trait TerrainSource: Send + Sync {
    /// 源级高度查询（经纬度，度）。返回值 = 椭球高（归一化后，4.2.2）。
    /// 空洞（NaN/NoData）或出界 → `None`（调用方按策略处理：邻近插值/告警）。
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64>;

    /// 实际覆盖范围（无边界声明 → None，此时出界判定依赖实现内部）。
    fn bounds(&self) -> Option<GeoBounds>;

    /// 分辨率语义描述（等经纬度弧秒 / 地面等距米）。
    fn resolution_desc(&self) -> String;
}

/// 按输入路径选择 reader（4.3：按扩展名/魔数自动选择）。
/// - `.hgt` → SRTM/HGT
/// - `.tif` / `.tiff` → GeoTIFF
/// - `.dt0` / `.dt1` / `.dt2` → DTED/DTED2
/// - `.zstd` / `.arpack`（自建）→ 内置紧凑格式
pub fn open_source(path: &Path) -> Result<Box<dyn TerrainSource>, AppError> {
    let ext = path
        .extension()
        .and_then(|e| e.to_str())
        .unwrap_or("")
        .to_ascii_lowercase();
    match ext.as_str() {
        "hgt" => Ok(Box::new(srtm::SrtmSource::open(path)?)),
        "tif" | "tiff" => Ok(Box::new(geotiff::GeoTiffSource::open(path)?)),
        "dt0" | "dt1" | "dt2" => Ok(Box::new(dted::DtedSource::open(path)?)),
        "zstd" | "arpack" => Ok(Box::new(builtin::BuiltinSource::open(path)?)),
        other => Err(AppError::Data(format!(
            "unsupported terrain file extension: .{other} (expect .hgt/.tif/.dt0-2/.zstd)"
        ))),
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn bounds_contains() {
        let b = GeoBounds {
            min_lon: 115.0,
            min_lat: 39.0,
            max_lon: 117.0,
            max_lat: 41.0,
        };
        assert!(b.contains(116.0, 40.0));
        assert!(!b.contains(118.0, 40.0));
        assert!(!b.contains(116.0, 42.0));
    }
}

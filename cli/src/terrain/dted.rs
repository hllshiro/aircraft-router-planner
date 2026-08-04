//! DTED/DTED2 数据源（技术方案 4.3：运行时用 `dted2` crate，纯 Rust）。
//!
//! DTED 为无压缩裸高度数据，读取性能接近 SRTM（5.1）；`dted2` 的
//! `get_elevation(lat, lon)` 直接匹配 TerrainSource 接口。

use std::path::Path;

use super::{GeoBounds, TerrainSource};
use crate::error::AppError;

/// DTED 源（内存常驻：dted2::DTEDData 全量持有）。
pub struct DtedSource {
    dted: dted2::DTEDData,
}

impl DtedSource {
    pub fn open(path: &Path) -> Result<Self, AppError> {
        let p = path
            .to_str()
            .ok_or_else(|| AppError::Data("non-UTF8 dted path".into()))?;
        let dted = dted2::DTEDData::read(p)
            .map_err(|e| AppError::Data(format!("dted2 read failed: {e:?}")))?;
        Ok(Self { dted })
    }
}

impl TerrainSource for DtedSource {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        // dted2 签名：get_elevation(lat, lon)
        self.dted.get_elevation(lat, lon)
    }

    fn bounds(&self) -> Option<GeoBounds> {
        let m = &self.dted.metadata;
        Some(GeoBounds {
            min_lon: m.origin.lon,
            min_lat: m.origin.lat,
            max_lon: m.origin.lon + m.count.lon as f64 * m.interval.lon,
            max_lat: m.origin.lat + m.count.lat as f64 * m.interval.lat,
        })
    }

    fn resolution_desc(&self) -> String {
        let m = &self.dted.metadata;
        format!(
            "dted2 {}x{} interval {:.3}deg x {:.3}deg",
            m.count.lon, m.count.lat, m.interval.lon, m.interval.lat
        )
    }
}

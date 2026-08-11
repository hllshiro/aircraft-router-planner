//! 地形数据源（技术方案 4.3 TerrainSource trait）。
//!
//! 实现：SRTM/HGT（自研 .hgt）、GeoTIFF（geotiff）、DTED/DTED2（dted2）、
//! 自建紧凑格式（16-bit 分块差分 + ruzstd，S9）。
//! 几何口径（4.2.3）：一切距离计算在本地等距投影平面；`height_at` 接口以
//! 经纬度（度）为源级查询键，投影平面射线经反投影后逐点查询（调用方负责）。

pub mod builtin;
pub mod convert;
pub mod dted;
pub mod geotiff;
pub mod mask;
pub mod memory;
pub mod srtm;
pub mod tiledir;

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

/// 地表类别（空洞分层策略，主管 2026-08-04 拍板：WATER/NODATA/OOB 三分类 + 湖泊）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum SurfaceClass {
    /// 陆地：正常 DEM 高程，地形遮挡由高度判定。
    Land,
    /// 海洋：高程固定 0（海平面），低代价，LOS 不遮挡。
    Water,
    /// 内陆湖：湖面高程（由 DEM 提供，一般高于海平面），低代价，LOS 不遮挡。
    Lake,
    /// 真缺失（NODATA）：高代价通行（初值 5x，主管拍板），LOS 不确定区间保守端。
    NoData,
    /// 超覆盖范围（OOB）：禁行墙，路径不得越出。
    OutOfBounds,
}

/// 地形采样结果（4.2.5 空洞分层：语义 + 可选高度）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Sample {
    /// 陆地，携带高度（米）。
    Land(f64),
    /// 海洋，高度 0（海平面）。
    Water,
    /// 内陆湖，携带湖面高度（米，由 DEM 提供）。
    Lake(f64),
    /// 真缺失（NODATA）。
    NoData,
    /// 越界（OOB）。
    OutOfBounds,
}

impl Sample {
    pub fn class(&self) -> SurfaceClass {
        match self {
            Sample::Land(_) => SurfaceClass::Land,
            Sample::Water => SurfaceClass::Water,
            Sample::Lake(_) => SurfaceClass::Lake,
            Sample::NoData => SurfaceClass::NoData,
            Sample::OutOfBounds => SurfaceClass::OutOfBounds,
        }
    }

    /// 有效高度（米）。Land/Lake 返回其高度；Water 返回 0（海平面）；NoData/OOB → None。
    pub fn height(&self) -> Option<f64> {
        match self {
            Sample::Land(h) | Sample::Lake(h) => Some(*h),
            Sample::Water => Some(0.0),
            Sample::NoData | Sample::OutOfBounds => None,
        }
    }

    /// 是否为水面（海洋/内陆湖）。
    pub fn is_water(&self) -> bool {
        matches!(self, Sample::Water | Sample::Lake(_))
    }

    /// 代价场基础代价（含 NODATA 5x 初值；OOB = 禁行 f32::INFINITY）。
    /// 地形高度本身的代价（越山爬升）由调用方按高度叠加。
    pub fn base_cost(&self, nodata_mult: f32) -> f32 {
        match self {
            Sample::Land(_) | Sample::Water | Sample::Lake(_) => 1.0,
            Sample::NoData => nodata_mult.max(1.0),
            Sample::OutOfBounds => f32::INFINITY,
        }
    }

    /// LOS 遮挡语义（空洞分层主管拍板）：
    /// - Land：地形遮挡由高度判定（`z <= height`），本方法不返回结论；
    /// - Water/Lake：水面不遮挡（无地形遮蔽）；
    /// - NoData：不确定区间保守端 → 视为不遮挡（探测概率高估 → 威胁高估 → 路径避开，
    ///   与高代价通行方向一致；修正 Phase 0"空洞不遮挡=非保守"的低估问题）；
    /// - OutOfBounds：禁行墙 → 视为遮挡（不可通过）。
    pub fn los_unblocked(&self) -> bool {
        match self {
            Sample::Land(_) | Sample::Water | Sample::Lake(_) => true,
            Sample::NoData => true,      // 保守端：不遮挡（威胁高估）
            Sample::OutOfBounds => false, // 禁行
        }
    }
}

/// 地形数据源 trait（4.3）：源级高度查询 + 语义采样 + 覆盖范围 + 分辨率语义。
pub trait TerrainSource: Send + Sync {
    /// 源级高度查询（经纬度，度）。返回值 = 椭球高（归一化后，4.2.2）。
    /// 空洞（NaN/NoData）或出界 → `None`（调用方按策略处理：邻近插值/告警）。
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64>;

    /// 语义采样（空洞分层入口）。默认实现：无掩膜源按 bounds 区分 OOB 与 NoData。
    fn sample_at(&self, lon: f64, lat: f64) -> Sample {
        if let Some(b) = self.bounds() {
            if !b.contains(lon, lat) {
                return Sample::OutOfBounds;
            }
        }
        match self.height_at(lon, lat) {
            Some(h) => Sample::Land(h),
            None => Sample::NoData,
        }
    }

    /// 实际覆盖范围（无边界声明 → None，此时出界判定依赖实现内部）。
    fn bounds(&self) -> Option<GeoBounds>;

    /// 分辨率语义描述（等经纬度弧秒 / 地面等距米）。
    fn resolution_desc(&self) -> String;
}

/// Box 透传委托（solver 外部格式直读路径，2026-08-11 主管：不需要转换）。
impl TerrainSource for Box<dyn TerrainSource> {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        (**self).height_at(lon, lat)
    }
    fn sample_at(&self, lon: f64, lat: f64) -> Sample {
        (**self).sample_at(lon, lat)
    }
    fn bounds(&self) -> Option<GeoBounds> {
        (**self).bounds()
    }
    fn resolution_desc(&self) -> String {
        (**self).resolution_desc()
    }
}

/// 无锁批量预取采样优化（候选，2026-08-07 对比验证）。
/// 可选实现：`BuiltinSource`（块级预取 + 无锁查表）与 `MaskedSource`（转发 inner）；
/// 其余源不实现。field_build 层可用 `prefetch_lonlat` 一次性锁外解压区域块，
/// 再 `sample_local` 无锁采样——消除 4M 次 Mutex 锁竞争（并行化验证失败的主因）。
///
/// 语义约束：`sample_local` 必须与 `sample_at` 完全一致（OOB/NoData/Land/Water/Lake），
/// 否则对比测试 bit-exact 断言会失败。
pub trait BulkPrefetch: TerrainSource {
    /// 预取经纬度矩形覆盖的全部块（锁外解压，返回局部块表；配合 `sample_local` 无锁访问）。
    /// 未实现/越界 → 空表（调用方回退带锁 `sample_at` 语义不变）。
    fn prefetch_lonlat(
        &self,
        min_lon: f64,
        min_lat: f64,
        max_lon: f64,
        max_lat: f64,
    ) -> std::collections::HashMap<usize, Vec<i16>>;

    /// 无锁采样（与 `sample_at` 同语义；local 未命中回退带锁路径）。
    fn sample_local(
        &self,
        local: &std::collections::HashMap<usize, Vec<i16>>,
        lon: f64,
        lat: f64,
    ) -> Sample;
}

/// 按输入路径选择 reader（4.3：按扩展名/魔数自动选择）。
/// - 目录 → 片级目录源（DTED/SRTM，M4/M5；按格式优先级 GeoTIFF > DTED > SRTM）
/// - `.hgt` → SRTM/HGT
/// - `.tif` / `.tiff` → GeoTIFF
/// - `.dt0` / `.dt1` / `.dt2` → DTED/DTED2
/// - `.zstd` / `.arpack`（自建）→ 内置紧凑格式
pub fn open_source(path: &Path) -> Result<Box<dyn TerrainSource>, AppError> {
    if path.is_dir() {
        return open_dir_source(path);
    }
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

/// 目录输入 → 片级目录源。格式优先级（主管 2026-08-07 拍板）：GeoTIFF > DTED > SRTM。
/// GeoTIFF 目录暂不支持（单文件 GeoTIFF 已按需 tile/strip LRU）——目录含 .tif 时
/// 明确报错并指引传单文件，避免静默降级到低优先级格式。
fn open_dir_source(dir: &Path) -> Result<Box<dyn TerrainSource>, AppError> {
    let mut has_geotiff = false;
    let mut has_dted = false;
    let mut has_srtm = false;
    for entry in std::fs::read_dir(dir)? {
        let Ok(e) = entry else { continue };
        let ext = e
            .path()
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        match ext.as_str() {
            "tif" | "tiff" => has_geotiff = true,
            "dt0" | "dt1" | "dt2" => has_dted = true,
            "hgt" => has_srtm = true,
            _ => {}
        }
    }
    if has_geotiff {
        return Err(AppError::Data(format!(
            "{}: directory contains GeoTIFF file(s); GeoTIFF 目录暂不支持（单文件 GeoTIFF 已支持 tile/strip 按需读取），请直接传入 .tif/.tiff 文件",
            dir.display()
        )));
    }
    if has_dted {
        return dted::open_dir(dir).map(|s| Box::new(s) as Box<dyn TerrainSource>);
    }
    if has_srtm {
        return srtm::open_dir(dir).map(|s| Box::new(s) as Box<dyn TerrainSource>);
    }
    Err(AppError::Data(format!(
        "{}: no supported terrain files (.hgt/.tif/.dt0-2) found in directory",
        dir.display()
    )))
}

/// 语义化 LOS 遮挡判定（空洞分层，主管 2026-08-04 拍板）。
///
/// 起点 `(olon, olat, oz)` 沿方向 `(dlon, dlat, dz)` 长度 `len_deg`（经度单位），
/// 等距采样 `n` 点；任一点：
/// - `Land(h)` 且 `z <= h` → 地形遮挡（blocked）；
/// - `Water` / `Lake` → 水面不遮挡；
/// - `NoData` → 不确定区间保守端：不遮挡（探测概率高估 → 威胁高估 → 路径避开，
///   与 NODATA 高代价通行方向一致；修正 Phase 0"空洞不遮挡=非保守"的低估问题）；
/// - `OutOfBounds` → 禁行墙，遮挡（blocked）。
///
/// 与 Phase 0 原型 `Terrain::ray_blocked_ll` 的差别：原型空洞/出界一律视为不遮挡，
/// 本函数为正式语义（OOB 遮挡、NoData 保守端）。
pub fn los_blocked<T: TerrainSource + ?Sized>(
    src: &T,
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
        match src.sample_at(lon, lat) {
            Sample::OutOfBounds => return true,
            Sample::Land(h) if z <= h => return true,
            _ => {}
        }
    }
    false
}

/// 语义退化统计（stats.degradations 输入，主管 2026-08-04：最坏降级警告）。
/// 在源覆盖范围内均匀采样 `n×n` 网格，统计 NoData 与 OutOfBounds 占比。
/// 覆盖范围未知（bounds()==None）时返回 (NaN, NaN)（调用方跳过统计）。
/// 防御：`n == 0` → (0.0, 0.0)；非有限输入不 panic。
pub fn semantic_degradation_ratios<T: TerrainSource + ?Sized>(src: &T, n: usize) -> (f64, f64) {
    let Some(b) = src.bounds() else {
        return (f64::NAN, f64::NAN);
    };
    if n == 0 {
        return (0.0, 0.0);
    }
    let mut nodata = 0u64;
    let mut oob = 0u64;
    let total = (n * n) as f64;
    for i in 0..n {
        let t = (i as f64 + 0.5) / n as f64;
        let lat = b.min_lat + (b.max_lat - b.min_lat) * t;
        for j in 0..n {
            let u = (j as f64 + 0.5) / n as f64;
            let lon = b.min_lon + (b.max_lon - b.min_lon) * u;
            match src.sample_at(lon, lat) {
                Sample::NoData => nodata += 1,
                Sample::OutOfBounds => oob += 1,
                _ => {}
            }
        }
    }
    (nodata as f64 / total, oob as f64 / total)
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

    /// 无掩膜默认源：全 Land(h)，右下角空洞 → NoData。
    struct FlatSrc {
        h: f64,
        hole: bool,
    }
    impl TerrainSource for FlatSrc {
        fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
            if self.hole && lon > 0.5 && lat < -89.5 {
                None
            } else {
                Some(self.h)
            }
        }
        fn bounds(&self) -> Option<GeoBounds> {
            Some(GeoBounds {
                min_lon: -2.0,
                min_lat: -92.0,
                max_lon: 2.0,
                max_lat: -88.0,
            })
        }
        fn resolution_desc(&self) -> String {
            "flat".into()
        }
    }

    #[test]
    fn default_sample_at_distinguishes_oob_nodata() {
        let s = FlatSrc { h: 1000.0, hole: true };
        // 边界内有效 → Land
        assert_eq!(s.sample_at(0.0, -90.0), Sample::Land(1000.0));
        // 边界内空洞 → NoData（默认实现区分 OOB 与空洞）
        assert_eq!(s.sample_at(1.0, -90.0), Sample::NoData);
        // 边界外 → OutOfBounds
        assert_eq!(s.sample_at(3.0, -90.0), Sample::OutOfBounds);
        assert_eq!(s.sample_at(0.0, -87.0), Sample::OutOfBounds);
    }

    #[test]
    fn los_water_nodata_unblocked_oob_blocked() {
        // 低空射线（z=500 < 地形 1000）：陆地遮挡
        let s = FlatSrc { h: 1000.0, hole: false };
        assert!(los_blocked(&s, 0.0, -90.0, 500.0, 1.0, 0.0, 0.0, 1.0, 100));
        // 高空（z=2000 > 1000）：不遮挡
        assert!(!los_blocked(&s, 0.0, -90.0, 2000.0, 1.0, 0.0, 0.0, 1.0, 100));
        // 射线进入 OOB（终点 lon=3 出界）→ 遮挡
        assert!(los_blocked(&s, 0.0, -90.0, 2000.0, 1.0, 0.0, 0.0, 3.0, 100));
    }

    #[test]
    fn sample_base_cost_semantics() {
        // NODATA 5x 初值
        assert_eq!(Sample::NoData.base_cost(5.0), 5.0);
        assert_eq!(Sample::Land(10.0).base_cost(5.0), 1.0);
        assert_eq!(Sample::Water.base_cost(5.0), 1.0);
        assert_eq!(Sample::Lake(10.0).base_cost(5.0), 1.0);
        assert_eq!(Sample::OutOfBounds.base_cost(5.0), f32::INFINITY);
        // height 语义
        assert_eq!(Sample::Water.height(), Some(0.0));
        assert_eq!(Sample::NoData.height(), None);
        assert!(Sample::Water.is_water());
        assert!(Sample::Lake(5.0).is_water());
        assert!(!Sample::Land(5.0).is_water());
    }

    #[test]
    fn degradation_ratios_counts_nodata_oob() {
        // FlatSrc：空洞区域 lon>0.5 && lat<-89.5（bounds: lon -2..2, lat -92..-88）
        let s = FlatSrc { h: 1000.0, hole: true };
        let (nd, oob) = semantic_degradation_ratios(&s, 4);
        // 采样点 4×4：lat ∈ [-91.5, -88.5]，lon ∈ [-1.5, 1.5]
        // 空洞条件 lon>0.5 且 lat<-89.5：lon=1.5 行 lat=-91.5/-90.5 → 2 点 NoData
        assert_eq!(nd, 2.0 / 16.0);
        // bounds 内无 OOB（采样点都在 bounds 内）
        assert_eq!(oob, 0.0);
        // n=0 防御
        assert_eq!(semantic_degradation_ratios(&s, 0), (0.0, 0.0));
    }

    #[test]
    fn degradation_ratios_oob_outside_sampled_bounds() {
        // bounds 未知 → (NaN, NaN)
        struct NoBounds;
        impl TerrainSource for NoBounds {
            fn height_at(&self, _lon: f64, _lat: f64) -> Option<f64> {
                Some(0.0)
            }
            fn bounds(&self) -> Option<GeoBounds> {
                None
            }
            fn resolution_desc(&self) -> String {
                "none".into()
            }
        }
        let s = NoBounds;
        let (nd, oob) = semantic_degradation_ratios(&s, 4);
        assert!(nd.is_nan() && oob.is_nan());
    }

    /// 最小 UHL-only DTED 文件（目录分发只读 UHL 建索引，不读片数据）。
    fn write_uhl_only_dted(path: &std::path::Path, lon: u32, lat: u32) -> std::io::Result<()> {
        use std::io::Write;
        let uhl = format!(
            "UHL1{:03}0000E{:03}0000N030003000005{:<15}{:04}{:04}{:<25}",
            lon, lat, "", 121, 121, ""
        );
        debug_assert_eq!(uhl.len(), 80);
        let mut f = std::fs::File::create(path)?;
        f.write_all(uhl.as_bytes())
    }

    #[test]
    fn open_source_dir_dispatches_dted() {
        let dir = std::env::temp_dir().join("open_source_dted");
        std::fs::create_dir_all(&dir).unwrap();
        write_uhl_only_dted(&dir.join("N42E015.dt0"), 15, 42).unwrap();
        write_uhl_only_dted(&dir.join("N42E016.dt0"), 16, 42).unwrap();
        let s = open_source(&dir).unwrap();
        assert!(s.resolution_desc().starts_with("dted dir"), "desc={}", s.resolution_desc());
        let b = s.bounds().unwrap();
        assert!((b.min_lon - 15.0).abs() < 1e-9 && (b.max_lon - 17.0).abs() < 1e-9);
        drop(std::fs::remove_dir_all(&dir));
    }

    #[test]
    fn open_source_dir_geotiff_errors() {
        let dir = std::env::temp_dir().join("open_source_geotiff");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("a.tif"), b"not really a tif").unwrap();
        match open_source(&dir) {
            Err(e) => assert!(e.to_string().contains("GeoTIFF"), "err={e}"),
            Ok(_) => panic!("expected GeoTIFF dir error"),
        }
        drop(std::fs::remove_dir_all(&dir));
    }

    #[test]
    fn open_source_dir_empty_errors() {
        let dir = std::env::temp_dir().join("open_source_empty");
        std::fs::create_dir_all(&dir).unwrap();
        assert!(open_source(&dir).is_err());
        drop(std::fs::remove_dir_all(&dir));
    }
}

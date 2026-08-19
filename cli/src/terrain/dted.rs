//! DTED/DTED2 数据源（技术方案 4.3：运行时用 `dted2` crate，纯 Rust）。
//!
//! DTED 为无压缩裸高度数据，读取性能接近 SRTM（5.1）；`dted2` 的
//! `get_elevation(lat, lon)` 直接匹配 TerrainSource 接口。
//!
//! 目录形态（M4，2026-08-07 主管拍板：外部格式按需读取）：单片 ≤26MB，
//! 按需粒度 = **片**——扫描时 O(1) 只读 UHL（64B）建片索引，采样按经纬度
//! 定位片 → dted2 全量读片 + 片级 LRU（8 片 ≈ 208MB，见 `tiledir`）。

use std::path::Path;

use super::tiledir::{TileDirSource, TileMeta, TileOpener};
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

// ==================== 目录形态（M4：片级 LRU） ====================

/// 打开 DTED 目录源：扫描 `.dt0/.dt1/.dt2` 片，O(1) 只读 UHL 建片索引。
/// 无效文件（UHL 无法解析 / 非 DTED）跳过；无有效片 → `AppError::Data`。
pub fn open_dir(dir: &Path) -> Result<TileDirSource, AppError> {
    let mut tiles = Vec::new();
    let rd = std::fs::read_dir(dir)?;
    for entry in rd {
        let Ok(e) = entry else { continue };
        let p = e.path();
        if !p.is_file() {
            continue;
        }
        let ext = p
            .extension()
            .and_then(|s| s.to_str())
            .unwrap_or("")
            .to_ascii_lowercase();
        if !matches!(ext.as_str(), "dt0" | "dt1" | "dt2") {
            continue;
        }
        let Ok(meta) = parse_uhl_meta(&p) else {
            continue;
        };
        tiles.push(meta);
    }
    let opener: TileOpener = Box::new(|p| Ok(Box::new(DtedSource::open(p)?)));
    TileDirSource::new(tiles, opener, "dted")
}

/// 解析 DTED UHL（O(1)：只读文件前 64B），产出片元数据。
/// 布局与 dted2-1.0.0 `parsers::dted_uhl_parser` 完全一致（实测 test_data.dt2 验证）：
/// `UHL<level>` + lon(3+2+2+符号) + lat(3+2+2+符号) + lon_interval×4 + lat_interval×4
/// + accuracy×4 + 跳过15 + lon_count×4 + lat_count×4 + 跳过25；interval 单位 0.1 弧秒。
fn parse_uhl_meta(path: &Path) -> Result<TileMeta, AppError> {
    use std::io::Read;
    let mut f = std::fs::File::open(path)?;
    let mut buf = [0u8; 64];
    f.read_exact(&mut buf)?;
    if &buf[0..3] != b"UHL" || !buf[3].is_ascii_digit() {
        return Err(AppError::Data(format!(
            "{}: not a DTED UHL (bad recognition sentinel)",
            path.display()
        )));
    }
    let mut pos = 4usize;
    let lon = parse_angle(&buf, &mut pos)?;
    let lat = parse_angle(&buf, &mut pos)?;
    let lon_interval_x10 = parse_uint(&buf, pos, 4)?;
    let lat_interval_x10 = parse_uint(&buf, pos + 4, 4)?;
    // interval 单位 0.1 弧秒 → 度（dted2: interval_secs_x_10 / 36000.0）
    let cell_lon_deg = lon_interval_x10 as f64 / 36000.0;
    let cell_lat_deg = lat_interval_x10 as f64 / 36000.0;
    if cell_lon_deg <= 0.0 || cell_lat_deg <= 0.0 {
        return Err(AppError::Data(format!(
            "{}: degenerate DTED interval",
            path.display()
        )));
    }
    Ok(TileMeta {
        path: path.to_path_buf(),
        min_lon: lon,
        min_lat: lat,
        cell_lon_deg,
        cell_lat_deg,
    })
}

/// 解析 DDMMSS + 可选 N/S/E/W（dted2 布局：3 位度 + 2 位分 + 2 位秒）。
fn parse_angle(buf: &[u8], pos: &mut usize) -> Result<f64, AppError> {
    let deg = parse_uint(buf, *pos, 3)? as f64;
    let min = parse_uint(buf, *pos + 3, 2)? as f64;
    let sec = parse_uint(buf, *pos + 5, 2)? as f64;
    *pos += 7;
    let neg = match buf.get(*pos) {
        Some(b'S') | Some(b'W') => {
            *pos += 1;
            true
        }
        Some(b'N') | Some(b'E') => {
            *pos += 1;
            false
        }
        _ => false, // 无符号字符 → 正（与 dted2 `opt(sign).unwrap_or(false)` 一致）
    };
    let v = deg + min / 60.0 + sec / 3600.0;
    Ok(if neg { -v } else { v })
}

fn parse_uint(buf: &[u8], pos: usize, n: usize) -> Result<u32, AppError> {
    let mut v = 0u32;
    for &b in &buf[pos..pos + n] {
        if !b.is_ascii_digit() {
            return Err(AppError::Data("DTED UHL: non-digit field".into()));
        }
        v = v * 10 + (b - b'0') as u32;
    }
    Ok(v)
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::sync::Arc;

    /// 构造最小 DTED 文件（121×121，30 弧秒，覆盖 1°×1°；UHL 标签用 "UHL1"
    /// —— dted2-1.0.0 的 `tag(b"UHL1")` 硬编码，其余级别标签无法被其读取）。
    /// 布局：UHL(80) + DSI(648) + ACC(2700) + 121 条记录（每条
    /// 0xAA + 1 + be_u16 blk + be_u16 lon_count + be_u16 lat_count + 121×i16 + 4 checksum）。
    fn write_min_dted(path: &Path, lon: u32, lat: u32, val: i16) -> std::io::Result<()> {
        let mut buf: Vec<u8> = Vec::new();
        // UHL（与 dted2 解析布局一致）
        let uhl = format!(
            "UHL1{:03}0000{}{:03}0000{}030003000005{:<15}{:04}{:04}{:<25}",
            lon,
            if lon < 180 { "E" } else { "W" },
            lat,
            "N",
            "",
            121,
            121,
            ""
        );
        debug_assert_eq!(uhl.len(), 80);
        buf.extend_from_slice(uhl.as_bytes());
        buf.resize(80 + 648 + 2700, b' '); // DSI + ACC 占位
        // 数据记录
        for _ in 0..121 {
            buf.push(0xAA);
            buf.push(0x00);
            buf.extend_from_slice(&0u16.to_be_bytes());
            buf.extend_from_slice(&121u16.to_be_bytes());
            buf.extend_from_slice(&121u16.to_be_bytes());
            for _ in 0..121 {
                let x = if val < 0 {
                    0x8000u16 | (val.unsigned_abs() as u16)
                } else {
                    val as u16
                };
                buf.extend_from_slice(&x.to_be_bytes());
            }
            buf.extend_from_slice(&[0u8; 4]); // checksum
        }
        std::fs::write(path, buf)
    }

    #[test]
    fn uhl_parse_matches_expected() {
        let dir = std::env::temp_dir();
        let p = dir.join("uhltest.dt0");
        write_min_dted(&p, 15, 42, 100).unwrap();
        let m = parse_uhl_meta(&p).unwrap();
        assert_eq!(m.min_lon, 15.0);
        assert_eq!(m.min_lat, 42.0);
        // UHL interval "0300" = 300×0.1 弧秒 = 30 弧秒 = 300/36000 deg
        assert!((m.cell_lon_deg - 300.0 / 36000.0).abs() < 1e-12);
        assert!((m.cell_lat_deg - 300.0 / 36000.0).abs() < 1e-12);
        drop(std::fs::remove_file(&p));
    }

    #[test]
    fn dir_open_sample_and_cross_tile() {
        let dir = std::env::temp_dir().join("dted_dir_test");
        std::fs::create_dir_all(&dir).unwrap();
        write_min_dted(&dir.join("N42E015.dt0"), 15, 42, 100).unwrap();
        write_min_dted(&dir.join("N42E016.dt0"), 16, 42, 200).unwrap();
        // 无关文件应被跳过
        std::fs::write(dir.join("readme.txt"), "not a tile").unwrap();

        let s = open_dir(&dir).unwrap();
        assert_eq!(s.tile_count(), 2);
        let b = s.global_bounds();
        assert!((b.min_lon - 15.0).abs() < 1e-9 && (b.max_lon - 17.0).abs() < 1e-9);
        // 片内采样（中心 = 15.5, 42.5）
        assert_eq!(s.height_at(15.5, 42.5), Some(100.0));
        assert_eq!(s.height_at(16.5, 42.5), Some(200.0));
        // 跨片边界：16.0 属右片（floor 键），右片左缘值 = 200
        assert_eq!(s.height_at(16.0, 42.5), Some(200.0));
        // 出界 → None
        assert_eq!(s.height_at(14.5, 42.5), None);
        // 缓存命中（同片两次 → 同 Arc 实例）
        let a1 = s.tile(0).unwrap();
        let a2 = s.tile(0).unwrap();
        assert!(Arc::ptr_eq(&a1, &a2));
        drop(std::fs::remove_dir_all(&dir));
    }

    #[test]
    fn dir_empty_errors() {
        let dir = std::env::temp_dir().join("dted_dir_empty");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("junk.txt"), "x").unwrap();
        assert!(open_dir(&dir).is_err());
        drop(std::fs::remove_dir_all(&dir));
    }
}

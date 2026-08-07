//! SRTM/HGT 数据源（自研 .hgt reader，技术方案 4.3 / 3.2.1：运行时全纯 Rust）。
//!
//! SRTM HGT 格式：1°×1° 瓦片，行优先，每个采样 2 字节**大端有符号 i16**；
//! SRTM3 = 1201×1201（3 弧秒，赤道≈92m），SRTM1 = 3601×3601（1 弧秒，赤道≈30m）；
//! 空洞 = -32768。瓦片第一行 = 最高纬度（北行优先）。
//! 文件名形如 `N39E116.hgt`（起点在文件名的南/西角）。
//!
//! 目录形态（M5，2026-08-07 主管拍板：外部格式按需读取）：单片 ≤26MB，
//! 按需粒度 = **片**——扫描文件名 + 文件尺寸 O(1) 建片索引，采样按经纬度
//! 定位片 → 片全量解码 + 片级 LRU（8 片 ≈ 208MB，见 `tiledir`）。

use std::path::{Path, PathBuf};

use super::tiledir::{TileDirSource, TileMeta, TileOpener};
use super::{GeoBounds, TerrainSource};
use crate::error::AppError;

/// SRTM HGT 瓦片源。
pub struct SrtmSource {
    /// 瓦片西南角（度）
    origin_lon: f64,
    origin_lat: f64,
    /// 网格边长（像素）：1201 / 3601
    size: usize,
    cell_deg: f64,
    h: Vec<i16>,
}

impl SrtmSource {
    /// 打开 .hgt 文件。文件名必须含 N/S/E/W（如 N39E116.hgt）。
    pub fn open(path: &Path) -> Result<Self, AppError> {
        let fname = path
            .file_name()
            .and_then(|s| s.to_str())
            .ok_or_else(|| AppError::Data("invalid .hgt filename".into()))?;
        // 解析 N39E116 / S12W080 等
        let bytes = path
            .extension()
            .and_then(|e| e.to_str())
            .map(|e| e.to_ascii_lowercase())
            .unwrap_or_default();
        if bytes != "hgt" {
            return Err(AppError::Data(format!("not an .hgt file: {fname}")));
        }
        let (lat, lon) = parse_hgt_filename(fname)?;

        let data = std::fs::read(path)?;
        // 1201²×2 = 2884802；3601²×2 = 25934402
        let size = match data.len() {
            2_884_802 => 1201,
            25_934_402 => 3601,
            n => {
                return Err(AppError::Data(format!(
                    "unexpected .hgt size {n} bytes (expect 1201² or 3601² samples)"
                )))
            }
        };
        // 大端 i16 解码
        let mut h = Vec::with_capacity(size * size);
        for chunk in data.chunks_exact(2) {
            h.push(i16::from_be_bytes([chunk[0], chunk[1]]));
        }
        let cell_deg = 1.0 / (size - 1) as f64;
        Ok(Self {
            origin_lon: lon,
            origin_lat: lat,
            size,
            cell_deg,
            h,
        })
    }

    /// 双线性插值采样（空洞 -32768 → None）。
    fn sample(&self, lon: f64, lat: f64) -> Option<f64> {
        let fc = (lon - self.origin_lon) / self.cell_deg;
        let fr = (lat - self.origin_lat) / self.cell_deg;
        let c0 = fc.floor() as isize;
        let r0 = fr.floor() as isize;
        // 瓦片行 = 北行优先：row = size-1 - r（纬度越高 row 越小）
        if c0 < 0 || r0 < 0 || c0 + 1 >= self.size as isize || r0 + 1 >= self.size as isize {
            return None;
        }
        let (r0, c0) = (r0 as usize, c0 as usize);
        let row_top = self.size - 1 - r0; // 纬度 fr 对应的上侧行
        let w_r = fr - r0 as f64;
        let w_c = fc - c0 as f64;
        // 北行优先：lat 增大 → fr 增大 → row 减小
        let i00 = row_top * self.size + c0; // (lat_hi, lon_lo)
        let i01 = i00 + 1; // (lat_hi, lon_hi)
        let i10 = i00 - self.size; // (lat_lo, lon_lo)
        let i11 = i10 + 1; // (lat_lo, lon_hi)
        let v00 = self.h[i00] as f64;
        let v01 = self.h[i01] as f64;
        let v10 = self.h[i10] as f64;
        let v11 = self.h[i11] as f64;
        if v00 <= -32767.0 || v01 <= -32767.0 || v10 <= -32767.0 || v11 <= -32767.0 {
            return None;
        }
        let top = v00 + (v01 - v00) * w_c;
        let bot = v10 + (v11 - v10) * w_c;
        Some(top + (bot - top) * w_r)
    }
}

/// 解析 `N39E116.hgt` → (lat0, lon0)（西南角）。
fn parse_hgt_filename(fname: &str) -> Result<(f64, f64), AppError> {
    let stem = fname.trim_end_matches(".hgt").trim_end_matches(".HGT");
    let b = stem.as_bytes();
    if b.len() < 7 {
        return Err(AppError::Data(format!("bad .hgt name: {fname}")));
    }
    let (ns, we) = (b[0] as char, b[3] as char);
    if !matches!(ns, 'N' | 'n' | 'S' | 's') || !matches!(we, 'E' | 'e' | 'W' | 'w') {
        return Err(AppError::Data(format!("bad .hgt name: {fname}")));
    }
    let lat: f64 = stem[1..3].parse().map_err(|_| AppError::Data(format!("bad .hgt name: {fname}")))?;
    let lon: f64 = stem[4..7].parse().map_err(|_| AppError::Data(format!("bad .hgt name: {fname}")))?;
    let lat0 = if matches!(ns, 'N' | 'n') { lat } else { -lat };
    let lon0 = if matches!(we, 'E' | 'e') { lon } else { -lon };
    Ok((lat0, lon0))
}

// ==================== 目录形态（M5：片级 LRU） ====================

/// 打开 SRTM 目录源：扫描 `.hgt` 片，文件名 + 文件尺寸 O(1) 建片索引。
/// 无效文件（文件名无 N/S/E/W / 尺寸非 1201² 或 3601²）跳过；无有效片 → 错误。
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
        if ext != "hgt" {
            continue;
        }
        let fname = match p.file_name().and_then(|s| s.to_str()) {
            Some(f) => f.to_string(),
            None => continue,
        };
        let Ok((lat0, lon0)) = parse_hgt_filename(&fname) else { continue };
        // 尺寸 → 网格边长 → cell（与 SrtmSource::open 同一判定）
        let Ok(len) = std::fs::metadata(&p).map(|m| m.len()) else { continue };
        let size = match len {
            2_884_802 => 1201,
            25_934_402 => 3601,
            _ => continue,
        };
        let cell = 1.0 / (size - 1) as f64;
        tiles.push(TileMeta {
            path: PathBuf::from(&p),
            min_lon: lon0,
            min_lat: lat0,
            cell_lon_deg: cell,
            cell_lat_deg: cell,
        });
    }
    let opener: TileOpener = Box::new(|p| Ok(Box::new(SrtmSource::open(p)?)));
    TileDirSource::new(tiles, opener, "srtm")
}

impl TerrainSource for SrtmSource {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        self.sample(lon, lat)
    }

    fn bounds(&self) -> Option<GeoBounds> {
        Some(GeoBounds {
            min_lon: self.origin_lon,
            min_lat: self.origin_lat,
            max_lon: self.origin_lon + 1.0,
            max_lat: self.origin_lat + 1.0,
        })
    }

    fn resolution_desc(&self) -> String {
        format!(
            "srtm {}x{} cell {:.6}deg ({:.2}m equator)",
            self.size,
            self.size,
            self.cell_deg,
            self.cell_deg * 111_320.0
        )
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use std::io::Write;

    /// 生成最小 SRTM3 瓦片（1201×1201，全 500m，中央空洞 -32768）。
    fn write_min_hgt(path: &Path) -> std::io::Result<()> {
        let mut f = std::fs::File::create(path)?;
        let mut buf = Vec::with_capacity(1201 * 1201 * 2);
        for i in 0..1201 * 1201 {
            let v = if i == 1201 * 600 + 600 { -32768i16 } else { 500i16 };
            buf.extend_from_slice(&v.to_be_bytes());
        }
        f.write_all(&buf)
    }

    #[test]
    fn parse_filename() {
        assert_eq!(parse_hgt_filename("N39E116.hgt").unwrap(), (39.0, 116.0));
        assert_eq!(parse_hgt_filename("S12W080.hgt").unwrap(), (-12.0, -80.0));
        assert!(parse_hgt_filename("bad.hgt").is_err());
    }

    #[test]
    fn open_and_sample() {
        let dir = std::env::temp_dir();
        let p = dir.join("N39E116_test.hgt");
        write_min_hgt(&p).unwrap();
        let s = SrtmSource::open(&p).unwrap();
        assert_eq!(s.size, 1201);
        // 正常区域（远离中央空洞）可采样
        assert!(s.height_at(116.4, 39.4).is_some());
        // 空洞中心 = 行 600 列 600 → lon=116.5, lat=39.5（北行优先换算）→ None
        let hole = s.height_at(116.5, 39.5);
        assert!(hole.is_none(), "expected None at hole, got {hole:?}");
        // 出界 → None
        assert!(s.height_at(118.0, 39.5).is_none());
        assert!(s.height_at(116.5, 40.5).is_none());
        drop(std::fs::remove_file(&p));
    }

    /// 写全常数值 SRTM3 瓦片（1201×1201）。
    fn write_flat_hgt(path: &Path, val: i16) -> std::io::Result<()> {
        use std::io::Write;
        let mut f = std::fs::File::create(path)?;
        let mut buf = Vec::with_capacity(1201 * 1201 * 2);
        for _ in 0..1201 * 1201 {
            buf.extend_from_slice(&val.to_be_bytes());
        }
        f.write_all(&buf)
    }

    #[test]
    fn dir_open_sample_and_cross_tile() {
        let dir = std::env::temp_dir().join("srtm_dir_test");
        std::fs::create_dir_all(&dir).unwrap();
        write_flat_hgt(&dir.join("N42E015.hgt"), 100).unwrap();
        write_flat_hgt(&dir.join("N42E016.hgt"), 200).unwrap();
        // 无关文件应被跳过
        std::fs::write(dir.join("note.txt"), "not a tile").unwrap();

        let s = open_dir(&dir).unwrap();
        assert_eq!(s.tile_count(), 2);
        let b = s.global_bounds();
        assert!((b.min_lon - 15.0).abs() < 1e-9 && (b.max_lon - 17.0).abs() < 1e-9);
        // 片内采样
        assert_eq!(s.height_at(15.5, 42.5), Some(100.0));
        assert_eq!(s.height_at(16.5, 42.5), Some(200.0));
        // 跨片边界：16.0 属右片（floor 键），右片左缘值 = 200
        assert_eq!(s.height_at(16.0, 42.5), Some(200.0));
        // 出界 → None
        assert_eq!(s.height_at(14.5, 42.5), None);
        drop(std::fs::remove_dir_all(&dir));
    }

    #[test]
    fn dir_skips_bad_size_and_empty_errors() {
        // 目录只有非标准尺寸 .hgt → 无有效片 → 错误
        let dir = std::env::temp_dir().join("srtm_dir_bad");
        std::fs::create_dir_all(&dir).unwrap();
        std::fs::write(dir.join("N42E015.hgt"), vec![0u8; 100]).unwrap();
        assert!(open_dir(&dir).is_err());
        drop(std::fs::remove_dir_all(&dir));

        // 目录只有非 .hgt → 错误
        let dir2 = std::env::temp_dir().join("srtm_dir_empty");
        std::fs::create_dir_all(&dir2).unwrap();
        std::fs::write(dir2.join("junk.txt"), "x").unwrap();
        assert!(open_dir(&dir2).is_err());
        drop(std::fs::remove_dir_all(&dir2));
    }
}

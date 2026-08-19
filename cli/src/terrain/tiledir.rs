//! 片级目录源（DTED / SRTM 目录共用，2026-08-07 主管拍板：外部格式按需读取）。
//!
//! 单片（DTED/SRTM .hgt）本来就小（2.9–26MB），按需粒度 = **片**：
//! 目录扫描时只读片元数据（origin/尺寸）建索引（O(片数)），采样按经纬度 O(1)
//! 定位片 → 片全量读入（dted2/SRTM 解析）+ LRU 缓存（Arc，内存有界）。
//! 跨片边界采样：双线性 4 角跨片定位（边界格点换相邻片取原值）。

use std::collections::{HashMap, VecDeque};
use std::path::{Path, PathBuf};
use std::sync::{Arc, Mutex};

use super::{GeoBounds, TerrainSource};
use crate::error::AppError;

/// 片级 LRU 上限（SRTM1/DTED1 单片 ≈ 26MB → 8 片 ≈ 208MB）。
pub const TILE_CACHE_MAX: usize = 8;

/// 片元数据（1°×1° 片；locate 用 floor 键 O(1)）。
#[derive(Debug, Clone)]
pub struct TileMeta {
    pub path: PathBuf,
    pub min_lon: f64,
    pub min_lat: f64,
    pub cell_lon_deg: f64,
    pub cell_lat_deg: f64,
}

/// 片打开工厂（DTED/SRTM 各自实现）。
pub type TileOpener = Box<dyn Fn(&Path) -> Result<Box<dyn TerrainSource>, AppError> + Send + Sync>;

/// 片级目录源。
pub struct TileDirSource {
    tiles: Vec<TileMeta>,
    /// (floor(lon), floor(lat)) → 片索引（1°×1° 键）
    lookup: HashMap<(i32, i32), usize>,
    opener: TileOpener,
    kind: String,
    cache: Mutex<TileCache>,
}

struct TileCache {
    map: HashMap<usize, Arc<dyn TerrainSource>>,
    order: VecDeque<usize>,
    max_tiles: usize,
}

impl TileCache {
    fn with_max(max_tiles: usize) -> Self {
        Self {
            map: HashMap::new(),
            order: VecDeque::new(),
            max_tiles: max_tiles.max(1),
        }
    }
}

fn lock_cache(m: &Mutex<TileCache>) -> std::sync::MutexGuard<'_, TileCache> {
    m.lock().unwrap_or_else(|e| e.into_inner())
}

impl TileDirSource {
    /// 从片列表构建目录源（构建方负责扫描目录 + 解析片元数据）。
    pub fn new(mut tiles: Vec<TileMeta>, opener: TileOpener, kind: &str) -> Result<Self, AppError> {
        if tiles.is_empty() {
            return Err(AppError::Data(format!("{kind} directory: no tiles found")));
        }
        tiles.sort_by(|a, b| a.path.cmp(&b.path)); // 确定性（扫描顺序不定）
        let mut lookup = HashMap::with_capacity(tiles.len());
        for (i, t) in tiles.iter().enumerate() {
            lookup.insert((t.min_lon.floor() as i32, t.min_lat.floor() as i32), i);
        }
        Ok(Self {
            tiles,
            lookup,
            opener,
            kind: kind.to_string(),
            cache: Mutex::new(TileCache::with_max(TILE_CACHE_MAX)),
        })
    }

    /// 定位含 (lon, lat) 的片（O(1) floor 键；无片 → None）。
    pub fn locate(&self, lon: f64, lat: f64) -> Option<usize> {
        self.lookup
            .get(&(lon.floor() as i32, lat.floor() as i32))
            .copied()
    }

    /// 取片源（Arc；缓存命中 clone Arc；未命中锁外 open + 双检插入）。
    pub fn tile(&self, idx: usize) -> Result<Arc<dyn TerrainSource>, AppError> {
        {
            let mut cache = lock_cache(&self.cache);
            if let Some(src) = cache.map.get(&idx).cloned() {
                cache.order.retain(|&i| i != idx);
                cache.order.push_back(idx);
                return Ok(src);
            }
        }
        let src: Arc<dyn TerrainSource> = Arc::from((self.opener)(&self.tiles[idx].path)?);
        let mut cache = lock_cache(&self.cache);
        if !cache.map.contains_key(&idx) {
            if cache.map.len() >= cache.max_tiles {
                if let Some(old) = cache.order.pop_front() {
                    cache.map.remove(&old);
                }
            }
            cache.map.insert(idx, Arc::clone(&src));
            cache.order.push_back(idx);
        }
        Ok(src)
    }

    /// 采样（双线性；4 角跨片定位）。
    pub fn sample(&self, lon: f64, lat: f64) -> Option<f64> {
        let idx = self.locate(lon, lat)?;
        let meta = &self.tiles[idx];
        let fc = (lon - meta.min_lon) / meta.cell_lon_deg;
        let fr = (lat - meta.min_lat) / meta.cell_lat_deg;
        let c0 = fc.floor() as isize;
        let r0 = fr.floor() as isize;
        if c0 < 0 || r0 < 0 {
            return None;
        }
        let w_c = fc - c0 as f64;
        let w_r = fr - r0 as f64;
        let lon0 = meta.min_lon + c0 as f64 * meta.cell_lon_deg;
        let lat0 = meta.min_lat + r0 as f64 * meta.cell_lat_deg;
        let v00 = self.cell_at(lon0, lat0)?;
        let v01 = self.cell_at(lon0 + meta.cell_lon_deg, lat0)?;
        let v10 = self.cell_at(lon0, lat0 + meta.cell_lat_deg)?;
        let v11 = self.cell_at(lon0 + meta.cell_lon_deg, lat0 + meta.cell_lat_deg)?;
        let top = v00 + (v01 - v00) * w_c;
        let bot = v10 + (v11 - v10) * w_c;
        Some(top + (bot - top) * w_r)
    }

    /// 格点值（格点 = 某片内部角；右/上边界格点由 floor 键自然落相邻片）。
    pub fn cell_at(&self, lon: f64, lat: f64) -> Option<f64> {
        let j = self.locate(lon, lat)?;
        let src = self.tile(j).ok()?;
        src.height_at(lon, lat)
    }

    /// 片数（调试/描述）。
    pub fn tile_count(&self) -> usize {
        self.tiles.len()
    }

    /// 描述（resolution_desc）。
    pub fn kind(&self) -> &str {
        &self.kind
    }

    /// 全局边界（所有片并集）。1°×1° 片 → 每片 max = min+1.0。
    pub fn global_bounds(&self) -> GeoBounds {
        let mut min_lon = f64::INFINITY;
        let mut min_lat = f64::INFINITY;
        let mut max_lon = f64::NEG_INFINITY;
        let mut max_lat = f64::NEG_INFINITY;
        for t in &self.tiles {
            min_lon = min_lon.min(t.min_lon);
            min_lat = min_lat.min(t.min_lat);
            max_lon = max_lon.max(t.min_lon + 1.0);
            max_lat = max_lat.max(t.min_lat + 1.0);
        }
        GeoBounds {
            min_lon,
            min_lat,
            max_lon,
            max_lat,
        }
    }
}

impl TerrainSource for TileDirSource {
    fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
        self.sample(lon, lat)
    }

    fn bounds(&self) -> Option<GeoBounds> {
        Some(self.global_bounds())
    }

    fn resolution_desc(&self) -> String {
        format!("{} dir: {} tiles", self.kind, self.tiles.len())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// 伪片源（固定值网格，验证定位/跨片/缓存）。
    struct FakeTile {
        lon0: f64,
        lat0: f64,
        val: f64,
    }
    impl TerrainSource for FakeTile {
        fn height_at(&self, lon: f64, lat: f64) -> Option<f64> {
            if lon >= self.lon0
                && lon <= self.lon0 + 1.0
                && lat >= self.lat0
                && lat <= self.lat0 + 1.0
            {
                Some(self.val)
            } else {
                None
            }
        }
        fn bounds(&self) -> Option<GeoBounds> {
            Some(GeoBounds {
                min_lon: self.lon0,
                min_lat: self.lat0,
                max_lon: self.lon0 + 1.0,
                max_lat: self.lat0 + 1.0,
            })
        }
        fn resolution_desc(&self) -> String {
            "fake".into()
        }
    }

    fn build_dir() -> TileDirSource {
        let tiles = vec![
            TileMeta {
                path: PathBuf::from("t0"),
                min_lon: 116.0,
                min_lat: 39.0,
                cell_lon_deg: 1.0 / 1200.0,
                cell_lat_deg: 1.0 / 1200.0,
            },
            TileMeta {
                path: PathBuf::from("t1"),
                min_lon: 117.0,
                min_lat: 39.0,
                cell_lon_deg: 1.0 / 1200.0,
                cell_lat_deg: 1.0 / 1200.0,
            },
        ];
        let opener: TileOpener = Box::new(|p| {
            let (lon0, lat0, val) = match p.to_str().unwrap() {
                "t0" => (116.0, 39.0, 100.0),
                "t1" => (117.0, 39.0, 200.0),
                _ => unreachable!(),
            };
            Ok(Box::new(FakeTile { lon0, lat0, val }))
        });
        TileDirSource::new(tiles, opener, "test").unwrap()
    }

    #[test]
    fn locate_and_sample() {
        let d = build_dir();
        assert_eq!(d.locate(116.5, 39.5).unwrap(), 0);
        assert_eq!(d.locate(117.5, 39.5).unwrap(), 1);
        assert!(d.locate(118.5, 39.5).is_none());
        // 采样（片内）
        assert_eq!(d.sample(116.5, 39.5), Some(100.0));
        assert_eq!(d.sample(117.5, 39.5), Some(200.0));
    }

    #[test]
    fn cross_tile_boundary() {
        let d = build_dir();
        // 边界 117.0（右片左边界）——locate floor(117.0)=117 → 片 1
        assert_eq!(d.sample(117.0, 39.5), Some(200.0));
        // 边界 116.999（左片右边界内）→ 片 0（双线性 4 角含 117.0 格点 → 跨片取 200）
        // 片 0 值 100 常数 → 插值仍 100
        assert_eq!(d.sample(116.999, 39.5), Some(100.0));
    }

    #[test]
    fn tile_cache_lru() {
        let d = build_dir();
        let a = d.tile(0).unwrap();
        let b = d.tile(1).unwrap();
        assert_eq!(a.height_at(116.5, 39.5), Some(100.0));
        assert_eq!(b.height_at(117.5, 39.5), Some(200.0));
        // 再取片 0（Arc clone——同实例）
        let a2 = d.tile(0).unwrap();
        assert_eq!(a2.height_at(116.5, 39.5), Some(100.0));
        assert_eq!(d.cache.lock().unwrap().map.len(), 2);
    }
}

//! 空间索引（技术方案 4.3 / 5.1：rstar 加速雷达/禁飞区邻域查询，~30% 预算环节）。
//!
//! - `RadarIndex`：雷达（膨胀后球体投影圆）R-tree，范围查询 + 最近邻；
//! - `CircleNoFlyIndex`：圆形禁飞/限飞区 R-tree；多边形禁飞区走线性扫
//!   （config::zone_contains），Phase 1 场景规模线性可接受。
//!
//! 确定性（13 轮共识热路径）：查询结果按 (id) 排序输出，迭代序与插入序无关。

use rstar::{RTree, RTreeObject, AABB};
use rstar::PointDistance;

/// 雷达条目（投影圆：中心经纬度 + 膨胀后半径）。
#[derive(Debug, Clone)]
pub struct RadarEntry {
    pub id: String,
    pub lon: f64,
    pub lat: f64,
    /// 膨胀后探测半径（米）
    pub radius_m: f64,
}

impl RTreeObject for RadarEntry {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        // 半径 → 经度/纬度上界（保守矩形；精确距离在查询后过滤）
        let d_lon = self.radius_m / 111_320.0 / self.lat.to_radians().cos().max(1e-6);
        let d_lat = self.radius_m / 110_540.0;
        AABB::from_corners(
            [self.lon - d_lon, self.lat - d_lat],
            [self.lon + d_lon, self.lat + d_lat],
        )
    }
}

impl PointDistance for RadarEntry {
    fn distance_2(&self, point: &[f64; 2]) -> f64 {
        // 经纬度差平方（最近邻排序用，非精确距离）
        let d_lon = self.lon - point[0];
        let d_lat = self.lat - point[1];
        d_lon * d_lon + d_lat * d_lat
    }

    fn contains_point(&self, point: &[f64; 2]) -> bool {
        let _ = point;
        true
    }
}

/// 圆形区域条目（禁飞/限飞/障碍物圆）。
#[derive(Debug, Clone)]
pub struct CircleEntry {
    pub id: String,
    pub lon: f64,
    pub lat: f64,
    pub radius_m: f64,
}

impl RTreeObject for CircleEntry {
    type Envelope = AABB<[f64; 2]>;

    fn envelope(&self) -> Self::Envelope {
        let d_lon = self.radius_m / 111_320.0 / self.lat.to_radians().cos().max(1e-6);
        let d_lat = self.radius_m / 110_540.0;
        AABB::from_corners(
            [self.lon - d_lon, self.lat - d_lat],
            [self.lon + d_lon, self.lat + d_lat],
        )
    }
}

/// 雷达索引。
pub struct RadarIndex {
    tree: RTree<RadarEntry>,
}

impl RadarIndex {
    pub fn build(entries: Vec<RadarEntry>) -> Self {
        Self {
            tree: RTree::bulk_load(entries),
        }
    }

    /// 查询点附近（半径内）的所有雷达，按 id 排序（确定性）。
    /// `radius_m`：查询半径（米），先 R-tree 粗筛再精确球面距离过滤。
    /// 非有限输入（NaN/Inf）→ 空结果（不 panic）。
    pub fn within(&self, lon: f64, lat: f64, radius_m: f64) -> Vec<&RadarEntry> {
        if !lon.is_finite() || !lat.is_finite() || !radius_m.is_finite() {
            return Vec::new();
        }
        let d_lon = radius_m / 111_320.0 / lat.to_radians().cos().max(1e-6);
        let d_lat = radius_m / 110_540.0;
        let env = AABB::from_corners([lon - d_lon, lat - d_lat], [lon + d_lon, lat + d_lat]);
        let mut hits: Vec<&RadarEntry> = self
            .tree
            .locate_in_envelope_intersecting(&env)
            .filter(|r| haversine_m(r.lon, r.lat, lon, lat) <= radius_m)
            .collect();
        hits.sort_by(|a, b| a.id.cmp(&b.id));
        hits
    }

    /// 最近雷达（非有限输入 → None）。
    pub fn nearest(&self, lon: f64, lat: f64) -> Option<&RadarEntry> {
        if !lon.is_finite() || !lat.is_finite() {
            return None;
        }
        self.tree.nearest_neighbor(&[lon, lat])
    }

    pub fn len(&self) -> usize {
        self.tree.size()
    }
}

/// 圆形禁飞/限飞区索引。
pub struct CircleIndex {
    tree: RTree<CircleEntry>,
}

impl CircleIndex {
    pub fn build(entries: Vec<CircleEntry>) -> Self {
        Self {
            tree: RTree::bulk_load(entries),
        }
    }

    /// 查询点所在的所有圆（按 id 排序，确定性）。
    pub fn containing(&self, lon: f64, lat: f64) -> Vec<&CircleEntry> {
        let env = AABB::from_point([lon, lat]);
        let mut hits: Vec<&CircleEntry> = self
            .tree
            .locate_in_envelope_intersecting(&env)
            .filter(|e| haversine_m(e.lon, e.lat, lon, lat) <= e.radius_m)
            .collect();
        hits.sort_by(|a, b| a.id.cmp(&b.id));
        hits
    }

    pub fn len(&self) -> usize {
        self.tree.size()
    }
}

/// 球面距离（米，haversine；经纬度 IO 层够用，计算层用等距投影见 coord）。
pub fn haversine_m(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    const R: f64 = 6_371_000.0;
    let d_lat = (lat2 - lat1).to_radians();
    let d_lon = (lon2 - lon1).to_radians();
    let a = (d_lat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
    2.0 * R * a.sqrt().atan2((1.0 - a).sqrt())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn radars() -> Vec<RadarEntry> {
        vec![
            RadarEntry {
                id: "r1".into(),
                lon: 116.0,
                lat: 39.0,
                radius_m: 50_000.0,
            },
            RadarEntry {
                id: "r2".into(),
                lon: 116.5,
                lat: 39.5,
                radius_m: 80_000.0,
            },
            RadarEntry {
                id: "r3".into(),
                lon: 120.0,
                lat: 40.0,
                radius_m: 60_000.0,
            },
        ]
    }

    #[test]
    fn within_radius_filters() {
        let idx = RadarIndex::build(radars());
        // 116.3, 39.3：r1（~30km）与 r2（~30km）在内；r3（~340km）在外
        let hits = idx.within(116.3, 39.3, 60_000.0);
        let ids: Vec<&str> = hits.iter().map(|r| r.id.as_str()).collect();
        assert_eq!(ids, vec!["r1", "r2"]);
    }

    #[test]
    fn nearest_radar() {
        let idx = RadarIndex::build(radars());
        let n = idx.nearest(116.2, 39.2).unwrap();
        assert_eq!(n.id, "r1");
    }

    #[test]
    fn circle_index_containing() {
        let idx = CircleIndex::build(vec![CircleEntry {
            id: "nf1".into(),
            lon: 116.5,
            lat: 39.5,
            radius_m: 30_000.0,
        }]);
        assert_eq!(idx.containing(116.5, 39.5).len(), 1);
        assert_eq!(idx.containing(117.0, 40.0).len(), 0);
    }

    #[test]
    fn empty_index_no_panic() {
        let idx = RadarIndex::build(vec![]);
        assert_eq!(idx.len(), 0);
        assert!(idx.within(0.0, 0.0, 1000.0).is_empty());
        assert!(idx.nearest(0.0, 0.0).is_none());
        let ci = CircleIndex::build(vec![]);
        assert!(ci.containing(0.0, 0.0).is_empty());
    }

    #[test]
    fn haversine_sanity() {
        // 北京→上海 ~1067km
        let d = haversine_m(116.397, 39.909, 121.473, 31.230);
        assert!((d - 1_067_000.0).abs() < 5_000.0, "d={d}");
    }
}

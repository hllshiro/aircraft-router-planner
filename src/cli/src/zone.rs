use crate::config::{Radar, Zone, ZoneShape};
use crate::path::haversine_m;

/// 区域类型枚举
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ZoneType {
    NoFly,
    Restricted,
    Sphere,
}

/// 统一区域 trait
pub trait UnifiedZone: Send + Sync {
    /// 点是否在区域内（含高度判定）
    fn contains(&self, lon: f64, lat: f64, alt_m: f64) -> bool;
    /// 区域类型
    fn zone_type(&self) -> ZoneType;
    /// 是否可穿越（端点穿透→true）
    fn is_traversable(&self) -> bool;
    /// 设置可穿性
    fn set_traversable(&mut self, traversable: bool);
    /// 基础代价倍率（硬墙=INF，软约束=200.0）
    fn base_cost_multiplier(&self) -> f64;
    /// 区域 ID（用于警告信息）
    fn id(&self) -> &str;
    /// 矩形与区域是否相交（格子贴墙判定）
    fn rect_intersects(&self, rx0: f64, ry0: f64, rx1: f64, ry1: f64) -> bool;
    /// 返回原始 Zone 数据（Circle/Polygon 用；Sphere 返回 None）
    fn as_zone(&self) -> Option<&Zone> {
        None
    }
}

// ─── 圆形区域 ───

pub struct CircleZone {
    zone: Zone,
    center_lon: f64,
    center_lat: f64,
    radius_km: f64,
    traversable: bool,
}

impl CircleZone {
    pub fn new(zone: &Zone) -> Self {
        let ZoneShape::Circle { center, radius_km } = &zone.shape else {
            panic!("CircleZone::new called with non-circle zone");
        };
        Self {
            zone: zone.clone(),
            center_lon: center[0],
            center_lat: center[1],
            radius_km: *radius_km,
            traversable: false,
        }
    }
}

impl UnifiedZone for CircleZone {
    fn contains(&self, lon: f64, lat: f64, alt_m: f64) -> bool {
        let Ok(g) = crate::coord::Geo::new(lon, lat) else {
            return false;
        };
        let Ok(c) = crate::coord::Geo::new(self.center_lon, self.center_lat) else {
            return false;
        };
        if c.distance_m(&g) > self.radius_km * 1000.0 {
            return false;
        }
        match (self.zone.alt_min_m, self.zone.alt_max_m) {
            (Some(min), Some(max)) => alt_m >= min && alt_m <= max,
            _ => true,
        }
    }

    fn zone_type(&self) -> ZoneType {
        if self.zone.alt_min_m.is_some() || self.zone.alt_max_m.is_some() {
            ZoneType::Restricted
        } else {
            ZoneType::NoFly
        }
    }

    fn is_traversable(&self) -> bool {
        self.traversable
    }

    fn set_traversable(&mut self, traversable: bool) {
        self.traversable = traversable;
    }

    fn base_cost_multiplier(&self) -> f64 {
        if self.traversable { 200.0 } else { f64::INFINITY }
    }

    fn id(&self) -> &str {
        &self.zone.id
    }

    fn rect_intersects(&self, rx0: f64, ry0: f64, rx1: f64, ry1: f64) -> bool {
        let closest_lon = self.center_lon.clamp(rx0, rx1);
        let closest_lat = self.center_lat.clamp(ry0, ry1);
        let Ok(g) = crate::coord::Geo::new(closest_lon, closest_lat) else {
            return false;
        };
        let Ok(c) = crate::coord::Geo::new(self.center_lon, self.center_lat) else {
            return false;
        };
        g.distance_m(&c) <= self.radius_km * 1000.0
    }

    fn as_zone(&self) -> Option<&Zone> {
        Some(&self.zone)
    }
}

// ─── 多边形区域 ───

pub struct PolygonZone {
    zone: Zone,
    traversable: bool,
}

impl PolygonZone {
    pub fn new(zone: &Zone) -> Self {
        let ZoneShape::Polygon { .. } = &zone.shape else {
            panic!("PolygonZone::new called with non-polygon zone");
        };
        Self {
            zone: zone.clone(),
            traversable: false,
        }
    }
}

impl UnifiedZone for PolygonZone {
    fn contains(&self, lon: f64, lat: f64, alt_m: f64) -> bool {
        let Ok(g) = crate::coord::Geo::new(lon, lat) else {
            return false;
        };
        let vertices = match &self.zone.shape {
            ZoneShape::Polygon { vertices } => vertices,
            _ => return false,
        };
        if !crate::config::point_in_polygon(&g, vertices) {
            return false;
        }
        match (self.zone.alt_min_m, self.zone.alt_max_m) {
            (Some(min), Some(max)) => alt_m >= min && alt_m <= max,
            _ => true,
        }
    }

    fn zone_type(&self) -> ZoneType {
        if self.zone.alt_min_m.is_some() || self.zone.alt_max_m.is_some() {
            ZoneType::Restricted
        } else {
            ZoneType::NoFly
        }
    }

    fn is_traversable(&self) -> bool {
        self.traversable
    }

    fn set_traversable(&mut self, traversable: bool) {
        self.traversable = traversable;
    }

    fn base_cost_multiplier(&self) -> f64 {
        if self.traversable { 200.0 } else { f64::INFINITY }
    }

    fn id(&self) -> &str {
        &self.zone.id
    }

    fn rect_intersects(&self, rx0: f64, ry0: f64, rx1: f64, ry1: f64) -> bool {
        let vertices = match &self.zone.shape {
            ZoneShape::Polygon { vertices } => vertices,
            _ => return false,
        };
        crate::config::rect_intersects_polygon(rx0, ry0, rx1, ry1, vertices)
    }

    fn as_zone(&self) -> Option<&Zone> {
        Some(&self.zone)
    }
}

// ─── 球形区域（原 RadarUnified） ───

pub struct SphereZone {
    radar_id: String,
    center_lon: f64,
    center_lat: f64,
    effective_radius_m: f64,
    traversable: bool,
}

impl SphereZone {
    pub fn new(radar: &Radar, inflation: f64) -> Self {
        let inflated = radar.radius_km * 1000.0 * inflation;
        let capped = radar.radius_km * 1000.0 + 100_000.0;
        let effective_radius_m = inflated.min(capped);
        Self {
            radar_id: radar.id.clone(),
            center_lon: radar.lon,
            center_lat: radar.lat,
            effective_radius_m,
            traversable: false,
        }
    }
}

impl UnifiedZone for SphereZone {
    fn contains(&self, lon: f64, lat: f64, _alt_m: f64) -> bool {
        let d = haversine_m(self.center_lon, self.center_lat, lon, lat);
        d <= self.effective_radius_m
    }

    fn zone_type(&self) -> ZoneType {
        ZoneType::Sphere
    }

    fn is_traversable(&self) -> bool {
        self.traversable
    }

    fn set_traversable(&mut self, traversable: bool) {
        self.traversable = traversable;
    }

    fn base_cost_multiplier(&self) -> f64 {
        if self.traversable { 200.0 } else { f64::INFINITY }
    }

    fn id(&self) -> &str {
        &self.radar_id
    }

    fn rect_intersects(&self, rx0: f64, ry0: f64, rx1: f64, ry1: f64) -> bool {
        let closest_lon = self.center_lon.clamp(rx0, rx1);
        let closest_lat = self.center_lat.clamp(ry0, ry1);
        haversine_m(self.center_lon, self.center_lat, closest_lon, closest_lat) <= self.effective_radius_m
    }
}

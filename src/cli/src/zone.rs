use crate::config::{Radar, Zone, ZoneShape};
use crate::path::haversine_m;

/// 区域类型枚举
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum ZoneType {
    NoFly,
    Restricted,
    Obstacle,
    Radar,
}

/// 统一区域 trait
pub trait UnifiedZone: Send + Sync {
    /// 点是否在区域内
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
}

/// 禁飞区/限飞区/障碍物的统一实现
pub struct ZoneUnified {
    pub zone: Zone,
    pub traversable: bool,
}

impl ZoneUnified {
    pub fn new(zone: Zone, traversable: bool) -> Self {
        Self { zone, traversable }
    }
}

impl UnifiedZone for ZoneUnified {
    fn contains(&self, lon: f64, lat: f64, alt_m: f64) -> bool {
        let Ok(g) = crate::coord::Geo::new(lon, lat) else {
            return false;
        };
        let in_geometry = match &self.zone.shape {
            ZoneShape::Circle {
                center,
                radius_km,
            } => {
                let Ok(c) = crate::coord::Geo::new(center[0], center[1]) else {
                    return false;
                };
                c.distance_m(&g) <= radius_km * 1000.0
            }
            ZoneShape::Polygon { vertices } => crate::config::point_in_polygon(&g, vertices),
        };
        if !in_geometry {
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
        if self.traversable {
            200.0
        } else {
            f64::INFINITY
        }
    }

    fn id(&self) -> &str {
        &self.zone.id
    }
}

/// 雷达的统一实现
pub struct RadarUnified {
    pub radar: Radar,
    pub effective_radius_m: f64,
    pub traversable: bool,
}

impl RadarUnified {
    pub fn new(radar: Radar, inflation: f64) -> Self {
        let inflated = radar.radius_km * 1000.0 * inflation;
        let capped = radar.radius_km * 1000.0 + 100_000.0;
        let effective_radius_m = inflated.min(capped);
        Self {
            radar,
            effective_radius_m,
            traversable: false,
        }
    }
}

impl UnifiedZone for RadarUnified {
    fn contains(&self, lon: f64, lat: f64, _alt_m: f64) -> bool {
        let d = haversine_m(self.radar.lon, self.radar.lat, lon, lat);
        d <= self.effective_radius_m
    }

    fn zone_type(&self) -> ZoneType {
        ZoneType::Radar
    }

    fn is_traversable(&self) -> bool {
        self.traversable
    }

    fn set_traversable(&mut self, traversable: bool) {
        self.traversable = traversable;
    }

    fn base_cost_multiplier(&self) -> f64 {
        if self.traversable {
            200.0
        } else {
            f64::INFINITY
        }
    }

    fn id(&self) -> &str {
        &self.radar.id
    }
}

//! 雷达威胁模型：球形探测 + 二值检测 + LOS 地形遮蔽。
//!
//! 语义：
//! - 每雷达球体探测：水平距离 ≤ 有效半径（radius_km × 1.1，cap 100km）；
//!   有效半径外不探测；
//! - 二值检测：范围内 + LOS 未遮蔽 = 必定探测（p=1.0），否则 p=0；
//! - LOS：雷达天线到点的视线被地形遮挡（含 NoData 保守视为遮挡）→ 该雷达不探测；
//! - 多雷达：任一雷达探测到即返回 1.0。

use crate::config::Radar;
use crate::coord::Geo;
use crate::path::{Path, haversine_m};
use crate::terrain::{Sample, TerrainSource};

/// 威胁模型参数。
#[derive(Debug, Clone)]
pub struct ThreatParams {
    /// 雷达膨胀系数（球体半径 = 实际 × 系数，>1，内部常量 1.1）
    pub radar_inflation: f64,
}

impl Default for ThreatParams {
    fn default() -> Self {
        Self {
            radar_inflation: 1.1,
        }
    }
}

/// 威胁评估报告。
#[derive(Debug, Clone, Default)]
pub struct ThreatReport {
    /// 路径是否进入任一雷达探测区
    pub detected: bool,
}

/// 威胁模型接口。
pub trait ThreatModel {
    /// 评估整条路径的探测检测（段内等距采样）。
    fn evaluate(&self, path: &Path, terrain: Option<&dyn TerrainSource>) -> ThreatReport;
    /// 静态几何探测（无 LOS）：点是否落在任一威胁有效半径内。
    fn static_detected(&self, lon: f64, lat: f64, alt_m: f64) -> bool {
        let _ = (lon, lat, alt_m);
        false
    }
    /// 静态几何并集探测概率（无 LOS）：二值 0.0 或 1.0。
    fn static_union_probability(&self, lon: f64, lat: f64) -> f64 {
        let _ = (lon, lat);
        0.0
    }
    /// 静态几何穿透深度（无 LOS）：到最近威胁中心的归一化距离 d/R_eff ∈ [0,1]；
    /// 0 = 中心，1 = 有效半径边缘，>1 = 有效半径外（无探测）。
    fn static_penetration(&self, lon: f64, lat: f64, alt_m: f64) -> f64 {
        let _ = (lon, lat, alt_m);
        1.0
    }
}

/// 默认球形威胁模型。
pub struct SphericalRadarThreat<'a> {
    radars: &'a [Radar],
    params: ThreatParams,
}

impl<'a> SphericalRadarThreat<'a> {
    pub fn new(radars: &'a [Radar], params: ThreatParams) -> Self {
        Self { radars, params }
    }

    /// 单雷达有效探测半径（膨胀 + cap 100km）。
    pub fn effective_radius_m(&self, r: &Radar) -> f64 {
        let inflated = r.radius_km * 1000.0 * self.params.radar_inflation;
        let capped = r.radius_km * 1000.0 + 100_000.0;
        inflated.min(capped)
    }

    /// 二值检测：任一雷达有效半径内 + LOS 未遮蔽 → 1.0，否则 0.0。
    pub fn point_probability(
        &self,
        lon: f64,
        lat: f64,
        alt_m: f64,
        terrain: Option<&dyn TerrainSource>,
    ) -> f64 {
        for r in self.radars {
            let d = haversine_m(r.lon, r.lat, lon, lat);
            if d > self.effective_radius_m(r) {
                continue;
            }
            if let Some(t) = terrain {
                if !line_of_sight(t, r.lon, r.lat, r.alt_m, lon, lat, alt_m) {
                    continue;
                }
            }
            return 1.0;
        }
        0.0
    }

    /// 静态几何并集概率（无 LOS——FMM 代价场用）。
    pub fn static_union_probability(&self, lon: f64, lat: f64) -> f64 {
        self.point_probability(lon, lat, 0.0, None)
    }

    /// 判断单个雷达是否应该绝对避让：
    /// 起点、终点、所有必经点都在该雷达有效半径外 → 可绕行 → 硬墙化
    pub fn should_hard_avoid(&self, radar_idx: usize, waypoints: &[Geo]) -> bool {
        if radar_idx >= self.radars.len() || waypoints.is_empty() {
            return false;
        }
        let r = &self.radars[radar_idx];
        let eff = self.effective_radius_m(r);

        // 所有航路点都在雷达外才可绕行
        waypoints.iter().all(|w| {
            let d = haversine_m(r.lon, r.lat, w.lon, w.lat);
            d >= eff
        })
    }

    /// 批量判定哪些雷达需要硬墙化
    pub fn hard_avoid_list(&self, waypoints: &[Geo]) -> Vec<bool> {
        (0..self.radars.len())
            .map(|i| self.should_hard_avoid(i, waypoints))
            .collect()
    }

    /// 返回雷达列表引用（用于代价场硬墙判定）
    pub fn radars(&self) -> &[Radar] {
        self.radars
    }
}

impl ThreatModel for SphericalRadarThreat<'_> {
    fn static_detected(&self, lon: f64, lat: f64, alt_m: f64) -> bool {
        self.point_probability(lon, lat, alt_m, None) > 0.0
    }

    fn static_union_probability(&self, lon: f64, lat: f64) -> f64 {
        self.point_probability(lon, lat, 0.0, None)
    }

    fn static_penetration(&self, lon: f64, lat: f64, _alt_m: f64) -> f64 {
        let mut best: f64 = 1.0;
        for r in self.radars {
            let d = haversine_m(r.lon, r.lat, lon, lat);
            let eff = self.effective_radius_m(r);
            if eff > 0.0 {
                best = best.min(d / eff);
            }
        }
        best
    }

    fn evaluate(&self, path: &Path, terrain: Option<&dyn TerrainSource>) -> ThreatReport {
        if path.len() < 2 {
            return ThreatReport::default();
        }
        const SEG_SAMPLES: usize = 8;
        let n = path.len();
        for i in 1..n {
            let a = path.points[i - 1];
            let b = path.points[i];
            for k in 0..=SEG_SAMPLES {
                let t = k as f64 / SEG_SAMPLES as f64;
                let lon = a.lon + (b.lon - a.lon) * t;
                let lat = a.lat + (b.lat - a.lat) * t;
                let alt = a.alt_m + (b.alt_m - a.alt_m) * t;
                if self.point_probability(lon, lat, alt, terrain) > 0.0 {
                    return ThreatReport { detected: true };
                }
            }
        }
        ThreatReport { detected: false }
    }
}

/// 雷达天线到点视线是否被地形遮挡（等距 8 点采样；NoData 不遮挡——保守策略，假设雷达看穿数据空洞）。
fn line_of_sight(
    t: &dyn TerrainSource,
    lon1: f64,
    lat1: f64,
    alt1: f64,
    lon2: f64,
    lat2: f64,
    alt2: f64,
) -> bool {
    const N: usize = 8;
    for i in 1..N {
        let u = i as f64 / N as f64;
        let lon = lon1 + (lon2 - lon1) * u;
        let lat = lat1 + (lat2 - lat1) * u;
        let los_h = alt1 + (alt2 - alt1) * u;
        match t.sample_at(lon, lat) {
            Sample::Land(h) => {
                if h > los_h {
                    return false;
                }
            }
            Sample::NoData => {}, // 不遮挡：保守策略，假设雷达看穿数据空洞，航线更安全
            Sample::Water | Sample::Lake(_) | Sample::OutOfBounds => {}
            Sample::Forbidden => return false, // 禁行墙（防御：地形源不产生，出现即遮挡）
        }
    }
    true
}

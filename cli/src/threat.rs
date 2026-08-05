//! 雷达威胁模型（Phase 4 M3 基础版，主管拍板：几何距离 + LOS + 压制 + detection_probability）。
//!
//! 语义（技术方案 4.5 + 十轮共识）：
//! - 每雷达球体探测：水平距离 ≤ 有效半径（radius_km × radar_inflation，被压制时再缩）；
//!   有效半径外探测概率 = 0；
//! - 探测概率随距离衰减：`DetectionCurve::Linear` p(d) = base·(1−u)；
//!   `Exponential` p(d) = base·exp(−4u)，u = d/R ∈ [0,1]（R 处 ≈ 1.8%）；
//! - LOS：雷达天线到点的视线被地形遮挡（含 NoData 保守视为遮挡）→ 该雷达不探测；
//! - 压制：`suppression_post_range_km` 直接给定压制后距离；否则
//!   `suppression_factor` δ → R' = R·(1−δ)（默认参数表 suppression_delta 兜底）；
//! - 多雷达探测概率主口径 = 概率并集 1−∏(1−pᵢ)（主管裁决）；
//! - 全程/每段累计探测概率：段内等距采样逐点并集。
//!
//! base_p（探测概率基准）与 P_cross（穿越阈值）为占位值（0.1），
//! 待真实雷达参数标定（docs/phase0_baseline.md Open Work）。

use crate::config::{DetectionCurve, Radar};
use crate::path::{Path, haversine_m};
use crate::terrain::{Sample, TerrainSource};

/// 威胁模型参数（缺省落默认参数表占位；覆盖来自 ParamsOverride/DefaultParams）。
#[derive(Debug, Clone)]
pub struct ThreatParams {
    /// 雷达膨胀系数（球体半径 = 实际 × 系数，>1）
    pub radar_inflation: f64,
    /// 探测概率衰减形态
    pub detection_curve: DetectionCurve,
    /// 穿越阈值 P_cross（累计探测概率超过 → 复验不通过）
    pub p_cross: f64,
    /// 压制修正因子 δ（无显式压制字段时兜底）
    pub suppression_delta: f64,
    /// 探测概率基准（占位 0.1，真实雷达参数标定前不得声称实现）
    pub base_p: f64,
}

impl Default for ThreatParams {
    fn default() -> Self {
        Self {
            radar_inflation: 1.2,
            detection_curve: DetectionCurve::Exponential,
            p_cross: 0.1,
            suppression_delta: 0.5,
            base_p: 0.1,
        }
    }
}

/// 威胁评估报告。
#[derive(Debug, Clone, Default)]
pub struct ThreatReport {
    /// 全程累计探测概率（概率并集）
    pub cumulative_p: f64,
    /// 单采样点峰值概率
    pub peak_p: f64,
    /// 峰值位置（lon, lat, alt_m）
    pub peak_point: Option<(f64, f64, f64)>,
    /// 累计概率是否超过 P_cross 阈值
    pub over_threshold: bool,
}

/// 威胁模型接口（M3：球形基础版；后续可替换多普勒/波束形态）。
pub trait ThreatModel {
    /// 评估整条路径的探测概率（段内等距采样）。
    fn evaluate(&self, path: &Path, terrain: Option<&dyn TerrainSource>) -> ThreatReport;
    /// 静态几何探测（无 LOS）：点是否落在任一威胁有效半径内。
    /// Theta* 去锯齿段检查用：直连穿威胁区则拒绝拉直（保住绕行路径）。
    fn static_detected(&self, lon: f64, lat: f64, alt_m: f64) -> bool {
        let _ = (lon, lat, alt_m);
        false
    }
    /// 静态几何穿透深度（无 LOS）：到最近威胁中心的归一化距离 d/R_eff ∈ [0,1]；
    /// 0 = 中心，1 = 有效半径边缘，>1 = 有效半径外（无探测）。
    /// Theta* 去锯齿用：仅"深穿"（< deep_ratio，默认 0.7）拒绝拉直；
    /// 低概率边缘（≥0.7）允许拉直，绕行路径才能平滑。
    fn static_penetration(&self, lon: f64, lat: f64, alt_m: f64) -> f64 {
        let _ = (lon, lat, alt_m);
        1.0
    }
    /// 穿越阈值 P_cross（复验软告警线）。
    fn p_cross(&self) -> f64 {
        0.1
    }
}

/// 默认球形威胁模型（基础版）。
pub struct SphericalRadarThreat<'a> {
    radars: &'a [Radar],
    params: ThreatParams,
}

impl<'a> SphericalRadarThreat<'a> {
    pub fn new(radars: &'a [Radar], params: ThreatParams) -> Self {
        Self { radars, params }
    }

    /// 单雷达有效探测半径（膨胀 + 压制）。
    fn effective_radius_m(&self, r: &Radar) -> f64 {
        let base = r.radius_km * 1000.0 * self.params.radar_inflation;
        if let Some(post) = r.suppression_post_range_km {
            return post * 1000.0;
        }
        if let Some(delta) = r.suppression_factor {
            return base * (1.0 - delta);
        }
        base
    }

    /// 单点累计探测概率（多雷达概率并集）。
    pub fn point_probability(
        &self,
        lon: f64,
        lat: f64,
        alt_m: f64,
        terrain: Option<&dyn TerrainSource>,
    ) -> f64 {
        let mut p_union = 0.0;
        for r in self.radars {
            let d = haversine_m(r.lon, r.lat, lon, lat);
            let eff = self.effective_radius_m(r);
            if d > eff {
                continue;
            }
            // LOS：视线被地形遮挡（含 NoData 保守）→ 该雷达不探测
            if let Some(t) = terrain
                && !line_of_sight(t, r.lon, r.lat, r.alt_m, lon, lat, alt_m)
            {
                continue;
            }
            let u = (d / eff).clamp(0.0, 1.0);
            let p = match self.params.detection_curve {
                DetectionCurve::Linear => self.params.base_p * (1.0 - u),
                DetectionCurve::Exponential => self.params.base_p * (-4.0 * u).exp(),
            };
            p_union = 1.0 - (1.0 - p_union) * (1.0 - p);
        }
        p_union
    }

    /// 静态几何并集概率（无 LOS——FMM 代价场用）。
    pub fn static_union_probability(&self, lon: f64, lat: f64) -> f64 {
        self.point_probability(lon, lat, 0.0, None)
    }
}

impl ThreatModel for SphericalRadarThreat<'_> {
    fn p_cross(&self) -> f64 {
        self.params.p_cross
    }

    fn static_detected(&self, lon: f64, lat: f64, alt_m: f64) -> bool {
        self.point_probability(lon, lat, alt_m, None) > 0.0
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
        let mut rep = ThreatReport::default();
        if path.len() < 2 {
            return rep;
        }
        const SEG_SAMPLES: usize = 8;
        let n = path.len();
        for i in 1..n {
            let a = path.points[i - 1];
            let b = path.points[i];
            for k in 0..=SEG_SAMPLES {
                let t = k as f64 / SEG_SAMPLES as f64;
                let (lon, lat, alt) = (
                    a.lon + (b.lon - a.lon) * t,
                    a.lat + (b.lat - a.lat) * t,
                    a.alt_m + (b.alt_m - a.alt_m) * t,
                );
                let p = self.point_probability(lon, lat, alt, terrain);
                rep.cumulative_p = 1.0 - (1.0 - rep.cumulative_p) * (1.0 - p);
                if p > rep.peak_p {
                    rep.peak_p = p;
                    rep.peak_point = Some((lon, lat, alt));
                }
            }
        }
        rep.over_threshold = rep.cumulative_p > self.params.p_cross;
        rep
    }
}

/// 雷达天线到点视线是否被地形遮挡（等距 8 点采样；NoData 保守视为遮挡）。
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
            Sample::NoData => return false,
            Sample::Water | Sample::Lake(_) | Sample::OutOfBounds => {}
        }
    }
    true
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::path::PathPoint;

    fn radar(lon: f64, lat: f64, radius_km: f64) -> Radar {
        Radar {
            id: "r1".into(),
            lon,
            lat,
            radar_type: crate::config::RadarType::Tracking,
            radius_km,
            alt_m: 10.0,
            suppression_post_range_km: None,
            suppression_factor: None,
        }
    }

    #[test]
    fn point_probability_linear_decay() {
        let rs = [radar(0.0, 0.0, 100.0)];
        let t = SphericalRadarThreat::new(&rs, ThreatParams {
            detection_curve: DetectionCurve::Linear,
            radar_inflation: 1.0,
            ..Default::default()
        });
        // 距离 0 → p = base
        let p0 = t.point_probability(0.0, 0.0, 3000.0, None);
        assert!((p0 - 0.1).abs() < 1e-9);
        // 距离 = R → p = 0
        let pr = t.point_probability(0.9, 0.0, 3000.0, None);
        assert!(pr < 1e-9);
        // 中间距离 → 线性期望（用 haversine 实际距离）
        let lon = 0.45;
        let d = haversine_m(0.0, 0.0, lon, 0.0);
        let expect = 0.1 * (1.0 - d / 100_000.0);
        let pm = t.point_probability(lon, 0.0, 3000.0, None);
        assert!((pm - expect).abs() < 1e-6, "p at half range = {pm} vs {expect}");
    }

    #[test]
    fn point_probability_exponential_decay() {
        let rs = [radar(0.0, 0.0, 100.0)];
        let t = SphericalRadarThreat::new(&rs, ThreatParams {
            detection_curve: DetectionCurve::Exponential,
            radar_inflation: 1.0,
            ..Default::default()
        });
        let p0 = t.point_probability(0.0, 0.0, 3000.0, None);
        assert!((p0 - 0.1).abs() < 1e-9);
        let lon = 0.45;
        let d = haversine_m(0.0, 0.0, lon, 0.0);
        let expect = 0.1 * (-4.0 * d / 100_000.0).exp();
        let pm = t.point_probability(lon, 0.0, 3000.0, None);
        assert!((pm - expect).abs() < 1e-6, "p at half range = {pm} vs {expect}");
    }

    #[test]
    fn union_of_multiple_radars() {
        let rs = [
            radar(0.0, 0.0, 100.0),
            radar(0.0, 0.1, 100.0), // 同一点第二雷达（0.1°≈11km）
        ];
        let t = SphericalRadarThreat::new(&rs, ThreatParams {
            detection_curve: DetectionCurve::Linear,
            radar_inflation: 1.0,
            ..Default::default()
        });
        // 第二雷达在 (0, 0.1°≈11.1km)：p2 = 0.1×(1−11.1/100) ≈ 0.0889
        // 并集 = 1−(1−0.1)(1−0.0889) ≈ 0.18
        let p = t.point_probability(0.0, 0.0, 3000.0, None);
        let d2 = haversine_m(0.0, 0.0, 0.0, 0.1);
        let expect = 1.0 - (1.0 - 0.1) * (1.0 - 0.1 * (1.0 - d2 / 100_000.0));
        assert!((p - expect).abs() < 1e-6, "union p = {p} vs {expect}");
    }

    #[test]
    fn suppression_post_range_shrinks() {
        let mut r = radar(0.0, 0.0, 100.0);
        r.suppression_post_range_km = Some(50.0);
        let rs = [r];
        let t = SphericalRadarThreat::new(&rs, ThreatParams {
            detection_curve: DetectionCurve::Linear,
            radar_inflation: 1.0,
            ..Default::default()
        });
        // 0.45° ≈ 50km 边界外 → p≈0；0.3° ≈ 33.4km 内 → p ≈ 0.1×(1−0.67) ≈ 0.033
        let p_edge = t.point_probability(0.45, 0.0, 3000.0, None);
        assert!(p_edge < 1e-9);
        let p_in = t.point_probability(0.3, 0.0, 3000.0, None);
        assert!(p_in > 0.02, "p_in = {p_in}");
    }

    #[test]
    fn los_terrain_blocks_detection() {
        // 山脊（采样点 Land 高 5000m）挡在雷达与飞机之间 → 探测概率 0
        struct Hill;
        impl crate::terrain::TerrainSource for Hill {
            fn height_at(&self, _lon: f64, _lat: f64) -> Option<f64> {
                Some(if _lat.abs() < 0.01 && _lon > 0.1 && _lon < 0.9 { 5000.0 } else { 0.0 })
            }
            fn bounds(&self) -> Option<crate::terrain::GeoBounds> {
                None
            }
            fn resolution_desc(&self) -> String {
                "test hill".into()
            }
        }
        let rs = [radar(0.0, 0.0, 200.0)];
        let t = SphericalRadarThreat::new(&rs, ThreatParams {
            detection_curve: DetectionCurve::Linear,
            radar_inflation: 1.0,
            ..Default::default()
        });
        let p_blocked = t.point_probability(1.0, 0.0, 3000.0, Some(&Hill));
        assert!(p_blocked < 1e-9, "山脊应遮挡: {p_blocked}");
        // 无地形（平地）→ 可见（1°≈111km < 200km → p ≈ 0.1×(1−0.56) ≈ 0.044）
        let p_open = t.point_probability(1.0, 0.0, 3000.0, None);
        assert!(p_open > 0.03, "open p = {p_open}");
    }

    #[test]
    fn evaluate_path_reports_union_and_threshold() {
        let rs = [radar(0.5, 0.0, 100.0)];
        let t = SphericalRadarThreat::new(&rs, ThreatParams {
            detection_curve: DetectionCurve::Linear,
            radar_inflation: 1.0,
            p_cross: 0.05,
            ..Default::default()
        });
        // 路径直穿雷达上方（0.0→1.0 经 0.5,0）→ 累计探测概率高 → 超阈值
        let path = Path::new(vec![
            PathPoint::new(0.0, 0.0, 3000.0),
            PathPoint::new(1.0, 0.0, 3000.0),
        ]);
        let rep = t.evaluate(&path, None);
        assert!(rep.over_threshold, "cum p = {}", rep.cumulative_p);
        assert!(rep.cumulative_p > 0.05);
        assert!(rep.peak_p > 0.05);
        // 远距离路径（0.0,5.0 → 1.0,5.0，雷达在 lat=0）→ 探测 ≈ 0
        let far = Path::new(vec![
            PathPoint::new(0.0, 5.0, 3000.0),
            PathPoint::new(1.0, 5.0, 3000.0),
        ]);
        let rep = t.evaluate(&far, None);
        assert!(!rep.over_threshold, "far cum p = {}", rep.cumulative_p);
        assert!(rep.cumulative_p < 1e-9);
    }
}

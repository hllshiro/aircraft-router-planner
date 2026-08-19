//! 路径类型与几何工具（Phase 3 输出美化链的基础）。
//!
//! - `PathPoint`：经纬度 + 高度（MSL 米）+ 可选航向（度，真北）；
//! - `Path`：点列（折线）；提供球面累计长度 / 航向 / 弦高（点到线段球面近似）等几何工具；
//! - 高程语义：`alt_m` 为 MSL 几何高度（技术方案 4.2.2 统一口径）。

/// 路径点（经纬度度 + MSL 高度米 + 可选航向度）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PathPoint {
    pub lon: f64,
    pub lat: f64,
    pub alt_m: f64,
    /// 航向（度，真北 0..360；None = 未指定，拟合时由前后点推算）。
    pub heading_deg: Option<f64>,
}

impl PathPoint {
    pub fn new(lon: f64, lat: f64, alt_m: f64) -> Self {
        Self {
            lon,
            lat,
            alt_m,
            heading_deg: None,
        }
    }

    pub fn with_heading(mut self, heading_deg: f64) -> Self {
        self.heading_deg = Some(heading_deg);
        self
    }

    /// 是否有效（有限坐标/高度）。
    pub fn is_finite(&self) -> bool {
        self.lon.is_finite() && self.lat.is_finite() && self.alt_m.is_finite()
    }
}

/// 折线路径。
#[derive(Debug, Clone, PartialEq)]
pub struct Path {
    pub points: Vec<PathPoint>,
}

impl Path {
    pub fn new(points: Vec<PathPoint>) -> Self {
        Self { points }
    }

    pub fn is_empty(&self) -> bool {
        self.points.is_empty()
    }

    pub fn len(&self) -> usize {
        self.points.len()
    }

    pub fn first(&self) -> Option<&PathPoint> {
        self.points.first()
    }

    pub fn last(&self) -> Option<&PathPoint> {
        self.points.last()
    }

    /// 球面累计长度（米）。<2 点 → 0。
    pub fn length_m(&self) -> f64 {
        self.points
            .windows(2)
            .map(|w| haversine_m(w[0].lon, w[0].lat, w[1].lon, w[1].lat))
            .sum()
    }

    /// 第 i 段航向（度，真北；两点大圆初始方位角）。
    pub fn segment_heading_deg(&self, i: usize) -> Option<f64> {
        let a = self.points.get(i)?;
        let b = self.points.get(i + 1)?;
        Some(bearing_deg(a.lon, a.lat, b.lon, b.lat))
    }

    /// 最后一段航向（段边界拼接用——下一段的入口航向）。
    pub fn last_segment_heading(&self) -> Option<f64> {
        if self.points.len() < 2 {
            return None;
        }
        self.segment_heading_deg(self.points.len() - 2)
    }

    /// 点 p 到折线 [a, b] 段的球面近似弦高（米）。
    /// 在平面（等距近似）上计算点到线段距离，用于抽稀弦高容差判定。
    pub fn chord_error_m(&self, idx: usize, a_idx: usize, b_idx: usize) -> Option<f64> {
        let p = self.points.get(idx)?;
        let a = self.points.get(a_idx)?;
        let b = self.points.get(b_idx)?;
        Some(point_seg_distance_m(
            p.lon, p.lat, a.lon, a.lat, b.lon, b.lat,
        ))
    }
}

/// 球面距离（米，haversine；与 spatial.rs 同口径）。
pub fn haversine_m(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    const R: f64 = 6_371_000.0;
    let d_lat = (lat2 - lat1).to_radians();
    let d_lon = (lon2 - lon1).to_radians();
    let a = (d_lat / 2.0).sin().powi(2)
        + lat1.to_radians().cos() * lat2.to_radians().cos() * (d_lon / 2.0).sin().powi(2);
    R * 2.0 * a.sqrt().min(1.0).asin()
}

/// 大圆初始方位角（度，真北 0..360）。
pub fn bearing_deg(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let (p1, l1, p2, l2) = (
        lat1.to_radians(),
        lon1.to_radians(),
        lat2.to_radians(),
        lon2.to_radians(),
    );
    let y = (l2 - l1).sin() * p2.cos();
    let x = p1.cos() * p2.sin() - p1.sin() * p2.cos() * (l2 - l1).cos();
    let deg = y.atan2(x).to_degrees();
    if deg < 0.0 { deg + 360.0 } else { deg }
}

/// 点到线段距离（米；等距平面近似：经度按 cos(lat) 缩放）。
/// 投影中心 = 线段中点，避免大范围经度余弦偏差。
pub fn point_seg_distance_m(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let mid_lat = (ay + by) / 2.0;
    let k = mid_lat.to_radians().cos().max(1e-6);
    let (px, py) = (px * k, py);
    let (ax, ay) = (ax * k, ay);
    let (bx, by) = (bx * k, by);
    let (vx, vy) = (bx - ax, by - ay);
    let len2 = vx * vx + vy * vy;
    let t = if len2 <= f64::EPSILON {
        0.0
    } else {
        (((px - ax) * vx + (py - ay) * vy) / len2).clamp(0.0, 1.0)
    };
    let (cx, cy) = (ax + vx * t, ay + vy * t);
    let d_deg = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
    // 弧长近似：纬向 1° ≈ 111320m，经向按 cos(mid_lat) 已在 k 中缩放
    d_deg * 111_320.0
}

/// 归一化角差（度，[-180, 180)）。
pub fn angle_diff_deg(a: f64, b: f64) -> f64 {
    let mut d = (a - b) % 360.0;
    if d >= 180.0 {
        d -= 360.0;
    } else if d < -180.0 {
        d += 360.0;
    }
    d
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn haversine_known_distance() {
        // 北京→上海约 1067km（0.1° 精度内）
        let d = haversine_m(116.4, 39.9, 121.5, 31.2);
        assert!((d - 1_067_000.0).abs() < 30_000.0, "d={d}");
    }

    #[test]
    fn bearing_east_is_90() {
        let b = bearing_deg(0.0, 0.0, 1.0, 0.0);
        assert!((b - 90.0).abs() < 0.01, "b={b}");
        // 北向 = 0
        let b2 = bearing_deg(0.0, 0.0, 0.0, 1.0);
        assert!(
            (b2 - 0.0).abs() < 0.01 || (b2 - 360.0).abs() < 0.01,
            "b2={b2}"
        );
    }

    #[test]
    fn point_seg_distance() {
        // 点 (0, 1) 到线段 (0,0)-(0,2)（同一经线）：距离 0
        assert!(point_seg_distance_m(0.0, 1.0, 0.0, 0.0, 0.0, 2.0) < 1e-6);
        // 点 (1, 1) 到同线段：约 111.32km
        let d = point_seg_distance_m(1.0, 1.0, 0.0, 0.0, 0.0, 2.0);
        assert!((d - 111_320.0).abs() < 200.0, "d={d}");
        // 投影超出线段 → 距离 = 到端点
        let d2 = point_seg_distance_m(3.0, 2.0, 0.0, 0.0, 1.0, 0.0);
        assert!(d2 > 222_000.0, "d2={d2}");
    }

    #[test]
    fn angle_diff() {
        assert!((angle_diff_deg(10.0, 20.0) - (-10.0)).abs() < 1e-9);
        assert!((angle_diff_deg(350.0, 10.0) - (-20.0)).abs() < 1e-9);
        assert!((angle_diff_deg(10.0, 350.0) - 20.0).abs() < 1e-9);
        assert!((angle_diff_deg(0.0, 180.0) - (-180.0)).abs() < 1e-9);
    }

    #[test]
    fn path_length() {
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 100.0),
            PathPoint::new(1.0, 0.0, 100.0),
            PathPoint::new(2.0, 0.0, 100.0),
        ]);
        let d = p.length_m();
        assert!((d - 2.0 * 111_320.0).abs() < 500.0, "d={d}");
    }
}

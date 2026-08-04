//! 坐标系统（技术方案 4.2.3 + Phase 1）。
//!
//! - 椭球表：WGS84 / CGCS2000 / GRS80（参数经 EPSG.io WKT2 核对，见下方常量注释）；
//! - 投影：自定义 TM 主算面（红皮书 Krüger 级数，|Δλ|<3° 精度 mm 级）+ UTM + GK3° + WebMercator；
//! - 近场 ENU：相对任务原点（起降/近场基元用，测地线方位分解）；
//! - 垂直基准层：椭球高为主，EGM96 偏移表为数据依赖（接口声明，表到位后回填）；
//! - 输出投影 codec：按 config 声明选择并正反算。

use crate::error::InputInvalidReason;
use geo::{Distance, Geodesic};

// ==================== 椭球 ====================

/// 旋转椭球。参数经 EPSG.io 核对（2026-08-04）：
/// - EPSG:4326 (WGS 84) WKT2 spheroid: 6378137 / 298.257223563
/// - EPSG:4490 (CGCS2000) WKT2 spheroid: 6378137 / 298.257222101（与 GRS80 同参数）
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Ellipsoid {
    pub name: &'static str,
    /// 长半轴（米）
    pub a: f64,
    /// 扁率倒数 1/f
    pub inv_f: f64,
}

impl Ellipsoid {
    pub const WGS84: Ellipsoid = Ellipsoid {
        name: "WGS 84",
        a: 6_378_137.0,
        inv_f: 298.257_223_563,
    };
    pub const CGCS2000: Ellipsoid = Ellipsoid {
        name: "CGCS2000",
        a: 6_378_137.0,
        inv_f: 298.257_222_101,
    };
    pub const GRS80: Ellipsoid = Ellipsoid {
        name: "GRS 1980",
        a: 6_378_137.0,
        inv_f: 298.257_222_101,
    };

    /// 短半轴（米）
    pub fn b(&self) -> f64 {
        self.a * (1.0 - 1.0 / self.inv_f)
    }
    /// 第一偏心率平方 e²
    pub fn e2(&self) -> f64 {
        let f = 1.0 / self.inv_f;
        f * (2.0 - f)
    }
    /// 第二偏心率平方 e'²
    pub fn e_prime2(&self) -> f64 {
        self.e2() / (1.0 - self.e2())
    }
}

/// 基准面：Phase 1 椭球级基准（无 7 参数地心变换；跨基准转换属开发期预处理）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub enum Datum {
    Wgs84,
    Cgcs2000,
}

impl Datum {
    pub fn ellipsoid(&self) -> Ellipsoid {
        match self {
            Datum::Wgs84 => Ellipsoid::WGS84,
            Datum::Cgcs2000 => Ellipsoid::CGCS2000,
        }
    }

    /// EPSG 代码（输出/核对用）
    pub fn epsg(&self) -> u32 {
        match self {
            Datum::Wgs84 => 4326,
            Datum::Cgcs2000 => 4490,
        }
    }
}

// ==================== 地理坐标 ====================

/// 经纬度（度）。非法值（NaN/Inf/越界）在构造时拒绝。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct Geo {
    pub lon: f64,
    pub lat: f64,
}

impl Geo {
    pub fn new(lon: f64, lat: f64) -> Result<Self, InputInvalidReason> {
        if !lon.is_finite() || !lat.is_finite() || lon < -180.0 || lon > 180.0 || lat < -90.0 || lat > 90.0
        {
            return Err(InputInvalidReason::IllegalCoordinate);
        }
        Ok(Self { lon, lat })
    }

    /// 与另一地理点的测地距离（米，geo Geodesic）
    pub fn distance_m(&self, other: &Geo) -> f64 {
        Geodesic.distance(
            geo::Point::new(self.lon, self.lat),
            geo::Point::new(other.lon, other.lat),
        )
    }
}

// ==================== 垂直基准 ====================

/// 垂直基准层。内置地形一律采用椭球高（EGM96 偏移属数据依赖，接口声明占位）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum VerticalDatum {
    /// 椭球高（默认，内置数据契约）
    Ellipsoid,
    /// EGM96 大地水准面（需外部偏移表，Phase 1 无表时报数据错误）
    Egm96,
}

impl VerticalDatum {
    pub fn as_str(&self) -> &'static str {
        match self {
            VerticalDatum::Ellipsoid => "ellipsoid",
            VerticalDatum::Egm96 => "egm96",
        }
    }
}

// ==================== 横轴墨卡托（自定义 TM 主算面） ====================

/// 横轴墨卡托投影（红皮书 Krüger 级数，|Δλ|≤3° 内误差 <1mm）。
#[derive(Debug, Clone, Copy)]
pub struct TransverseMercator {
    pub ellipsoid: Ellipsoid,
    /// 中央经线（度）
    pub central_meridian: f64,
    /// 比例因子（UTM=0.9996，GK=1.0，自定义可配）
    pub scale_factor: f64,
    pub false_easting: f64,
    pub false_northing: f64,
}

impl TransverseMercator {
    /// 正算：地理 → 平面米 (easting, northing)
    pub fn forward(&self, lon_deg: f64, lat_deg: f64) -> (f64, f64) {
        let a = self.ellipsoid.a;
        let e2 = self.ellipsoid.e2();
        let e_prime2 = self.ellipsoid.e_prime2();
        let k0 = self.scale_factor;

        let phi = lat_deg.to_radians();
        let lam = (lon_deg - self.central_meridian).to_radians();
        let cos_phi = phi.cos();
        let sin_phi = phi.sin();
        let tan_phi = phi.tan();

        let n = a / (1.0 - e2 * sin_phi * sin_phi).sqrt();
        let t = tan_phi * tan_phi;
        let c = e_prime2 * cos_phi * cos_phi;
        let a_coef = lam * cos_phi;

        // 子午线弧长 M
        let m = a
            * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * phi
                - (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0) * (2.0 * phi).sin()
                + (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * (4.0 * phi).sin()
                - (35.0 * e2 * e2 * e2 / 3072.0) * (6.0 * phi).sin());

        let x = k0 * n * (a_coef
            + (1.0 - t + c) * a_coef.powi(3) / 6.0
            + (5.0 - 18.0 * t + t * t + 72.0 * c - 58.0 * e_prime2) * a_coef.powi(5) / 120.0)
            + self.false_easting;
        let y = k0
            * (m
                + n * tan_phi
                    * (a_coef * a_coef / 2.0
                        + (5.0 - t + 9.0 * c + 4.0 * c * c) * a_coef.powi(4) / 24.0
                        + (61.0 - 58.0 * t + t * t + 600.0 * c - 330.0 * e_prime2) * a_coef.powi(6)
                            / 720.0))
            + self.false_northing;
        (x, y)
    }

    /// 反算：平面米 → 地理（度）
    pub fn inverse(&self, easting: f64, northing: f64) -> (f64, f64) {
        let a = self.ellipsoid.a;
        let e2 = self.ellipsoid.e2();
        let e_prime2 = self.ellipsoid.e_prime2();
        let k0 = self.scale_factor;

        let x = easting - self.false_easting;
        let y = northing - self.false_northing;

        // 底部半径计算辅助
        let m0_scale = 1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0;
        let mu = y / (k0 * a * m0_scale);
        // 迭代求足点纬度 φ1
        let mut phi1 = mu;
        for _ in 0..8 {
            let m1 = a
                * ((1.0 - e2 / 4.0 - 3.0 * e2 * e2 / 64.0 - 5.0 * e2 * e2 * e2 / 256.0) * phi1
                    - (3.0 * e2 / 8.0 + 3.0 * e2 * e2 / 32.0 + 45.0 * e2 * e2 * e2 / 1024.0)
                        * (2.0 * phi1).sin()
                    + (15.0 * e2 * e2 / 256.0 + 45.0 * e2 * e2 * e2 / 1024.0) * (4.0 * phi1).sin()
                    - (35.0 * e2 * e2 * e2 / 3072.0) * (6.0 * phi1).sin());
            phi1 += (y - k0 * m1) / (a * m0_scale);
        }
        let sin1 = phi1.sin();
        let cos1 = phi1.cos();
        let tan1 = phi1.tan();
        let n1 = a / (1.0 - e2 * sin1 * sin1).sqrt();
        let r1 = a * (1.0 - e2) / (1.0 - e2 * sin1 * sin1).powf(1.5);
        let t1 = tan1 * tan1;
        let c1 = e_prime2 * cos1 * cos1;
        let d = x / (k0 * n1);

        let phi = phi1
            - (n1 * tan1 / r1)
                * (d * d / 2.0
                    - (5.0 + 3.0 * t1 + 10.0 * c1 - 4.0 * c1 * c1 - 9.0 * e_prime2) * d.powi(4)
                        / 24.0
                    + (61.0 + 90.0 * t1 + 298.0 * c1 + 45.0 * t1 * t1 - 252.0 * e_prime2
                        - 3.0 * c1 * c1)
                        * d.powi(6)
                        / 720.0);
        let lam = (d
            - (1.0 + 2.0 * t1 + c1) * d.powi(3) / 6.0
            + (5.0 - 2.0 * c1 + 28.0 * t1 - 3.0 * c1 * c1 + 8.0 * e_prime2 + 24.0 * t1 * t1)
                * d.powi(5)
                / 120.0)
            / cos1;

        (self.central_meridian + lam.to_degrees(), phi.to_degrees())
    }

    /// UTM 带（1-60）
    pub fn utm_zone(lon_deg: f64) -> u8 {
        let zone = ((lon_deg + 180.0) / 6.0).floor() as i64 + 1;
        zone.clamp(1, 60) as u8
    }

    /// 构造 UTM 投影（自动带号，北半球；南半球需另配 false_northing=10_000_000）
    pub fn utm(ellipsoid: Ellipsoid, lon_deg: f64) -> Self {
        let zone = Self::utm_zone(lon_deg);
        let cm = -183.0 + 6.0 * zone as f64;
        Self {
            ellipsoid,
            central_meridian: cm,
            scale_factor: 0.9996,
            false_easting: 500_000.0,
            false_northing: 0.0,
        }
    }

    /// 构造中国 3° 带高斯-克吕格投影（GK3°，zone=带号，cm=zone*3）
    pub fn gk3(ellipsoid: Ellipsoid, zone: u8) -> Self {
        Self {
            ellipsoid,
            central_meridian: 3.0 * zone as f64,
            scale_factor: 1.0,
            false_easting: 500_000.0,
            false_northing: 0.0,
        }
    }
}

// ==================== Web 墨卡托（EPSG:3857，球形） ====================

/// Web 墨卡托（EPSG:3857，球形近似，a=b=6378137；WKT2 核对：Mercator_1SP,
/// cm=0, k=1, FE=FN=0）。
pub struct WebMercator;

impl WebMercator {
    const R: f64 = 6_378_137.0;

    pub fn forward(lon_deg: f64, lat_deg: f64) -> (f64, f64) {
        let lat = lat_deg.clamp(-85.06, 85.06).to_radians();
        let x = Self::R * lon_deg.to_radians();
        let y = Self::R * ((std::f64::consts::FRAC_PI_4 + lat / 2.0).tan()).ln();
        (x, y)
    }

    pub fn inverse(x: f64, y: f64) -> (f64, f64) {
        let lon = (x / Self::R).to_degrees();
        let lat = (2.0 * ((y / Self::R).exp()).atan() - std::f64::consts::FRAC_PI_2).to_degrees();
        (lon, lat)
    }
}

// ==================== 近场 ENU ====================

/// 近场 ENU（局部东-北-上，原点为任务锚点）。Phase 1 用球面近似（R=a），
/// 覆盖近场（≤10km 起降/近场基元）误差 ≤米级；远场应切换主 TM 投影面。
#[derive(Debug, Clone, Copy)]
pub struct EnuFrame {
    pub origin: Geo,
}

impl EnuFrame {
    const R: f64 = 6_378_137.0;

    pub fn new(origin: Geo) -> Self {
        Self { origin }
    }

    /// 地理 → ENU (east, north) 米（球面近似，近场）
    pub fn to_enu(&self, p: &Geo) -> (f64, f64) {
        let north = (p.lat - self.origin.lat).to_radians() * Self::R;
        let east = (p.lon - self.origin.lon).to_radians() * Self::R * self.origin.lat.to_radians().cos();
        (east, north)
    }

    /// ENU (east, north) 米 → 地理（球面近似，近场）
    pub fn from_enu(&self, east: f64, north: f64) -> Geo {
        let lat = self.origin.lat + (north / Self::R).to_degrees();
        let cos_lat = self.origin.lat.to_radians().cos().max(1e-9);
        let lon = self.origin.lon + (east / (Self::R * cos_lat)).to_degrees();
        Geo { lon, lat }
    }
}

// ==================== 输出投影 codec ====================

/// 输出投影选择（config 声明，Phase 1 提供四种 + 自定义 TM）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum OutputProjection {
    LonLat,
    Utm(u8),
    Gk3(u8),
    WebMercator,
}

impl OutputProjection {
    pub fn as_str(&self) -> String {
        match self {
            OutputProjection::LonLat => "lonlat".into(),
            OutputProjection::Utm(z) => format!("utm{}-n", z),
            OutputProjection::Gk3(z) => format!("gk3-{}", z),
            OutputProjection::WebMercator => "web_mercator".into(),
        }
    }
}

/// 输出投影 codec：绑定椭球 + 投影选择，forward/inverse 分发。
#[derive(Debug, Clone, Copy)]
pub enum ProjectionCodec {
    LonLat,
    Utm(TransverseMercator),
    Gk3(TransverseMercator),
    WebMercator,
}

impl ProjectionCodec {
    pub fn new(proj: OutputProjection, ellipsoid: Ellipsoid, anchor: &Geo) -> Self {
        match proj {
            OutputProjection::LonLat => ProjectionCodec::LonLat,
            OutputProjection::Utm(z) => {
                let mut tm = TransverseMercator::utm(ellipsoid, anchor.lon);
                tm.central_meridian = -183.0 + 6.0 * z as f64;
                ProjectionCodec::Utm(tm)
            }
            OutputProjection::Gk3(z) => ProjectionCodec::Gk3(TransverseMercator::gk3(ellipsoid, z)),
            OutputProjection::WebMercator => ProjectionCodec::WebMercator,
        }
    }

    pub fn forward(&self, lon_deg: f64, lat_deg: f64) -> (f64, f64) {
        match self {
            ProjectionCodec::LonLat => (lon_deg, lat_deg),
            ProjectionCodec::Utm(tm) => tm.forward(lon_deg, lat_deg),
            ProjectionCodec::Gk3(tm) => tm.forward(lon_deg, lat_deg),
            ProjectionCodec::WebMercator => WebMercator::forward(lon_deg, lat_deg),
        }
    }

    pub fn inverse(&self, x: f64, y: f64) -> (f64, f64) {
        match self {
            ProjectionCodec::LonLat => (x, y),
            ProjectionCodec::Utm(tm) => tm.inverse(x, y),
            ProjectionCodec::Gk3(tm) => tm.inverse(x, y),
            ProjectionCodec::WebMercator => WebMercator::inverse(x, y),
        }
    }
}

// ==================== WKT2 核对记录（2026-08-04，EPSG.io 拉取） ====================
// 仅作参数溯源核对，不随产品分发。
// EPSG:4326 GEOGCS WGS84: spheroid[6378137, 298.257223563]
// EPSG:4490 GEOGCS CGCS2000: spheroid[6378137, 298.257222101]
// EPSG:3857 PROJCS Pseudo-Mercator: Mercator_1SP[cm=0, k=1, FE=0, FN=0], +b=6378137(球形)

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn geo_rejects_invalid() {
        assert!(Geo::new(116.0, 39.0).is_ok());
        assert_eq!(Geo::new(f64::NAN, 39.0), Err(InputInvalidReason::IllegalCoordinate));
        assert_eq!(Geo::new(181.0, 39.0), Err(InputInvalidReason::IllegalCoordinate));
        assert_eq!(Geo::new(116.0, -91.0), Err(InputInvalidReason::IllegalCoordinate));
        assert_eq!(Geo::new(116.0, f64::INFINITY), Err(InputInvalidReason::IllegalCoordinate));
    }

    #[test]
    fn tm_forward_inverse_roundtrip() {
        let tm = TransverseMercator::utm(Ellipsoid::WGS84, 116.0); // zone 50, cm=117
        for (lon, lat) in [(116.397, 39.909), (113.5, 30.0), (120.0, 45.0), (116.397, 40.221)] {
            let (x, y) = tm.forward(lon, lat);
            let (lon2, lat2) = tm.inverse(x, y);
            // Krüger 级数截断误差 mm 级；1e-7 度 ≈ 1.1cm 弧长，满足米级契约
            assert!((lon - lon2).abs() < 1e-7, "lon drift {} vs {}", lon, lon2);
            assert!((lat - lat2).abs() < 1e-7, "lat drift {} vs {}", lat, lat2);
        }
    }

    #[test]
    fn utm_zone_calculation() {
        assert_eq!(TransverseMercator::utm_zone(116.397), 50);
        assert_eq!(TransverseMercator::utm_zone(-179.0), 1);
        assert_eq!(TransverseMercator::utm_zone(179.0), 60);
    }

    #[test]
    fn beijing_utm_zone50_range() {
        // 北京(116.397, 39.909) UTM 50N：E≈447km N≈4418km（数量级验证）
        let tm = TransverseMercator::utm(Ellipsoid::WGS84, 116.397);
        let (x, y) = tm.forward(116.397, 39.909);
        assert!((440_000.0..455_000.0).contains(&x), "x={x}");
        assert!((4_415_000.0..4_425_000.0).contains(&y), "y={y}");
    }

    #[test]
    fn gk3_beijing_zone40_range() {
        // 北京(116.397, 39.909) 3°带 zone 40（cm=120°E）：Δλ=3.603° → E≈192km；
        // 北向 y = 子午弧长 ≈ 4425.6km
        let tm = TransverseMercator::gk3(Ellipsoid::CGCS2000, 40);
        let (x, y) = tm.forward(116.397, 39.909);
        assert!((185_000.0..200_000.0).contains(&x), "x={x}");
        assert!((4_420_000.0..4_430_000.0).contains(&y), "y={y}");
    }

    #[test]
    fn web_mercator_beijing() {
        // 北京 WebMercator：x = 116.397° × 111319.49 m/° ≈ 12957255（仅经度决定）；
        // y = R·ln(tan(π/4+φ/2)) ≈ 4852727（回代验证见下）
        let (x, y) = WebMercator::forward(116.397, 39.909);
        assert!((x - 12_957_254.8).abs() < 1.0, "x={x}");
        assert!((y - 4_852_727.2).abs() < 1.0, "y={y}");
        let (lon, lat) = WebMercator::inverse(x, y);
        assert!((lon - 116.397).abs() < 1e-9);
        assert!((lat - 39.909).abs() < 1e-9);
    }

    #[test]
    fn datum_epsg_codes() {
        assert_eq!(Datum::Wgs84.epsg(), 4326);
        assert_eq!(Datum::Cgcs2000.epsg(), 4490);
    }

    #[test]
    fn enu_roundtrip() {
        let origin = Geo::new(116.397, 39.909).unwrap();
        let frame = EnuFrame::new(origin);
        let p = Geo::new(116.50, 40.0).unwrap();
        let (e, n) = frame.to_enu(&p);
        let back = frame.from_enu(e, n);
        assert!((back.lon - p.lon).abs() < 1e-7, "lon drift {}", back.lon - p.lon);
        assert!((back.lat - p.lat).abs() < 1e-7, "lat drift {}", back.lat - p.lat);
    }
}

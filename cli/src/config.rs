//! 输入/输出 JSON 契约（技术方案 4.2.1 / 4.5 + Phase 1）。
//!
//! - 输入：`crs` 坐标系声明、A/B 起终点顶层字段、`vehicles` 多机数组、
//!   红方雷达、禁飞/限飞区、武器映射表、地形配置、默认参数表覆盖；
//! - 输出：`schema_version` + `status` 四态 + 错误体 + 车辆结果 + 统计；
//! - `InputValidator`：畸形/退化输入在解析层即拦截 → `input_invalid` + 原因码。
//!
//! 严格契约：`deny_unknown_fields`（未知字段 = 畸形，属 MalformedJson）。

use crate::coord::{Datum, Geo, VerticalDatum};
use crate::error::{AppError, InputInvalidReason};
use serde::{Deserialize, Serialize};

/// 当前 schema 版本（与方案 v0.20 对齐）。
pub const SCHEMA_VERSION: &str = "0.20";

// ==================== 输入契约 ====================

/// 任务输入（顶层）。未知字段 → 畸形。
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Input {
    pub schema_version: String,
    #[serde(default)]
    pub crs: CrsConfig,
    #[serde(default)]
    pub output_crs: OutputCrsConfig,
    pub mission: Mission,
}

#[derive(Debug, Clone, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct CrsConfig {
    /// WGS84 / CGCS2000 / GRS80（默认 WGS84；其余 datum fail-fast，见 4.2.3）
    #[serde(default)]
    pub datum: DatumName,
    /// MSL（默认，输入高程基准；内部统一椭球高见垂直基准层）
    #[serde(default)]
    pub vertical: VerticalName,
    /// lonlat / utm / gk3（输入本身是投影坐标时，默认 lonlat）
    #[serde(default)]
    pub input_projection: InputProjection,
}

#[derive(Debug, Clone, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct OutputCrsConfig {
    /// lonlat / utm / gk3 / web_mercator / custom_tm
    #[serde(default)]
    pub projection: OutputProjectionName,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "UPPERCASE")]
pub enum DatumName {
    #[default]
    Wgs84,
    Cgcs2000,
    Grs80,
}

impl DatumName {
    pub fn to_datum(&self) -> Result<Datum, AppError> {
        match self {
            DatumName::Wgs84 | DatumName::Grs80 => Ok(Datum::Wgs84),
            DatumName::Cgcs2000 => Ok(Datum::Cgcs2000),
        }
    }
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "UPPERCASE")]
pub enum VerticalName {
    #[default]
    Msl,
    /// 椭球高（显式声明时输入高度为椭球高）
    Ellipsoid,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum InputProjection {
    #[default]
    Lonlat,
    Utm(u8),
    Gk3(u8),
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum OutputProjectionName {
    #[default]
    Lonlat,
    Utm(u8),
    Gk3(u8),
    WebMercator,
}

/// 任务（mission）—— A/B 起终点为显式顶层字段（四轮共识）。
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Mission {
    pub start: Waypoint,
    pub target: Waypoint,
    #[serde(default)]
    pub vehicles: Vec<VehicleInput>,
    #[serde(default)]
    pub red_forces: RedForces,
    #[serde(default)]
    pub no_fly_zones: Vec<Zone>,
    #[serde(default)]
    pub restricted_zones: Vec<Zone>,
    #[serde(default)]
    pub obstacles: Vec<Zone>,
    #[serde(default)]
    pub terrain: TerrainConfig,
    #[serde(default)]
    pub weapons: Vec<WeaponEntry>,
    #[serde(default)]
    pub parameters: ParamsOverride,
}

/// 航路点（经纬度 + 高程，米；高程 = MSL 或椭球高，随 crs.vertical）。
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Waypoint {
    pub lon: f64,
    pub lat: f64,
    pub alt_m: f64,
}

impl Waypoint {
    pub fn to_geo(&self) -> Result<Geo, AppError> {
        Geo::new(self.lon, self.lat).map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))
    }
}

/// 车辆输入（十三轮共识多机契约）：profile + 起点 pose + 目标引用 + 任务场景。
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleInput {
    pub id: String,
    pub profile: VehicleProfile,
    pub start_pose: VehiclePose,
    /// 目标引用（缺省 = mission.target）
    #[serde(default)]
    pub target_ref: Option<String>,
    /// 任务场景（缺省 = A1 通用语义）：launch_position / evade_detection
    #[serde(default)]
    pub scenario: Option<String>,
}

/// 车辆性能参数集（八轮共识：输入显式提供，缺省落默认参数表占位，缺参 fail-fast）。
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehicleProfile {
    pub aircraft_type: AircraftType,
    /// 巡航速度 m/s（或 speed_range 二选一）
    #[serde(default)]
    pub cruise_speed_mps: Option<f64>,
    /// 速度范围 [v_min, v_max] m/s（十轮主管裁决：速度为核心输入）
    #[serde(default)]
    pub speed_range_mps: Option<[f64; 2]>,
    /// 最小转弯半径 m（缺省默认参数表；A6 自洽：r_min ≥ v²/(g·tan φ_max)）
    #[serde(default)]
    pub min_turn_radius_m: Option<f64>,
    /// 最大爬升角 °
    #[serde(default)]
    pub max_climb_angle_deg: Option<f64>,
    /// 最大坡度 φ_max °（ρ = v²/(g·tan φ_max) 物理耦合）
    #[serde(default)]
    pub max_bank_deg: Option<f64>,
    #[serde(default)]
    pub ceiling_m: Option<f64>,
    /// 飞机探测概率参数（隐蔽突防折算项，0..=1）
    #[serde(default)]
    pub detection_probability: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AircraftType {
    FixedWing,
    Rotorcraft,
}

/// 起点位姿（多机时每机独立起点；单机时与 mission.start 一致）。
#[derive(Debug, Clone, Copy, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct VehiclePose {
    pub lon: f64,
    pub lat: f64,
    /// 初始航向 °（真北，0-360）
    #[serde(default)]
    pub heading_deg: f64,
    pub alt_m: f64,
}

/// 红方部署。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct RedForces {
    #[serde(default)]
    pub radars: Vec<Radar>,
    #[serde(default)]
    pub sams: Vec<Sam>,
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Radar {
    pub id: String,
    pub lon: f64,
    pub lat: f64,
    /// early_warning / tracking / fire_control
    pub radar_type: RadarType,
    /// 探测距离（km，球体模型基础半径）
    pub radius_km: f64,
    /// 天线高度 m（默认 10）
    #[serde(default = "default_antenna_m")]
    pub alt_m: f64,
    /// 压制后有效距离 km（十三轮共识可选字段，输入优先）
    #[serde(default)]
    pub suppression_post_range_km: Option<f64>,
    /// 压制因子 δ（0..1，探测距离 × (1−δ)）
    #[serde(default)]
    pub suppression_factor: Option<f64>,
}

fn default_antenna_m() -> f64 {
    10.0
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum RadarType {
    EarlyWarning,
    Tracking,
    FireControl,
}

/// 红方地空导弹（Phase 2 威胁建模占位；Phase 1 仅解析）。
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Sam {
    pub id: String,
    pub lon: f64,
    pub lat: f64,
    pub radius_km: f64,
}

/// 禁飞/限飞/非地形障碍物（几何语义同禁飞区硬阈值，4.2.1/4.5）。
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct Zone {
    pub id: String,
    pub zone_type: ZoneType,
    #[serde(flatten)]
    pub shape: ZoneShape,
    /// 高度区间（MSL；AGL 语义由 height_semantics 声明，解析层换算）
    pub alt_min_m: f64,
    pub alt_max_m: f64,
    #[serde(default)]
    pub height_semantics: HeightSemantics,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum ZoneType {
    NoFly,
    Restricted,
    Obstacle,
}

impl Zone {
    /// 是否代价场硬墙（Phase 4 M2）：NoFly/Obstacle 全高度水平禁入；
    /// Restricted 为高度层禁入（区间外可穿越），不画墙。
    pub fn is_wall(&self) -> bool {
        matches!(self.zone_type, ZoneType::NoFly | ZoneType::Obstacle)
    }
}

#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields, tag = "shape", content = "geometry", rename_all = "snake_case")]
pub enum ZoneShape {
    Circle { center: [f64; 2], radius_km: f64 },
    Polygon { vertices: Vec<[f64; 2]> },
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum HeightSemantics {
    #[default]
    Msl,
    Agl,
}

/// 地形配置（4.2.4 默认场景 / 4.2.5 内置契约）。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct TerrainConfig {
    /// none（默认，海拔 0 平面）/ builtin（内置数据包）/ path（外部文件）
    #[serde(default)]
    pub source: TerrainSourceType,
    #[serde(default)]
    pub path: Option<String>,
    #[serde(default)]
    pub resolution_m: Option<f64>,
    #[serde(default)]
    pub vertical_datum: Option<VerticalDatumName>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum TerrainSourceType {
    #[default]
    None,
    Builtin,
    Path,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize)]
#[serde(rename_all = "snake_case")]
pub enum VerticalDatumName {
    Ellipsoid,
    Egm96,
}

impl VerticalDatumName {
    pub fn to_datum(&self) -> VerticalDatum {
        match self {
            VerticalDatumName::Ellipsoid => VerticalDatum::Ellipsoid,
            VerticalDatumName::Egm96 => VerticalDatum::Egm96,
        }
    }
}

/// 武器映射条目（十一轮共识结构：射程区间 / 引信 / 发射包线 / 打击目标引用）。
#[derive(Debug, Clone, Deserialize)]
#[serde(deny_unknown_fields)]
pub struct WeaponEntry {
    pub weapon_id: String,
    /// 射程 [Rmin, Rmax] km（Rmin 可为 0；带最小射程不得停在 Rmin 内，九轮共识）
    pub range_km: [f64; 2],
    #[serde(default)]
    pub fuze_type: String,
    #[serde(default)]
    pub envelope: Option<LaunchEnvelope>,
    /// 打击目标引用（缺省 = mission.target）
    #[serde(default)]
    pub target_ref: Option<String>,
}

/// 发射包线（航向/高度/速度窗）。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct LaunchEnvelope {
    #[serde(default)]
    pub heading_deg: Option<[f64; 2]>,
    #[serde(default)]
    pub alt_m: Option<[f64; 2]>,
    #[serde(default)]
    pub speed_mps: Option<[f64; 2]>,
}

/// 默认参数表覆盖（全部可选，未提供用 DefaultParams）。
#[derive(Debug, Clone, Deserialize, Default)]
#[serde(deny_unknown_fields)]
pub struct ParamsOverride {
    #[serde(default)]
    pub radar_inflation: Option<f64>,
    #[serde(default)]
    pub detection_curve: Option<DetectionCurve>,
    #[serde(default)]
    pub p_cross: Option<f64>,
    #[serde(default)]
    pub suppression_delta: Option<f64>,
    #[serde(default)]
    pub los_mask_coef: Option<f64>,
    #[serde(default)]
    pub main_budget_ms: Option<u64>,
    #[serde(default)]
    pub degrade_budget_ms: Option<u64>,
    #[serde(default)]
    pub z_resolution_m: Option<f64>,
    #[serde(default)]
    pub fine_success_threshold: Option<f64>,
    #[serde(default)]
    pub coarse_cell_m: Option<f64>,
    #[serde(default)]
    pub default_weapon_radius_km: Option<f64>,
    #[serde(default)]
    pub weapon_map: Option<Vec<WeaponEntry>>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default)]
#[serde(rename_all = "snake_case")]
pub enum DetectionCurve {
    #[default]
    Exponential,
    Linear,
}

// ==================== 默认参数表（三轮共识；占位值 Phase 0 校准后生效） ====================

/// 默认参数表（编译期常量，输入未提供时使用；全部为占位值，Phase 0 校准）。
#[derive(Debug, Clone)]
pub struct DefaultParams {
    /// 雷达膨胀系数（球体半径 = 实际 × 系数，>1）
    pub radar_inflation: f64,
    /// 探测概率衰减形态（指数为默认推荐：R⁻⁴ 简化）
    pub detection_curve: DetectionCurve,
    /// 穿越阈值 P_cross（占位保守低值，Phase 0 定值；未标定不得声称实现）
    pub p_cross: f64,
    /// 压制修正因子 δ（探测距离 × (1−δ)，占位）
    pub suppression_delta: f64,
    /// LOS mask 系数（默认 0.05–0.1 区间内取 0.08；守保守口径不取 0，十二轮共识）
    pub los_mask_coef: f64,
    /// 主算法预算（ms，5.1 建议初始分配）
    pub main_budget_ms: u64,
    /// 降级链预算（ms）
    pub degrade_budget_ms: u64,
    /// 垂直分层固定层高（十三轮定稿，写入数据契约元数据）
    pub z_resolution_m: f64,
    /// 细层成功率阈值（建议 90%，十一轮收敛判据）
    pub fine_success_threshold: f64,
    /// FMM 粗层格距（m，1-2km 范围）
    pub coarse_cell_m: f64,
    /// 表外武器兜底通用值（km，最保守）
    pub default_weapon_radius_km: f64,
    /// 通用武器默认射程 [Rmin, Rmax] km（未输入时）
    pub default_weapon_range_km: [f64; 2],
    /// 最大爬升角占位（°）
    pub default_max_climb_angle_deg: f64,
    /// 最大坡度占位（°）
    pub default_max_bank_deg: f64,
    /// 最小转弯半径占位（m，固定翼）
    pub default_fixed_wing_turn_radius_m: f64,
    /// 最小转弯半径占位（m，旋翼机——悬停/原地转向 r→0 显式建模，九轮共识）
    pub default_rotorcraft_turn_radius_m: f64,
    /// 巡航速度占位（m/s，固定翼）
    pub default_fixed_wing_speed_mps: f64,
    /// 巡航速度占位（m/s，旋翼机）
    pub default_rotorcraft_speed_mps: f64,
}

impl Default for DefaultParams {
    fn default() -> Self {
        Self {
            radar_inflation: 1.2,
            detection_curve: DetectionCurve::Exponential,
            p_cross: 0.1,
            suppression_delta: 0.5,
            los_mask_coef: 0.08,
            main_budget_ms: 2_500,
            degrade_budget_ms: 500,
            z_resolution_m: 50.0,
            fine_success_threshold: 0.9,
            coarse_cell_m: 2_000.0,
            default_weapon_radius_km: 10.0,
            default_weapon_range_km: [5.0, 40.0],
            default_max_climb_angle_deg: 15.0,
            default_max_bank_deg: 30.0,
            default_fixed_wing_turn_radius_m: 5_000.0,
            default_rotorcraft_turn_radius_m: 0.0,
            default_fixed_wing_speed_mps: 250.0,
            default_rotorcraft_speed_mps: 100.0,
        }
    }
}

impl DefaultParams {
    /// 默认武器映射表（占位条目，Phase 0 校准填充；表外武器走兜底 + 告警）。
    pub fn default_weapon_map() -> Vec<WeaponEntry> {
        vec![
            WeaponEntry {
                weapon_id: "aam_medium".into(),
                range_km: [5.0, 40.0],
                fuze_type: "proximity".into(),
                envelope: None,
                target_ref: None,
            },
            WeaponEntry {
                weapon_id: "asm_air_ground".into(),
                range_km: [3.0, 120.0],
                fuze_type: "impact".into(),
                envelope: None,
                target_ref: None,
            },
            WeaponEntry {
                weapon_id: "jdam".into(),
                range_km: [1.0, 15.0],
                fuze_type: "impact".into(),
                envelope: None,
                target_ref: None,
            },
        ]
    }

    /// 合并参数覆盖（覆盖优先，未覆盖用默认）。
    pub fn merge(&self, o: &ParamsOverride) -> DefaultParams {
        let mut d = self.clone();
        if let Some(v) = o.radar_inflation {
            d.radar_inflation = v;
        }
        if let Some(v) = o.detection_curve {
            d.detection_curve = v;
        }
        if let Some(v) = o.p_cross {
            d.p_cross = v;
        }
        if let Some(v) = o.suppression_delta {
            d.suppression_delta = v;
        }
        if let Some(v) = o.los_mask_coef {
            d.los_mask_coef = v;
        }
        if let Some(v) = o.main_budget_ms {
            d.main_budget_ms = v;
        }
        if let Some(v) = o.degrade_budget_ms {
            d.degrade_budget_ms = v;
        }
        if let Some(v) = o.z_resolution_m {
            d.z_resolution_m = v;
        }
        if let Some(v) = o.fine_success_threshold {
            d.fine_success_threshold = v;
        }
        if let Some(v) = o.coarse_cell_m {
            d.coarse_cell_m = v;
        }
        if let Some(v) = o.default_weapon_radius_km {
            d.default_weapon_radius_km = v;
        }
        d
    }

    /// A6 物理自洽：给定速度与最大坡度的最小转弯半径 r_min ≥ v²/(g·tan φ_max)。
    pub fn physical_turn_radius_m(v_mps: f64, bank_deg: f64) -> f64 {
        const G: f64 = 9.80665;
        v_mps * v_mps / (G * bank_deg.to_radians().tan())
    }
}

// ==================== 输出契约 ====================

/// 输出 JSON（C11：顶层携带 schema_version；status 四态契约）。
#[derive(Debug, Clone, Serialize)]
pub struct Output {
    pub schema_version: String,
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<crate::error::ErrorBody>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elapsed_ms: Option<u64>,
    pub vehicles: Vec<VehicleOutput>,
    pub stats: Stats,
}

impl Output {
    pub fn success(elapsed_ms: u64) -> Self {
        Self {
            schema_version: SCHEMA_VERSION.into(),
            status: "success".into(),
            error: None,
            elapsed_ms: Some(elapsed_ms),
            vehicles: Vec::new(),
            stats: Stats::default(),
        }
    }

    pub fn failure(status: &str, error: crate::error::ErrorBody, elapsed_ms: u64) -> Self {
        Self {
            schema_version: SCHEMA_VERSION.into(),
            status: status.into(),
            error: Some(error),
            elapsed_ms: Some(elapsed_ms),
            vehicles: Vec::new(),
            stats: Stats::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize)]
#[serde(rename_all = "snake_case")]
pub struct VehicleOutput {
    pub id: String,
    /// planned / no_solution / degraded
    pub status: String,
    pub path: Vec<PathPoint>,
    pub distance_m: f64,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Copy, Serialize)]
#[serde(rename_all = "snake_case")]
pub struct PathPoint {
    /// 输出投影坐标（默认经纬度；projection 声明时按 codec 转换）
    pub x: f64,
    pub y: f64,
    pub alt_m: f64,
}

#[derive(Debug, Clone, Default, Serialize)]
#[serde(rename_all = "snake_case")]
pub struct Stats {
    pub fmm_ms: f64,
    pub los_checks: u64,
    pub degradations: Vec<String>,
}

// ==================== 解析与校验 ====================

impl Input {
    /// 从 JSON 字符串解析（畸形 JSON → input_invalid: malformed_json）。
    pub fn from_json_str(s: &str) -> Result<Self, AppError> {
        serde_json::from_str(s).map_err(|e| AppError::Json(e))
    }
}

/// InputValidator 前置模块：解析后立即校验，退化输入不进入算法。
pub fn validate(input: &Input) -> Result<(), AppError> {
    // schema 版本
    if input.schema_version != SCHEMA_VERSION {
        return Err(AppError::Data(format!(
            "schema_version mismatch: input {} vs supported {}",
            input.schema_version, SCHEMA_VERSION
        )));
    }
    // A/B 起终点
    let start = input.mission.start.to_geo()?;
    let target = input.mission.target.to_geo()?;
    if start.distance_m(&target) < 100.0 {
        return Err(AppError::InputInvalid(InputInvalidReason::DegenerateStartEqualsTarget));
    }
    // 多机
    let mut ids = std::collections::HashSet::new();
    for v in &input.mission.vehicles {
        if !ids.insert(v.id.clone()) {
            return Err(AppError::InputInvalid(InputInvalidReason::VehicleParamsInconsistent));
        }
        validate_vehicle(v)?;
        let pose = Geo::new(v.start_pose.lon, v.start_pose.lat)
            .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))?;
        // 起点在禁飞区
        for z in &input.mission.no_fly_zones {
            if zone_contains(z, &pose) {
                return Err(AppError::InputInvalid(InputInvalidReason::TargetInNoFly));
            }
        }
    }
    // B 在禁飞区
    for z in &input.mission.no_fly_zones {
        if zone_contains(z, &target) {
            return Err(AppError::InputInvalid(InputInvalidReason::TargetInNoFly));
        }
    }
    // 雷达 ∩ 禁飞区重叠（雷达位置在禁飞多边形内）
    for r in &input.mission.red_forces.radars {
        let g = Geo::new(r.lon, r.lat)
            .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))?;
        for z in &input.mission.no_fly_zones {
            if zone_contains(z, &g) {
                return Err(AppError::InputInvalid(InputInvalidReason::RadarOverlapNoFly));
            }
        }
        if r.radius_km <= 0.0 {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    // 参数覆盖数值域
    let o = &input.mission.parameters;
    if let Some(v) = o.radar_inflation {
        if v <= 1.0 {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    if let Some(v) = o.p_cross {
        if !(0.0..=1.0).contains(&v) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    if let Some(v) = o.los_mask_coef {
        if !(0.0..=1.0).contains(&v) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    Ok(())
}

fn validate_vehicle(v: &VehicleInput) -> Result<(), AppError> {
    let p = &v.profile;
    let speed = p
        .cruise_speed_mps
        .or_else(|| p.speed_range_mps.map(|r| r[0]));
    if let Some(s) = speed {
        if !(1.0..=1_000.0).contains(&s) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    if let Some([vmin, vmax]) = p.speed_range_mps {
        if vmin <= 0.0 || vmax < vmin {
            return Err(AppError::InputInvalid(InputInvalidReason::VehicleParamsInconsistent));
        }
    }
    // A6 物理自洽（十二轮共识）：r_min ≥ v²/(g·tan φ_max)，输入同时提供时校验
    if let (Some(v), Some(bank), Some(r)) = (speed, p.max_bank_deg, p.min_turn_radius_m) {
        let r_min = DefaultParams::physical_turn_radius_m(v, bank);
        if r < r_min * 0.999 {
            return Err(AppError::InputInvalid(InputInvalidReason::VehicleParamsInconsistent));
        }
    }
    if let Some(dp) = p.detection_probability {
        if !(0.0..=1.0).contains(&dp) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    Ok(())
}

/// 点是否在禁飞/限飞区内部（圆 / 多边形；水平几何，高度层判定 Phase 4 M2）。
pub(crate) fn zone_contains(z: &Zone, p: &Geo) -> bool {
    match &z.shape {
        ZoneShape::Circle { center, radius_km } => {
            let c = Geo::new(center[0], center[1]).ok();
            match c {
                Some(c) => c.distance_m(p) <= radius_km * 1000.0,
                None => false,
            }
        }
        ZoneShape::Polygon { vertices } => point_in_polygon(p, vertices),
    }
}

/// 射线法点在多边形内（经纬度平面近似，Phase 1 足够——区域校验用途）。
fn point_in_polygon(p: &Geo, vertices: &[[f64; 2]]) -> bool {
    if vertices.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = vertices.len() - 1;
    for i in 0..vertices.len() {
        let (xi, yi) = (vertices[i][0], vertices[i][1]);
        let (xj, yj) = (vertices[j][0], vertices[j][1]);
        if (yi > p.lat) != (yj > p.lat)
            && p.lon < (xj - xi) * (p.lat - yi) / (yj - yi) + xi
        {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// 点是否在 Zone 内且高度落入禁入区间 [alt_min, alt_max]（Phase 4 M2 高度层）。
/// - MSL：alt_m 直接比较区间；
/// - AGL：地面高度 ground_m 提供时换算 MSL（alt_min+ground .. alt_max+ground）；
///   ground 未知 → 保守视为在区间内（净空不确定，安全优先）。
pub(crate) fn zone_contains_at(z: &Zone, p: &Geo, alt_m: f64, ground_m: Option<f64>) -> bool {
    if !zone_contains(z, p) {
        return false;
    }
    let (lo, hi) = match z.height_semantics {
        HeightSemantics::Msl => (z.alt_min_m, z.alt_max_m),
        HeightSemantics::Agl => match ground_m {
            Some(g) => (z.alt_min_m + g, z.alt_max_m + g),
            None => return true,
        },
    };
    alt_m >= lo && alt_m <= hi
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIN_JSON: &str = r#"{
        "schema_version": "0.20",
        "mission": {
            "start": {"lon": 116.30, "lat": 39.90, "alt_m": 500},
            "target": {"lon": 117.10, "lat": 40.20, "alt_m": 1000}
        }
    }"#;

    #[test]
    fn parse_minimal_input() {
        let input = Input::from_json_str(MIN_JSON).unwrap();
        assert_eq!(input.schema_version, "0.20");
        assert_eq!(input.mission.start.lon, 116.30);
        assert!(input.mission.vehicles.is_empty());
        assert!(validate(&input).is_ok());
    }

    #[test]
    fn unknown_field_is_malformed() {
        let s = r#"{"schema_version":"0.20","mission":{"start":{"lon":1,"lat":2,"alt_m":0},"target":{"lon":3,"lat":4,"alt_m":0}},"bogus":1}"#;
        let err = Input::from_json_str(s).unwrap_err();
        assert!(matches!(err, AppError::Json(_)));
    }

    #[test]
    fn degenerate_a_equals_b_rejected() {
        let s = r#"{"schema_version":"0.20","mission":{"start":{"lon":116.0,"lat":39.0,"alt_m":0},"target":{"lon":116.0,"lat":39.0,"alt_m":0}}}"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::DegenerateStartEqualsTarget)) => {}
            other => panic!("expected degenerate, got {other:?}"),
        }
    }

    #[test]
    fn target_in_no_fly_rejected() {
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                "target":{"lon":116.5,"lat":39.9,"alt_m":0},
                "no_fly_zones":[{"id":"nf1","zone_type":"no_fly",
                    "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10},
                    "alt_min_m":0,"alt_max_m":10000}]
            }
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::TargetInNoFly)) => {}
            other => panic!("expected target_in_no_fly, got {other:?}"),
        }
    }

    #[test]
    fn radar_overlap_no_fly_rejected() {
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                "target":{"lon":117.0,"lat":40.0,"alt_m":0},
                "no_fly_zones":[{"id":"nf1","zone_type":"no_fly",
                    "shape":"polygon","geometry":{"vertices":[[116.0,39.5],[116.5,39.5],[116.5,40.0],[116.0,40.0]]},
                    "alt_min_m":0,"alt_max_m":10000}],
                "red_forces":{"radars":[{"id":"r1","lon":116.25,"lat":39.75,"radar_type":"tracking","radius_km":100}]}
            }
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::RadarOverlapNoFly)) => {}
            other => panic!("expected radar_overlap, got {other:?}"),
        }
    }

    #[test]
    fn vehicles_multi_input() {
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":116.30,"lat":39.90,"alt_m":500},
                "target":{"lon":117.10,"lat":40.20,"alt_m":1000},
                "vehicles":[
                    {"id":"v1",
                     "profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                                "min_turn_radius_m":12000,"max_bank_deg":30},
                     "start_pose":{"lon":116.30,"lat":39.90,"heading_deg":90,"alt_m":500}},
                    {"id":"v2",
                     "profile":{"aircraft_type":"ROTORCRAFT","cruise_speed_mps":100},
                     "start_pose":{"lon":116.40,"lat":39.80,"heading_deg":0,"alt_m":300}}
                ]
            }
        }"#;
        let input = Input::from_json_str(s).unwrap();
        assert_eq!(input.mission.vehicles.len(), 2);
        assert!(validate(&input).is_ok());
    }

    #[test]
    fn duplicate_vehicle_id_rejected() {
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":116.30,"lat":39.90,"alt_m":500},
                "target":{"lon":117.10,"lat":40.20,"alt_m":1000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING"},"start_pose":{"lon":116.3,"lat":39.9,"alt_m":500}},
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING"},"start_pose":{"lon":116.4,"lat":39.8,"alt_m":500}}
                ]
            }
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::VehicleParamsInconsistent)) => {}
            other => panic!("expected dup id, got {other:?}"),
        }
    }

    #[test]
    fn a6_physical_self_consistency() {
        // 250 m/s @ 30° bank → r_min ≈ 11045m；输入 5000m 应拒绝
        let r_min = DefaultParams::physical_turn_radius_m(250.0, 30.0);
        assert!((r_min - 11_045.0).abs() < 10.0, "r_min={r_min}");
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":116.30,"lat":39.90,"alt_m":500},
                "target":{"lon":117.10,"lat":40.20,"alt_m":1000},
                "vehicles":[
                    {"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                                "max_bank_deg":30,"min_turn_radius_m":5000},
                     "start_pose":{"lon":116.3,"lat":39.9,"alt_m":500}}
                ]
            }
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::VehicleParamsInconsistent)) => {}
            other => panic!("expected A6 reject, got {other:?}"),
        }
    }

    #[test]
    fn zone_contains_at_msl_band() {
        let z = Zone {
            id: "z1".into(),
            zone_type: ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [115.0, 39.0],
                radius_km: 10.0,
            },
            alt_min_m: 0.0,
            alt_max_m: 2000.0,
            height_semantics: HeightSemantics::Msl,
        };
        let p = Geo::new(115.0, 39.0).unwrap();
        assert!(zone_contains_at(&z, &p, 500.0, None));
        assert!(zone_contains_at(&z, &p, 2000.0, None));
        assert!(!zone_contains_at(&z, &p, 3000.0, None));
        // 水平外 → false 不论高度
        let q = Geo::new(116.5, 39.0).unwrap();
        assert!(!zone_contains_at(&z, &q, 500.0, None));
    }

    #[test]
    fn zone_contains_at_agl_conversion() {
        let z = Zone {
            id: "z2".into(),
            zone_type: ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [115.0, 39.0],
                radius_km: 10.0,
            },
            alt_min_m: 0.0,
            alt_max_m: 100.0,
            height_semantics: HeightSemantics::Agl,
        };
        let p = Geo::new(115.0, 39.0).unwrap();
        // 地面 500m：区间换算 [500, 600]
        assert!(zone_contains_at(&z, &p, 550.0, Some(500.0)));
        assert!(!zone_contains_at(&z, &p, 700.0, Some(500.0)));
        assert!(!zone_contains_at(&z, &p, 400.0, Some(500.0)));
        // ground 未知 → 保守在区间内
        assert!(zone_contains_at(&z, &p, 9000.0, None));
    }

    #[test]
    fn point_in_polygon_basic() {
        let poly = [[116.0, 39.5], [116.5, 39.5], [116.5, 40.0], [116.0, 40.0]];
        let inside = Geo::new(116.25, 39.75).unwrap();
        let outside = Geo::new(117.0, 41.0).unwrap();
        assert!(point_in_polygon(&inside, &poly));
        assert!(!point_in_polygon(&outside, &poly));
    }

    #[test]
    fn default_params_merge() {
        let d = DefaultParams::default();
        let o = ParamsOverride {
            radar_inflation: Some(1.5),
            ..Default::default()
        };
        let m = d.merge(&o);
        assert_eq!(m.radar_inflation, 1.5);
        assert_eq!(m.los_mask_coef, 0.08);
    }
}

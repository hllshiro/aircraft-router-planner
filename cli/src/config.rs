//! 输入/输出 JSON 契约（技术方案 4.2.1 / 4.5 + Phase 1；v0.21 契约统一与飞行器化）。
//!
//! - 输入：顶层 `aircraft[]`（逐机显式 start/target/profile/weapon，mission 包裹层已拍平）+ 红方雷达、三区、地形、参数覆盖；
//! - 输出：`status` 四态 + 错误体 + 飞行器结果 + 统计；
//! - `InputValidator`：畸形/退化输入在解析层即拦截 → `input_invalid` + 原因码。
//!
//! 严格契约：`deny_unknown_fields`（未知字段 = 畸形，属 MalformedJson）；
//! `zone_type` 不由 JSON 提供，由解析层按所属数组注入（三数组是类型唯一标记）。

use crate::coord::Geo;
use crate::error::{AppError, InputInvalidReason};
use schemars::JsonSchema;
use serde::{Deserialize, Serialize};

// ==================== 输入契约 ====================

/// 任务输入（顶层）。未知字段 → 畸形。
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Input {
    /// 飞行器数组（必填非空；空数组 → missing_aircraft）
    pub aircraft: Vec<AircraftInput>,
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
    pub parameters: ParamsOverride,
}

/// 飞行器（固定翼 / 旋翼；十三轮共识多机契约）：profile + 起点 + 目标 + 任务场景。
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AircraftInput {
    pub id: String,
    /// 机型性能参数（整段省略 → 缺省固定翼占位）
    #[serde(default)]
    pub profile: AircraftProfile,
    /// 起点（必填）
    pub start: Waypoint,
    /// 目标点（必填）
    pub target: Waypoint,
    /// 中途必经点（Phase 4 M5 每机独立序列）：start → mid[0..] → target。
    /// 分段 FMM（共享代价场）→ 拼接 → 整路径平滑复验。alt_m 为垂直剖面分段锚点（起→必经点→终点按段内比例插值）。
    #[serde(default)]
    pub mid_waypoints: Vec<Waypoint>,
    /// 武器（出现即启用；缺省 = 点目标语义）
    #[serde(default)]
    pub weapon: Option<Weapon>,
}

/// 航路点（经纬度 + 高程，米；高程 = MSL 或椭球高，随 crs.vertical）。
#[derive(Debug, Clone, Copy, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Waypoint {
    pub lon: f64,
    pub lat: f64,
    pub alt_m: f64,
}

impl Waypoint {
    pub fn to_geo(&self) -> Result<Geo, AppError> {
        Geo::new(self.lon, self.lat)
            .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))
    }
}

/// 飞行器性能参数集（八轮共识：输入显式提供，缺省落默认参数表占位，缺参 fail-fast）。
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AircraftProfile {
    pub aircraft_type: AircraftType,
    /// 巡航速度 m/s（或 speed_range 二选一）
    #[serde(default)]
    pub cruise_speed_mps: Option<f64>,
    /// 速度范围 [v_min, v_max] m/s（十轮主管裁决：速度为核心输入）
    #[serde(default)]
    pub speed_range_mps: Option<[f64; 2]>,
    /// 最小转弯半径 m（缺省默认参数表；A6 自洽：r_min ≥ v²/(g·tan φ_max)，
    /// 2026-08-07 主管放宽：显式半径信任，转弯段降速实现，不再按巡航物理下限拒）
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
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "SCREAMING_SNAKE_CASE")]
pub enum AircraftType {
    FixedWing,
    Rotorcraft,
}

impl Default for AircraftProfile {
    /// 缺省机型配置（固定翼 + 全占位 None；solver 单机兜底构造）。
    fn default() -> Self {
        Self {
            aircraft_type: AircraftType::FixedWing,
            cruise_speed_mps: None,
            speed_range_mps: None,
            min_turn_radius_m: None,
            max_climb_angle_deg: None,
            max_bank_deg: None,
            ceiling_m: None,
        }
    }
}

/// 红方部署。
#[derive(Debug, Clone, Deserialize, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct RedForces {
    #[serde(default)]
    pub radars: Vec<Radar>,
}

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Radar {
    pub id: String,
    pub lon: f64,
    pub lat: f64,
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

/// 禁飞/限飞/非地形障碍物（几何语义同禁飞区硬阈值，4.2.1/4.5）。
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Zone {
    pub id: String,
    /// 区域类型。**不由 JSON 提供**（`deny_unknown_fields` 会拒绝 zone_type 键）；
    /// 由 `Input::from_json_str` 按所属数组注入（no_fly_zones→NoFly / restricted_zones→Restricted / obstacles→Obstacle）。
    #[serde(skip)]
    pub zone_type: ZoneType,
    #[serde(flatten)]
    pub shape: ZoneShape,
    /// 高度区间下界（MSL）。
    /// 仅 Restricted（限飞区）使用——NoFly/Obstacle 全高度禁入，**不需要高度范围**，
    /// 可省略（省略 = 全高度；2026-08-12 主管：禁飞区无高度范围）。
    #[serde(default)]
    pub alt_min_m: Option<f64>,
    /// 高度区间上界（同 alt_min_m：仅 Restricted 使用，NoFly/Obstacle 可省略）。
    #[serde(default)]
    pub alt_max_m: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Default, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum ZoneType {
    #[default]
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

#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(
    deny_unknown_fields,
    tag = "shape",
    content = "geometry",
    rename_all = "snake_case"
)]
pub enum ZoneShape {
    Circle { center: [f64; 2], radius_km: f64 },
    Polygon { vertices: Vec<[f64; 2]> },
}

/// 地形配置（4.2.4 默认场景 / 4.2.5 内置契约）。
#[derive(Debug, Clone, Deserialize, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TerrainConfig {
    /// none（默认，海拔 0 平面）/ builtin（内置数据包）/ path（外部文件）
    #[serde(default)]
    pub source: TerrainSourceType,
    #[serde(default)]
    pub path: Option<String>,
    /// 海岸掩膜文件（GSHHG 3 态；None 时自动探测默认掩膜）
    #[serde(default)]
    pub mask_path: Option<String>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum TerrainSourceType {
    #[default]
    None,
    Builtin,
    Path,
}

/// 武器类型（2026-08-12 主管定案：空空导弹 / 空地导弹 / 航空炸弹；2026-08-19 起
/// weapon 出现即启用、weapon_type 必填）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum WeaponType {
    /// 空空导弹（AAM）
    Aam,
    /// 空地导弹（AGM）
    Agm,
    /// 航空炸弹（JDAM）
    Bomb,
}

impl WeaponType {
    /// 该类型默认射程 [Rmin, Rmax] km（主管 2026-08-12：按类型设置默认值；
    /// 沿用占位表 aam_medium / asm_air_ground / jdam，docs/02 §3.6）。
    pub fn default_range_km(self) -> [f64; 2] {
        match self {
            WeaponType::Aam => [5.0, 40.0],
            WeaponType::Agm => [3.0, 120.0],
            WeaponType::Bomb => [1.0, 15.0],
        }
    }
}

/// 武器（2026-08-12 主管定案语义不变：类型 + 射程 + 发射包线；2026-08-19 移入飞行器）。
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Weapon {
    /// 武器类型（aam / agm / bomb）。出现即启用，类型必填。
    pub weapon_type: WeaponType,
    /// 射程 [Rmin, Rmax] km。缺省 = 按 weapon_type 对应默认射程
    ///（aam [5,40] / agm [3,120] / bomb [1,15]）。
    #[serde(default)]
    pub range_km: Option<[f64; 2]>,
    /// 发射包线（航向/高度/速度窗；heading/alt 硬校验、speed 软校验）。
    #[serde(default)]
    pub envelope: Option<LaunchEnvelope>,
}

impl Weapon {
    /// 有效射程：显式 range_km 优先，否则按类型默认。
    pub fn effective_range_km(&self) -> Option<[f64; 2]> {
        Some(self.range_km.unwrap_or_else(|| self.weapon_type.default_range_km()))
    }
}

/// 发射包线（航向/高度/速度窗）。
#[derive(Debug, Clone, Deserialize, Default, JsonSchema)]
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
#[derive(Debug, Clone, Deserialize, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct ParamsOverride {
    #[serde(default)]
    pub radar_inflation: Option<f64>,
    /// 探测曲线形态字符串（"swerling1"/"exponential"/"linear" 不区分大小写；无效 → 默认 swerling1，
    /// 主管决策 2026-08-05：无外部参数或参数无效使用默认值）。
    #[serde(default)]
    pub detection_curve: Option<String>,
    #[serde(default)]
    pub p_cross: Option<f64>,
    #[serde(default)]
    pub suppression_delta: Option<f64>,
    /// 雷达探测概率代价系数（FMM 代价 ×(1+coef×p)；越大航路越倾向绕行躲避）
    #[serde(default)]
    pub radar_cost_coef: Option<f64>,
    #[serde(default)]
    pub los_mask_coef: Option<f64>,
}

#[derive(Debug, Clone, Copy, PartialEq, Eq, Deserialize, Default, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum DetectionCurve {
    /// Swerling I 模型（默认，2026-08-13 base_p 标定定案）：
    /// Pd(d) = exp(−VT/(1 + SNR₀·(R_eff/d)⁴))，Pfa=1e-6 → VT=13.8155，
    /// R_eff = 90% 探测距离（SNR₀ = VT/(−ln base_p) − 1 = 130.1 ≈ 21.1 dB）。
    #[default]
    Swerling1,
    /// 指数衰减（R⁻⁴ 简化近似，保留供显式选择）。
    Exponential,
    Linear,
}

// ==================== 默认参数表（三轮共识；占位值 Phase 0 校准后生效） ====================

/// 默认参数表（编译期常量，输入未提供时使用；全部为占位值，Phase 0 校准）。
#[derive(Debug, Clone)]
pub struct DefaultParams {
    /// 雷达膨胀系数（球体半径 = 实际 × 系数，>1）
    pub radar_inflation: f64,
    /// 探测概率衰减形态（Swerling I 为默认：典型监视雷达模型标定，2026-08-13）
    pub detection_curve: DetectionCurve,
    /// 穿越阈值 P_cross（占位保守低值，Phase 0 定值；未标定不得声称实现）
    pub p_cross: f64,
    /// 探测概率基准 base_p（2026-08-13 标定：Swerling I 下有效半径 R_eff 处探测概率 =
    /// 0.9，即 R_eff = 90% 探测距离；中心概率由模型推导 ≈1.0）。
    /// 仅内部默认参数表字段，不进 ParamsOverride 外部覆盖（与 p_cross 解耦定案）。
    pub base_p: f64,
    /// 压制修正因子 δ（探测距离 × (1−δ)，占位）
    pub suppression_delta: f64,
    /// 雷达探测概率代价系数（FMM 代价 ×(1+coef×(p+geom))；>0，默认 200：
    /// p = 几何并集探测概率，geom = 有效半径内归一化深穿惩罚 (1−u)。
    /// 中心 ×(1+200×(0.1+1))≈×221、0.8R 处 ×(1+200×0.15)≈×31——
    /// 穿探测区（含并排双雷达重叠/间隙边缘）明确绕行，探测区外无几何项。
    /// 主管 2026-08-06：并排双雷达不得直穿探测区（即使 P_cross 调高）。
    pub radar_cost_coef: f64,
    /// LOS mask 系数（默认 0.05–0.1 区间内取 0.08；守保守口径不取 0，十二轮共识）
    pub los_mask_coef: f64,
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
            detection_curve: DetectionCurve::Swerling1,
            p_cross: 0.1,
            base_p: 0.9, // Swerling I 标定：R_eff = 90% 探测距离（方案 A，2026-08-13）
            suppression_delta: 0.5,
            radar_cost_coef: 200.0,
            los_mask_coef: 0.08,
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
    /// 合并参数覆盖（覆盖优先，未覆盖用默认）。
    /// 数值参数无效（非有限/出域）与无效曲线字符串 → 回落默认（主管决策 2026-08-05：
    /// 无外部参数或参数无效使用默认值；回落由 solver 记入 stats.degradations）。
    pub fn merge(&self, o: &ParamsOverride) -> DefaultParams {
        let mut d = self.clone();
        // 合法域与旧契约一致（radar_inflation>1 为膨胀、p_cross/los_mask_coef∈[0,1]、
        // suppression_delta∈[0,1)）；出域/非有限 → 回落默认。
        if let Some(v) = o.radar_inflation.filter(|v| v.is_finite() && *v > 1.0) {
            d.radar_inflation = v;
        }
        if let Some(v) = o
            .p_cross
            .filter(|v| v.is_finite() && *v >= 0.0 && *v <= 1.0)
        {
            d.p_cross = v;
        }
        if let Some(v) = o
            .suppression_delta
            .filter(|v| v.is_finite() && *v >= 0.0 && *v < 1.0)
        {
            d.suppression_delta = v;
        }
        if let Some(v) = o.radar_cost_coef.filter(|v| v.is_finite() && *v > 0.0) {
            d.radar_cost_coef = v;
        }
        if let Some(v) = o
            .los_mask_coef
            .filter(|v| v.is_finite() && *v >= 0.0 && *v <= 1.0)
        {
            d.los_mask_coef = v;
        }
        if let Some(s) = o.detection_curve.as_deref() {
            match s.to_ascii_lowercase().as_str() {
                "swerling1" => d.detection_curve = DetectionCurve::Swerling1,
                "exponential" => d.detection_curve = DetectionCurve::Exponential,
                "linear" => d.detection_curve = DetectionCurve::Linear,
                _ => {} // 无效 → 默认
            }
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

/// 输出 JSON（status 四态契约）。
#[derive(Debug, Clone, Serialize, JsonSchema)]
pub struct Output {
    pub status: String,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub error: Option<crate::error::ErrorBody>,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub elapsed_ms: Option<u64>,
    pub aircraft: Vec<AircraftOutput>,
    pub stats: Stats,
}

impl Output {
    pub fn success(elapsed_ms: u64) -> Self {
        Self {
            status: "success".into(),
            error: None,
            elapsed_ms: Some(elapsed_ms),
            aircraft: Vec::new(),
            stats: Stats::default(),
        }
    }

    pub fn failure(status: &str, error: crate::error::ErrorBody, elapsed_ms: u64) -> Self {
        Self {
            status: status.into(),
            error: Some(error),
            elapsed_ms: Some(elapsed_ms),
            aircraft: Vec::new(),
            stats: Stats::default(),
        }
    }
}

#[derive(Debug, Clone, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct AircraftOutput {
    pub id: String,
    /// planned / no_solution / degraded
    pub status: String,
    pub path: Vec<PathPoint>,
    pub distance_m: f64,
    pub warnings: Vec<String>,
}

#[derive(Debug, Clone, Copy, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct PathPoint {
    /// 经度（度，WGS84；主管决策 2026-08-05：输入输出坐标点均为经纬高定义，x=经度）
    pub x: f64,
    /// 纬度（度，WGS84；y=纬度）
    pub y: f64,
    /// MSL 几何高度（米；与 path.rs 高程口径一致）
    pub alt_m: f64,
}

#[derive(Debug, Clone, Default, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub struct Stats {
    pub fmm_ms: f64,
    pub los_checks: u64,
    pub degradations: Vec<String>,
}

// ==================== 解析与校验 ====================

impl Input {
    /// 从 JSON 字符串解析（畸形 JSON → input_invalid: malformed_json）。
    /// 解析后按所属数组注入 zone_type（三数组是类型唯一标记）。
    pub fn from_json_str(s: &str) -> Result<Self, AppError> {
        let mut input: Self = serde_json::from_str(s).map_err(AppError::Json)?;
        for z in &mut input.no_fly_zones {
            z.zone_type = ZoneType::NoFly;
        }
        for z in &mut input.restricted_zones {
            z.zone_type = ZoneType::Restricted;
        }
        for z in &mut input.obstacles {
            z.zone_type = ZoneType::Obstacle;
        }
        Ok(input)
    }
}

/// InputValidator 前置模块：解析后立即校验，退化输入不进入算法。
pub fn validate(input: &Input) -> Result<(), AppError> {
    // 逐机显式契约：aircraft 必填非空（空数组 → missing_aircraft）。
    if input.aircraft.is_empty() {
        return Err(AppError::InputInvalid(
            InputInvalidReason::MissingAircraft,
        ));
    }
    let mut ids = std::collections::HashSet::new();
    for a in &input.aircraft {
        if !ids.insert(a.id.clone()) {
            return Err(AppError::InputInvalid(
                InputInvalidReason::VehicleParamsInconsistent,
            ));
        }
        validate_aircraft(a)?;
        // 逐机：start/target 合法坐标 + 间距 ≥ 100m
        let start = a.start.to_geo()?;
        let target = a.target.to_geo()?;
        if start.distance_m(&target) < 100.0 {
            return Err(AppError::InputInvalid(
                InputInvalidReason::DegenerateStartEqualsTarget,
            ));
        }
        // 起点在禁飞区
        for z in &input.no_fly_zones {
            if zone_contains(z, &start) {
                return Err(AppError::InputInvalid(InputInvalidReason::TargetInNoFly));
            }
        }
        // 目标在禁飞区
        for z in &input.no_fly_zones {
            if zone_contains(z, &target) {
                return Err(AppError::InputInvalid(InputInvalidReason::TargetInNoFly));
            }
        }
        // 必经点在禁飞区（P5：必经点不可绕行 → fail-fast，禁飞区绝对禁入语义）。
        for wp in &a.mid_waypoints {
            let g = wp.to_geo()?;
            for z in &input.no_fly_zones {
                if zone_contains(z, &g) {
                    return Err(AppError::InputInvalid(
                        InputInvalidReason::MidWaypointInNoFly,
                    ));
                }
            }
        }
    }
    // 雷达 ∩ 禁飞区重叠（雷达位置在禁飞多边形内）
    for r in &input.red_forces.radars {
        let g = Geo::new(r.lon, r.lat)
            .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))?;
        for z in &input.no_fly_zones {
            if zone_contains(z, &g) {
                return Err(AppError::InputInvalid(
                    InputInvalidReason::RadarOverlapNoFly,
                ));
            }
        }
        if r.radius_km <= 0.0 {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    // 参数覆盖数值域：主管决策 2026-08-05（无外部参数或参数无效使用默认值）——
    // 不再 fail-fast，无效值由 DefaultParams::merge 回落默认，事实记入 stats.degradations。
    // Zone 高度区间（2026-08-12 主管：禁飞区无高度范围）：限飞区必须有
    // [alt_min, alt_max]（alt_min < alt_max，有限值）；禁飞/障碍全高度禁入，
    // 不要求 alt（可省略）。
    for z in input
        .no_fly_zones
        .iter()
        .chain(&input.restricted_zones)
        .chain(&input.obstacles)
    {
        validate_zone(z)?;
    }
    Ok(())
}

/// Zone 高度区间校验：Restricted 必须有 [alt_min, alt_max]（lo < hi，有限值）；
/// NoFly/Obstacle 全高度禁入，不要求 alt（None = 全高度，2026-08-12）。
fn validate_zone(z: &Zone) -> Result<(), AppError> {
    if z.zone_type == ZoneType::Restricted {
        match (z.alt_min_m, z.alt_max_m) {
            (Some(lo), Some(hi)) => {
                if !(lo.is_finite() && hi.is_finite() && lo < hi) {
                    return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
                }
            }
            _ => return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds)),
        }
    }
    Ok(())
}

fn validate_aircraft(a: &AircraftInput) -> Result<(), AppError> {
    let p = &a.profile;
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
            return Err(AppError::InputInvalid(
                InputInvalidReason::VehicleParamsInconsistent,
            ));
        }
    }
    // A6 物理自洽（十二轮共识，2026-08-07 主管放宽）：r_min ≥ v²/(g·tan φ_max)。
    // 速度非锁定——极端条件下转弯段可降速（v_turn = sqrt(r·g·tanφ)）实现小半径，
    // 显式 min_turn_radius_m 不再按巡航速度物理下限拒绝（solver 信任输入并输出降速
    // 提示；smooth_options_for 的 A6 有效下限 = min(phys, turn_radius) 恒满足）。
    // 仅保留正数防线：固定翼 r < 1m 无物理意义（旋翼机 r→0 合法，悬停原地转向）。
    if let (Some(_v), Some(_bank), Some(r)) = (speed, p.max_bank_deg, p.min_turn_radius_m) {
        if p.aircraft_type == AircraftType::FixedWing && r < 1.0 {
            return Err(AppError::InputInvalid(
                InputInvalidReason::VehicleParamsInconsistent,
            ));
        }
    }
    // 武器射程（2026-08-12 主管定案：类型 + 射程；W2-P3 逐机化——校验移至本函数）：
    // 显式 range_km 提供时校验有限且 lo < hi（倒置/非有限 → out_of_bounds）。
    if let Some(w) = &a.weapon {
        if let Some([lo, hi]) = w.range_km {
            if !(lo.is_finite() && hi.is_finite() && lo < hi) {
                return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
            }
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
pub(crate) fn point_in_polygon(p: &Geo, vertices: &[[f64; 2]]) -> bool {
    point_in_polygon_xy(p.lon, p.lat, vertices)
}

/// 点在多边形内（xy 经纬度平面近似，射线法；与 point_in_polygon 同口径）。
pub(crate) fn point_in_polygon_xy(px: f64, py: f64, vertices: &[[f64; 2]]) -> bool {
    if vertices.len() < 3 {
        return false;
    }
    let mut inside = false;
    let mut j = vertices.len() - 1;
    for i in 0..vertices.len() {
        let (xi, yi) = (vertices[i][0], vertices[i][1]);
        let (xj, yj) = (vertices[j][0], vertices[j][1]);
        if (yi > py) != (yj > py) && px < (xj - xi) * (py - yi) / (yj - yi) + xi {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// 轴对齐矩形与多边形相交（保守）：任一边相交 / 任一多边形顶点在矩形内 /
/// 任一矩形角点在多边形内。经纬度平面近似（与 point_in_polygon 同口径）。
///
/// 用于代价场墙光栅化——**中心点采样会漏掉 < 1 格的窄带**（多边形尖角/斜边附近
/// 无格中心命中），FMM 沿漏格穿过 → verify 精确几何（zone_segment_clearance_km）
/// 拒 → 平滑链全败 → 误报 no_solution（2026-08-11 zz_nosolution_case：禁飞区
/// 三角形西顶点窄带 ~30m，256 网格下 3.5 列无墙格，FMM 贴顶点 84m 穿入）。
pub(crate) fn rect_intersects_polygon(
    rx0: f64,
    ry0: f64,
    rx1: f64,
    ry1: f64,
    verts: &[[f64; 2]],
) -> bool {
    if verts.len() < 3 {
        return false;
    }
    // 1) 多边形任一顶点在矩形内
    for v in verts {
        if v[0] >= rx0 && v[0] <= rx1 && v[1] >= ry0 && v[1] <= ry1 {
            return true;
        }
    }
    // 2) 矩形角点任一在多边形内
    for (px, py) in [(rx0, ry0), (rx1, ry0), (rx0, ry1), (rx1, ry1)] {
        if point_in_polygon_xy(px, py, verts) {
            return true;
        }
    }
    // 3) 任一边与矩形 4 边相交（含端点接触，保守）
    let rect = [
        (rx0, ry0, rx1, ry0),
        (rx1, ry0, rx1, ry1),
        (rx1, ry1, rx0, ry1),
        (rx0, ry1, rx0, ry0),
    ];
    let n = verts.len();
    for i in 0..n {
        let (ax, ay) = (verts[i][0], verts[i][1]);
        let (bx, by) = (verts[(i + 1) % n][0], verts[(i + 1) % n][1]);
        for (cx, cy, dx, dy) in rect {
            if segs_intersect_plane(ax, ay, bx, by, cx, cy, dx, dy) {
                return true;
            }
        }
    }
    false
}

// ---- 线段-多边形几何净距（主管 2026-08-06：绕飞太贴边→考虑飞机机动，绕行需留转弯空间）----

/// 线段到 Zone 的水平最小净距（km）。0 = 段穿入或贴边界。
/// 多边形：端点在内为 0，否则取段到每条边的两线段最近距离的最小值；
/// 圆形：段到圆心最近距离 − 半径（≤0 为穿/贴）。经纬度平面近似（与 point_in_polygon 同口径）。
pub(crate) fn zone_segment_clearance_km(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    z: &Zone,
) -> f64 {
    match &z.shape {
        ZoneShape::Circle { center, radius_km } => match Geo::new(center[0], center[1]) {
            Ok(c) => (pt_seg_dist_km(lon1, lat1, lon2, lat2, c.lon, c.lat) - *radius_km).max(0.0),
            Err(_) => f64::MAX,
        },
        ZoneShape::Polygon { vertices } => {
            if vertices.len() < 3 {
                return f64::MAX;
            }
            let p1 = Geo::new(lon1, lat1).ok();
            let p2 = Geo::new(lon2, lat2).ok();
            if p1.as_ref().map_or(false, |g| point_in_polygon(g, vertices))
                || p2.as_ref().map_or(false, |g| point_in_polygon(g, vertices))
            {
                return 0.0;
            }
            let mut best = f64::MAX;
            let mut j = vertices.len() - 1;
            for i in 0..vertices.len() {
                let d = seg_seg_dist_km(
                    lon1,
                    lat1,
                    lon2,
                    lat2,
                    vertices[j][0],
                    vertices[j][1],
                    vertices[i][0],
                    vertices[i][1],
                );
                best = best.min(d);
                j = i;
            }
            best.max(0.0)
        }
    }
}

/// 点到线段最近距离（km；经纬度平面近似，经度按 lat0 余弦缩放）。
pub(crate) fn pt_seg_dist_km(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    plon: f64,
    plat: f64,
) -> f64 {
    let lat0 = lat1.to_radians();
    let kx = 111.320 * lat0.cos();
    let ky = 111.0;
    let (ax, ay) = (lon1 * kx, lat1 * ky);
    let (bx, by) = (lon2 * kx, lat2 * ky);
    let (px, py) = (plon * kx, plat * ky);
    let (vx, vy) = (bx - ax, by - ay);
    let (wx, wy) = (px - ax, py - ay);
    let l2 = vx * vx + vy * vy;
    let t = if l2 > 0.0 {
        ((wx * vx + wy * vy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (qx, qy) = (ax + t * vx, ay + t * vy);
    ((px - qx).powi(2) + (py - qy).powi(2)).sqrt()
}

/// 两线段最近距离（km；平面近似）。相交（含端点/共线接触）→ 0；
/// 不相交时最近点在 4 个"端点到对段"垂足或端点之一，取四者最小。
fn seg_seg_dist_km(
    ax1: f64,
    ay1: f64,
    ax2: f64,
    ay2: f64,
    bx1: f64,
    by1: f64,
    bx2: f64,
    by2: f64,
) -> f64 {
    let lat0 = ((ay1 + by1) / 2.0).to_radians();
    let kx = 111.320 * lat0.cos();
    let ky = 111.0;
    let (ax1, ay1) = (ax1 * kx, ay1 * ky);
    let (ax2, ay2) = (ax2 * kx, ay2 * ky);
    let (bx1, by1) = (bx1 * kx, by1 * ky);
    let (bx2, by2) = (bx2 * kx, by2 * ky);
    if segs_intersect_plane(ax1, ay1, ax2, ay2, bx1, by1, bx2, by2) {
        return 0.0;
    }
    let d1 = pt_seg_plane(ax1, ay1, bx1, by1, bx2, by2);
    let d2 = pt_seg_plane(ax2, ay2, bx1, by1, bx2, by2);
    let d3 = pt_seg_plane(bx1, by1, ax1, ay1, ax2, ay2);
    let d4 = pt_seg_plane(bx2, by2, ax1, ay1, ax2, ay2);
    d1.min(d2).min(d3).min(d4)
}

/// 平面点到线段距离（米，已投影）。
fn pt_seg_plane(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    let (vx, vy) = (bx - ax, by - ay);
    let (wx, wy) = (px - ax, py - ay);
    let l2 = vx * vx + vy * vy;
    let t = if l2 > 0.0 {
        ((wx * vx + wy * vy) / l2).clamp(0.0, 1.0)
    } else {
        0.0
    };
    let (qx, qy) = (ax + t * vx, ay + t * vy);
    ((px - qx).powi(2) + (py - qy).powi(2)).sqrt()
}

/// 平面线段相交（含端点/共线接触——保守，接触即相交）。
pub(crate) fn segs_intersect_plane(
    ax: f64,
    ay: f64,
    bx: f64,
    by: f64,
    cx: f64,
    cy: f64,
    dx: f64,
    dy: f64,
) -> bool {
    let d1 = cross2(cx - ax, cy - ay, bx - ax, by - ay);
    let d2 = cross2(dx - ax, dy - ay, bx - ax, by - ay);
    let d3 = cross2(ax - cx, ay - cy, dx - cx, dy - cy);
    let d4 = cross2(bx - cx, by - cy, dx - cx, dy - cy);
    if ((d1 > 0.0 && d2 < 0.0) || (d1 < 0.0 && d2 > 0.0))
        && ((d3 > 0.0 && d4 < 0.0) || (d3 < 0.0 && d4 > 0.0))
    {
        return true;
    }
    if d1 == 0.0 && on_seg2(cx, cy, ax, ay, bx, by) {
        return true;
    }
    if d2 == 0.0 && on_seg2(dx, dy, ax, ay, bx, by) {
        return true;
    }
    if d3 == 0.0 && on_seg2(ax, ay, cx, cy, dx, dy) {
        return true;
    }
    if d4 == 0.0 && on_seg2(bx, by, cx, cy, dx, dy) {
        return true;
    }
    false
}

fn cross2(ax: f64, ay: f64, bx: f64, by: f64) -> f64 {
    ax * by - ay * bx
}

fn on_seg2(px: f64, py: f64, ax: f64, ay: f64, bx: f64, by: f64) -> bool {
    const EPS: f64 = 1e-9;
    px >= ax.min(bx) - EPS
        && px <= ax.max(bx) + EPS
        && py >= ay.min(by) - EPS
        && py <= ay.max(by) + EPS
}

/// 点是否在 Zone 内且高度落入禁入区间 [alt_min, alt_max]（Phase 4 M2 高度层）。
/// - NoFly/Obstacle（全高度禁入）或未提供高度区间 → 几何命中即禁入；
/// - Restricted：高度一律 MSL 直比（v0.21 起无 AGL 语义）。
pub(crate) fn zone_contains_at(z: &Zone, p: &Geo, alt_m: f64) -> bool {
    if !zone_contains(z, p) {
        return false;
    }
    let Some(min) = z.alt_min_m else { return true };
    let Some(max) = z.alt_max_m else { return true };
    alt_m >= min && alt_m <= max
}

#[cfg(test)]
mod tests {
    use super::*;

    const MIN_JSON: &str = r#"{
        "aircraft": [
            {"id": "a1",
             "start": {"lon": 116.30, "lat": 39.90, "alt_m": 500},
             "target": {"lon": 117.10, "lat": 40.20, "alt_m": 1000}}
        ]
    }"#;

    #[test]
    fn parse_minimal_input() {
        let input = Input::from_json_str(MIN_JSON).unwrap();
        assert_eq!(input.aircraft.len(), 1);
        assert_eq!(input.aircraft[0].start.lon, 116.30);
        assert!(validate(&input).is_ok());
    }

    #[test]
    fn unknown_field_is_malformed() {
        let s = r#"{"aircraft":[{"id":"a1","start":{"lon":1,"lat":2,"alt_m":0},"target":{"lon":3,"lat":4,"alt_m":0}}],"bogus":1}"#;
        let err = Input::from_json_str(s).unwrap_err();
        assert!(matches!(err, AppError::Json(_)));
    }

    #[test]
    fn aircraft_empty_missing() {
        // W2-P1：aircraft 空数组 → missing_aircraft（逐机显式契约下无任务可规划）。
        let s = r#"{"aircraft":[]}"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::MissingAircraft)) => {}
            other => panic!("expected missing_aircraft, got {other:?}"),
        }
    }

    #[test]
    fn zone_segment_clearance_km_basic() {
        // 多边形：段穿内部 → 0；段平行于边外 0.1°≈11.1km；
        // 圆：段过圆心 → 0；段距圆心 20km（半径 10km）→ 净距 10km。
        let poly = Zone {
            id: "p".into(),
            zone_type: ZoneType::NoFly,
            shape: ZoneShape::Polygon {
                vertices: vec![[116.0, 39.5], [116.5, 39.5], [116.5, 40.0], [116.0, 40.0]],
            },
            alt_min_m: Some(0.0),
            alt_max_m: Some(10000.0),
        };
        let c = zone_segment_clearance_km(116.2, 39.6, 116.3, 39.9, &poly);
        assert_eq!(c, 0.0, "段穿内部 → 净距 0, got {c}");
        let c = zone_segment_clearance_km(116.1, 40.1, 116.4, 40.1, &poly);
        assert!((c - 11.1).abs() < 1.0, "平行净距 ~11.1km, got {c}");
        let circ = Zone {
            id: "c".into(),
            zone_type: ZoneType::NoFly,
            shape: ZoneShape::Circle {
                center: [116.25, 39.75],
                radius_km: 10.0,
            },
            alt_min_m: Some(0.0),
            alt_max_m: Some(10000.0),
        };
        let c = zone_segment_clearance_km(116.25, 39.70, 116.25, 39.80, &circ);
        assert!(c < 0.01, "段过圆心 → 净距 0, got {c}");
        let c = zone_segment_clearance_km(116.25, 39.30, 116.25, 39.40, &circ);
        // 段距圆心最近 0.35°≈38.85km − 半径 10km = 28.85km
        assert!((c - 28.85).abs() < 1.5, "净距 ~28.85km, got {c}");
    }

    #[test]
    fn degenerate_a_equals_b_rejected() {
        let s = r#"{"aircraft":[{"id":"a1","start":{"lon":116.0,"lat":39.0,"alt_m":0},"target":{"lon":116.0,"lat":39.0,"alt_m":0}}]}"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::DegenerateStartEqualsTarget)) => {}
            other => panic!("expected degenerate, got {other:?}"),
        }
    }

    #[test]
    fn target_in_no_fly_rejected() {
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":116.5,"lat":39.9,"alt_m":0}}
            ],
            "no_fly_zones":[{"id":"nf1",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10},
                "alt_min_m":0,"alt_max_m":10000}]
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
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0}}
            ],
            "no_fly_zones":[{"id":"nf1",
                "shape":"polygon","geometry":{"vertices":[[116.0,39.5],[116.5,39.5],[116.5,40.0],[116.0,40.0]]},
                "alt_min_m":0,"alt_max_m":10000}],
            "red_forces":{"radars":[{"id":"r1","lon":116.25,"lat":39.75,"radius_km":100}]}
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::RadarOverlapNoFly)) => {}
            other => panic!("expected radar_overlap, got {other:?}"),
        }
    }

    #[test]
    fn aircraft_target_outside_no_fly_ok() {
        // P5-M1（主管 2026-08-12 案例）逐机化：aircraft.target 显式给出圆外目标
        // → validate 通过；对照：target 在圆内（49.37km < 50km）→ target_in_no_fly。
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                 "target":{"lon":114.26335909078654,"lat":41.99101176729852,"alt_m":3000}}
            ],
            "no_fly_zones":[{"id":"nf1",
                "shape":"circle","geometry":{"center":[116.54581607527983,39.90085583451849],"radius_km":50},
                "alt_min_m":0,"alt_max_m":12000}]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        if let Err(e) = validate(&input) {
            panic!("expected ok (target outside no-fly), got {e:?}");
        }
        // 对照：target（圆内）被拒
        let s2 = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.9,"lat":39.8,"alt_m":3000},
                 "target":{"lon":116.8,"lat":40.3,"alt_m":3000}}
            ],
            "no_fly_zones":[{"id":"nf1",
                "shape":"circle","geometry":{"center":[116.54581607527983,39.90085583451849],"radius_km":50},
                "alt_min_m":0,"alt_max_m":12000}]
        }"#;
        let input2 = Input::from_json_str(s2).unwrap();
        match validate(&input2) {
            Err(AppError::InputInvalid(InputInvalidReason::TargetInNoFly)) => {}
            other => panic!("expected target_in_no_fly, got {other:?}"),
        }
    }

    #[test]
    fn mid_waypoint_in_no_fly_rejected() {
        // P5-M2：必经点在禁飞区 → fail-fast（必经点不可绕行）
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0},
                 "mid_waypoints":[{"lon":116.5,"lat":39.9,"alt_m":0}]}
            ],
            "no_fly_zones":[{"id":"nf1",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10},
                "alt_min_m":0,"alt_max_m":10000}]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::MidWaypointInNoFly)) => {}
            other => panic!("expected mid_waypoint_in_no_fly, got {other:?}"),
        }
        // 必经点在禁飞区外 → 通过
        let s2 = s.replace("lon\":116.5,\"lat\":39.9", "lon\":116.0,\"lat\":39.0");
        let input2 = Input::from_json_str(&s2).unwrap();
        if let Err(e) = validate(&input2) {
            panic!("expected ok, got {e:?}");
        }
    }

    #[test]
    fn no_fly_zone_without_alt_range_ok() {
        // 2026-08-12 主管：禁飞区本身不允许进入、没有高度范围——NoFly 可省略
        // alt_min/alt_max（全高度禁入），解析与校验都必须通过。
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0}}
            ],
            "no_fly_zones":[{"id":"nf1",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10}}]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        assert_eq!(input.no_fly_zones[0].zone_type, ZoneType::NoFly);
        assert!(input.no_fly_zones[0].alt_min_m.is_none());
        assert!(input.no_fly_zones[0].alt_max_m.is_none());
        if let Err(e) = validate(&input) {
            panic!("expected ok (no_fly without alt), got {e:?}");
        }
    }

    #[test]
    fn restricted_zone_requires_alt_range() {
        // 限飞区必须有 [alt_min, alt_max]；缺失 → out_of_bounds 拒绝。
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0}}
            ],
            "restricted_zones":[{"id":"rz1",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10}}]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds)) => {}
            other => panic!("expected out_of_bounds, got {other:?}"),
        }
    }

    #[test]
    fn zone_injection_by_array() {
        // 三数组各解析一条 → 按所属数组注入 zone_type（三数组是类型唯一标记）。
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0}}
            ],
            "no_fly_zones":[{"id":"nf1",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10}}],
            "restricted_zones":[{"id":"rz1",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10},
                "alt_min_m":0,"alt_max_m":5000}],
            "obstacles":[{"id":"ob1",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10}}]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        assert_eq!(input.no_fly_zones[0].zone_type, ZoneType::NoFly);
        assert_eq!(input.restricted_zones[0].zone_type, ZoneType::Restricted);
        assert_eq!(input.obstacles[0].zone_type, ZoneType::Obstacle);
    }

    #[test]
    fn zone_type_key_rejected() {
        // zone_type 键不再属于 JSON 契约（由解析层按数组注入）→ 畸形。
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0}}
            ],
            "no_fly_zones":[{"id":"nf1","zone_type":"no_fly",
                "shape":"circle","geometry":{"center":[116.5,39.9],"radius_km":10}}]
        }"#;
        let err = Input::from_json_str(s).unwrap_err();
        assert!(matches!(err, AppError::Json(_)));
    }

    #[test]
    fn profile_omitted_defaults() {
        // 无 profile → AircraftProfile::default()（缺省固定翼占位）。
        let input = Input::from_json_str(MIN_JSON).unwrap();
        assert_eq!(input.aircraft[0].profile.aircraft_type, AircraftType::FixedWing);
        assert!(input.aircraft[0].profile.cruise_speed_mps.is_none());
        assert!(input.aircraft[0].mid_waypoints.is_empty());
        assert!(input.aircraft[0].weapon.is_none());
    }

    #[test]
    fn weapon_in_aircraft_parsed() {
        // 武器移入飞行器：weapon 出现即启用、weapon_type 必填；缺 weapon_type → 畸形。
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":116.30,"lat":39.90,"alt_m":500},
                 "target":{"lon":117.10,"lat":40.20,"alt_m":1000},
                 "weapon":{"weapon_type":"agm","range_km":[2.0,80.0],
                           "envelope":{"heading_deg":[0,360],"alt_m":[0,5000],"speed_mps":[50,300]}}}
            ]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        let w = input.aircraft[0].weapon.as_ref().expect("weapon 应解析");
        assert_eq!(w.weapon_type, WeaponType::Agm);
        assert_eq!(w.effective_range_km(), Some([2.0, 80.0]));
        assert!(w.envelope.is_some());
        if let Err(e) = validate(&input) {
            panic!("expected ok, got {e:?}");
        }
        // 缺 weapon_type → malformed_json（weapon 出现即要求类型必填）
        let bad = r#"{
            "aircraft":[
                {"id":"a1",
                 "start":{"lon":116.30,"lat":39.90,"alt_m":500},
                 "target":{"lon":117.10,"lat":40.20,"alt_m":1000},
                 "weapon":{"range_km":[2.0,80.0]}}
            ]
        }"#;
        let err = Input::from_json_str(bad).unwrap_err();
        assert!(matches!(err, AppError::Json(_)));
    }

    #[test]
    fn weapon_semantics_type_and_default_range() {
        // 2026-08-12 主管定案：武器语义 = 类型 + 射程；weapon_type 必填（出现即启用）；
        // 射程缺省 = 按类型默认值（aam [5,40] / agm [3,120] / bomb [1,15]）。
        let s = r#"{
            "aircraft":[
                {"id":"a_aam",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0},
                 "weapon":{"weapon_type":"aam"}},
                {"id":"a_agm",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0},
                 "weapon":{"weapon_type":"agm","range_km":[2.0,80.0]}},
                {"id":"a_bomb",
                 "start":{"lon":115.0,"lat":39.0,"alt_m":0},
                 "target":{"lon":117.0,"lat":40.0,"alt_m":0},
                 "weapon":{"weapon_type":"bomb"}}
            ]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        let ws: Vec<&Weapon> = input
            .aircraft
            .iter()
            .filter_map(|a| a.weapon.as_ref())
            .collect();
        assert_eq!(ws.len(), 3);
        // 缺省射程按类型
        assert_eq!(ws[0].effective_range_km(), Some([5.0, 40.0]));
        // 显式射程优先
        assert_eq!(ws[1].effective_range_km(), Some([2.0, 80.0]));
        assert_eq!(ws[2].effective_range_km(), Some([1.0, 15.0]));
        if let Err(e) = validate(&input) {
            panic!("expected ok, got {e:?}");
        }
        // 倒置射程 → out_of_bounds（W2-P3：逐机 validate_aircraft 校验）
        let bad = s.replace("[2.0,80.0]", "[80.0,2.0]");
        let input2 = Input::from_json_str(&bad).unwrap();
        match validate(&input2) {
            Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds)) => {}
            other => panic!("expected out_of_bounds, got {other:?}"),
        }
    }

    #[test]
    fn aircraft_multi_input() {
        let s = r#"{
            "aircraft":[
                {"id":"a1",
                 "profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                            "min_turn_radius_m":12000,"max_bank_deg":30},
                 "start":{"lon":116.30,"lat":39.90,"alt_m":500},
                 "target":{"lon":117.10,"lat":40.20,"alt_m":1000}},
                {"id":"a2",
                 "profile":{"aircraft_type":"ROTORCRAFT","cruise_speed_mps":100},
                 "start":{"lon":116.40,"lat":39.80,"alt_m":300},
                 "target":{"lon":117.10,"lat":40.20,"alt_m":1000}}
            ]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        assert_eq!(input.aircraft.len(), 2);
        assert!(validate(&input).is_ok());
    }

    #[test]
    fn duplicate_aircraft_id_rejected() {
        let s = r#"{
            "aircraft":[
                {"id":"a1","profile":{"aircraft_type":"FIXED_WING"},
                 "start":{"lon":116.3,"lat":39.9,"alt_m":500},
                 "target":{"lon":117.1,"lat":40.2,"alt_m":1000}},
                {"id":"a1","profile":{"aircraft_type":"FIXED_WING"},
                 "start":{"lon":116.4,"lat":39.8,"alt_m":500},
                 "target":{"lon":117.1,"lat":40.2,"alt_m":1000}}
            ]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        match validate(&input) {
            Err(AppError::InputInvalid(InputInvalidReason::VehicleParamsInconsistent)) => {}
            other => panic!("expected dup id, got {other:?}"),
        }
    }

    #[test]
    fn a6_physical_self_consistency_relaxed() {
        // 2026-08-07 主管放宽：250 m/s @ 30° bank → r_min ≈ 11045m；
        // 显式 5000m < r_min 不再拒绝（转弯段降速实现，solver 输出降速提示）。
        let r_min = DefaultParams::physical_turn_radius_m(250.0, 30.0);
        assert!((r_min - 11_045.0).abs() < 10.0, "r_min={r_min}");
        let s = r#"{
            "aircraft":[
                {"id":"a1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                            "max_bank_deg":30,"min_turn_radius_m":5000},
                 "start":{"lon":116.3,"lat":39.9,"alt_m":500},
                 "target":{"lon":117.1,"lat":40.2,"alt_m":1000}}
            ]
        }"#;
        let input = Input::from_json_str(s).unwrap();
        validate(&input).expect("A6 放宽：显式半径信任，不再拒绝");

        // 正数防线仍生效：固定翼 r=0（<1m）拒绝；旋翼机 r→0 合法（悬停原地转向）
        let bad = r#"{
            "aircraft":[
                {"id":"a1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                            "max_bank_deg":30,"min_turn_radius_m":0},
                 "start":{"lon":116.3,"lat":39.9,"alt_m":500},
                 "target":{"lon":117.1,"lat":40.2,"alt_m":1000}}
            ]
        }"#;
        let bad_input = Input::from_json_str(bad).unwrap();
        match validate(&bad_input) {
            Err(AppError::InputInvalid(InputInvalidReason::VehicleParamsInconsistent)) => {}
            other => panic!("expected reject for fixed-wing r<=0, got {other:?}"),
        }
        let ok_rotor = r#"{
            "aircraft":[
                {"id":"a1","profile":{"aircraft_type":"ROTORCRAFT","cruise_speed_mps":60,
                            "max_bank_deg":30,"min_turn_radius_m":0},
                 "start":{"lon":116.3,"lat":39.9,"alt_m":500},
                 "target":{"lon":117.1,"lat":40.2,"alt_m":1000}}
            ]
        }"#;
        let rotor_input = Input::from_json_str(ok_rotor).unwrap();
        validate(&rotor_input).expect("旋翼机 r=0 合法（悬停原地转向）");
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
            alt_min_m: Some(0.0),
            alt_max_m: Some(2000.0),
        };
        let p = Geo::new(115.0, 39.0).unwrap();
        assert!(zone_contains_at(&z, &p, 500.0));
        assert!(zone_contains_at(&z, &p, 2000.0));
        assert!(!zone_contains_at(&z, &p, 3000.0));
        // 水平外 → false 不论高度
        let q = Geo::new(116.5, 39.0).unwrap();
        assert!(!zone_contains_at(&z, &q, 500.0));
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

    #[test]
    fn invalid_params_fall_back_to_defaults() {
        // 主管决策 2026-08-05：无外部参数或参数无效使用默认值。
        let d = DefaultParams::default();
        let o = ParamsOverride {
            radar_inflation: Some(-3.0),           // 无效：≤1
            p_cross: Some(5.0),                    // 无效：>1
            suppression_delta: Some(2.0),          // 无效：≥1
            detection_curve: Some("weird".into()), // 无效：非 exponential/linear
            los_mask_coef: Some(-0.5),             // 无效：<0
            ..Default::default()
        };
        let m = d.merge(&o);
        assert_eq!(m.radar_inflation, d.radar_inflation);
        assert_eq!(m.p_cross, d.p_cross);
        assert_eq!(m.suppression_delta, d.suppression_delta);
        assert_eq!(m.detection_curve, DetectionCurve::Swerling1);
        assert_eq!(m.los_mask_coef, d.los_mask_coef);
    }

    #[test]
    fn invalid_detection_curve_parsed_case_insensitive() {
        let d = DefaultParams::default();
        let o = ParamsOverride {
            detection_curve: Some("LINEAR".into()),
            ..Default::default()
        };
        assert_eq!(d.merge(&o).detection_curve, DetectionCurve::Linear);
        let o2 = ParamsOverride {
            detection_curve: Some("linear".into()),
            ..Default::default()
        };
        assert_eq!(d.merge(&o2).detection_curve, DetectionCurve::Linear);
    }
}

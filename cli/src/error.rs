//! 错误类型与 JSON 错误码（技术方案 4.2 输出 status 契约）。
//!
//! 输出契约四态：`success` / `degraded_timeout` / `no_solution` / `input_invalid`。
//! 任何错误最终以 JSON 形式从 stdout 输出（管道形态），硬故障（IO/内部）才走 stderr。

use schemars::JsonSchema;
use serde::Serialize;

/// 输入非法原因码（`status = "input_invalid"` 时必填，供下游精确诊断）。
#[derive(Debug, Clone, Copy, PartialEq, Eq, Serialize, JsonSchema)]
#[serde(rename_all = "snake_case")]
pub enum InputInvalidReason {
    /// JSON 语法/结构畸形（不可解析、类型错误、缺必填字段、多余未知字段）
    MalformedJson,
    /// 非法经纬度（纬度越界 / 经度越界 / NaN / 无穷）
    IllegalCoordinate,
    /// 字段超界（半径非正、速度越界、高度越界、时间非正等数值域违反）
    OutOfBounds,
    /// 退化输入：A 与 B 相同或间距小于最小可行距离
    DegenerateStartEqualsTarget,
    /// B（目标）落在禁飞区/限飞区内部
    TargetInNoFly,
    /// 中途必经点落在禁飞区/限飞区内部（必经点不可绕行 → fail-fast，P5）
    MidWaypointInNoFly,
    /// 雷达部署区与禁飞区重叠
    RadarOverlapNoFly,
    /// 地形数据空洞残留（加载后仍有 NaN/NoData 且未声明容忍）
    TerrainHolesRemain,
    /// 飞行器参数不自洽（如多机速度/半径/场景互相矛盾）
    VehicleParamsInconsistent,
    /// 飞行器数组为空（逐机显式契约下无任务可规划）
    MissingAircraft,
}

impl std::fmt::Display for InputInvalidReason {
    fn fmt(&self, f: &mut std::fmt::Formatter<'_>) -> std::fmt::Result {
        let s = match self {
            Self::MalformedJson => "malformed_json",
            Self::IllegalCoordinate => "illegal_coordinate",
            Self::OutOfBounds => "out_of_bounds",
            Self::DegenerateStartEqualsTarget => "degenerate_start_equals_target",
            Self::TargetInNoFly => "target_in_no_fly",
            Self::MidWaypointInNoFly => "mid_waypoint_in_no_fly",
            Self::RadarOverlapNoFly => "radar_overlap_no_fly",
            Self::TerrainHolesRemain => "terrain_holes_remain",
            Self::VehicleParamsInconsistent => "vehicle_params_inconsistent",
            Self::MissingAircraft => "missing_aircraft",
        };
        write!(f, "{s}")
    }
}

/// 顶层错误（内部错误类型，最终序列化为输出 JSON）。
#[derive(Debug, thiserror::Error)]
pub enum AppError {
    #[error("io error: {0}")]
    Io(#[from] std::io::Error),

    #[error("json error: {0}")]
    Json(#[from] serde_json::Error),

    #[error("input invalid: {0}")]
    InputInvalid(InputInvalidReason),

    #[error("no solution: {0}")]
    NoSolution(String),

    #[error("degraded timeout: {0}")]
    DegradedTimeout(String),

    #[error("data error: {0}")]
    Data(String),

    #[error("internal error: {0}")]
    Internal(String),
}

/// 输出 JSON 错误体（status != success 时使用）。
#[derive(Debug, Clone, Serialize, JsonSchema)]
pub struct ErrorBody {
    pub code: String,
    pub message: String,
}

impl ErrorBody {
    pub fn input_invalid(reason: InputInvalidReason, message: impl Into<String>) -> Self {
        Self {
            code: reason.to_string(),
            message: message.into(),
        }
    }

    pub fn code(mut self, code: impl Into<String>) -> Self {
        self.code = code.into();
        self
    }
}

impl From<&AppError> for ErrorBody {
    fn from(e: &AppError) -> Self {
        match e {
            AppError::InputInvalid(r) => Self::input_invalid(*r, e.to_string()),
            AppError::NoSolution(_) => Self {
                code: "no_solution".into(),
                message: e.to_string(),
            },
            AppError::DegradedTimeout(_) => Self {
                code: "degraded_timeout".into(),
                message: e.to_string(),
            },
            AppError::Data(_) => Self {
                code: "data_error".into(),
                message: e.to_string(),
            },
            _ => Self {
                code: "internal_error".into(),
                message: e.to_string(),
            },
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn reason_display_snake_case() {
        assert_eq!(
            InputInvalidReason::DegenerateStartEqualsTarget.to_string(),
            "degenerate_start_equals_target"
        );
    }

    #[test]
    fn error_body_input_invalid_code() {
        let b = ErrorBody::input_invalid(InputInvalidReason::TargetInNoFly, "B in no-fly");
        assert_eq!(b.code, "target_in_no_fly");
        assert_eq!(b.message, "B in no-fly");
    }
}

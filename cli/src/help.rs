//! 自定义 help 输出（风格指南：纯文本 + 定宽对齐，三层递进 Root → Category → Endpoint）。

/// 参数树节点。
struct ParamNode {
    name: &'static str,
    type_label: &'static str,
    required: bool,
    description: &'static str,
    children: &'static [ParamNode],
}

/// 输出完整 help。
pub fn print_help(bin: &str, version: &str) {
    let request_body = build_request_body();
    let return_body = build_return_body();

    print!(
        "{bin} v{version} - Aircraft Route Planner\n\
         \n\
         Usage: {bin} <command>\n\
         \n\
         Commands:\n\
           help  显示完整使用说明\n\
           plan  路径规划\n\
         \n\
         Description:\n\
         从文件读取任务 JSON，执行路径规划，输出结果 JSON。\n\
         状态四态：success / degraded_timeout / no_solution / input_invalid。\n\
         \n\
         Usage: {bin} plan --file <file> --out <path>\n\
         \n\
         Options:\n\
          * --file <file>   任务 JSON 文件\n\
          * --out <path>    结果 JSON 文件\n\
         \n\
         Rules:\n\
           - * is Required\n\
         \n\
         Request Body:\n\
         {request_body}\n\
         \n\
         Return:\n\
         {return_body}\n\
         \n\
         Examples:\n\
           {bin} plan --file task.json --out result.json\n",
    );
}

// ---- Request Body ----

fn build_request_body() -> String {
    let root = vec![
        ParamNode {
            name: "aircraft",
            type_label: "array<object>",
            required: true,
            description: "飞行器数组（必填非空）",
            children: &[
                ParamNode { name: "id", type_label: "string", required: true, description: "飞行器 ID", children: &[] },
                ParamNode { name: "start", type_label: "object", required: true, description: "起点", children: &[
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度（度，WGS84）", children: &[] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度（度，WGS84）", children: &[] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高程（米，MSL）", children: &[] },
                ]},
                ParamNode { name: "target", type_label: "object", required: true, description: "目标点", children: &[
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度（度，WGS84）", children: &[] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度（度，WGS84）", children: &[] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高程（米，MSL）", children: &[] },
                ]},
                ParamNode { name: "profile", type_label: "object", required: false, description: "机型性能参数", children: &[
                    ParamNode { name: "aircraft_type", type_label: "string", required: true, description: "FIXED_WING / ROTORCRAFT", children: &[] },
                    ParamNode { name: "cruise_speed_mps", type_label: "f64", required: false, description: "巡航速度 m/s", children: &[] },
                    ParamNode { name: "speed_range_mps", type_label: "array<f64>", required: false, description: "[v_min, v_max] m/s", children: &[] },
                    ParamNode { name: "min_turn_radius_m", type_label: "f64", required: false, description: "最小转弯半径 m", children: &[] },
                    ParamNode { name: "max_climb_angle_deg", type_label: "f64", required: false, description: "最大爬升角 °", children: &[] },
                    ParamNode { name: "max_bank_deg", type_label: "f64", required: false, description: "最大坡度 °", children: &[] },
                    ParamNode { name: "ceiling_m", type_label: "f64", required: false, description: "升限 m", children: &[] },
                ]},
                ParamNode { name: "mid_waypoints", type_label: "array<object>", required: false, description: "中途必经点", children: &[
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度", children: &[] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度", children: &[] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高度", children: &[] },
                ]},
                ParamNode { name: "weapon", type_label: "object", required: false, description: "武器配置（出现即启用）", children: &[
                    ParamNode { name: "weapon_type", type_label: "string", required: true, description: "aam / agm / bomb", children: &[] },
                    ParamNode { name: "range_km", type_label: "array<f64>", required: false, description: "[Rmin, Rmax] km", children: &[] },
                    ParamNode { name: "envelope", type_label: "object", required: false, description: "发射包线", children: &[
                        ParamNode { name: "heading_deg", type_label: "array<f64>", required: false, description: "航向角范围", children: &[] },
                        ParamNode { name: "alt_m", type_label: "array<f64>", required: false, description: "高度范围", children: &[] },
                        ParamNode { name: "speed_mps", type_label: "array<f64>", required: false, description: "速度范围", children: &[] },
                    ]},
                ]},
            ],
        },
        ParamNode {
            name: "red_forces",
            type_label: "object",
            required: false,
            description: "红方部署",
            children: &[
                ParamNode { name: "radars", type_label: "array<object>", required: false, description: "雷达数组", children: &[
                    ParamNode { name: "id", type_label: "string", required: true, description: "雷达 ID", children: &[] },
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度", children: &[] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度", children: &[] },
                    ParamNode { name: "radius_km", type_label: "f64", required: true, description: "探测距离 km", children: &[] },
                    ParamNode { name: "alt_m", type_label: "f64", required: false, description: "天线高度 m（默认 10）", children: &[] },
                    ParamNode { name: "suppression_post_range_km", type_label: "f64", required: false, description: "压制后有效距离 km", children: &[] },
                    ParamNode { name: "suppression_factor", type_label: "f64", required: false, description: "压制因子 δ", children: &[] },
                ]},
            ],
        },
        ParamNode {
            name: "no_fly_zones",
            type_label: "array<object>",
            required: false,
            description: "禁飞区",
            children: &[
                ParamNode { name: "id", type_label: "string", required: true, description: "区域 ID", children: &[] },
                ParamNode { name: "shape", type_label: "tagged-union", required: true, description: "circle / polygon", children: &[
                    ParamNode { name: "circle", type_label: "object", required: false, description: "", children: &[
                        ParamNode { name: "center", type_label: "array<f64,2>", required: true, description: "圆心 [lon, lat]", children: &[] },
                        ParamNode { name: "radius_km", type_label: "f64", required: true, description: "半径 km", children: &[] },
                    ]},
                    ParamNode { name: "polygon", type_label: "object", required: false, description: "", children: &[
                        ParamNode { name: "vertices", type_label: "array<array<f64,2>>", required: true, description: "顶点列表", children: &[] },
                    ]},
                ]},
                ParamNode { name: "alt_min_m", type_label: "f64", required: false, description: "高度下界（仅限飞区）", children: &[] },
                ParamNode { name: "alt_max_m", type_label: "f64", required: false, description: "高度上界（仅限飞区）", children: &[] },
            ],
        },
        ParamNode {
            name: "restricted_zones",
            type_label: "array<object>",
            required: false,
            description: "限飞区",
            children: &[
                ParamNode { name: "id", type_label: "string", required: true, description: "区域 ID", children: &[] },
                ParamNode { name: "shape", type_label: "tagged-union", required: true, description: "circle / polygon", children: &[
                    ParamNode { name: "circle", type_label: "object", required: false, description: "", children: &[
                        ParamNode { name: "center", type_label: "array<f64,2>", required: true, description: "圆心 [lon, lat]", children: &[] },
                        ParamNode { name: "radius_km", type_label: "f64", required: true, description: "半径 km", children: &[] },
                    ]},
                    ParamNode { name: "polygon", type_label: "object", required: false, description: "", children: &[
                        ParamNode { name: "vertices", type_label: "array<array<f64,2>>", required: true, description: "顶点列表", children: &[] },
                    ]},
                ]},
                ParamNode { name: "alt_min_m", type_label: "f64", required: true, description: "高度下界", children: &[] },
                ParamNode { name: "alt_max_m", type_label: "f64", required: true, description: "高度上界", children: &[] },
            ],
        },
        ParamNode {
            name: "obstacles",
            type_label: "array<object>",
            required: false,
            description: "障碍物",
            children: &[
                ParamNode { name: "id", type_label: "string", required: true, description: "障碍物 ID", children: &[] },
                ParamNode { name: "shape", type_label: "tagged-union", required: true, description: "circle / polygon", children: &[
                    ParamNode { name: "circle", type_label: "object", required: false, description: "", children: &[
                        ParamNode { name: "center", type_label: "array<f64,2>", required: true, description: "圆心 [lon, lat]", children: &[] },
                        ParamNode { name: "radius_km", type_label: "f64", required: true, description: "半径 km", children: &[] },
                    ]},
                    ParamNode { name: "polygon", type_label: "object", required: false, description: "", children: &[
                        ParamNode { name: "vertices", type_label: "array<array<f64,2>>", required: true, description: "顶点列表", children: &[] },
                    ]},
                ]},
                ParamNode { name: "alt_min_m", type_label: "f64", required: false, description: "高度下界（仅限飞区）", children: &[] },
                ParamNode { name: "alt_max_m", type_label: "f64", required: false, description: "高度上界（仅限飞区）", children: &[] },
            ],
        },
        ParamNode {
            name: "terrain",
            type_label: "object",
            required: false,
            description: "地形配置",
            children: &[
                ParamNode { name: "source", type_label: "string", required: false, description: "none / path（默认: none）", children: &[] },
                ParamNode { name: "path", type_label: "string", required: false, description: "地形文件路径", children: &[] },
                ParamNode { name: "mask_path", type_label: "string", required: false, description: "海岸掩膜路径", children: &[] },
            ],
        },
        ParamNode {
            name: "parameters",
            type_label: "object",
            required: false,
            description: "参数覆盖",
            children: &[
                ParamNode { name: "grid_resolution", type_label: "int", required: false, description: "粗网格分辨率（8..1024，默认 256）", children: &[] },
                ParamNode { name: "radar_inflation", type_label: "f64", required: false, description: "雷达膨胀系数（>1）", children: &[] },
                ParamNode { name: "detection_curve", type_label: "string", required: false, description: "swerling1 / exponential / linear", children: &[] },
                ParamNode { name: "p_cross", type_label: "f64", required: false, description: "穿越阈值（0..1）", children: &[] },
                ParamNode { name: "suppression_delta", type_label: "f64", required: false, description: "压制因子 δ（0..1）", children: &[] },
                ParamNode { name: "radar_cost_coef", type_label: "f64", required: false, description: "雷达代价系数（>0）", children: &[] },
                ParamNode { name: "los_mask_coef", type_label: "f64", required: false, description: "LOS mask 系数（0..1）", children: &[] },
            ],
        },
    ];

    render_tree(&root, "  ")
}

// ---- Return Body ----

fn build_return_body() -> String {
    let root = vec![
        ParamNode {
            name: "status",
            type_label: "string",
            required: true,
            description: "success / degraded_timeout / no_solution / input_invalid",
            children: &[],
        },
        ParamNode {
            name: "error",
            type_label: "object",
            required: false,
            description: "错误体（status != success 时）",
            children: &[
                ParamNode { name: "code", type_label: "string", required: true, description: "错误码", children: &[] },
                ParamNode { name: "message", type_label: "string", required: true, description: "错误信息", children: &[] },
            ],
        },
        ParamNode {
            name: "elapsed_ms",
            type_label: "u64",
            required: false,
            description: "耗时（ms）",
            children: &[],
        },
        ParamNode {
            name: "aircraft",
            type_label: "array<object>",
            required: true,
            description: "飞行器结果",
            children: &[
                ParamNode { name: "id", type_label: "string", required: true, description: "飞行器 ID", children: &[] },
                ParamNode { name: "status", type_label: "string", required: true, description: "planned / no_solution / degraded", children: &[] },
                ParamNode { name: "path", type_label: "array<object>", required: true, description: "路径点", children: &[
                    ParamNode { name: "x", type_label: "f64", required: true, description: "经度", children: &[] },
                    ParamNode { name: "y", type_label: "f64", required: true, description: "纬度", children: &[] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高度", children: &[] },
                ]},
                ParamNode { name: "distance_m", type_label: "f64", required: true, description: "距离（m）", children: &[] },
                ParamNode { name: "warnings", type_label: "array<string>", required: true, description: "警告信息", children: &[] },
            ],
        },
        ParamNode {
            name: "stats",
            type_label: "object",
            required: true,
            description: "统计信息",
            children: &[
                ParamNode { name: "fmm_ms", type_label: "f64", required: true, description: "FMM 耗时（ms）", children: &[] },
                ParamNode { name: "los_checks", type_label: "u64", required: true, description: "LOS 检查次数", children: &[] },
                ParamNode { name: "degradations", type_label: "array<string>", required: true, description: "降级原因列表", children: &[] },
            ],
        },
    ];

    render_tree(&root, "  ")
}

// ---- 树渲染 ----

/// 渲染参数树为固定宽度对齐的字符串。
fn render_tree(nodes: &[ParamNode], indent: &str) -> String {
    if nodes.is_empty() {
        return String::new();
    }

    // 计算最大字段名宽度（用于对齐）
    let max_name_width = col_width(nodes);

    let mut out = String::new();
    for node in nodes {
        let prefix = if node.required { "* " } else { "  " };
        // name + type_label 对齐：字段名占 max_name_width，后接 " : type_label  描述"
        let name_col_width = max_name_width;
        let line = format!(
            "{indent}{prefix}{:<width$} : {:<24} {}\n",
            node.name,
            node.type_label,
            node.description,
            width = name_col_width,
        );
        out.push_str(&line);

        // 递归渲染子节点（缩进 +2）
        if !node.children.is_empty() {
            let child_indent = format!("{indent}  ");
            out.push_str(&render_tree(node.children, &child_indent));
        }
    }
    out
}

/// 计算一组节点名称的最大字符宽度。
fn col_width(nodes: &[ParamNode]) -> usize {
    nodes
        .iter()
        .map(|n| n.name.len())
        .max()
        .unwrap_or(0)
}

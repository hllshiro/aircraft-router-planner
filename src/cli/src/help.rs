//! 自定义 help 输出（风格指南：纯文本 + 定宽对齐，三层递进 Root → Category → Endpoint）。

/// 参数树节点（owned 描述，支持动态候选项）。
struct ParamNode {
    name: &'static str,
    type_label: &'static str,
    required: bool,
    description: String,
    children: Vec<ParamNode>,
}

/// 输出完整 help。
pub fn print_help(bin: &str, version: &str, index: Option<&crate::config::TerrainIndex>) {
    let request_body = build_request_body(index);
    let return_body = render_tree(&build_return_body(), "  ");

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

fn build_request_body(index: Option<&crate::config::TerrainIndex>) -> String {
    let terrain_children = build_terrain_children(index);
    let root = vec![
        ParamNode {
            name: "aircraft",
            type_label: "array<object>",
            required: true,
            description: "飞行器数组（必填非空）".into(),
            children: vec![
                ParamNode { name: "id", type_label: "string", required: true, description: "飞行器 ID".into(), children: vec![] },
                ParamNode { name: "start", type_label: "object", required: true, description: "起点".into(), children: vec![
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度（度，WGS84）".into(), children: vec![] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度（度，WGS84）".into(), children: vec![] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高程（米，MSL）".into(), children: vec![] },
                ]},
                ParamNode { name: "target", type_label: "object", required: true, description: "目标点".into(), children: vec![
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度（度，WGS84）".into(), children: vec![] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度（度，WGS84）".into(), children: vec![] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高程（米，MSL）".into(), children: vec![] },
                ]},
                ParamNode { name: "profile", type_label: "object", required: false, description: "机型性能参数".into(), children: vec![
                    ParamNode { name: "aircraft_type", type_label: "string", required: true, description: "FIXED_WING / ROTORCRAFT".into(), children: vec![] },
                    ParamNode { name: "maximum_speed_mps", type_label: "f64", required: false, description: "最大速度 m/s".into(), children: vec![] },
                    ParamNode { name: "maximum_turn_rate_dps", type_label: "f64", required: false, description: "最大转弯角速率 °/s".into(), children: vec![] },
                    ParamNode { name: "maximum_climb_rate_mps", type_label: "f64", required: false, description: "最大爬升率 m/s".into(), children: vec![] },
                    ParamNode { name: "maximum_altitude_m", type_label: "f64", required: false, description: "最大飞行高度 m".into(), children: vec![] },
                ]},
                ParamNode { name: "mid_waypoints", type_label: "array<object>", required: false, description: "中途必经点".into(), children: vec![
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度".into(), children: vec![] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度".into(), children: vec![] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高度".into(), children: vec![] },
                ]},
                ParamNode { name: "weapon", type_label: "object", required: false, description: "武器配置（出现即启用）".into(), children: vec![
                    ParamNode { name: "weapon_type", type_label: "string", required: true, description: "aam / agm / bomb".into(), children: vec![] },
                    ParamNode { name: "range_km", type_label: "array<f64>", required: false, description: "[Rmin, Rmax] km".into(), children: vec![] },
                    ParamNode { name: "envelope", type_label: "object", required: false, description: "发射包线".into(), children: vec![
                        ParamNode { name: "heading_deg", type_label: "array<f64>", required: false, description: "航向角范围".into(), children: vec![] },
                        ParamNode { name: "alt_m", type_label: "array<f64>", required: false, description: "高度范围".into(), children: vec![] },
                        ParamNode { name: "speed_mps", type_label: "array<f64>", required: false, description: "速度范围".into(), children: vec![] },
                    ]},
                ]},
            ],
        },
        ParamNode {
            name: "red_forces",
            type_label: "object",
            required: false,
            description: "红方部署".into(),
            children: vec![
                ParamNode { name: "radars", type_label: "array<object>", required: false, description: "雷达数组".into(), children: vec![
                    ParamNode { name: "id", type_label: "string", required: true, description: "雷达 ID".into(), children: vec![] },
                    ParamNode { name: "lon", type_label: "f64", required: true, description: "经度".into(), children: vec![] },
                    ParamNode { name: "lat", type_label: "f64", required: true, description: "纬度".into(), children: vec![] },
                    ParamNode { name: "radius_km", type_label: "f64", required: true, description: "探测距离 km".into(), children: vec![] },
                    ParamNode { name: "alt_m", type_label: "f64", required: false, description: "天线高度 m（默认 10）".into(), children: vec![] },
                    ParamNode { name: "suppression_post_range_km", type_label: "f64", required: false, description: "压制后有效距离 km".into(), children: vec![] },
                    ParamNode { name: "suppression_factor", type_label: "f64", required: false, description: "压制因子 δ".into(), children: vec![] },
                ]},
            ],
        },
        ParamNode {
            name: "zones",
            type_label: "array<object>",
            required: false,
            description: "区域数组".into(),
            children: vec![
                ParamNode { name: "id", type_label: "string", required: true, description: "区域 ID".into(), children: vec![] },
                ParamNode { name: "shape", type_label: "tagged-union", required: true, description: "circle / polygon".into(), children: vec![
                    ParamNode { name: "circle", type_label: "object", required: false, description: "".into(), children: vec![
                        ParamNode { name: "center", type_label: "array<f64,2>", required: true, description: "圆心 [lon, lat]".into(), children: vec![] },
                        ParamNode { name: "radius_km", type_label: "f64", required: true, description: "半径 km".into(), children: vec![] },
                    ]},
                    ParamNode { name: "polygon", type_label: "object", required: false, description: "".into(), children: vec![
                        ParamNode { name: "vertices", type_label: "array<array<f64,2>>", required: true, description: "顶点列表".into(), children: vec![] },
                    ]},
                ]},
                ParamNode { name: "alt_min_m", type_label: "f64", required: false, description: "高度下界（restricted 必填）".into(), children: vec![] },
                ParamNode { name: "alt_max_m", type_label: "f64", required: false, description: "高度上界（restricted 必填）".into(), children: vec![] },
            ],
        },
        ParamNode {
            name: "terrain",
            type_label: "object",
            required: false,
            description: "地形配置（索引 id）".into(),
            children: terrain_children,
        },
        ParamNode {
            name: "parameters",
            type_label: "object",
            required: false,
            description: "参数覆盖".into(),
            children: vec![
                ParamNode { name: "precision", type_label: "string", required: false, description: "fast / balanced / accurate（默认 balanced；无解时自动提升精度）".into(), children: vec![] },
                ParamNode { name: "detection_curve", type_label: "string", required: false, description: "swerling1 / exponential / linear".into(), children: vec![] },
                ParamNode { name: "p_cross", type_label: "f64", required: false, description: "穿越阈值（0..1）".into(), children: vec![] },
                ParamNode { name: "radar_cost_coef", type_label: "f64", required: false, description: "雷达代价系数（>0）".into(), children: vec![] },
            ],
        },
    ];

    render_tree(&root, "  ")
}

// ---- Return Body ----

fn build_return_body() -> Vec<ParamNode> {
    vec![
        ParamNode {
            name: "status",
            type_label: "string",
            required: true,
            description: "success / degraded_timeout / no_solution / input_invalid".into(),
            children: vec![],
        },
        ParamNode {
            name: "error",
            type_label: "object",
            required: false,
            description: "错误体（status != success 时）".into(),
            children: vec![
                ParamNode { name: "code", type_label: "string", required: true, description: "错误码".into(), children: vec![] },
                ParamNode { name: "message", type_label: "string", required: true, description: "错误信息".into(), children: vec![] },
            ],
        },
        ParamNode {
            name: "elapsed_ms",
            type_label: "u64",
            required: false,
            description: "耗时（ms）".into(),
            children: vec![],
        },
        ParamNode {
            name: "aircraft",
            type_label: "array<object>",
            required: true,
            description: "飞行器结果".into(),
            children: vec![
                ParamNode { name: "id", type_label: "string", required: true, description: "飞行器 ID".into(), children: vec![] },
                ParamNode { name: "status", type_label: "string", required: true, description: "planned / no_solution / degraded".into(), children: vec![] },
                ParamNode { name: "path", type_label: "array<object>", required: true, description: "路径点".into(), children: vec![
                    ParamNode { name: "x", type_label: "f64", required: true, description: "经度".into(), children: vec![] },
                    ParamNode { name: "y", type_label: "f64", required: true, description: "纬度".into(), children: vec![] },
                    ParamNode { name: "alt_m", type_label: "f64", required: true, description: "高度".into(), children: vec![] },
                ]},
                ParamNode { name: "distance_m", type_label: "f64", required: true, description: "距离（m）".into(), children: vec![] },
                ParamNode { name: "warnings", type_label: "array<string>", required: true, description: "警告信息".into(), children: vec![] },
            ],
        },
        ParamNode {
            name: "stats",
            type_label: "object",
            required: true,
            description: "统计信息".into(),
            children: vec![
                ParamNode { name: "fmm_ms", type_label: "f64", required: true, description: "FMM 耗时（ms）".into(), children: vec![] },
                ParamNode { name: "los_checks", type_label: "u64", required: true, description: "LOS 检查次数".into(), children: vec![] },
                ParamNode { name: "degradations", type_label: "array<string>", required: true, description: "降级原因列表".into(), children: vec![] },
            ],
        },
    ]
}

// ---- Terrain 动态候选项 ----

fn build_terrain_children(index: Option<&crate::config::TerrainIndex>) -> Vec<ParamNode> {
    match index {
        Some(idx) => {
            let arpack_desc = if idx.arpacks.is_empty() {
                "无可用 arpack".to_string()
            } else {
                idx.arpacks
                    .iter()
                    .map(|e| format!("{}({})", e.id, e.desc))
                    .collect::<Vec<_>>()
                    .join(" / ")
            };
            let mask_desc = if idx.masks.is_empty() {
                "无可用 mask".to_string()
            } else {
                idx.masks
                    .iter()
                    .map(|e| format!("{}({})", e.id, e.desc))
                    .collect::<Vec<_>>()
                    .join(" / ")
            };
            vec![
                ParamNode {
                    name: "arpack",
                    type_label: "string",
                    required: false,
                    description: format!("arpack 索引 id：{arpack_desc}"),
                    children: vec![],
                },
                ParamNode {
                    name: "mask",
                    type_label: "string",
                    required: false,
                    description: format!("mask 索引 id：{mask_desc}"),
                    children: vec![],
                },
            ]
        }
        None => vec![ParamNode {
            name: "(无数据)",
            type_label: "",
            required: false,
            description: "当前无数据文件，无法提供地形支持".into(),
            children: vec![],
        }],
    }
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
            out.push_str(&render_tree(&node.children, &child_indent));
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

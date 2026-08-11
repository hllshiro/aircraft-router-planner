//! Phase 3 输出美化链（技术方案 Phase 3 + 九轮链序修正 + 十轮复验清单）。
//!
//! 默认链序：Theta* 去锯齿 → Dubins/样条拟合 → 贪心抽稀 → 弦高/运动学复验；
//! 优先级：安全（不撞山/不越禁飞） > 运动学约束（机动可飞） > 美化（平滑/抽稀）。
//! 任一复验失败回退上一策略环节；全链失败回退美化前原始折线并显式告警
//! `smoothing_failed`（宁丑勿违，不静默交付违规路径）。
//!
//! 本模块为纯几何后处理：输入折线路径（经纬度+高度），输出平滑路径；
//! 与 Phase 2 细层搜索解耦，地形净空/禁飞检查通过注入接口接入。

use crate::config::{AircraftType, DefaultParams, VehicleProfile};
use crate::dubins::dubins_path;
use crate::path::{Path, PathPoint, angle_diff_deg, point_seg_distance_m};

/// Dubins 拟合阶段的弦高放宽值（米）：Dubins 输出是"物理修正"圆角
/// （转弯半径 r_m），绕行 L 形/多雷达弧线的圆角相对 raw 折线可达数百米，
/// 100m 逼近容差会误杀合法绕行；运动学/地形/禁飞仍严格复验。
const DUBINS_CHORD_TOL_M: f64 = 1000.0;

/// Catmull-Rom 样条阶段的弦高放宽值（米）：样条对"绕行折线"（含 Theta* 无法
/// 拉直的急转弯，如多边形尖角绕行 90° 折角）的平滑代价——折角处样条内切，
/// 弦高可达数百米（zigzag18 92° 折角 116m），100m 逼近容差会误杀合法平滑；
/// 运动学/地形/禁飞仍严格复验。
const CATMULL_CHORD_TOL_M: f64 = 500.0;

/// 平滑器参数（Phase 0 标定前用保守初值，参数化可调；标定项见 docs/phase0_baseline.md）。
#[derive(Debug, Clone)]
pub struct SmoothOptions {
    /// 机型（Phase 4 机型分流；旋翼机可悬停/极小转弯半径——急转/垂直机动合法，
    /// 复验按机型放宽，链不拟合 Dubins 圆弧）。
    pub aircraft_type: AircraftType,
    /// 抽稀弦高容差（米）。Phase 0 标定项，初值 100m（粗层 1-2km 格距的 ~1/10）。
    pub chord_tol_m: f64,
    /// 最小转弯半径（米，Dubins 拟合 + 运动学复验；仅固定翼）。
    pub turn_radius_m: f64,
    /// 最大爬升角（度，运动学复验；仅固定翼）。
    pub max_climb_deg: f64,
    /// 地形净空（米，复验）。
    pub clearance_m: f64,
    /// 最大转角（度，相邻航段航向差；运动学复验；仅固定翼）。
    pub max_turn_deg: f64,
    /// Dubins 采样密度（每段点数）。
    pub dubins_sample_n: usize,
    /// 复验每段等距采样数（段内最坏情况检查）。
    pub verify_seg_samples: usize,
}

impl Default for SmoothOptions {
    fn default() -> Self {
        Self {
            aircraft_type: AircraftType::FixedWing,
            chord_tol_m: 100.0,
            turn_radius_m: 5_000.0,
            max_climb_deg: 8.0,
            clearance_m: 100.0,
            max_turn_deg: 60.0,
            dubins_sample_n: 32,
            verify_seg_samples: 8,
        }
    }
}

/// Phase 4 M4：机型分流参数派生（VehicleProfile → SmoothOptions + A6 物理下限）。
///
/// 派生规则（技术方案 A6 自洽 + 八轮共识缺省落默认参数表）：
/// - 速度 v：`cruise_speed_mps` → `speed_range_mps` 中值 → 机型默认
///   （固定翼 250 m/s / 旋翼机 100 m/s）；
/// - A6 物理下限 r_phys = v²/(g·tan φ_max)，φ_max = `max_bank_deg` → 默认 30°
///   （与 Phase 0 标定表一致：442m@50m/s、11039m@250m/s 即此式）；
/// - 固定翼 turn_radius 信任输入 `min_turn_radius_m` / 默认表 5000m，不再钳到巡航
///   物理下限（2026-08-07 主管：速度非锁定，转弯段可降速实现小半径；v_turn 由
///   turn_radius 反算，A6 恒满足）；返回值的第二个分量是 **A6 有效下限** =
///   min(巡航物理下限, turn_radius)，solver 的 verify A6 检查与预取 slack 用它；
/// - 旋翼机 r→0 合法（可悬停原地转向，九轮共识），turn_radius 不钳；
/// - max_climb = `max_climb_angle_deg` → 默认表 15°（固定翼运动学复验用）。
pub fn smooth_options_for(profile: &VehicleProfile, params: &DefaultParams) -> (SmoothOptions, f64) {
    let v = profile
        .cruise_speed_mps
        .or_else(|| profile.speed_range_mps.map(|[a, b]| (a + b) / 2.0))
        .unwrap_or(match profile.aircraft_type {
            AircraftType::FixedWing => params.default_fixed_wing_speed_mps,
            AircraftType::Rotorcraft => params.default_rotorcraft_speed_mps,
        });
    let bank_deg = profile.max_bank_deg.unwrap_or(params.default_max_bank_deg);
    let phys_min_radius_m = v * v / (9.81 * bank_deg.to_radians().tan());
    let base_r = profile
        .min_turn_radius_m
        .unwrap_or(match profile.aircraft_type {
            AircraftType::FixedWing => params.default_fixed_wing_turn_radius_m,
            AircraftType::Rotorcraft => params.default_rotorcraft_turn_radius_m,
        });
    let turn_radius_m = match profile.aircraft_type {
        AircraftType::Rotorcraft => base_r, // r→0 合法，不钳
        AircraftType::FixedWing => base_r,  // 信任输入/默认表；转弯段降速实现（主管 2026-08-07）
    };
    let opts = SmoothOptions {
        aircraft_type: profile.aircraft_type,
        turn_radius_m,
        max_climb_deg: profile
            .max_climb_angle_deg
            .unwrap_or(params.default_max_climb_angle_deg),
        ..Default::default()
    };
    // A6 有效下限：降速语义下 = min(巡航物理下限, 规划半径)。v_turn 由 turn_radius
    // 反算（sqrt(r·g·tanφ)），其物理下限恰为 turn_radius——verify 的 A6 检查
    // `turn_radius < phys` 恒不触发；大半径输入时仍按巡航物理下限兜底。
    (opts, phys_min_radius_m.min(turn_radius_m))
}

/// 直线段合法性检查（Theta* 去锯齿用）：从 (lon1,lat1,alt1) 直飞到
/// (lon2,lat2,alt2) 是否合法（不穿地形/禁飞/限飞）。true = 可直连。
pub type SegmentCheck<'a> = dyn Fn(f64, f64, f64, f64, f64, f64) -> bool + 'a;

// ==================== 基础算法 ====================

/// Theta* 去锯齿（LOS 捷径简化）：贪心跳点——锚点 i 起，从路径末尾向前找
/// 第一个 `check` 通过的跳跃点 j（i,j 之间直连合法），保 i、j，删中间点。
/// `max_turn_deg`：跳点额外受航向连续性约束（相邻航段转角 ≤ max，
/// 防"弧线拉直成 61°+ 折线"导致运动学复验失败；None = 不限制）。
/// `entry_heading`：段首点（index 0）的入航向（度，真北，来自前一段输出方向）。
/// 段首点本身无法从段内计算入航向，但它是受限区剖面/过渡段的拼接边界——若
/// 首跳（index0→j）方向与入航向夹角 > max_turn，拼接后段边界转角超限，单段
/// verify 无法发现（2026-08-07 主管 1755 点场景：climb 点转角 61.94° > 60°，
/// seg3 过渡 out→climb 与 seg4 首跳 climb→A 夹角超限，终检 vertex 5 拒）。
/// None = 不约束段首（起点段无前段，保持原语义）。
/// 复杂度 O(n²)（点列短，粗层走廊点数百级可接受）。
pub fn theta_star_smooth(
    path: &Path,
    check: &SegmentCheck,
    max_turn_deg: Option<f64>,
    entry_heading: Option<f64>,
    min_r_m: f64,
    entry_max_deg: f64,
) -> Path {
    if path.len() < 3 {
        return path.clone();
    }
    let mut out: Vec<PathPoint> = Vec::with_capacity(path.len());
    out.push(path.points[0]);
    let mut i = 0usize;
    while i < path.len() - 1 {
        // 跳点插弧（2026-08-11 zz33）成功后已 push 弧点，外层跳过普通 push。
        let mut advanced = false;
        let mut j = path.len() - 1;
        loop {
            if j == i + 1 || j == i {
                break;
            }
            let a = path.points[i];
            let b = path.points[j];
            if !check(a.lon, a.lat, a.alt_m, b.lon, b.lat, b.alt_m) {
                if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() && out.len() < 4 {
                    eprintln!("[ts-dbg] i={i} j={j} reject=check ({:.4},{:.4})->({:.4},{:.4})", a.lon, a.lat, b.lon, b.lat);
                }
                j -= 1;
                continue;
            }
            if let Some(max_turn) = max_turn_deg {
                // 首跳（out 只有段首点）用入航向；之后用上一跳方向。
                let h0 = if out.len() >= 2 {
                    let p_prev = out[out.len() - 2];
                    let p_cur = out[out.len() - 1];
                    Some(heading_deg_pts(&p_prev, &p_cur))
                } else {
                    entry_heading
                };
                if let Some(h0) = h0 {
                    let p_cur = out[out.len() - 1];
                    let h1 = heading_deg_pts(&p_cur, &b);
                    let d = crate::path::angle_diff_deg(h0, h1).abs();
                    // 首跳 entry 约束放宽（2026-08-10 zigzag25）：mid 恰在雷达
                    // 盘内时，SEG0 从南侧绕入向北（末段方向 0°），SEG1 需向西
                    // 离开（首跳 88.5°）——60° 约束把首跳全拒 → theta_star 只能
                    // 走相邻点（b→c 800m 向南）→ 段边界 180° 掉头 → arc 病态
                    // （tan(90°) 发散）→ FINAL 拒 → 回退 471 点锯齿。放宽到
                    // 95°：b→target 直接拉直（check 已通过），solver boundary
                    // arc 用标准弧切 b 处理 88.5°（r×tan(44.25°)≈431m，物理
                    // 半径内）。zigzag11 首跳 61.94° 场景仍受约束（<95）。
                    // 必经点段再放宽（2026-08-10 zigzag27）：主管输入 4 必经点
                    // 之字形，wp3 处几何需 160° 大转向，95° 仍挡 → 拉直全拒
                    // → 走相邻点 → 段边界 179.9° 掉头 → arc tan(89.95°)≈114.6
                    // 发散退化（d_m 截断 0.75×|bc| 后 r_eff≈5m）→ FINAL 拒 →
                    // 回退 541 点锯齿。必经点是用户硬约束（必须经过），到达后
                    // 大转向合法（solver 会插切弧处理 160°：r×tan(80°)≈2.5km
                    // 需 bc 段足够长——SEG3 wp3→wp4 直线 65km 满足）；非必经点
                    // 段仍 95°（zigzag11 135° 语义保护）。首跳上限直接取
                    // entry_max_deg（不再乘 1.6——zigzag25 的 min(96,95)=95 语义
                    // 由非必经点传 95 保持）。
                    let effective_max = if out.len() == 1 && entry_heading.is_some() {
                        entry_max_deg
                    } else {
                        max_turn
                    };
                    if d > effective_max {
                        if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() && out.len() < 4 {
                            eprintln!("[ts-dbg] i={i} j={j} reject=turn {d:.1}>={effective_max} h0={h0:.1} h1={h1:.1} entry={entry_heading:?} outlen={}", out.len());
                        }
                        // 跳点转角超限：尝试圆弧过渡拆分（2026-08-11 zz33）。
                        // 绕多边形北缘时 raw 折返段与拉直方向冲突（wp1→wp2 需转
                        // 91°>60°），j 递减到相邻点（raw 折返段微步 ~270m）才插弧 →
                        // |bc| 短 → d_m 截断 → r_eff 塌缩（178m<442m）→ verify
                        // radius 拒 → 全链回退 1061 点密集锯齿。这里直接在拉直目标
                        // j 处插弧（c=j，|bc| 大，r_eff 保持 ≥ min_r）：弧从入段方向
                        // 切到 b→j 出段方向，E 在 b→j 直线上，接续点 k 由
                        // arc_transition 的 E→path[k] 搜索决定。仅非首跳
                        // （out.len()>=2，有入段 a）可插弧；首跳 entry 约束保持
                        // （无入段，必经点段首跳放宽 175° 已覆盖）。
                        if out.len() >= 2 {
                            let a_prev = out[out.len() - 2];
                            let b_cur = out[out.len() - 1];
                            let c_tgt = path.points[j];
                            if let Some((pts, k)) = arc_transition(
                                &a_prev,
                                &b_cur,
                                &c_tgt,
                                max_turn,
                                min_r_m,
                                check,
                                &path.points,
                                i,
                                true,
                                false,
                                0,
                            ) {
                                let npts = pts.len();
                                // 弹出 b（急转弯点）：弧从入段线上的 S 开始，不再经过 b
                                // （否则 b→S 是掉头 180°，verify 拒）。
                                out.pop();
                                for p in pts {
                                    out.push(p);
                                }
                                // 接续点 k == 终点时，弧末点 E 在 b→c 线上（d_m ≤
                                // 0.75×|bc|）并不等于终点 c——终点必须保留在输出中。
                                if k == path.len() - 1 {
                                    out.push(c_tgt);
                                }
                                i = k;
                                advanced = true;
                                if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                    eprintln!(
                                        "[ts-dbg] jump arc_transition at i={i} j={j} pts={npts} k={k}"
                                    );
                                }
                                break;
                            }
                        }
                        j -= 1;
                        continue;
                    }
                }
            }
            break;
        }
        // 保 i、j；若 j 直接是下一邻点则普通前进一步
        if j > i {
            if j == i + 1 {
                // 相邻点：若转弯超限（FMM 楼梯 90° 角，绕窄通道时跳点拉直被墙挡），
                // 插入圆弧过渡点拆分转弯（2026-08-07 zigzag18）。
                if let (Some(max_turn), true) = (max_turn_deg, out.len() >= 2) {
                    let a = out[out.len() - 2];
                    let b = out[out.len() - 1];
                    let c = path.points[i + 1];
                    let h0 = heading_deg_pts(&a, &b);
                    let h1 = heading_deg_pts(&b, &c);
                    if crate::path::angle_diff_deg(h0, h1).abs() > max_turn {
                        if let Some((pts, k)) =
                            arc_transition(&a, &b, &c, max_turn, min_r_m, check, &path.points, i, true, false, 0)
                        {
                            let npts = pts.len();
                            // 弹出 b（急转弯点）：弧从入段线上的 S 开始，不再经过 b
                            // （否则 b→S 是掉头 180°，verify 拒）。
                            out.pop();
                            for p in pts {
                                out.push(p);
                            }
                            i = k;
                            advanced = true;
                            if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                                eprintln!("[ts-dbg] arc_transition at i={i} pts={npts} k={k}");
                            }
                        } else if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                            eprintln!(
                                "[ts-dbg] arc_transition FAIL at i={i} turn={:.1} b=({:.4},{:.4}) c=({:.4},{:.4})",
                                crate::path::angle_diff_deg(h0, h1).abs(),
                                b.lon,
                                b.lat,
                                c.lon,
                                c.lat
                            );
                        }
                    }
                }
            }
            if !advanced {
                out.push(path.points[j]);
                i = j;
            }
        } else {
            i += 1;
            out.push(path.points[i.min(path.len() - 1)]);
        }
    }
    if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
        eprintln!(
            "[ts-dbg] RESULT n={} path={}",
            out.len(),
            out.iter()
                .map(|p| format!("({:.4},{:.4})", p.lon, p.lat))
                .collect::<Vec<_>>()
                .join(" -> ")
        );
    }
    Path::new(out)
}

/// Theta* fallback 急转弯的圆弧过渡插入（2026-08-07 zigzag18）。
/// 绕多边形窄通道（poly2 北顶点 / poly1 南边）时 FMM 楼梯 90° 角无法被跳点拉直
/// （向北/向西都穿墙），fallback 直接保留相邻点 → 转弯 > max_turn → verify 拒 →
/// 全链失败回退密集锯齿。这里在急转弯处用等距投影圆弧插入过渡点：从入段方向
/// 转到出段方向，每步转弯 ≤ max_turn 且弧半径 ≥ min_r_m；插入点逐段 check
/// （不穿墙 + 离墙净距）。成功返回 (插入点[含末点 E], 接续点索引 k)；失败返回
/// None（调用方保持原 fallback，宁丑勿违）。
/// `find_k`：false 时跳过 E 后的接续点搜索（2026-08-08 solver 段边界修复——
/// 只需弧点，后续段尚未拼接，k 无意义）。
/// `min_steps`：最小细分段数（2026-08-11 zz33）。大掉头（θ≈167°）时 ceil(θ/60)=3
/// 段，弧末段步进 55.6°；boundary arc 外推 E' 到出段直线后，倒数第二点（距 b 仅
/// ~730m）→E'（距 b 1768m）方向偏离弧末段 → 转角 101°>60° → verify 拒。细分到
/// 5 段后末段步进 33.4°，p_{n-1}→E' 趋近出段方向（转角≈步进<60）。0 = 默认
/// ceil(θ/max_turn)。
pub(crate) fn arc_transition(
    a: &crate::path::PathPoint,
    b: &crate::path::PathPoint,
    c: &crate::path::PathPoint,
    max_turn_deg: f64,
    min_r_m: f64,
    check: &SegmentCheck,
    path: &[crate::path::PathPoint],
    i: usize,
    find_k: bool,
    keep_b: bool,
    min_steps: usize,
) -> Option<(Vec<crate::path::PathPoint>, usize)> {
    use crate::path::{angle_diff_deg, bearing_deg};
    let h_ab = heading_deg_pts(a, b);
    let h_bc = heading_deg_pts(b, c);
    // 有向转角：angle_diff_deg(x, y) = normalize(x − y)；取 (h_bc − h_ab) →
    // 顺时针为正（右转 +，左转 −）。注意 2026-08-07 zigzag18 曾用 (h_ab − h_bc)
    // 作 dir，北→西（左转 94°）被当右转 → 圆心放错侧 → 弧向东鼓包 170° 折返。
    let signed = angle_diff_deg(h_bc, h_ab);
    let theta = signed.abs();
    if theta <= max_turn_deg || theta < 1e-6 {
        return None;
    }
    // +1 右转（顺时针），-1 左转。**180° 掉头边界**：angle_diff_deg 归一化
    // 到 [-180,180) 时 x−y=180 恰返回 −180 → side=−1 → 弧从 S 沿入段切线
    // **反向**转出（2026-08-10 zigzag25：mid 恰在雷达盘内，SEG0 从南侧绕入
    // 向北、SEG1 向南出盘 → 段边界 180° 掉头 → 弧折返 180° → FINAL 拒）。
    // 180° 左右转等价，强制 side=+1：弧从 S 沿入段切线方向（向 b 侧）转出，
    // U 形弯方向正确。
    let side = if theta > 179.999 {
        1.0
    } else if signed > 0.0 {
        1.0
    } else {
        -1.0
    };
    let n = ((theta / max_turn_deg).ceil() as usize).max(min_steps).max(2);
    let delta = theta / n as f64;
    // 步长（米）：保证弧半径 ≥ min_r_m，且不小于 2km（避免点过密）。
    // keep_b（必经点/段端点硬点）：用物理转弯半径——弧紧贴 b（切点偏差
    // r·tan(θ/2) 小，必经点语义保留）；大半径弧会让 E 远离 b → 越过后段
    // 下一节点 → 拼接折返（2026-08-10 zigzag25：E 距 b 2.6km > c 距 b
    // 0.7km → E→c 反向 178°）。
    let half = (delta / 2.0).to_radians();
    let step_m = if keep_b {
        (2.0 * min_r_m * half.sin()).max(50.0)
    } else {
        (2.0 * min_r_m * half.sin()).max(2_000.0)
    };
    let r_m = step_m / (2.0 * half.sin()).max(1e-9);
    let d_m_raw = r_m * (theta / 2.0).to_radians().tan();
    // 弧末点 E 不得越过出段下一节点 c：E 在 c 的出段侧 → E→c 反向折返。
    // 截断 d_m ≤ 0.75×|bc|（有效半径同步缩小，弧点转角不变）。
    // keep_b（必经点/段端点硬点）**不用切弧模型**（2026-08-10 zigzag29）：
    // 切弧的 S/E 在 b 两侧 d_m=r·tan(θ/2) 处，θ→180° 时 d_m 发散——174.9°
    // 大转角 d_m≈10km，必经点 b 被甩到弧外 10km（远超 0.05°≈5.5km 容差），
    // 且 bc 短时 d_m 截断 → r_eff<物理半径 → verify radius 拒 → 全链回退
    // 2549 点网格楼梯。改用 **U 形弧（S=b）**：弧从必经点出发（b 在弧上，
    // 精确经过），半径恒 min_r_m，转 θ 后 E 为弧上自然点——E 距 b 仅
    // 2r·sin(θ/2)（174.9° 时 883m），E 不在出段直线上但 E→c 方向偏差角
    // ≈atan(2r·sin²(θ/2)/|bc|)<3°（c 距 b≥数 km），verify 转角放行。
    let bc_m = crate::path::haversine_m(b.lon, b.lat, c.lon, c.lat);
    let d_m = d_m_raw.min(bc_m * 0.75);
    // 等距投影 dest（以 b 为中纬；弧长 ~10km 级，误差 <0.1%）。
    let lat0 = b.lat.to_radians();
    let kx = 111_320.0 * lat0.cos();
    let ky = 111_320.0;
    let dest = |lon: f64, lat: f64, h_deg: f64, dist_m: f64| -> (f64, f64) {
        let e = dist_m * h_deg.to_radians().sin();
        let n_ = dist_m * h_deg.to_radians().cos();
        (lon + e / kx, lat + n_ / ky)
    };
    // **180° 掉头无平滑弧**：入/出段同线（S/E 必须在同一条直线上）时，
    // 切线约束（S 处=入段方向、E 处=出段方向）与直径几何矛盾；且
    // tan(θ/2)=tan(90°) 发散 → 标准弧退化（d_m 截断后 r_eff≈0）。
    // theta_star 首跳 95° 放宽后（2026-08-10 zigzag25）此分支不应触发；
    // 触发则宁丑勿违（返回 None，调用方保持原样）。
    if theta > 179.999 {
        return None;
    }
    // keep_b：S=b（弧从必经点出发，U 形弯）；非 keep_b：S 在入段直线上
    // b 前 d_m 处（切弧模型，b 是两切线交点，弧绕过 b 急转弯点）。
    let (s_lon, s_lat) = if keep_b {
        (b.lon, b.lat)
    } else {
        dest(b.lon, b.lat, h_ab, -d_m)
    };
    let r_eff = if keep_b {
        min_r_m
    } else if d_m_raw > 0.0 {
        d_m / (theta / 2.0).to_radians().tan()
    } else {
        r_m
    };
    // 半径方向（center→S）：右转圆心在入段右侧（半径 = h_ab−90，φ 随弧顺时针增），
    // 左转圆心在入段左侧（半径 = h_ab+90，φ 逆时针减）。圆心在 S 的对面：
    // center = S − r·(sin φ0, cos φ0)；P(t) = center + r·(sin φ(t), cos φ(t))，
    // φ(t) = φ0 + side·t·θ —— t=0 恰为 S、t=1 恰为 E，且切线 = 入/出段航向。
    let phi0 = h_ab - side * 90.0;
    let (c_lon, c_lat) = dest(s_lon, s_lat, phi0 + 180.0, r_eff);
    let mut pts: Vec<crate::path::PathPoint> = Vec::with_capacity(n + 1);
    pts.push(crate::path::PathPoint {
        lon: s_lon,
        lat: s_lat,
        alt_m: b.alt_m,
        heading_deg: None,
    });
    for k in 1..=n {
        let t = k as f64 / n as f64;
        let h_rad = (phi0 + side * t * theta).to_radians();
        let (p_lon, p_lat) = (c_lon + r_eff * h_rad.sin() / kx, c_lat + r_eff * h_rad.cos() / ky);
        pts.push(crate::path::PathPoint {
            lon: p_lon,
            lat: p_lat,
            alt_m: b.alt_m,
            heading_deg: None,
        });
    }
    // 末点：keep_b 用弧上自然点（U 形弯，E 由弧几何决定，不在出段直线上
    // 但 E→c 方向偏差 <3°）；非 keep_b 用出段直线上的 E=dest(b,h_bc,d_m)。
    let (e_lon, e_lat) = if keep_b {
        (pts[n].lon, pts[n].lat)
    } else {
        dest(b.lon, b.lat, h_bc, d_m)
    };
    pts[n] = crate::path::PathPoint {
        lon: e_lon,
        lat: e_lat,
        alt_m: b.alt_m,
        heading_deg: None,
    };
    // 逐段 check：入段 a→S 及弧内每段合法才接受。
    if !check(a.lon, a.lat, a.alt_m, s_lon, s_lat, b.alt_m) {
        return None;
    }
    let mut prev = pts[0];
    for p in pts.iter().skip(1) {
        if !check(prev.lon, prev.lat, prev.alt_m, p.lon, p.lat, p.alt_m) {
            return None;
        }
        prev = *p;
    }
    // 从 E 找接续点 k：E→path[k] 方向与出段方向夹角 ≤ max_turn 且 check 合法。
    // 从 i+1 开始：E 在 c 前（d_m ≤ |bc|）则 E→c 方向 = h_bc（0° 转角）自然选中；
    // E 在 c 后则 E→c 反向（≈180°）自动被转弯约束拒掉，继续向后找。
    let e = crate::path::PathPoint {
        lon: e_lon,
        lat: e_lat,
        alt_m: b.alt_m,
        heading_deg: None,
    };
    if !find_k {
        return Some((pts, 0));
    }
    for k in (i + 1)..path.len() {
        let pk = path[k];
        let h_ek = bearing_deg(e.lon, e.lat, pk.lon, pk.lat);
        if angle_diff_deg(h_bc, h_ek).abs() <= max_turn_deg
            && check(e.lon, e.lat, e.alt_m, pk.lon, pk.lat, pk.alt_m)
        {
            return Some((pts, k));
        }
    }
    None
}

/// 两点航向（度，真北 0..360；大圆初始方位角）。
/// 必须与 verify 的 `Path::segment_heading_deg`（大圆 bearing）**完全同口径**——
/// 2026-08-07 zigzag15：Theta* 用等距投影检查转角放行 59.99°，verify 大圆算出
/// 60.66° > 60° 拒 → 该阶段被拒、后续链从 raw 继续全失败 → 回退 1978 点密集锯齿。
fn heading_deg_pts(a: &crate::path::PathPoint, b: &crate::path::PathPoint) -> f64 {
    crate::path::bearing_deg(a.lon, a.lat, b.lon, b.lat)
}

/// Chaikin 角切割平滑：每段取 1/4、3/4 插值点，保持端点。
/// `iterations` 轮（1-2 轮即够；过多会收敛到线段本身）。
pub fn chaikin_smooth(path: &Path, iterations: usize) -> Path {
    if path.len() < 3 || iterations == 0 {
        return path.clone();
    }
    let mut cur = path.clone();
    for _ in 0..iterations {
        if cur.len() < 3 {
            break;
        }
        let pts = &cur.points;
        let mut out = Vec::with_capacity(pts.len() * 2);
        out.push(pts[0]);
        for w in pts.windows(2) {
            let (a, b) = (w[0], w[1]);
            let q = |t: f64| PathPoint {
                lon: a.lon + (b.lon - a.lon) * t,
                lat: a.lat + (b.lat - a.lat) * t,
                alt_m: a.alt_m + (b.alt_m - a.alt_m) * t,
                heading_deg: None,
            };
            out.push(q(0.25));
            out.push(q(0.75));
        }
        out.push(*pts.last().unwrap());
        cur = Path::new(out);
    }
    cur
}

/// Catmull-Rom 样条：每段插 `samples_per_seg` 个点（含段内），保持端点。
/// 首末段用端点复制（自然边界）。
pub fn catmull_rom_spline(path: &Path, samples_per_seg: usize) -> Path {
    let n = path.len();
    if n < 3 || samples_per_seg == 0 {
        return path.clone();
    }
    let pts = &path.points;
    let mut out: Vec<PathPoint> = Vec::with_capacity(n * (samples_per_seg + 1));
    out.push(pts[0]);
    for i in 0..n - 1 {
        let p0 = if i == 0 { pts[0] } else { pts[i - 1] };
        let p1 = pts[i];
        let p2 = pts[i + 1];
        let p3 = if i + 2 < n { pts[i + 2] } else { pts[i + 1] };
        for k in 1..=samples_per_seg {
            let t = k as f64 / samples_per_seg as f64;
            let t2 = t * t;
            let t3 = t2 * t;
            let cr = |a: f64, b: f64, c: f64, d: f64| {
                0.5 * ((2.0 * b) + (-a + c) * t + (2.0 * a - 5.0 * b + 4.0 * c - d) * t2
                    + (-a + 3.0 * b - 3.0 * c + d) * t3)
            };
            out.push(PathPoint {
                lon: cr(p0.lon, p1.lon, p2.lon, p3.lon),
                lat: cr(p0.lat, p1.lat, p2.lat, p3.lat),
                alt_m: cr(p0.alt_m, p1.alt_m, p2.alt_m, p3.alt_m),
                heading_deg: None,
            });
        }
    }
    Path::new(out)
}

/// 贪心双指针抽稀：锚点 i，指针 j 从末尾向前找最大跳跃——(i, j) 内全部点
/// 相对线段 [i, j] 的弦高 ≤ 容差则保 j 删中间，否则 j 前移。
/// 保留首末点。复杂度 O(n²)（点列短可接受）。
pub fn greedy_simplify(path: &Path, chord_tol_m: f64) -> Path {
    let n = path.len();
    if n < 3 {
        return path.clone();
    }
    let mut keep: Vec<usize> = vec![0];
    let mut i = 0usize;
    while i < n - 1 {
        let mut j = n - 1;
        while j > i + 1 {
            let mut ok = true;
            for k in (i + 1)..j {
                let e = path.chord_error_m(k, i, j).unwrap_or(f64::INFINITY);
                if e > chord_tol_m {
                    ok = false;
                    break;
                }
            }
            if ok {
                break;
            }
            j -= 1;
        }
        keep.push(j);
        i = j;
    }
    if *keep.last().unwrap() != n - 1 {
        keep.push(n - 1);
    }
    Path::new(keep.into_iter().map(|k| path.points[k]).collect())
}

/// 局部等距投影（以中点为中心）：经纬度 → 平面米（y 向北，x 向东）。
pub struct LocalProjection {
    pub clon: f64,
    pub clat: f64,
    pub k: f64,
}

impl LocalProjection {
    pub fn new(clon: f64, clat: f64) -> Self {
        Self {
            clon,
            clat,
            k: clat.to_radians().cos().max(1e-6),
        }
    }
    pub fn to_xy(&self, lon: f64, lat: f64) -> (f64, f64) {
        ((lon - self.clon) * self.k * 111_320.0, (lat - self.clat) * 111_320.0)
    }
    pub fn to_lonlat(&self, x: f64, y: f64) -> (f64, f64) {
        (self.clon + x / (self.k * 111_320.0), self.clat + y / 111_320.0)
    }
}

/// 在折线上按弧长找对应高度（高程剖面独立插值）：采样点距离最近段线性插值。
fn height_on_polyline(path: &Path, lon: f64, lat: f64) -> f64 {
    let n = path.len();
    if n == 0 {
        return 0.0;
    }
    if n == 1 {
        return path.points[0].alt_m;
    }
    let mut best = f64::INFINITY;
    let mut h = path.points[0].alt_m;
    let proj = LocalProjection::new(lon, lat);
    let (px, py) = (0.0, 0.0);
    for w in path.points.windows(2) {
        let (ax, ay) = proj.to_xy(w[0].lon, w[0].lat);
        let (bx, by) = proj.to_xy(w[1].lon, w[1].lat);
        let (vx, vy) = (bx - ax, by - ay);
        let len2 = vx * vx + vy * vy;
        let t = if len2 <= f64::EPSILON {
            0.0
        } else {
            (((px - ax) * vx + (py - ay) * vy) / len2).clamp(0.0, 1.0)
        };
        let (cx, cy) = (ax + vx * t, ay + vy * t);
        let d = ((px - cx).powi(2) + (py - cy).powi(2)).sqrt();
        if d < best {
            best = d;
            h = w[0].alt_m + (w[1].alt_m - w[0].alt_m) * t;
        }
    }
    h
}

/// Dubins 2D 拟合 + 高程剖面独立插值：首末 pose（航向 = 显式 heading 或首末段航向），
/// 平面 = 局部等距投影；高度沿原折线插值（爬升角由复验检查）。
/// 无解（d < 2R 等）→ None。
pub fn dubins_fit(path: &Path, r_m: f64, sample_n: usize) -> Option<Path> {
    if path.len() < 2 {
        return None;
    }
    let first = path.points[0];
    let last = *path.points.last().unwrap();
    if !first.is_finite() || !last.is_finite() || !r_m.is_finite() || r_m <= 0.0 {
        return None;
    }
    let th0 = match first.heading_deg {
        Some(h) => h,
        None => path.segment_heading_deg(0)?,
    };
    let th1 = match last.heading_deg {
        Some(h) => h,
        None => path.segment_heading_deg(path.len() - 2)?,
    };
    // 航向（度，真北）→ 平面角（rad，y 向北：北=90° → π/2；Dubins 坐标系 y 向上、航向逆时针）
    let ang = |hd: f64| hd.to_radians() - std::f64::consts::FRAC_PI_2;
    let clon = (first.lon + last.lon) / 2.0;
    let clat = (first.lat + last.lat) / 2.0;
    let proj = LocalProjection::new(clon, clat);
    let (x0, y0) = proj.to_xy(first.lon, first.lat);
    let (x1, y1) = proj.to_xy(last.lon, last.lat);
    let path2 = dubins_path((x0, y0), ang(th0), (x1, y1), ang(th1), r_m)?;
    let pts2 = path2.sample(sample_n.max(4));
    let out: Vec<PathPoint> = pts2
        .into_iter()
        .map(|(x, y)| {
            let (lon, lat) = proj.to_lonlat(x, y);
            PathPoint::new(lon, lat, height_on_polyline(path, lon, lat))
        })
        .collect();
    Some(Path::new(out))
}

#[cfg(test)]
mod base_tests {
    use super::*;
    use crate::path::haversine_m;

    fn straight_path(n: usize) -> Path {
        Path::new(
            (0..n)
                .map(|i| PathPoint::new(i as f64 * 0.01, 0.0, 100.0))
                .collect(),
        )
    }

    #[test]
    fn theta_star_removes_collinear() {
        // 全共线点：check 恒 true → 应压缩为两点
        let p = straight_path(20);
        let out = theta_star_smooth(&p, &|_, _, _, _, _, _| true, None, None, 0.0, 95.0);
        assert_eq!(out.len(), 2, "got {}", out.len());
    }

    #[test]
    fn theta_star_blocked_mid() {
        // 中间点直连被 check 拒绝：保留分段
        let p = straight_path(10);
        // 只允许相邻直连（j=i+1 总是允许）
        let out = theta_star_smooth(&p, &|_, _, _, _, _, _| false, None, None, 0.0, 95.0);
        assert_eq!(out.len(), p.len(), "got {}", out.len());
    }

    #[test]
    fn theta_star_first_jump_respects_entry_heading() {
        // 段边界回归（2026-08-07 zigzag11）：Theta* 首跳（段首 index0 → j）必须受
        // 入航向约束，否则拼接后段边界转角超限（climb 点 61.94°>60°）→ 终检拒 →
        // 全链回退 1755 点网格楼梯。构造：入航向朝东（0°），首跳点向东北偏 70°
        // （超 60°）应被拒；放宽到 90° 时允许。
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 100.0),
            PathPoint::new(0.01, 0.01, 100.0), // 偏 45°（首跳候选）
            PathPoint::new(0.02, 0.02, 100.0),
            PathPoint::new(0.03, 0.03, 100.0),
        ]);
        // 入航向 0°（东），首跳点方向 45°（东北）→ 45° <= 60° 允许
        let out = theta_star_smooth(&p, &|_, _, _, _, _, _| true, Some(60.0), Some(0.0), 0.0, 95.0);
        assert_eq!(out.len(), 2, "45° 首跳应允许: got {}", out.len());
        // 入航向 90°（北），首跳点方向 45° → 差 45° <= 60° 允许
        let out = theta_star_smooth(&p, &|_, _, _, _, _, _| true, Some(60.0), Some(90.0), 0.0, 95.0);
        assert_eq!(out.len(), 2, "45° 差应允许: got {}", out.len());
        // 入航向 180°（西），首跳点方向 45° → 差 135° > 60° → 拒跳，退邻点
        // （首跳被拒但后续跳点仍可拉直 → 3 点：(0,0)→(0.01,0.01)→(0.03,0.03)）
        let out = theta_star_smooth(&p, &|_, _, _, _, _, _| true, Some(60.0), Some(180.0), 0.0, 95.0);
        assert_eq!(out.len(), 3, "135° 首跳应拒: got {}", out.len());
        // 无入航向（起点段）→ 不约束首跳（保持原语义）
        let out = theta_star_smooth(&p, &|_, _, _, _, _, _| true, Some(60.0), None, 0.0, 95.0);
        assert_eq!(out.len(), 2, "无入航向应直接拉直: got {}", out.len());
    }

    #[test]
    fn chaikin_endpoints_kept() {
        let p = straight_path(5);
        let out = chaikin_smooth(&p, 1);
        assert_eq!(out.points[0].lon, p.points[0].lon);
        assert_eq!(out.last().unwrap().lon, p.last().unwrap().lon);
        assert!(out.len() > p.len());
        // 全共线 → 中间点仍在直线上
        for pt in &out.points {
            assert!(pt.lat.abs() < 1e-12);
        }
    }

    #[test]
    fn catmull_endpoints_kept() {
        let p = straight_path(5);
        let out = catmull_rom_spline(&p, 4);
        assert!((out.points[0].lon - p.points[0].lon).abs() < 1e-9);
        assert!((out.last().unwrap().lon - p.last().unwrap().lon).abs() < 1e-9);
        assert!(out.len() > p.len());
        for pt in &out.points {
            assert!(pt.lat.abs() < 1e-9, "catmull on line deviates: {}", pt.lat);
        }
    }

    #[test]
    fn greedy_simplify_tolerance() {
        // 锯齿路径：相邻点偏离基线，但容差内应被抽稀
        let pts: Vec<PathPoint> = (0..10)
            .map(|i| {
                let lat = if i % 2 == 0 { 0.0 } else { 0.001 }; // ~111m 偏移
                PathPoint::new(i as f64 * 0.01, lat, 100.0)
            })
            .collect();
        let p = Path::new(pts);
        let out = greedy_simplify(&p, 200.0); // 容差 > 偏移 → 抽到首末
        assert_eq!(out.len(), 2, "got {}", out.len());
        let out2 = greedy_simplify(&p, 50.0); // 容差 < 偏移 → 保留锯齿点
        assert!(out2.len() >= 6, "got {}", out2.len());
    }

    #[test]
    fn dubins_fit_horizontal() {
        // 水平直线 1° 经度（~111km），R=5km：Dubins 可解
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 100.0).with_heading(90.0),
            PathPoint::new(1.0, 0.0, 100.0).with_heading(90.0),
        ]);
        let out = dubins_fit(&p, 5_000.0, 32).expect("dubins fit");
        assert!(out.len() >= 4);
        // 首末点保持
        let d0 = haversine_m(out.points[0].lon, out.points[0].lat, 0.0, 0.0);
        let d1 = haversine_m(
            out.last().unwrap().lon,
            out.last().unwrap().lat,
            1.0,
            0.0,
        );
        assert!(d0 < 100.0 && d1 < 100.0, "d0={d0} d1={d1}");
        // 高度剖面保持 100
        for pt in &out.points {
            assert!((pt.alt_m - 100.0).abs() < 1e-6);
        }
    }

    #[test]
    fn dubins_fit_too_close_none() {
        // 垂直转向近距（11m）：所有圆心距 < 2R → 无解 → None（不 panic）
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 100.0).with_heading(90.0), // 朝东
            PathPoint::new(0.0001, 0.0, 100.0).with_heading(0.0), // 朝北
        ]);
        assert!(dubins_fit(&p, 5_000.0, 32).is_none());
        // 非有限输入 → None
        let bad = Path::new(vec![
            PathPoint::new(f64::NAN, 0.0, 100.0).with_heading(90.0),
            PathPoint::new(1.0, 0.0, 100.0).with_heading(90.0),
        ]);
        assert!(dubins_fit(&bad, 5_000.0, 32).is_none());
    }

    #[test]
    fn dubins_fit_same_heading_close_s_shape() {
        // 同向近距（11m < 2R）：RSL 内切线圆心距略 > 2R → 刚可解（S 形），
        // 首末 pose 必须精确匹配（防 phase0 center 单位尺度 bug 回归，B5 未覆盖）。
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 100.0).with_heading(90.0),
            PathPoint::new(0.0001, 0.0, 100.0).with_heading(90.0),
        ]);
        let out = dubins_fit(&p, 5_000.0, 64).expect("RSL S-shape solvable");
        let proj = LocalProjection::new(0.0, 0.0);
        let (x0, y0) = proj.to_xy(out.points[0].lon, out.points[0].lat);
        assert!(x0.abs() < 1.0 && y0.abs() < 1.0, "start ({x0},{y0})");
        let (x1, y1) = proj.to_xy(out.last().unwrap().lon, out.last().unwrap().lat);
        let (ex, ey) = proj.to_xy(0.0001, 0.0);
        assert!(
            (x1 - ex).abs() < 1.0 && (y1 - ey).abs() < 1.0,
            "end ({x1},{y1}) vs ({ex},{ey})"
        );
        // 末航向 ≈ 东（90°）：末两点方向
        let n = out.len();
        let (ax, ay) = proj.to_xy(out.points[n - 2].lon, out.points[n - 2].lat);
        let (bx, by) = proj.to_xy(out.points[n - 1].lon, out.points[n - 1].lat);
        let hd = ((by - ay).atan2(bx - ax)).to_degrees() + 90.0;
        let dh = crate::path::angle_diff_deg(hd, 90.0);
        assert!(dh.abs() < 5.0, "final heading {hd} (dh={dh})");
    }

    #[test]
    fn projection_roundtrip() {
        let pr = LocalProjection::new(116.4, 39.9);
        let (x, y) = pr.to_xy(116.4, 39.9);
        assert!(x.abs() < 1e-6 && y.abs() < 1e-6);
        let (lon, lat) = pr.to_lonlat(x + 1000.0, y + 1000.0);
        // 1km 误差 < 2m（局部等距近似）
        let d = haversine_m(116.4, 39.9, lon, lat);
        assert!((d - 1414.2).abs() < 2.0, "d={d}");
    }
}

// ==================== 复验器（全链复验清单） ====================

/// 复验报告。
#[derive(Debug, Clone, Default)]
pub struct VerifyReport {
    pub ok: bool,
    /// 硬性违规（导致不通过）。
    pub issues: Vec<String>,
    /// 软性提示（不阻断，但下游可见；如 NoData 净空不确定）。
    pub warnings: Vec<String>,
}

/// 复验上下文：地形净空 + 禁飞/限飞包含 + 雷达威胁（接口化；Phase 4 机型接入后扩展）。
#[derive(Default)]
pub struct VerifyContext<'a> {
    pub terrain: Option<&'a dyn crate::terrain::TerrainSource>,
    pub nofly: Option<&'a crate::spatial::CircleIndex>,
    /// Phase 4 M2 高度层：完整 Zone 语义（水平 + [alt_min, alt_max] + AGL 换算）。
    /// 提供时优先于 nofly（nofly 仅水平圆快查）；None 时回退 nofly。
    pub zones: Option<&'a [crate::config::Zone]>,
    /// Phase 4 M3 雷达威胁模型：累计探测概率超 P_cross → 软性告警（degradation，不阻断平滑）。
    pub threat: Option<&'a dyn crate::threat::ThreatModel>,
    /// Zone 硬墙（NoFly/Obstacle）水平膨胀距离（m；主管 2026-08-06：绕飞贴边→考虑
    /// 飞机机动——绕行需留物理转弯空间）。段到墙净距 < 该值即判定不合法；0 = 不膨胀。
    pub zone_inflation_m: f64,
}

/// 全链复验（十轮复验清单）：
/// { 地形净空 } ∪ { 禁飞/限飞包含（A4/A5 硬阈值） } ∪ { 转弯半径/转弯角/航向连续性（A3） }
/// ∪ { 爬升角 } ∪ { 弦高（几何，相对美化前参考路径） } ∪ { A6 速度-转弯半径自洽（参数化，缺省跳过） }。
///
/// 机型分支（旋翼机可悬停/极小转弯半径，九轮共识）：
/// `AircraftType::Rotorcraft` 跳过固定翼运动学硬拦（转弯半径/转角/爬升角）——
/// 悬停原地转向/垂直机动在位置空间表现为急转（类方波/尖角），是**合法航路**，不得误判；
/// 净空/禁飞/弦高对两机型一律硬约束。A6 仅固定翼物理自洽（r_min ≥ v²/(g·tan φ_max)）。
///
/// - 采样密度：每段等距 `verify_seg_samples` 点（段内最坏情况，端点+中点+加密）；
/// - 地形净空：Land → z ≥ h + clearance；Water/Lake → 水面（净空 0 起算）；
///   NoData → 降级警告（空洞策略：净空不确定但不阻断，调用方汇总进 degradations）；
///   OutOfBounds → 不通过（越界墙）；
/// - 转弯半径：三点外接圆半径（等距平面近似），≥ opts.turn_radius_m（仅固定翼）；
/// - 转角：相邻段航向差 ≤ opts.max_turn_deg（仅固定翼）；
/// - 爬升角：段高度差/水平距离 ≤ tan(max_climb_deg)（仅固定翼）；
/// - 弦高：相对 `reference`（美化前）逐点弦高 ≤ opts.chord_tol_m（几何逼近误差）；
/// - A6：`phys_min_radius_m` 提供时校验 r_min ≥ phys_min_radius_m（参数化，Phase 4 接入）。
/// 段-圆水平相交参数区间（解析二次方程，局部等距投影）。返回 Some((t1,t2))
/// 当相交区间与 [0,1] 有重叠（含边界接触，保守）；否则 None。
/// 统一 check（Theta* 拉直）与 verify 的圆判定口径——此前 check 用净距
/// （起点纬度投影，圆边缘 ±2% 误差可翻转"穿/不穿"判定），verify 用解析法，
/// 边缘场景 check 放行 verify 会拒的穿区段（2026-08-06 zigzag9：theta_star
/// 拉直段擦过 restricted 圆边缘被放行 → verify 拒 → 全链失败回退密集楼梯）。
pub(crate) fn segment_circle_intersect_t(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    cx: f64,
    cy: f64,
    r_km: f64,
) -> Option<(f64, f64)> {
    let mlat = ((lat1 + lat2) / 2.0).to_radians();
    let kx = mlat.cos() * 111.32;
    let ky = 111.32;
    let dx = (lon2 - lon1) * kx;
    let dy = (lat2 - lat1) * ky;
    let ox = (cx - lon1) * kx;
    let oy = (cy - lat1) * ky;
    let aa = dx * dx + dy * dy;
    if aa < 1e-12 {
        return None;
    }
    let bb = -2.0 * (dx * ox + dy * oy);
    // 长段投影误差补偿（2026-08-08 主管 zigzag22）：等距投影把段当平面直线，但
    // 实际大圆路径在北半球向极地弯曲；1200km 长段上投影与大圆偏差 ~0.5-3km，
    // 贴圆擦过（真实穿入 0.5km）会被投影算成不穿 → verify/check 放行 → 交付穿
    // restricted 高度带。求交用 r+slack（slack=max(1km, 段长×0.5%)）扩大搜索区间，
    // 区间内采样点由 zone_contains_at 精确（球面距离）判定——不会误报合法贴边。
    let seg_km = aa.sqrt();
    let slack = (seg_km * 0.005).max(1.0);
    let cc = ox * ox + oy * oy - (r_km + slack) * (r_km + slack);
    let disc = bb * bb - 4.0 * aa * cc;
    if disc <= 0.0 {
        return None;
    }
    let sq = disc.sqrt();
    let t1 = ((-bb - sq) / (2.0 * aa)).clamp(0.0, 1.0);
    let t2 = ((-bb + sq) / (2.0 * aa)).clamp(0.0, 1.0);
    if t2 <= 0.0 || t1 >= 1.0 {
        return None;
    }
    Some((t1, t2))
}

#[allow(clippy::too_many_arguments)]
pub fn verify_path(
    path: &Path,
    reference: Option<&Path>,
    opts: &SmoothOptions,
    ctx: &VerifyContext,
    phys_min_radius_m: Option<f64>,
) -> VerifyReport {
    let mut rep = VerifyReport::default();
    // 路径级去重：数据范围外（OOB）采样点逐点警告会洪水（起点/终点在外部小范围
    // 地形外时整段 OOB，~200m 一点 → 数百条）——整个路径只报首条（2026-08-11）。
    let mut oob_warned = false;
    let n = path.len();
    if n < 2 {
        rep.issues.push("path has < 2 points".into());
        rep.ok = false;
        return rep;
    }
    for pt in &path.points {
        if !pt.is_finite() {
            rep.issues.push("non-finite path point".into());
            rep.ok = false;
            return rep;
        }
    }

    // 固定翼运动学约束（旋翼机跳过：悬停原地转向/垂直机动为合法急转/陡爬升）
    let kinematic = opts.aircraft_type == AircraftType::FixedWing;

    // 1) 航向连续性 + 转角 + 转弯半径 + 爬升角（逐段/逐三点）
    let proj = {
        let mid = path.points[n / 2];
        LocalProjection::new(mid.lon, mid.lat)
    };
    let xy: Vec<(f64, f64)> = path.points.iter().map(|p| proj.to_xy(p.lon, p.lat)).collect();
    for i in 1..n {
        // 爬升角（仅固定翼）
        if kinematic {
            let (ax, ay) = xy[i - 1];
            let (bx, by) = xy[i];
            let horiz = ((bx - ax).powi(2) + (by - ay).powi(2)).sqrt();
            let dh = (path.points[i].alt_m - path.points[i - 1].alt_m).abs();
            if horiz > 1e-9 {
                let climb = (dh / horiz).atan().to_degrees();
                if climb > opts.max_climb_deg {
                    rep.issues.push(format!(
                        "segment {i}: climb {climb:.2}deg > max {:.1}deg",
                        opts.max_climb_deg
                    ));
                }
            }
        }
        if i + 1 < n && kinematic {
            // 转角 + 转弯半径（点 i 处，三点 (i-1, i, i+1)）。
            // 原 `i > 1` 盲区：段内 index 1 的点（Theta* 首跳点/段边界点）单段
            // 平滑时不查转角，拼接后 index 变大终检才暴露段边界转角超限，如
            // 2026-08-07 主管 1755 点场景 vertex 5 turn 61.94° > 60°。统一语义：
            // 点 i 的转角 = angle(seg(i-1), seg(i))，半径 = 三点外接圆——对全部
            // 内部点（i = 1..n-2）生效；末点无出段，跳过。
            let h0 = path.segment_heading_deg(i - 1).unwrap_or(0.0);
            let h1 = path.segment_heading_deg(i).unwrap_or(h0);
            let turn = angle_diff_deg(h0, h1).abs();
            if turn > opts.max_turn_deg {
                rep.issues.push(format!(
                    "vertex {i}: turn {turn:.2}deg > max {:.1}deg",
                    opts.max_turn_deg
                ));
            }
            // 半径是**局部几何量**，必须用三点局部投影测量（2026-08-10 zigzag28：
            // verify 曾用全路径中点投影测弧半径，弧远离路径中点时等距投影经度
            // 缩放失配（cos 随纬度变化 0.8%）→ 442m 物理转弯弧被量成 434m <
            // 0.99×442=437.58 → 误拒 → 全链回退 1017 点网格锯齿。arc_transition
            // 生成弧用 b 点局部投影，验证同样要局部口径，二者才一致）。
            let p0 = &path.points[i - 1];
            let p1 = &path.points[i];
            let p2 = &path.points[i + 1];
            let lp = LocalProjection::new(
                (p0.lon + p1.lon + p2.lon) / 3.0,
                (p0.lat + p1.lat + p2.lat) / 3.0,
            );
            let (a, b, c) = (
                lp.to_xy(p0.lon, p0.lat),
                lp.to_xy(p1.lon, p1.lat),
                lp.to_xy(p2.lon, p2.lat),
            );
            if let Some(r) = circumradius(a, b, c)
                && r < opts.turn_radius_m * 0.99
            {
                rep.issues.push(format!(
                    "vertex {i}: radius {r:.0}m < min {:.0}m",
                    opts.turn_radius_m
                ));
            }
        }
    }

    // 2) 地形净空 + 禁飞包含（每段等距采样）
    // 采样密度按段长自适应（2026-08-10 主管输入撞山修复）：固定 verify_seg_samples
    // 点对长段（>20km）采样间隔过大，山峰会漏过——276km 直线段 9 点采样间隔
    // ~30km，2137m 峰被漏 → 穿山路径通过复验交付（dubins 96 点段短采样密能发现，
    // 链却选点数最少的 2 点直线）。目标间隔 ~1km（7.5as 地形 ~230m 分辨率，
    // 1km 足够捕捉山峰宽度），下限 = 原固定值。
    let infl_km = ctx.zone_inflation_m / 1000.0;
    for i in 1..n {
        let a = path.points[i - 1];
        let b = path.points[i];
        let seg_len_m = crate::path::haversine_m(a.lon, a.lat, b.lon, b.lat);
        // 目标间隔 ~200m（7.5as 地形 ~230m 分辨率）：段边界弧插入后起点偏移
        // （E 距 b 数百米）会使 1km 采样网格整体移相，窄山峰（宽 <1km）可被
        // 网格错过（2026-08-10 zigzag29：SEG23 121km 直线 terrain 2082m 峰，
        // check 与 verify 相位差 379m → 一个抓到另一个漏）。200m < 格距 230m，
        // 任意相位下 ≥1 个采样点必落入 ≥230m 宽的山峰格。下限 = 原固定值。
        let segs = ((seg_len_m / 200.0).ceil() as usize).max(opts.verify_seg_samples.max(2));
        // Zone 硬墙（NoFly/Obstacle）：段到墙水平净距 ≥ zone_inflation_m
        // （几何精确，无采样漏判；主管 2026-08-06：绕飞贴边→考虑飞机机动留转弯空间。
        //  inflation=0 时仍拒绝穿入（clr≤0）——与原 zone_contains_at 采样语义一致）
        if let Some(zs) = ctx.zones {
            for z in zs {
                if z.is_wall() {
                    let clr = crate::config::zone_segment_clearance_km(
                        a.lon, a.lat, b.lon, b.lat, z,
                    );
                    if clr <= 1e-9 || clr < infl_km {
                        rep.issues.push(format!(
                            "segment {i}: clearance {clr:.2}km < inflation {infl_km:.2}km (zone wall)"
                        ));
                    }
                }
            }
            // Restricted（非墙，高度层语义）：**几何精确**穿圆判定——等距采样会漏掉
            // 浅穿/短弦（段贴圆擦过时采样点可能全在圆外，如 2026-08-06 new_rz 浅穿
            // 0.5km、弦 8.9km 但采样点漏过 → 3000m 直穿 restricted 违规交付）。
            // 段与圆相交区间 [t1,t2]（解析二次方程，局部等距投影），区间内采样高度。
            for z in zs {
                if z.is_wall() {
                    continue;
                }
                let crate::config::ZoneShape::Circle { center, radius_km } = &z.shape else {
                    continue;
                };
                let (cx, cy, r) = (center[0], center[1], *radius_km);
                let Some((t1, t2)) =
                    segment_circle_intersect_t(a.lon, a.lat, b.lon, b.lat, cx, cy, r)
                else {
                    continue; // 段直线不穿圆
                };
                for kk in 0..=8 {
                    let tt = t1 + (t2 - t1) * kk as f64 / 8.0;
                    let (lon, lat, alt) = (
                        a.lon + (b.lon - a.lon) * tt,
                        a.lat + (b.lat - a.lat) * tt,
                        a.alt_m + (b.alt_m - a.alt_m) * tt,
                    );
                    if let Ok(g) = crate::coord::Geo::new(lon, lat) {
                        let ground = ctx.terrain.and_then(|t| t.height_at(lon, lat));
                        if crate::config::zone_contains_at(z, &g, alt, ground) {
                            rep.issues.push(format!(
                                "segment {i}: inside zone (alt band) at t={tt:.3} alt={alt:.0}"
                            ));
                            break;
                        }
                    }
                }
            }
        }
        for k in 0..=segs {
            let t = k as f64 / segs as f64;
            let (lon, lat, alt) = (
                a.lon + (b.lon - a.lon) * t,
                a.lat + (b.lat - a.lat) * t,
                a.alt_m + (b.alt_m - a.alt_m) * t,
            );
            if let Some(zs) = ctx.zones {
                let geo_ok = crate::coord::Geo::new(lon, lat).ok();
                if let Some(g) = geo_ok {
                    // 完整 Zone 语义：水平 + 高度区间（AGL 需地面高度）；硬墙已在上面
                    // 净距检查覆盖（全高度墙），这里只查 Restricted（高度层语义）。
                    let ground = ctx.terrain.and_then(|t| t.height_at(lon, lat));
                    if zs
                        .iter()
                        .any(|z| !z.is_wall() && crate::config::zone_contains_at(z, &g, alt, ground))
                    {
                        rep.issues.push(format!(
                            "sample (lon={lon:.4},lat={lat:.4},alt={alt:.0}) inside zone (alt band)"
                        ));
                    }
                }
            } else if let Some(circ) = ctx.nofly
                && !circ.containing(lon, lat).is_empty()
            {
                rep.issues.push(format!(
                    "sample (lon={lon:.4},lat={lat:.4}) inside no-fly zone"
                ));
            }
            if let Some(ter) = ctx.terrain {
                match ter.sample_at(lon, lat) {
                    crate::terrain::Sample::Land(h) => {
                        if alt < h + opts.clearance_m {
                            rep.issues.push(format!(
                                "sample (lon={lon:.4},lat={lat:.4}) clearance {:.0}m < {:.0}m (terrain {h:.0}m)",
                                alt - h, opts.clearance_m
                            ));
                        }
                    }
                    crate::terrain::Sample::Water | crate::terrain::Sample::Lake(_) => {
                        if alt < opts.clearance_m {
                            rep.issues.push(format!(
                                "sample (lon={lon:.4},lat={lat:.4}) water clearance {alt:.0}m < {:.0}m",
                                opts.clearance_m
                            ));
                        }
                    }
                    crate::terrain::Sample::NoData => {
                        // 空洞策略（主管 2026-08-04）：不设数据合格判断，对任意空洞形态
                        // 给出可用结果，最坏降级警告进 stats.degradations。空洞处高度未知
                        // 不阻断航路——固定端点（start/target）落在空洞时硬拒会让全链被拒、
                        // 回退密集网格楼梯（主管 2026-08-06 zigzag8：渤海/内蒙 NoData 洞
                        // → 1196 点锯齿）。降级警告由调用方汇总进 degradations。
                        rep.warnings.push(format!(
                            "sample (lon={lon:.4},lat={lat:.4}) NoData terrain: clearance unknown (degraded)"
                        ));
                    }
                    crate::terrain::Sample::OutOfBounds => {
                        // 2026-08-11 放开输入点限制：数据范围外同 NoData——
                        // 高度未知不阻断，降级警告（起终点落在小范围外部地形外时
                        // 平滑链不再被 OOB 硬拒，回退密集网格楼梯）。逐点去重：
                        // 整个路径只报首条，避免数百条洪水。
                        if !oob_warned {
                            oob_warned = true;
                            rep.warnings.push(format!(
                                "sample (lon={lon:.4},lat={lat:.4}) out of terrain bounds (degraded)"
                            ));
                        }
                    }
                    crate::terrain::Sample::Forbidden => {
                        rep.issues.push(format!(
                            "sample (lon={lon:.4},lat={lat:.4}) inside forbidden wall"
                        ));
                    }
                }
            }
        }
    }

    // 3) 弦高（相对美化前参考路径）
    if let Some(rev) = reference {
        for (idx, p) in path.points.iter().enumerate() {
            // 找参考路径中最近点做弦高近似：简化 = 到参考折线的最小距离
            let mut min_d = f64::INFINITY;
            for w in rev.points.windows(2) {
                let d = point_seg_distance_m(p.lon, p.lat, w[0].lon, w[0].lat, w[1].lon, w[1].lat);
                if d < min_d {
                    min_d = d;
                }
            }
            if min_d > opts.chord_tol_m {
                rep.issues.push(format!(
                    "point {idx}: chord {min_d:.0}m > tol {:.0}m",
                    opts.chord_tol_m
                ));
            }
        }
    }

    // 4) A6 速度-转弯半径自洽（仅固定翼；旋翼机 r_min→0 无物理下限）
    if opts.aircraft_type == AircraftType::FixedWing
        && let Some(r_phys) = phys_min_radius_m
        && opts.turn_radius_m < r_phys - 1e-9
    {
        rep.issues.push(format!(
            "A6: r_min {:.0}m < phys min {:.0}m",
            opts.turn_radius_m, r_phys
        ));
    }

    // 5) 雷达威胁（Phase 4 M3）：累计探测概率超 P_cross → 软性告警（不阻断平滑）。
    //    雷达是软约束：穿雷达区（避不开）时路径仍应平滑交付，概率超标由
    //    stats.degradations 记录；硬失败会迫使整条平滑链回退 → 网格锯齿暴露。
    if let Some(thr) = ctx.threat {
        let tr = thr.evaluate(path, ctx.terrain);
        if tr.over_threshold {
            rep.warnings.push(format!(
                "radar: cumulative detection p {:.4} > threshold {:.4}",
                tr.cumulative_p, thr.p_cross()
            ));
        } else if tr.cumulative_p > 0.0 {
            rep.warnings.push(format!(
                "radar: cumulative detection p {:.4} (peak {:.4})",
                tr.cumulative_p, tr.peak_p
            ));
        }
    }

    rep.ok = rep.issues.is_empty();
    rep
}

/// 三点外接圆半径（等距平面；共线 → None）。
pub fn circumradius(a: (f64, f64), b: (f64, f64), c: (f64, f64)) -> Option<f64> {
    let (ax, ay) = (b.0 - a.0, b.1 - a.1);
    let (bx, by) = (c.0 - a.0, c.1 - a.1);
    let det = ax * by - ay * bx;
    if det.abs() < 1e-12 {
        return None; // 共线
    }
    let (la, lb) = (ax * ax + ay * ay, bx * bx + by * by);
    let cx = (by * la - ay * lb) / (2.0 * det);
    let cy = (ax * lb - bx * la) / (2.0 * det);
    Some((cx * cx + cy * cy).sqrt())
}

// ==================== 平滑器策略链（trait 化 + 回退语义） ====================

/// 平滑策略 trait（链序执行；`smooth` 失败返回 None = 该策略不可用，保持当前路径）。
pub trait Smoother {
    fn name(&self) -> &str;
    fn smooth(&self, path: &Path) -> Option<Path>;
}

/// Theta* 去锯齿策略。
pub struct ThetaStarSmoother<'a> {
    pub check: &'a SegmentCheck<'a>,
    /// 航向连续性约束（相邻航段转角上限；None = 不限制）
    pub max_turn_deg: Option<f64>,
    /// 段首点入航向（度，真北；None = 不约束段首）
    pub entry_heading: Option<f64>,
    /// 圆弧过渡的最小转弯半径（米，物理约束；fallback 急转弯插入弧点用）
    pub min_r_m: f64,
    /// 首跳 entry 约束放宽上限（度；必经点段 175°，非必经点 95°）
    pub entry_max_deg: f64,
}
impl Smoother for ThetaStarSmoother<'_> {
    fn name(&self) -> &str {
        "theta_star"
    }
    fn smooth(&self, path: &Path) -> Option<Path> {
        Some(theta_star_smooth(
            path,
            self.check,
            self.max_turn_deg,
            self.entry_heading,
            self.min_r_m,
            self.entry_max_deg,
        ))
    }
}

/// Dubins 拟合策略（无解 → None，回退链继续）。
pub struct DubinsSmoother {
    pub r_m: f64,
    pub sample_n: usize,
}
impl Smoother for DubinsSmoother {
    fn name(&self) -> &str {
        "dubins"
    }
    fn smooth(&self, path: &Path) -> Option<Path> {
        dubins_fit(path, self.r_m, self.sample_n)
    }
}

/// Catmull-Rom 样条策略。
pub struct CatmullRomSmoother {
    pub samples_per_seg: usize,
}
impl Smoother for CatmullRomSmoother {
    fn name(&self) -> &str {
        "catmull_rom"
    }
    fn smooth(&self, path: &Path) -> Option<Path> {
        Some(catmull_rom_spline(path, self.samples_per_seg))
    }
}

/// 贪心抽稀策略。
pub struct GreedySimplifySmoother {
    pub tol_m: f64,
}

impl Smoother for GreedySimplifySmoother {
    fn name(&self) -> &str {
        "greedy_simplify"
    }
    fn smooth(&self, path: &Path) -> Option<Path> {
        Some(greedy_simplify(path, self.tol_m))
    }
}

/// 默认策略链（九轮链序修正 + 旋翼机分支）：
/// - 固定翼：Theta* 去锯齿 → Catmull-Rom 样条（绕行弧线平滑）→ Dubins 拟合 → 贪心抽稀；
/// - 旋翼机：Theta* 去锯齿 → 贪心抽稀（**不含 Dubins 全局拟合**——悬停原地转向
///   是合法机动，圆弧拟合会拉直/破坏转向点；尖角由机型感知复验放行）。
pub fn default_chain<'a>(
    opts: &SmoothOptions,
    check: &'a SegmentCheck<'a>,
    entry_heading: Option<f64>,
    entry_max_deg: f64,
) -> Vec<Box<dyn Smoother + 'a>> {
    let mut chain: Vec<Box<dyn Smoother + 'a>> = vec![Box::new(ThetaStarSmoother {
        check,
        max_turn_deg: Some(opts.max_turn_deg),
        entry_heading,
        min_r_m: opts.turn_radius_m,
        entry_max_deg,
    })];
    if opts.aircraft_type == AircraftType::FixedWing {
        // Catmull-Rom 过点样条：对"绕行弧线"（Theta* 拉直受深探测 check 限制只能折线逼近）
        // 输出曲率≈绕行半径的平滑样条，复验（转角/半径/弦高）自然通过；
        // 对"锯齿直穿"则样条曲率小 → 半径复验失败 → 回退 Dubins/直线替代（正确语义）。
        chain.push(Box::new(CatmullRomSmoother { samples_per_seg: 16 }));
        chain.push(Box::new(DubinsSmoother {
            r_m: opts.turn_radius_m,
            sample_n: opts.dubins_sample_n,
        }));
    }
    chain.push(Box::new(GreedySimplifySmoother {
        tol_m: opts.chord_tol_m,
    }));
    chain
}

/// 平滑结果。
#[derive(Debug, Clone)]
pub struct SmoothResult {
    /// 最终路径（复验通过的最后阶段；全链失败 = 原始折线）。
    pub path: Path,
    /// 实际执行的链步骤名（回退时少于策略数）。
    pub applied: Vec<String>,
    /// 软性告警（如 `smoothing_failed`）。
    pub warning: Option<String>,
    /// 最后阶段复验报告（供下游签收精度度量）。
    pub verify: VerifyReport,
    /// 各阶段复验中地形净空不足的最大地形高度（米）；None = 无。
    /// 2026-08-11 zz30：Theta* 拉直段穿山（check 长段采样 1024 上限截断漏窄峰）
    /// 时，中间阶段 verify 的 terrain issue 会被"回退楼梯阶段"（沿 FMM 走廊地形
    /// OK）吞掉 → 最终 verify 无 terrain issue。记录中间阶段最大地形高度供
    /// solver 抬升重跑。
    pub terrain_gap_m: Option<f64>,
}

/// 策略链执行 + 复验门（回退语义，十轮共识）：
/// 从链末阶段向前回退，第一个复验通过的阶段即交付；
/// 全部失败 → 交付美化前原始折线 + `warning: smoothing_failed`（宁丑勿违）。
pub fn smooth_path_chain<'a>(
    input: &Path,
    chain: &[Box<dyn Smoother + 'a>],
    opts: &SmoothOptions,
    ctx: &VerifyContext,
    phys_min_radius_m: Option<f64>,
) -> SmoothResult {
    // 执行链，记录每阶段
    let mut stages: Vec<(String, Path)> = vec![("input".into(), input.clone())];
    let mut cur = input.clone();
    for s in chain {
        if let Some(p) = s.smooth(&cur) {
            cur = p;
            stages.push((s.name().to_string(), cur.clone()));
        }
    }
    // 各阶段复验中地形净空不足的最大地形高度（供 solver 抬升重跑，见 SmoothResult）
    let mut terrain_gap_m: Option<f64> = None;
    // 从链末向前回退（包含 input 阶段）
    // Dubins 拟合阶段：输出是"物理修正"圆角（转弯半径 r_m），相对 raw 折线的
    // 弦高可达数百米（绕行 L 形圆角 ~0.7km）——100m 逼近容差会误杀合法绕行，
    // 对该阶段放宽弦高（DUBINS_CHORD_TOL_M），运动学/地形/禁飞仍严格复验。
    // 弦高 reference：用 Theta* 合法拉直输出（若无则原始折线）。FMM 对 NoData
    // 5x 高代价区（如渤海空洞）产生"伪影绕行"，把原始楼梯当 reference 会让
    // Theta*/后续阶段的合法拉直被弦高门（100m）误杀（伪影偏差可达数 km）→
    // 全链失败回退密集楼梯（2026-08-06 zigzag9：NoData 伪影 968km raw 全链拒）。
    // 语义：弦高门只约束"相对合法参考的几何逼近"；穿墙/穿区由 zone/地形复验
    // 兜底（Theta* 输出本身 check-legal，后续阶段偏离会 zone 拒）。
    let reference: Path = stages
        .iter()
        .find(|(n, _)| *n == "theta_star")
        .map(|(_, p)| p.clone())
        .unwrap_or_else(|| input.clone());
    // 从后向前遍历全部阶段，在"通过全复验"的阶段中选点数最少的交付。
    // 弦高 reference 放宽后（见上），多个阶段可能同时合法（如 Theta* 折线
    // 与 Dubins 圆角）；交付点数少的既合法又简洁，且段边界拼接的运动学风险
    // 更小（2026-08-06 zigzag3：reference=Theta* 时 seg 交付 96 点 Dubins，
    // 与受限区剖面段拼接处半径 7431m<11035m → 全路径终检拒 → 回退 raw）。
    let mut best: Option<(usize, &Path)> = None;
    for (idx, (_name, stage)) in stages.iter().enumerate().rev() {
        let rep = if _name == "dubins" {
            let mut o = opts.clone();
            o.chord_tol_m = DUBINS_CHORD_TOL_M;
            verify_path(stage, Some(&reference), &o, ctx, phys_min_radius_m)
        } else if _name == "catmull_rom" || _name == "theta_star" {
            // Theta* 的圆弧过渡插入点偏离 raw（楼梯）是合理平滑代价（zigzag18 239m）；
            // 运动学/地形/禁飞仍严格复验。
            let mut o = opts.clone();
            o.chord_tol_m = CATMULL_CHORD_TOL_M;
            if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
                eprintln!("[smooth-dbg] verify {_name} chord_tol={:.0}", o.chord_tol_m);
            }
            verify_path(stage, Some(&reference), &o, ctx, phys_min_radius_m)
        } else {
            verify_path(stage, Some(&reference), opts, ctx, phys_min_radius_m)
        };
        if rep.ok && best.is_none_or(|(_, p)| stage.points.len() < p.points.len()) {
            best = Some((idx, stage));
        }
        // 该阶段复验失败 → 跳过；若含地形净空不足（"(terrain Nm)"）记录最大地形
        // 高度（verify 采样 ~200m 密于 check，窄峰不会被漏检）
        if !rep.ok {
            let t = rep
                .issues
                .iter()
                .filter_map(|s| {
                    let pos = s.find("(terrain ")?;
                    let tail = s[pos + "(terrain ".len()..].trim_end_matches(')').trim();
                    tail.trim_end_matches('m').trim().parse::<f64>().ok()
                })
                .fold(0.0_f64, f64::max);
            if t > 0.0 {
                terrain_gap_m = Some(terrain_gap_m.map_or(t, |o| o.max(t)));
            }
        }
        if std::env::var_os("ARP_DEBUG_SMOOTH").is_some() {
            let status = if rep.ok { "OK" } else { "FAIL" };
            eprintln!(
                "[smooth-dbg] stage={} points={} status={} issues={} warnings={}",
                _name,
                stage.points.len(),
                status,
                rep.issues.len(),
                rep.warnings.len()
            );
            for iss in rep.issues.iter().take(6) {
                eprintln!("[smooth-dbg]   issue: {iss}");
            }
            for (pi, pp) in stage.points.iter().enumerate().take(10) {
                eprintln!(
                    "[smooth-dbg]   st-pt{pi}: lon={:.6} lat={:.6} alt={:.0}",
                    pp.lon, pp.lat, pp.alt_m
                );
            }
            if std::env::var_os("ARP_DEBUG_SMOOTH_DEEP").is_some() {
                for (pi, pp) in stage.points.iter().enumerate().skip(30).take(40) {
                    eprintln!(
                        "[smooth-dbg]   st-pt{pi}: lon={:.6} lat={:.6} alt={:.0}",
                        pp.lon, pp.lat, pp.alt_m
                    );
                }
            }
        }
    }
    if let Some((idx, stage)) = best {
        let applied: Vec<String> = stages[1..=idx].iter().map(|(n, _)| n.clone()).collect();
        let verify_opts = if stages[idx].0 == "dubins" {
            let mut o = opts.clone();
            o.chord_tol_m = DUBINS_CHORD_TOL_M;
            o
        } else if stages[idx].0 == "catmull_rom" || stages[idx].0 == "theta_star" {
            let mut o = opts.clone();
            o.chord_tol_m = CATMULL_CHORD_TOL_M;
            o
        } else {
            opts.clone()
        };
        let rep = verify_path(stage, Some(&reference), &verify_opts, ctx, phys_min_radius_m);
        return SmoothResult {
            path: stage.clone(),
            applied,
            warning: None,
            verify: rep,
            terrain_gap_m,
        };
    }
    // 全链失败：原始折线 + 显式告警
    let rep = verify_path(input, None, opts, ctx, phys_min_radius_m);
    SmoothResult {
        path: input.clone(),
        applied: Vec::new(),
        warning: Some("smoothing_failed: no smoothed stage passed full verification".into()),
        verify: rep,
        terrain_gap_m,
    }
}

#[cfg(test)]
mod chain_tests {
    use super::*;
    use crate::path::PathPoint;
    use crate::spatial::CircleEntry;
    use crate::terrain::{GeoBounds, TerrainSource};

    /// 平坦地形源（全 100m 平地，无空洞）。
    struct FlatTerrain;
    impl TerrainSource for FlatTerrain {
        fn height_at(&self, _lon: f64, _lat: f64) -> Option<f64> {
            Some(100.0)
        }
        fn bounds(&self) -> Option<GeoBounds> {
            Some(GeoBounds {
                min_lon: -10.0,
                min_lat: -10.0,
                max_lon: 10.0,
                max_lat: 10.0,
            })
        }
        fn resolution_desc(&self) -> String {
            "flat".into()
        }
    }

    #[test]
    fn segment_circle_intersect_shallow_graze() {
        // zigzag9（2026-08-06）：段"擦过"圆边缘，相交区间窄（0.592..0.622）
        // → 必须判相交（旧净距法在此翻转：起点纬度投影误差 ~2% 恰在半径边缘）。
        let t = segment_circle_intersect_t(
            118.28982699875671,
            38.42208802408725,
            114.2076,
            42.3648,
            116.27050736818683,
            41.08978345198258,
            50.0,
        );
        assert!(t.is_some(), "浅穿必须判相交");
        if let Some((t1, t2)) = t {
            assert!(t1 > 0.5 && t2 < 0.7, "浅穿区间应在 0.59..0.63 附近, got {t1:.3}..{t2:.3}");
        }
        // 远线段（不穿圆）→ None
        assert!(
            segment_circle_intersect_t(
                118.0, 38.0, 117.0, 38.5, 116.27050736818683, 41.08978345198258, 50.0
            )
            .is_none()
        );
        // 深穿（段穿过圆心附近）→ Some
        assert!(
            segment_circle_intersect_t(
                116.27,
                40.5,
                116.27,
                41.7,
                116.27050736818683,
                41.08978345198258,
                50.0
            )
            .is_some()
        );
    }

    #[test]
    fn circumradius_known() {
        // 直角三角形三点 (0,0),(1,0),(0,1)：外接圆半径 = √2/2
        let r = circumradius((0.0, 0.0), (1.0, 0.0), (0.0, 1.0)).unwrap();
        assert!((r - std::f64::consts::FRAC_1_SQRT_2).abs() < 1e-9);
        // 共线 → None
        assert!(circumradius((0.0, 0.0), (1.0, 0.0), (2.0, 0.0)).is_none());
    }

    #[test]
    fn verify_flat_ok() {
        let opts = SmoothOptions::default();
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 500.0),
            PathPoint::new(0.5, 0.0, 500.0),
            PathPoint::new(1.0, 0.0, 500.0),
        ]);
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let rep = verify_path(&p, None, &opts, &ctx, None);
        assert!(rep.ok, "issues: {:?}", rep.issues);
    }

    #[test]
    fn verify_terrain_collision() {
        let opts = SmoothOptions::default();
        // 地形 100m + 净空 100m → 需 ≥200m；高度 150m 违规
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 150.0),
            PathPoint::new(1.0, 0.0, 150.0),
        ]);
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let rep = verify_path(&p, None, &opts, &ctx, None);
        assert!(!rep.ok);
        assert!(rep.issues.iter().any(|s| s.contains("clearance")));
    }

    #[test]
    fn verify_zones_altitude_band() {
        // M2：同水平面不同高度——区间内违禁、区间外放行
        let z = crate::config::Zone {
            id: "R1".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: crate::config::ZoneShape::Circle {
                center: [0.5, 0.0],
                radius_km: 10.0,
            },
            alt_min_m: 0.0,
            alt_max_m: 1000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let zones = [z];
        let opts = SmoothOptions::default();
        // 高度 500 在 [0,1000] 内 → 违禁
        let p_in = Path::new(vec![
            PathPoint::new(0.0, 0.0, 500.0),
            PathPoint::new(1.0, 0.0, 500.0),
        ]);
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: Some(&zones),
            threat: None,
              zone_inflation_m: 0.0,
        };
        let rep = verify_path(&p_in, None, &opts, &ctx, None);
        assert!(!rep.ok);
        assert!(rep.issues.iter().any(|s| s.contains("zone")), "{:?}", rep.issues);
        // 高度 3000 在区间外 → 放行（水平位置相同）
        let p_out = Path::new(vec![
            PathPoint::new(0.0, 0.0, 3000.0),
            PathPoint::new(1.0, 0.0, 3000.0),
        ]);
        let rep = verify_path(&p_out, None, &opts, &ctx, None);
        assert!(rep.ok, "高度区间外应放行: {:?}", rep.issues);
    }

    #[test]
    fn verify_nofly_violation() {
        let opts = SmoothOptions::default();
        let idx = crate::spatial::CircleIndex::build(vec![CircleEntry {
            id: "NF1".into(),
            lon: 0.5,
            lat: 0.0,
            radius_m: 5_000.0,
        }]);
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 500.0),
            PathPoint::new(1.0, 0.0, 500.0),
        ]);
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: Some(&idx),
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let rep = verify_path(&p, None, &opts, &ctx, None);
        assert!(!rep.ok);
        assert!(rep.issues.iter().any(|s| s.contains("no-fly")));
    }

    #[test]
    fn verify_turn_radius_violation() {
        let opts = SmoothOptions {
            max_turn_deg: 180.0, // 只测转弯半径
            turn_radius_m: 100_000.0, // 极小转弯必违规
            ..Default::default()
        };
        // 90° 直角路径
        let p = Path::new(vec![
            PathPoint::new(0.0, 0.0, 500.0),
            PathPoint::new(0.5, 0.0, 500.0),
            PathPoint::new(0.5, 0.5, 500.0),
        ]);
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let rep = verify_path(&p, None, &opts, &ctx, None);
        assert!(!rep.ok);
        assert!(rep.issues.iter().any(|s| s.contains("radius")), "{:?}", rep.issues);
    }

    #[test]
    fn chain_full_pipeline() {
        let opts = SmoothOptions::default();
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        // 带锯齿的折线（共线 + 轻微偏转），高度 500 满足净空
        let pts: Vec<PathPoint> = (0..20)
            .map(|i| {
                let lat = if i % 2 == 0 { 0.0 } else { 0.002 }; // ~222m 偏转
                PathPoint::new(i as f64 * 0.05, lat, 500.0)
            })
            .collect();
        let input = Path::new(pts);
        let check = |_: f64, _: f64, _: f64, _: f64, _: f64, _: f64| true;
        let chain = default_chain(&opts, &check, None, 95.0);
        let out = smooth_path_chain(&input, &chain, &opts, &ctx, None);
        assert!(!out.applied.is_empty(), "applied: {:?}", out.applied);
        assert!(out.warning.is_none(), "warning: {:?}", out.warning);
        assert!(out.path.len() <= input.len());
        // 交付路径过复验
        let rep = verify_path(&out.path, Some(&input), &opts, &ctx, None);
        assert!(rep.ok, "issues: {:?}", rep.issues);
    }

    #[test]
    fn chain_nodata_degrades_not_fails() {
        // 地形全 NoData → 空洞策略（主管 2026-08-04）：净空不确定但不阻断航路，
        // 降级警告（不产生 smoothing_failed 回退楼梯；主管 2026-08-06 zigzag8 根因）。
        struct NoDataTerrain;
        impl TerrainSource for NoDataTerrain {
            fn height_at(&self, _lon: f64, _lat: f64) -> Option<f64> {
                None
            }
            fn bounds(&self) -> Option<GeoBounds> {
                Some(GeoBounds {
                    min_lon: -10.0,
                    min_lat: -10.0,
                    max_lon: 10.0,
                    max_lat: 10.0,
                })
            }
            fn resolution_desc(&self) -> String {
                "nodata".into()
            }
        }
        let opts = SmoothOptions::default();
        let ctx = VerifyContext {
            terrain: Some(&NoDataTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let input = Path::new(vec![
            PathPoint::new(0.0, 0.0, 500.0),
            PathPoint::new(0.5, 0.5, 500.0),
            PathPoint::new(1.0, 0.0, 500.0),
        ]);
        let check = |_: f64, _: f64, _: f64, _: f64, _: f64, _: f64| true;
        let chain = default_chain(&opts, &check, None, 95.0);
        let out = smooth_path_chain(&input, &chain, &opts, &ctx, None);
        // NoData 不再阻断：链应应用至少一个阶段（不再回退 smoothing_failed 楼梯）
        assert!(!out.applied.is_empty(), "applied: {:?}", out.applied);
        assert_eq!(out.warning, None, "NoData 地形不应 smoothing_failed");
        assert!(
            out.verify.warnings.iter().any(|w| w.contains("NoData terrain")),
            "应保留 NoData 降级警告，实际 {:?}",
            out.verify.warnings
        );
    }

    #[test]
    fn chain_nan_input_no_panic() {
        let opts = SmoothOptions::default();
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let input = Path::new(vec![
            PathPoint::new(f64::NAN, 0.0, 500.0),
            PathPoint::new(1.0, 0.0, 500.0),
        ]);
        let check = |_: f64, _: f64, _: f64, _: f64, _: f64, _: f64| true;
        let chain = default_chain(&opts, &check, None, 95.0);
        let out = smooth_path_chain(&input, &chain, &opts, &ctx, None);
        // 不 panic；NaN 输入复验 fail → 回退原始
        assert!(!out.verify.ok);
    }

    /// 水平梳齿方波（近 90° 急转阶梯，齿距 step_deg 度、齿深 step_deg 度）。
    fn square_wave(teeth: usize, step_deg: f64) -> Path {
        let mut pts = vec![PathPoint::new(0.0, 0.0, 500.0)];
        for t in 0..teeth {
            let (x, y) = (t as f64 * 2.0 * step_deg, t as f64 * step_deg);
            pts.push(PathPoint::new(x + 2.0 * step_deg, y, 500.0));
            pts.push(PathPoint::new(x + 2.0 * step_deg, y + step_deg, 500.0));
        }
        pts.push(PathPoint::new(
            teeth as f64 * 2.0 * step_deg,
            teeth as f64 * step_deg,
            500.0,
        ));
        Path::new(pts)
    }

    #[test]
    fn fixed_wing_square_wave_rejected() {
        // 固定翼 + 绕障方波（check 拒绝斜穿）：截不了直 → 90° 尖角违反运动学约束
        // → 复验拦截，链回退原始 + smoothing_failed
        let opts = SmoothOptions::default(); // FixedWing
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let wave = square_wave(4, 0.02);
        let check = |lon1: f64, lat1: f64, _: f64, lon2: f64, lat2: f64, _: f64| {
            !((lon2 - lon1).abs() > 0.025 && (lat2 - lat1).abs() > 0.01)
        };
        let chain = default_chain(&opts, &check, None, 95.0);
        let out = smooth_path_chain(&wave, &chain, &opts, &ctx, None);
        assert_eq!(
            out.warning.as_deref(),
            Some("smoothing_failed: no smoothed stage passed full verification")
        );
        assert!(!out.verify.ok);
        assert!(out.verify.issues.iter().any(|s| s.contains("turn")));
    }

    #[test]
    fn fixed_wing_clean_square_wave_straightened() {
        // 固定翼 + 无障碍方波（纯锯齿伪影）：Theta* 直接截弯取直，输出直线
        let opts = SmoothOptions::default(); // FixedWing
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let wave = square_wave(4, 0.02);
        let check = |_: f64, _: f64, _: f64, _: f64, _: f64, _: f64| true;
        let chain = default_chain(&opts, &check, None, 95.0);
        let out = smooth_path_chain(&wave, &chain, &opts, &ctx, None);
        assert!(out.warning.is_none(), "warning: {:?}", out.warning);
        assert!(out.verify.ok, "issues: {:?}", out.verify.issues);
        // 截直成 2 点直线
        assert_eq!(out.path.len(), 2, "got {}", out.path.len());
    }

    #[test]
    fn rotorcraft_square_wave_passes() {
        // 旋翼机：急转/悬停原地转向为合法机动（九轮共识）→ 复验放行，不误判
        let opts = SmoothOptions {
            aircraft_type: AircraftType::Rotorcraft,
            ..Default::default()
        };
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        let wave = square_wave(4, 0.02);
        let rep = verify_path(&wave, None, &opts, &ctx, None);
        assert!(rep.ok, "rotorcraft square wave misjudged: {:?}", rep.issues);
        assert!(rep.issues.is_empty());
    }

    #[test]
    fn rotorcraft_chain_keeps_turns_not_dubins() {
        // 旋翼机链不含 Dubins：方波通过链后保持急转结构（不得被圆弧拟合拉直）
        let opts = SmoothOptions {
            aircraft_type: AircraftType::Rotorcraft,
            ..Default::default()
        };
        let ctx = VerifyContext {
            terrain: Some(&FlatTerrain),
            nofly: None,
            zones: None,
            threat: None,
              zone_inflation_m: 0.0,
        };
        // check 拒绝斜穿（模拟绕障方波）→ theta_star 截不了 → 保持
        let wave = square_wave(4, 0.02);
        let check = |lon1: f64, lat1: f64, _: f64, lon2: f64, lat2: f64, _: f64| {
            !((lon2 - lon1).abs() > 0.025 && (lat2 - lat1).abs() > 0.01)
        };
        let chain = default_chain(&opts, &check, None, 95.0);
        let out = smooth_path_chain(&wave, &chain, &opts, &ctx, None);
        assert!(out.verify.ok, "issues: {:?}", out.verify.issues);
        assert!(out.warning.is_none(), "warning: {:?}", out.warning);
        // 尖角保留：输出仍含近 90° 转角（复验放行，未被截直）
        assert!(out.path.len() >= 6, "turns lost: {}", out.path.len());
    }

    #[test]
    fn m4_profile_smooth_options_derivation() {
        use crate::config::VehicleProfile;
        let p = crate::config::DefaultParams::default();

        // 默认固定翼：v=250 → phys@cruise ≈ 11035；turn_radius = 默认表 5000（信任，
        // 2026-08-07 起不再钳到 phys）；返回的 A6 有效下限 = min(phys, turn_radius)
        let prof = VehicleProfile::default();
        let (o, a6) = smooth_options_for(&prof, &p);
        assert!((a6 - 5000.0).abs() < 1e-9, "a6 = {a6}");
        assert_eq!(o.aircraft_type, AircraftType::FixedWing);
        assert!((o.turn_radius_m - 5000.0).abs() < 1e-9, "turn {}", o.turn_radius_m);
        // 默认 max_climb = 默认表 15°
        assert!((o.max_climb_deg - 15.0).abs() < 1e-9);

        // 显式慢速固定翼：v=50 → phys ≈ 442；默认表 5000 更大 → turn_radius = 5000
        let prof = VehicleProfile {
            cruise_speed_mps: Some(50.0),
            ..Default::default()
        };
        let (o, a6) = smooth_options_for(&prof, &p);
        assert!((a6 - 441.6).abs() < 1.0, "a6 = {a6}");
        assert!((o.turn_radius_m - 5000.0).abs() < 1e-9);

        // 显式 min_turn_radius 大于 phys → 用输入值；A6 有效下限 = 巡航 phys
        let prof = VehicleProfile {
            cruise_speed_mps: Some(250.0),
            min_turn_radius_m: Some(20_000.0),
            ..Default::default()
        };
        let (o, a6) = smooth_options_for(&prof, &p);
        assert!((o.turn_radius_m - 20_000.0).abs() < 1e-9);
        assert!((a6 - 11_035.0).abs() < 2.0, "a6 = {a6}");

        // 显式 min_turn_radius 小于 phys（主管场景 v=250 + r=442）→ 信任输入不再钳制；
        // 转弯段降速到 v_turn = sqrt(442·g·tan30°) ≈ 50 m/s 实现；A6 有效下限 =
        // turn_radius → verify 的 A6 检查恒过
        let prof = VehicleProfile {
            cruise_speed_mps: Some(250.0),
            min_turn_radius_m: Some(442.0),
            ..Default::default()
        };
        let (o, a6) = smooth_options_for(&prof, &p);
        assert!((o.turn_radius_m - 442.0).abs() < 1e-9, "turn {}", o.turn_radius_m);
        assert!((a6 - 442.0).abs() < 1e-9, "a6 = {a6}");
        let v_turn = (442.0 * 9.81 * 30f64.to_radians().tan()).sqrt();
        assert!((v_turn - 50.0).abs() < 1.0, "v_turn = {v_turn}");

        // 旋翼机：r→0 合法不钳；speed_range 中值取速；A6 有效下限 = min(phys, 0) = 0
        let prof = VehicleProfile {
            aircraft_type: AircraftType::Rotorcraft,
            speed_range_mps: Some([40.0, 80.0]),
            ..Default::default()
        };
        let (o, a6) = smooth_options_for(&prof, &p);
        assert_eq!(o.aircraft_type, AircraftType::Rotorcraft);
        assert_eq!(o.turn_radius_m, 0.0);
        assert_eq!(a6, 0.0);
    }
}

//! Phase 0 反馈错误输入回归套件（主管拍板 2026-08-06：落点 A + 地形依赖检测降级）。
//!
//! 背景：phase0_out/ 是 Phase 0 调试产物目录（已 gitignore）。其中 18 个**输入型**
//! mission JSON 是历史上主管反馈 bug 的原始输入（锯齿/直穿 restricted/禁飞区陷阱/
//! restricted 高度层剖面判据等）。本套件把它们复制为正式回归用例，防止错误复现。
//!
//! 回归语义（核心断言）：
//!   1. 每个输入 parse + validate + solve 全过（不 panic、不 Err）；
//!   2. 输出路径逐点不穿任何 zone（水平包含 + 高度带，与 zone_contains_at 同语义）——
//!      no_fly/obstacle 全高度禁入，restricted 仅禁入高度带（底部/顶部剖面穿越合法）。
//!
//! 地形依赖（主管决策）：cases 中 terrain.path 指向
//! `data/east_asia_7p5as.arpack`（发布版默认地形，数据文件已 gitignore；
//! 2026-08-10 起 china_dem_l12 已退出测试流程）。
//! 运行期检测：文件存在 → 改写 path 为绝对路径使用真实地形；缺失 → terrain.source=none
//! 合成平面（覆盖不到真实地形 bug，但保证用例在无数据环境仍可跑）。
//!
//! 新增用例：把输入 JSON 放进 tests/regression/cases/ 即可自动纳入（遍历发现）。

use std::path::{Path, PathBuf};

use aircraft_router_planner_cli::config::{self, Input, Zone, ZoneShape};
use aircraft_router_planner_cli::coord::Geo;
use aircraft_router_planner_cli::solver::{self, SolveParams};

/// cases 目录：<crate>/tests/regression/cases/
fn cases_dir() -> PathBuf {
    Path::new(env!("CARGO_MANIFEST_DIR")).join("tests/regression/cases")
}

/// 真实地形数据文件候选（workspace 根/data/...）
fn real_terrain_path() -> Option<PathBuf> {
    let root = Path::new(env!("CARGO_MANIFEST_DIR")).join("..");
    let cand = root.join("data/east_asia_7p5as.arpack");
    cand.exists().then_some(cand)
}

/// 解析用例输入并按地形依赖检测改写 terrain：
/// - 数据存在 → terrain.path 改写为绝对路径（真实地形）；
/// - 数据缺失 → terrain.source=none（合成平面）。
fn load_case(name: &str) -> Input {
    let p = cases_dir().join(name);
    let raw =
        std::fs::read_to_string(&p).unwrap_or_else(|e| panic!("{name}: read case failed: {e}"));
    let mut input: Input =
        Input::from_json_str(&raw).unwrap_or_else(|e| panic!("{name}: parse failed: {e}"));
    if input.mission.terrain.source == config::TerrainSourceType::None {
        return input;
    }
    match real_terrain_path() {
        Some(abs) => {
            input.mission.terrain.path = Some(abs.to_string_lossy().into_owned());
        }
        None => {
            input.mission.terrain.source = config::TerrainSourceType::None;
            input.mission.terrain.path = None;
        }
    }
    input
}

/// 水平包含：圆（测地距离）或 多边形（射线法）。
fn point_in_zone_shape(z: &Zone, p: &Geo) -> bool {
    match &z.shape {
        ZoneShape::Circle { center, radius_km } => {
            let c = Geo::new(center[0], center[1]).ok();
            match c {
                Some(c) => c.distance_m(p) <= radius_km * 1000.0,
                None => false,
            }
        }
        ZoneShape::Polygon { vertices } => point_in_polygon(vertices, p),
    }
}

/// 射线法 point-in-polygon（lon=x, lat=y，平面近似足够回归语义）。
fn point_in_polygon(verts: &[[f64; 2]], p: &Geo) -> bool {
    let mut inside = false;
    let (x, y) = (p.lon, p.lat);
    let mut j = verts.len() - 1;
    for i in 0..verts.len() {
        let (xi, yi) = (verts[i][0], verts[i][1]);
        let (xj, yj) = (verts[j][0], verts[j][1]);
        if ((yi > y) != (yj > y)) && (x < (xj - xi) * (y - yi) / (yj - yi) + xi) {
            inside = !inside;
        }
        j = i;
    }
    inside
}

/// 高度带判定（与 zone_contains_at 同语义：MSL 直比；无高度区间
/// （NoFly/Obstacle 全高度）→ 拦截）。
fn alt_in_band(z: &Zone, alt_m: f64) -> bool {
    match (z.alt_min_m, z.alt_max_m) {
        (Some(lo), Some(hi)) => alt_m >= lo && alt_m <= hi,
        _ => true, // 无高度区间 → 全高度拦截
    }
}

/// 路径点是否违规进入某 zone（水平包含 + 高度带）。
fn point_violates(z: &Zone, p: &Geo, alt_m: f64) -> bool {
    point_in_zone_shape(z, p) && alt_in_band(z, alt_m)
}

/// 核心断言：输出路径逐点不穿任何 zone。
fn assert_path_clear(input: &Input, out: &config::Output, name: &str) {
    let mut zones = input.mission.no_fly_zones.clone();
    zones.extend(input.mission.restricted_zones.clone());
    zones.extend(input.mission.obstacles.clone());
    if zones.is_empty() {
        return;
    }
    for v in &out.vehicles {
        for (i, pt) in v.path.iter().enumerate() {
            let geo = match Geo::new(pt.x, pt.y) {
                Ok(g) => g,
                Err(_) => continue,
            };
            for z in &zones {
                if point_violates(z, &geo, pt.alt_m) {
                    panic!(
                        "{name}: vehicle {} path point {i} (lon={}, lat={}, alt={}) violates zone {} ({:?})",
                        v.id, pt.x, pt.y, pt.alt_m, z.id, z.zone_type
                    );
                }
            }
        }
    }
}

/// 遍历 cases 目录，逐个回归。
#[test]
fn phase0_feedback_inputs_regression() {
    let dir = cases_dir();
    let mut names: Vec<String> = std::fs::read_dir(&dir)
        .unwrap_or_else(|e| panic!("cannot read cases dir {}: {e}", dir.display()))
        .map(|e| e.unwrap().file_name().to_string_lossy().into_owned())
        .filter(|n| n.ends_with(".json"))
        .collect();
    names.sort();

    assert!(
        !names.is_empty(),
        "no regression cases found under {}",
        dir.display()
    );

    let terrain_ok = real_terrain_path().is_some();
    eprintln!(
        "[regress] {} cases, real terrain = {}",
        names.len(),
        if terrain_ok {
            "present"
        } else {
            "MISSING (using terrain none)"
        }
    );

    for name in &names {
        let input = load_case(name);
        // 输入必须过契约校验（退化输入本身即 bug，反馈输入都应是合法任务）
        config::validate(&input).unwrap_or_else(|e| panic!("{name}: validate failed: {e:?}"));

        let mut params = SolveParams::default();
        if input.mission.terrain.source != config::TerrainSourceType::None {
            if let Some(p) = &input.mission.terrain.path {
                params.terrain_path = Some(PathBuf::from(p));
            }
        }

        let out = solver::solve(&input, &params, 0)
            .unwrap_or_else(|e| panic!("{name}: solve error: {e:?}"));

        // status 契约：success / degraded_timeout / no_solution 均合法，但 input_invalid 不应在此出现
        assert_ne!(
            out.status, "input_invalid",
            "{name}: unexpected input_invalid after validate passed"
        );

        // 有路径则断言不穿 zone（核心回归语义）
        assert_path_clear(&input, &out, name);

        // zigzag11（2026-08-07 主管）：双 100km 雷达 + no_fly 多边形 + no_fly 圆 +
        // 双 restricted，raw 1755 点网格楼梯回退（smoothing_failed）。修复：段边界
        // 入口航向约束 Theta* 首跳（seg3 out→climb 与 seg4 climb→A 夹角 61.94°>
        // 60°→ 拼接后终检拒→全链回退）→ 10 点平滑交付。强断言防锯齿复发。
        if name == "zigzag11.json" {
            let v = &out.vehicles[0];
            assert!(
                v.path.len() < 30,
                "{name}: still staircase dense ({} pts), entry_heading regression",
                v.path.len()
            );
            assert!(
                !v.warnings.iter().any(|w| w.contains("smoothing_failed")),
                "{name}: smoothing_failed re-appeared"
            );
        }

        // zigzag12（2026-08-07 主管）：start 黄海→target 内蒙，双 100km 雷达 +
        // no_fly 多边形 + no_fly 圆 + 双 restricted（rz1 0-5000 顶部绕飞）。raw 1893
        // 点网格楼梯回退（smoothing_failed）。根因：rz1 顶部剖面 out→climb 过渡直线
        // (119.75,37.27)→(117.16,37.30) 穿 no_fly 多边形（clearance=0）——剖面段穿
        // 硬墙从未检测（need_wall 恒 false）→ 拼接后终检拒 → 全链回退。修复：过渡
        // 直线（desc_in/out_climb）做硬墙净距检查 → need_wall → 画墙水平绕行兜底 →
        // 6 点平滑交付。强断言防锯齿复发。
        if name == "zigzag12.json" {
            let v = &out.vehicles[0];
            assert!(
                v.path.len() < 30,
                "{name}: still staircase dense ({} pts), profile-through-wall regression",
                v.path.len()
            );
            assert!(
                !v.warnings.iter().any(|w| w.contains("smoothing_failed")),
                "{name}: smoothing_failed re-appeared"
            );
        }

        // zigzag13（2026-08-07 主管 2000km 大跨度）：青岛黄海→俄境（lon 120.89→113.12,
        // lat 36.06→53.00, span 17.4°），no_fly 多边形 + 2 no_fly 圆 + 3 restricted +
        // 3 雷达，v=250m/s 固定翼（r_phys=11035m 钳制）。raw 1605 点网格楼梯回退
        // （smoothing_failed）。根因：FMM 8 邻域楼梯沿膨胀墙走对角线切角 ~0.71×cell
        // → 路径离原始墙 < verify 要求的 inflation（cell 1.89km 时 3 格膨胀 5.67km −
        // 切角 1.34km = 4.33km < 5.52km）→ 平滑链全失败。修复：span>2.5° 时膨胀距离
        // 补 0.71×cell（3→4 格）→ 3 点平滑交付。强断言防锯齿复发。
        if name == "zigzag13.json" {
            let v = &out.vehicles[0];
            assert!(
                v.path.len() < 30,
                "{name}: still staircase dense ({} pts), inflation corner-cut regression",
                v.path.len()
            );
            assert!(
                !v.warnings.iter().any(|w| w.contains("smoothing_failed")),
                "{name}: smoothing_failed re-appeared"
            );
            assert!(
                v.distance_m < 2_500_000.0,
                "{name}: detour too long ({} km), inflation regression",
                v.distance_m / 1000.0
            );
        }

        // zigzag17（2026-08-07 主管第 4 版场景）：3 no_fly 多边形 + restricted 圆
        // （0-6000m）。raw 1140 点网格楼梯回退（smoothing_failed）。根因：多边形
        // **尖角顶点**（poly3 西南角 (116.198,37.111)）处 FMM 格点墙角是钝的（墙格在
        // 顶点东北），路径从顶点西侧绕过时离**几何边** 1.33km < inflation 2km——
        // 0.71×cell 切角补偿（ceil((2000+0.71×1953)/1953)=2 格）不够 → verify 拒 →
        // 平滑链全失败。修复：span>2.5° 时膨胀格数再 +1 兜底尖角偏差 → 11 点平滑交付。
        if name == "zigzag17.json" {
            let v = &out.vehicles[0];
            assert!(
                v.path.len() < 30,
                "{name}: still staircase dense ({} pts), polygon corner clearance regression",
                v.path.len()
            );
            assert!(
                !v.warnings.iter().any(|w| w.contains("smoothing_failed")),
                "{name}: smoothing_failed re-appeared"
            );
        }

        // zigzag18（2026-08-07 主管第 5 版场景）：3 no_fly 多边形 + restricted 圆
        // （0-6000m）。raw 2006 点网格楼梯回退（smoothing_failed）。根因：FMM 90°
        // 楼梯角 + Theta* 无法跳远（全被墙挡）→ fallback 走相邻点产生 92° 急转弯 →
        // 拼接后终检拒 → 全链回退。修复：Theta* fallback 插入圆弧过渡点
        // （arc_transition）拆分急转弯（2026-08-08 修正两处几何符号：① 有向转角
        // 应取 (h_bc−h_ab) 顺时针为正，原 (h_ab−h_bc) 致北→西左转被当右转 → 弧向东
        // 鼓包 170° 折返；② 圆心应在 S 对侧（S−r·(sinφ0,cosφ0)），原 S+r 同侧致
        // t=0 弧点偏移 2r）→ 8 点 theta 全过 → 12 点平滑交付。强断言防锯齿复发。
        if name == "zigzag18.json" {
            let v = &out.vehicles[0];
            assert!(
                v.path.len() < 30,
                "{name}: still staircase dense ({} pts), theta fallback arc regression",
                v.path.len()
            );
            assert!(
                !v.warnings.iter().any(|w| w.contains("smoothing_failed")),
                "{name}: smoothing_failed re-appeared"
            );
        }

        eprintln!(
            "[regress] {name}: status={} vehicles={}",
            out.status,
            out.vehicles.len()
        );
    }
}

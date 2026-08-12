//! 两级分辨率细层——可见图最小原型（P1）
//!
//! 依据 docs/12 §3.2/3.3/3.4 + §8 + §13 定案范围（主管 2026-08-12 拍板）：
//! - 单 patch、单凸障碍、无地形、无限飞区触发；
//! - 排除圆障碍（`shape:"circle"`）触发 patch——禁飞区绝对禁入、不可上下绕行；
//! - 排除 mid_waypoint 触发源；排除多 patch 串接；
//! - 纳入 C2（search_truncated 截断降级标注）+ C3（裁决器归因分层）；
//! - 出口：stderr 分类 JSON + stdout stats.degradations 汇总。
//!
//! 实现细节定案（§13.1）：
//! - C1：凸包用 Andrew monotone chain（纯叉积 + `f64::total_cmp` 字典序，零三角函数）；
//! - C4：细层凸化基于原多边形矢量膨胀（边平移求交），不复用粗层栅格 INF 格点化；
//! - C5：求交/贴边判定用保守侧常量 `GEOM_EPS_DEG`；
//! - C6：feature-flag 默认关（`ARP_PATCH=1` 开启）。

use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// C5：求交/贴边保守侧 epsilon（1e-9° ≈ 0.11 mm，仅用于几何退化/边界接触判定）。
pub const GEOM_EPS_DEG: f64 = 1e-9;
/// §3.2 PATCH_R：patch 尺寸 30km（度）。
pub const PATCH_R_DEG: f64 = 30.0 / 111.32;
/// §7 可见图顶点数上限：超过即搜索截断（search_truncated 触发条件）。
pub const MAX_VIS_VERTICES: usize = 64;
/// 边平移求交行列式退化阈值。
const DET_EPS: f64 = 1e-12;

/// C6：patch 功能默认关；环境变量 `ARP_PATCH=1` 开启（验收 flag-on 全量）。
pub fn patch_enabled() -> bool {
    std::env::var("ARP_PATCH").map(|v| v == "1").unwrap_or(false)
}

/// P1 可应用性判定（触发前的排除项检查）。
///
/// - `has_terrain`：P1 无地形（data 契约只吃 ARPK1，patch 先不做地形净空边权）；
/// - `has_mid_waypoints`：P1 排除 mid_waypoint 触发源（§13.2 R3）；
/// - `zones` 中圆障碍/限飞区（Restricted）触发均被 P1 排除（主管拍板，§8/§13.4）。
pub fn patch_applicable(zones: &[crate::config::Zone], has_terrain: bool, has_mid_waypoints: bool) -> bool {
    if has_terrain || has_mid_waypoints {
        return false;
    }
    if !zones.iter().all(|z| {
        // P1 只接受多边形硬墙（NoFly/Obstacle）；圆障碍与限飞区（含圆）排除。
        z.is_wall() && matches!(z.shape, crate::config::ZoneShape::Polygon { .. })
    }) {
        return false;
    }
    true
}

// ==================== 几何原语 ====================

/// 三点点积叉积：`cross(o, a, b)` = (a-o)×(b-o)，逆时针为正。
fn cross(o: [f64; 2], a: [f64; 2], b: [f64; 2]) -> f64 {
    (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
}

/// C1：Andrew monotone chain 凸包（逆时针，去共线中间点）。
///
/// 排序用 `f64::total_cmp`（IEEE 754 totalOrder，跨平台逐位一致），tie-break 按
/// 坐标字典序——确定性红线（契约 2）。零三角函数。
pub fn convex_hull(points: &[[f64; 2]]) -> Vec<[f64; 2]> {
    let n = points.len();
    if n <= 2 {
        return points.to_vec();
    }
    let mut pts = points.to_vec();
    pts.sort_by(|a, b| {
        let c = a[0].total_cmp(&b[0]);
        if c == Ordering::Equal {
            a[1].total_cmp(&b[1])
        } else {
            c
        }
    });
    // 去重（坐标完全相同的顶点——锚点重合类 B9 用例）。
    pts.dedup_by(|a, b| a[0].to_bits() == b[0].to_bits() && a[1].to_bits() == b[1].to_bits());
    if pts.len() <= 2 {
        return pts;
    }
    // lower + upper（cross <= 0 弹出，保严格凸、共线点去中间）。
    let mut lower: Vec<[f64; 2]> = Vec::new();
    for &p in &pts {
        while lower.len() >= 2 && cross(lower[lower.len() - 2], lower[lower.len() - 1], p) <= 0.0 {
            lower.pop();
        }
        lower.push(p);
    }
    let mut upper: Vec<[f64; 2]> = Vec::new();
    for &p in pts.iter().rev() {
        while upper.len() >= 2 && cross(upper[upper.len() - 2], upper[upper.len() - 1], p) <= 0.0 {
            upper.pop();
        }
        upper.push(p);
    }
    lower.pop();
    upper.pop();
    lower.extend(upper);
    lower
}

/// C4：凸多边形矢量膨胀（Minkowski 圆膨胀的边平移等价形）。
///
/// 对每条边沿外法线平移 `dist_deg`，相邻平移边求交得新顶点（零三角函数，
/// 仅叉积/归一化 sqrt）。输入须为逆时针凸多边形（`convex_hull` 输出）。
/// 退化（<3 顶点 / 行列式过小）时返回空 Vec 由调用方判不适用。
pub fn inflate_convex(poly: &[[f64; 2]], dist_deg: f64) -> Vec<[f64; 2]> {
    let n = poly.len();
    if n < 3 || dist_deg <= 0.0 {
        return Vec::new();
    }
    // 每条边：外法线 (a,b)（单位）与平移后常数 c。
    let mut lines: Vec<([f64; 2], f64)> = Vec::with_capacity(n);
    for i in 0..n {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % n];
        let ex = p2[0] - p1[0];
        let ey = p2[1] - p1[1];
        let len = (ex * ex + ey * ey).sqrt();
        if len <= 0.0 {
            return Vec::new();
        }
        // 逆时针多边形的外法线 = (ey, -ex)/len
        let (a, b) = (ey / len, -ex / len);
        let c = a * p1[0] + b * p1[1] + dist_deg;
        lines.push(([a, b], c));
    }
    let mut out: Vec<[f64; 2]> = Vec::with_capacity(n);
    for i in 0..n {
        let ([a1, b1], c1) = lines[i];
        let ([a2, b2], c2) = lines[(i + 1) % n];
        let det = a1 * b2 - a2 * b1;
        if det.abs() < DET_EPS {
            return Vec::new();
        }
        let x = (c1 * b2 - c2 * b1) / det;
        let y = (a1 * c2 - a2 * c1) / det;
        out.push([x, y]);
    }
    out
}

/// 点是否在凸多边形内（含边界，保守侧：边界 eps 内视为内部）。
pub fn point_in_convex(p: [f64; 2], poly: &[[f64; 2]], eps: f64) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    for i in 0..n {
        if cross(poly[i], poly[(i + 1) % n], p) < -eps {
            return false;
        }
    }
    true
}

/// 线段与凸多边形是否**穿入内部**（保守侧：深入边界 eps 以内不算穿入，边界接触放行）。
///
/// 可见图语义：边允许贴着（膨胀后）多边形边界走（膨胀已含机动空间，契约 9），
/// 只禁止进入内部。半平面约束区间法：对每条边求线段参数 t 的可行区间（内部约束
/// `f(t) >= eps`），交叠为空即不相交。
pub fn seg_intersects_convex(a: [f64; 2], b: [f64; 2], poly: &[[f64; 2]], eps: f64) -> bool {
    let n = poly.len();
    if n < 3 {
        return false;
    }
    let mut lo = 0.0_f64;
    let mut hi = 1.0_f64;
    for i in 0..n {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % n];
        let ex = p2[0] - p1[0];
        let ey = p2[1] - p1[1];
        // f(t) = cross(e, a + t*(b-a) - p1) = f0 + t*f1；内部要求 f(t) >= eps
        let f0 = ex * (a[1] - p1[1]) - ey * (a[0] - p1[0]);
        let f1 = ex * (b[1] - a[1]) - ey * (b[0] - a[0]);
        if f1.abs() < 1e-15 {
            if f0 < eps {
                return false;
            }
        } else {
            let t_crit = (eps - f0) / f1;
            if f1 > 0.0 {
                lo = lo.max(t_crit);
            } else {
                hi = hi.min(t_crit);
            }
        }
        if lo > hi + 1e-12 {
            return false;
        }
    }
    true
}

/// 线段到凸多边形边界的最小距离（度，粗估；用于 C3 归因的顶点邻域判定）。
pub fn seg_poly_dist_deg(a: [f64; 2], b: [f64; 2], poly: &[[f64; 2]]) -> f64 {
    let n = poly.len();
    if n < 3 {
        return f64::INFINITY;
    }
    let mut best = f64::INFINITY;
    for i in 0..n {
        let p1 = poly[i];
        let p2 = poly[(i + 1) % n];
        // 点到线段距离（度，欧氏近似）
        let ex = p2[0] - p1[0];
        let ey = p2[1] - p1[1];
        let len2 = ex * ex + ey * ey;
        let proj = |p: [f64; 2]| -> f64 {
            if len2 <= 0.0 {
                return ((p[0] - p1[0]).powi(2) + (p[1] - p1[1]).powi(2)).sqrt();
            }
            let t = ((p[0] - p1[0]) * ex + (p[1] - p1[1]) * ey) / len2;
            let t = t.clamp(0.0, 1.0);
            let qx = p1[0] + t * ex;
            let qy = p1[1] + t * ey;
            ((p[0] - qx).powi(2) + (p[1] - qy).powi(2)).sqrt()
        };
        best = best.min(proj(a)).min(proj(b));
    }
    best
}

// ==================== 可见图 + 搜索 ====================

/// Dijkstra 状态（确定性：cost 按 total_cmp 全序，tie-break 顶点整数索引）。
#[derive(Clone, Copy, Debug)]
struct State {
    cost: f64,
    id: usize,
}

impl PartialEq for State {
    fn eq(&self, other: &Self) -> bool {
        self.cost.to_bits() == other.cost.to_bits() && self.id == other.id
    }
}
impl Eq for State {}
impl PartialOrd for State {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for State {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap 是最大堆：反转 cost 使 cost 小者先弹出（Dijkstra 按距离递增）；
        // cost 相等时 id 小者先弹出（显式整数 tie-break，确定性）。
        other
            .cost
            .total_cmp(&self.cost)
            .then_with(|| other.id.cmp(&self.id))
    }
}

/// C3 裁决分类（verify 失败后的归因）。
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub enum PatchFailureClass {
    /// 折线自身硬违规（穿障碍/净空不足）——确定性几何无解。
    GeometricImpossible,
    /// 失败坐标落在转折顶点邻域（转弯机动空间不足）——拟合/机动缺陷，非几何无解。
    FittingDefect,
}

/// patch 规划结果。
#[derive(Debug, Clone, PartialEq)]
pub enum PatchOutcome {
    /// 合法折线 + 几何总长（km）。
    Path(Vec<[f64; 2]>, f64),
    /// C2：可见图顶点/尺寸达到上限被截断（≠ 几何无解；调用方须在 degradations 标注）。
    SearchTruncated,
    /// 图不连通（无合法路径）。
    GeometricImpossible,
}

/// 主入口：单凸障碍可见图最短路径规划（P1）。
///
/// `obstacle_poly` 为原始障碍多边形顶点（任意凹凸/方向）；流程：
/// 凸化（C1）→ 矢量膨胀（C4，`inflation_m` 机动空间）→ 可见图 → Dijkstra（确定性）。
pub fn plan_patch(
    start: [f64; 2],
    target: [f64; 2],
    obstacle_poly: &[[f64; 2]],
    inflation_m: f64,
) -> PatchOutcome {
    // 零障碍 / 退化障碍：直接直线（P1 无地形无限飞区，直线必然合法）。
    if obstacle_poly.is_empty() {
        return PatchOutcome::Path(vec![start, target], crate::path::haversine_m(start[0], start[1], target[0], target[1]) / 1000.0);
    }
    let hull = convex_hull(obstacle_poly);
    if hull.len() < 3 {
        // <3 顶点凸化（共线/单点）→ 不构成障碍 → 直线。
        return PatchOutcome::Path(vec![start, target], crate::path::haversine_m(start[0], start[1], target[0], target[1]) / 1000.0);
    }
    let inflated = inflate_convex(&hull, inflation_m / 111_320.0);
    if inflated.len() < 3 {
        return PatchOutcome::GeometricImpossible;
    }

    // 节点：0=start，1=target，2..=凸顶点（整数索引顺序 = 凸包输出序，确定性）。
    let n_verts = inflated.len();
    let n_nodes = 2 + n_verts;
    if n_nodes > MAX_VIS_VERTICES {
        return PatchOutcome::SearchTruncated;
    }
    let node = |id: usize| -> [f64; 2] {
        if id == 0 {
            start
        } else if id == 1 {
            target
        } else {
            inflated[id - 2]
        }
    };

    // 起点/终点被膨胀后障碍吞入 → 无机动空间，几何无解（P1 明确归因）。
    if point_in_convex(start, &inflated, GEOM_EPS_DEG) || point_in_convex(target, &inflated, GEOM_EPS_DEG) {
        return PatchOutcome::GeometricImpossible;
    }

    // 邻接表：任意两节点间线段不穿（膨胀后）障碍 → 合法边，权重 = 几何长度 km。
    let mut adj: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n_nodes];
    for i in 0..n_nodes {
        for j in (i + 1)..n_nodes {
            let pi = node(i);
            let pj = node(j);
            if !seg_intersects_convex(pi, pj, &inflated, GEOM_EPS_DEG) {
                let w = crate::path::haversine_m(pi[0], pi[1], pj[0], pj[1]) / 1000.0;
                adj[i].push((j, w));
                adj[j].push((i, w));
            }
        }
    }

    // Dijkstra（BinaryHeap 小根堆 + 显式 tie-break；确定性）。
    let mut dist = vec![f64::INFINITY; n_nodes];
    let mut prev = vec![usize::MAX; n_nodes];
    let mut heap = BinaryHeap::new();
    dist[0] = 0.0;
    heap.push(State { cost: 0.0, id: 0 });
    while let Some(State { cost, id }) = heap.pop() {
        if cost > dist[id] {
            continue;
        }
        if id == 1 {
            break;
        }
        for &(nid, w) in &adj[id] {
            let nc = cost + w;
            if nc < dist[nid] || (nc.to_bits() == dist[nid].to_bits() && nid < prev.iter().position(|&p| p == id).unwrap_or(usize::MAX)) {
                // 相等时按整数 id 序保确定性（prev 相同则跳过）
            }
            if nc < dist[nid] {
                dist[nid] = nc;
                prev[nid] = id;
                heap.push(State { cost: nc, id: nid });
            }
        }
    }
    if !dist[1].is_finite() {
        return PatchOutcome::GeometricImpossible;
    }
    // 回溯路径。
    let mut path = Vec::new();
    let mut cur = 1;
    while cur != usize::MAX {
        path.push(node(cur));
        if cur == 0 {
            break;
        }
        cur = prev[cur];
    }
    path.reverse();
    PatchOutcome::Path(path, dist[1])
}

/// P1 定位器（简化单簇）：提取 verify 硬闸失败点；若所有失败点均落在 `patch_r_deg`
/// 半径内（单簇）返回簇心（首点）；无坐标 / 多簇 → None（P1 不适用，归原流程）。
///
/// 依据 §3.2：patch 由硬闸失败点聚类触发，PATCH_R=30km 合并；P1 仅支持单簇单障碍。
pub fn locate_single_cluster(issues: &[String], patch_r_deg: f64) -> Option<[f64; 2]> {
    let mut pts = Vec::new();
    for s in issues {
        if let Some((lon, lat)) = extract_issue_coord(s) {
            pts.push([lon, lat]);
        }
    }
    if pts.is_empty() {
        return None;
    }
    let first = pts[0];
    for p in pts.iter().skip(1) {
        let d = crate::path::haversine_m(first[0], first[1], p[0], p[1]) / 1000.0;
        if d > patch_r_deg * 111.32 + 1.0 {
            return None;
        }
    }
    Some(first)
}

/// C3 归因分层：verify 硬闸失败后，按失败坐标判定缺陷类型。
///
/// - 失败坐标落在路径转折顶点邻域（距障碍 < `defect_deg`）→ `FittingDefect`；
/// - 否则（折线自身穿障碍/净空不足）→ `GeometricImpossible`。
///
/// `issues` 为 verify 报告中的硬性 issue（含 `lon=...,lat=...` 或 `(terrain ...)` 坐标）。
pub fn classify_verify_failure(
    path: &[[f64; 2]],
    issues: &[String],
    inflated: &[[f64; 2]],
    defect_deg: f64,
) -> PatchFailureClass {
    for s in issues {
        if let Some((lon, lat)) = extract_issue_coord(s) {
            // 失败点在转折顶点邻域（任一路径顶点到失败点距离 < defect_deg）
            for v in path {
                let d = ((v[0] - lon).powi(2) + (v[1] - lat).powi(2)).sqrt();
                if d < defect_deg {
                    return PatchFailureClass::FittingDefect;
                }
            }
            // 失败点本身在障碍（膨胀后）内/贴边 → 折线硬违规
            if point_in_convex([lon, lat], inflated, GEOM_EPS_DEG) {
                return PatchFailureClass::GeometricImpossible;
            }
        }
    }
    PatchFailureClass::GeometricImpossible
}

/// 从 verify issue 字符串提取坐标（格式 `lon=...,lat=...`）。
pub fn extract_issue_coord(s: &str) -> Option<(f64, f64)> {
    let find = |key: &str| -> Option<f64> {
        let idx = s.find(key)?;
        let tail = &s[idx + key.len()..];
        let end = tail.find(|c: char| !(c.is_ascii_digit() || c == '.' || c == '-' || c == '+' || c == 'e' || c == 'E'));
        let num = match end {
            Some(e) => &tail[..e],
            None => tail,
        };
        num.trim().parse().ok()
    };
    let lon = find("lon=")?;
    let lat = find("lat=")?;
    Some((lon, lat))
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn convex_hull_square() {
        let pts = [[116.0, 39.0], [117.0, 39.0], [117.0, 40.0], [116.0, 40.0]];
        let hull = convex_hull(&pts);
        assert_eq!(hull.len(), 4);
        // 逆时针
        for i in 0..hull.len() {
            let a = hull[i];
            let b = hull[(i + 1) % hull.len()];
            let c = hull[(i + 2) % hull.len()];
            assert!(cross(a, b, c) > 0.0, "not ccw at {i}");
        }
    }

    #[test]
    fn convex_hull_dedup_and_collinear() {
        let pts = [[116.0, 39.0], [116.0, 39.0], [116.5, 39.0], [117.0, 39.0], [117.0, 40.0], [116.0, 40.0]];
        let hull = convex_hull(&pts);
        assert_eq!(hull.len(), 4);
    }

    #[test]
    fn inflate_grows_area() {
        let poly = [[116.0, 39.0], [117.0, 39.0], [117.0, 40.0], [116.0, 40.0]];
        let inf = inflate_convex(&poly, 0.01);
        assert_eq!(inf.len(), 4);
        // 原顶点在膨胀多边形内部
        for p in &poly {
            assert!(point_in_convex(*p, &inf, 1e-9), "{p:?} should be inside inflated");
        }
    }

    #[test]
    fn point_in_convex_basic() {
        let poly = [[116.0, 39.0], [117.0, 39.0], [117.0, 40.0], [116.0, 40.0]];
        assert!(point_in_convex([116.5, 39.5], &poly, 0.0));
        assert!(!point_in_convex([118.0, 39.5], &poly, 0.0));
    }

    #[test]
    fn seg_intersects_convex_crossing() {
        let poly = [[116.0, 39.0], [117.0, 39.0], [117.0, 40.0], [116.0, 40.0]];
        // 穿越内部
        assert!(seg_intersects_convex([115.5, 39.5], [117.5, 39.5], &poly, GEOM_EPS_DEG));
        // 不穿
        assert!(!seg_intersects_convex([115.0, 38.0], [115.5, 38.5], &poly, GEOM_EPS_DEG));
        // 贴边（沿边界 y=39.0 行走）= 边界接触，不穿入内部 → 放行（可见图语义：
        // 膨胀后边界行走合法，机动空间已含）
        assert!(!seg_intersects_convex([115.5, 39.0], [117.5, 39.0], &poly, GEOM_EPS_DEG));
        // 起点在多边形内部 → 必穿内部 → 相交
        assert!(seg_intersects_convex([115.5, 39.5], [116.5, 39.0], &poly, GEOM_EPS_DEG));
    }

    #[test]
    fn plan_patch_no_obstacle_straight() {
        let out = plan_patch([116.0, 39.0], [117.0, 40.0], &[], 5_000.0);
        match out {
            PatchOutcome::Path(p, d) => {
                assert_eq!(p.len(), 2);
                assert!(d > 100.0 && d < 200.0);
            }
            other => panic!("expected straight, got {other:?}"),
        }
    }

    #[test]
    fn plan_patch_obstacle_detours() {
        // 障碍在起点-目标直线上 → 绕行长度 > 直线
        let obs = [[116.4, 39.3], [116.6, 39.3], [116.6, 39.7], [116.4, 39.7]];
        let out = plan_patch([116.0, 39.0], [117.0, 40.0], &obs, 5_000.0);
        match out {
            PatchOutcome::Path(p, d) => {
                assert!(p.len() >= 3, "should detour around obstacle: {p:?}");
                let straight = crate::path::haversine_m(116.0, 39.0, 117.0, 40.0) / 1000.0;
                assert!(d > straight, "detour longer than straight: {d} vs {straight}");
                // 路径不穿障碍（膨胀后）
                let hull = convex_hull(&obs);
                let inf = inflate_convex(&hull, 5_000.0 / 111_320.0);
                for w in p.windows(2) {
                    assert!(!seg_intersects_convex(w[0], w[1], &inf, GEOM_EPS_DEG), "edge crosses inflated");
                }
            }
            other => panic!("expected path, got {other:?}"),
        }
    }

    #[test]
    fn plan_patch_start_inside_inflated_impossible() {
        let obs = [[116.4, 39.3], [116.6, 39.3], [116.6, 39.7], [116.4, 39.7]];
        // start 在膨胀带内（贴近障碍但原始障碍外）→ 几何无解（P1 明确归因）
        let out = plan_patch([116.42, 39.3], [117.0, 40.0], &obs, 100_000.0);
        assert_eq!(out, PatchOutcome::GeometricImpossible);
    }

    #[test]
    fn plan_patch_search_truncated() {
        // 构造大量顶点 → 超过上限 → SearchTruncated（C2）
        let mut obs = Vec::new();
        let n = MAX_VIS_VERTICES;
        for i in 0..n {
            let t = i as f64 / n as f64 * std::f64::consts::TAU;
            obs.push([116.5 + 0.1 * t.cos(), 39.5 + 0.1 * t.sin()]);
        }
        let out = plan_patch([116.0, 39.0], [117.0, 40.0], &obs, 5_000.0);
        assert_eq!(out, PatchOutcome::SearchTruncated);
    }

    #[test]
    fn plan_patch_degenerate_hull_straight() {
        // B9：<3 顶点凸化（共线障碍不构成障碍）→ 直线交付
        let obs = [[116.4, 39.3], [116.6, 39.3], [116.5, 39.3]];
        match plan_patch([116.0, 39.0], [117.0, 40.0], &obs, 5_000.0) {
            PatchOutcome::Path(p, _) => assert_eq!(p.len(), 2, "collinear obstacle -> straight"),
            other => panic!("expected straight, got {other:?}"),
        }
    }

    #[test]
    fn plan_patch_start_equals_target() {
        // B9：锚点重合（start == target）→ 零长路径
        let obs = [[116.4, 39.3], [116.6, 39.3], [116.6, 39.7], [116.4, 39.7]];
        match plan_patch([116.0, 39.0], [116.0, 39.0], &obs, 5_000.0) {
            PatchOutcome::Path(p, d) => {
                assert_eq!(p.len(), 2);
                assert_eq!(d.to_bits(), 0.0_f64.to_bits());
            }
            other => panic!("expected zero-length path, got {other:?}"),
        }
    }

    #[test]
    fn tie_break_deterministic() {
        // 同一输入两次规划结果逐字节一致（确定性红线）
        let obs = [[116.4, 39.3], [116.6, 39.3], [116.6, 39.7], [116.4, 39.7]];
        let a = plan_patch([116.0, 39.0], [117.0, 40.0], &obs, 5_000.0);
        let b = plan_patch([116.0, 39.0], [117.0, 40.0], &obs, 5_000.0);
        assert_eq!(a, b);
        if let PatchOutcome::Path(pa, da) = &a {
            if let PatchOutcome::Path(pb, db) = &b {
                assert_eq!(pa, pb);
                assert_eq!(da.to_bits(), db.to_bits());
            }
        }
    }

    #[test]
    fn extract_issue_coord_parses() {
        let s = "sample (lon=116.42,lat=39.30) clearance fail";
        assert_eq!(extract_issue_coord(s), Some((116.42, 39.30)));
        assert_eq!(extract_issue_coord("no coord here"), None);
    }

    #[test]
    fn classify_fitting_vs_geometric() {
        let path = [[116.0, 39.0], [116.6, 39.7], [117.0, 40.0]];
        let obs = [[116.4, 39.3], [116.6, 39.3], [116.6, 39.7], [116.4, 39.7]];
        let inf = inflate_convex(&convex_hull(&obs), 5_000.0 / 111_320.0);
        // 失败点在转折顶点邻域 → FittingDefect
        let issues = vec!["sample (lon=116.599,lat=39.701) clearance fail".to_string()];
        assert_eq!(classify_verify_failure(&path, &issues, &inf, 0.01), PatchFailureClass::FittingDefect);
        // 失败点远离转折 → GeometricImpossible
        let issues2 = vec!["sample (lon=116.2,lat=39.1) clearance fail".to_string()];
        assert_eq!(classify_verify_failure(&path, &issues2, &inf, 0.01), PatchFailureClass::GeometricImpossible);
    }

    #[test]
    fn patch_applicable_excludes() {
        use crate::config::{Zone, ZoneShape, ZoneType};
        let mk = |t: ZoneType, s: ZoneShape| Zone {
            id: "z".into(),
            zone_type: t,
            shape: s,
            alt_min_m: 0.0,
            alt_max_m: 10000.0,
            height_semantics: Default::default(),
        };
        let poly_wall = mk(ZoneType::NoFly, ZoneShape::Polygon { vertices: vec![[116.0, 39.0], [117.0, 39.0], [117.0, 40.0]] });
        let circle_wall = mk(ZoneType::NoFly, ZoneShape::Circle { center: [116.5, 39.5], radius_km: 10.0 });
        let restricted = mk(ZoneType::Restricted, ZoneShape::Polygon { vertices: vec![[116.0, 39.0], [117.0, 39.0], [117.0, 40.0]] });
        // 纯多边形硬墙 → 适用
        assert!(patch_applicable(&[poly_wall.clone()], false, false));
        // 圆障碍 → 排除（R1：禁飞区绝对禁入）
        assert!(!patch_applicable(&[circle_wall], false, false));
        // 限飞区 → 排除
        assert!(!patch_applicable(&[restricted], false, false));
        // 地形 → 排除
        assert!(!patch_applicable(&[poly_wall.clone()], true, false));
        // 必经点 → 排除
        assert!(!patch_applicable(&[poly_wall], false, true));
    }
}

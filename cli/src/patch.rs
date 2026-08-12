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

use crate::threat::ThreatModel;
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

    // ---------- P2 ----------

    #[test]
    fn cluster_failures_merges_within_patch_r() {
        let issues = vec![
            "sample (lon=116.0,lat=39.0) fail".to_string(),
            "sample (lon=116.1,lat=39.0) fail".to_string(),
            "sample (lon=117.5,lat=40.5) fail".to_string(),
        ];
        // 前两点相距 ~11km（PATCH_R=30km 内合并），第三点 ~180km 外（新簇）
        let clusters = cluster_failures(&issues, PATCH_R_DEG);
        assert_eq!(clusters.len(), 2);
        // 簇心 = 字典序首点（确定性）
        assert_eq!(clusters[0], [116.0, 39.0]);
        assert_eq!(clusters[1], [117.5, 40.5]);
    }

    #[test]
    fn cluster_failures_deterministic_order() {
        let issues = vec![
            "sample (lon=117.5,lat=40.5) fail".to_string(),
            "sample (lon=116.1,lat=39.0) fail".to_string(),
            "sample (lon=116.0,lat=39.0) fail".to_string(),
        ];
        let a = cluster_failures(&issues, PATCH_R_DEG);
        let b = cluster_failures(&issues, PATCH_R_DEG);
        assert_eq!(a, b);
        // 字典序：116.0 先于 117.5
        assert!(a[0][0] < a[1][0]);
    }

    #[test]
    fn boundary_anchors_finds_enter_exit() {
        let rect = PatchRect::from_center([116.5, 39.5], PATCH_R_DEG);
        let skel = [
            [116.0, 39.0],
            [116.5, 39.0],  // 进入 patch（x 方向）
            [116.5, 39.5],
            [116.5, 40.0],  // 离开 patch
            [117.0, 40.0],
        ];
        let (ein, eout) = boundary_anchors(&skel, &rect);
        assert!(ein.is_some(), "enter anchor missing");
        assert!(eout.is_some(), "exit anchor missing");
        let ein = ein.unwrap();
        let eout = eout.unwrap();
        // 进入点在下边界（y = c - half）：骨架从 y=39.0 进入矩形（x=116.5 已居中）
        assert!((ein[1] - (39.5 - PATCH_R_DEG)).abs() < 1e-6, "enter on bottom edge: {ein:?}");
        // 离开点在上边界（y = c + half）：骨架沿 x=116.5 从 y=40.0 方向离开矩形
        assert!((eout[1] - (39.5 + PATCH_R_DEG)).abs() < 1e-6, "exit on top edge: {eout:?}");
    }

    #[test]
    fn plan_patch_multi_two_obstacles() {
        let obs1 = [[116.30, 39.30], [116.50, 39.30], [116.50, 39.70], [116.30, 39.70]];
        let obs2 = [[116.70, 39.30], [116.90, 39.30], [116.90, 39.70], [116.70, 39.70]];
        let out = plan_patch_multi([116.0, 39.0], [117.0, 40.0], &[obs1.to_vec(), obs2.to_vec()], &[], 5_000.0, 3000.0, None);
        match out {
            PatchOutcome::Path(p, d) => {
                assert!(p.len() >= 3, "should detour two obstacles: {p:?}");
                let straight = crate::path::haversine_m(116.0, 39.0, 117.0, 40.0) / 1000.0;
                assert!(d > straight, "detour longer than straight: {d} vs {straight}");
            }
            other => panic!("expected path, got {other:?}"),
        }
    }

    #[test]
    fn plan_patch_multi_restricted_chord_check() {
        use crate::config::{Zone, ZoneShape, ZoneType};
        // 限飞区（多边形）覆盖直线路径；高度 3000 在禁行带 [2000, 4000] 内 → 边被拒；
        // 高度 1000（禁行带外底部）→ 直穿合法。
        let z = Zone {
            id: "rz".into(),
            zone_type: ZoneType::Restricted,
            shape: ZoneShape::Polygon { vertices: vec![[116.30, 39.30], [116.70, 39.30], [116.70, 39.70], [116.30, 39.70]] },
            alt_min_m: 2000.0,
            alt_max_m: 4000.0,
            height_semantics: Default::default(),
        };
        let restricted = vec![z];
        // 带内 → 直线被拒（restricted 覆盖直线）→ 无路径（单障碍 none → 直线但受限拒）
        let out_blocked = plan_patch_multi(
            [116.0, 39.0],
            [117.0, 40.0],
            &[],
            &[&restricted[0]],
            5_000.0,
            3000.0,
            None,
        );
        match out_blocked {
            PatchOutcome::GeometricImpossible => {}
            other => panic!("expected blocked (in band), got {other:?}"),
        }
        // 带外底部 → 直穿合法
        let out_pass = plan_patch_multi(
            [116.0, 39.0],
            [117.0, 40.0],
            &[],
            &[&restricted[0]],
            5_000.0,
            1000.0,
            None,
        );
        match out_pass {
            PatchOutcome::Path(p, _) => assert_eq!(p.len(), 2, "straight pass under band: {p:?}"),
            other => panic!("expected pass (under band), got {other:?}"),
        }
    }

    #[test]
    fn stitch_joins_skeleton_and_patch() {
        let skel = [[116.0, 39.0], [116.4, 39.4], [116.8, 39.8], [117.0, 40.0]];
        let patch = [[116.4, 39.4], [116.55, 39.7], [116.8, 39.8]];
        let out = stitch(&skel, &patch, [116.4, 39.4], [116.8, 39.8]);
        assert_eq!(out[0], skel[0]);
        assert_eq!(*out.last().unwrap(), skel[3]);
        // 拼接后经过 patch 中点（绕行点）
        assert!(out.contains(&[116.55, 39.7]));
        // 无重复接缝点（相邻点不相等）
        for w in out.windows(2) {
            assert!(w[0] != w[1], "duplicate seam point: {:?}", w);
        }
    }

    #[test]
    fn boundary_anchors_none_when_skeleton_fully_in_out() {
        let rect = PatchRect::from_center([116.5, 39.5], PATCH_R_DEG);
        // 骨架全程在矩形内 → 无进入/离开锚点
        let inside = [[116.5, 39.4], [116.5, 39.5], [116.5, 39.6]];
        let (ein, eout) = boundary_anchors(&inside, &rect);
        assert!(ein.is_none() && eout.is_none(), "fully inside -> no anchors: {ein:?} {eout:?}");
        // 骨架全程在矩形外 → 无锚点
        let outside = [[116.0, 39.0], [116.0, 39.1], [116.0, 39.2]];
        let (ein, eout) = boundary_anchors(&outside, &rect);
        assert!(ein.is_none() && eout.is_none(), "fully outside -> no anchors: {ein:?} {eout:?}");
    }

    #[test]
    fn plan_patch_multi_anchor_in_obstacle_impossible() {
        // start/target 被障碍吞入（膨胀后）→ 无机动空间 → 几何无解
        let obs = [[116.30, 39.30], [116.70, 39.30], [116.70, 39.70], [116.30, 39.70]];
        let out = plan_patch_multi([116.5, 39.5], [117.0, 40.0], &[obs.to_vec()], &[], 10_000.0, 3000.0, None);
        assert!(
            matches!(out, PatchOutcome::GeometricImpossible),
            "anchor swallowed by obstacle -> impossible: {out:?}"
        );
    }

    #[test]
    fn stitch_exact_skeleton_no_duplicate() {
        // patch 与骨架完全重合（无绕行）→ 拼接后与骨架一致且无重复点
        let skel = [[116.0, 39.0], [116.5, 39.5], [117.0, 40.0]];
        let patch = skel.to_vec();
        let out = stitch(&skel, &patch, skel[0], skel[2]);
        assert_eq!(out, skel, "coincident patch -> skeleton unchanged: {out:?}");
    }

    #[test]
    fn patch_retry_budget_fixed_steps() {
        // C7：重试上限 + 固定扩张步长（确定性，不写死绝对耗时）
        assert_eq!(PATCH_RETRY_MAX, 2);
        assert_eq!(PATCH_RETRY_EXPAND, 1.5);
        let mut rect = PatchRect::from_center([116.5, 39.5], PATCH_R_DEG);
        let h0 = rect.half_deg;
        for _ in 0..PATCH_RETRY_MAX {
            rect.half_deg *= PATCH_RETRY_EXPAND;
        }
        assert!((rect.half_deg - h0 * 1.5f64.powi(PATCH_RETRY_MAX as i32)).abs() < 1e-12);
        assert!(rect.half_deg > h0, "retry expands patch deterministically");
    }

    #[test]
    fn multi_patch_stitch_two_clusters() {
        // 多 patch 串接（§11.2）：两簇独立 patch 依次拼入骨架，首尾锚点保留
        let skel = [[116.0, 38.0], [116.5, 38.5], [116.5, 39.0], [116.5, 39.5], [117.0, 40.0]];
        let rect1 = PatchRect::from_center([116.5, 38.5], PATCH_R_DEG);
        let rect2 = PatchRect::from_center([116.5, 39.5], PATCH_R_DEG);
        let (ein1, eout1) = boundary_anchors(&skel, &rect1);
        let (ein2, eout2) = boundary_anchors(&skel, &rect2);
        assert!(ein1.is_some() && eout1.is_some() && ein2.is_some() && eout2.is_some());
        // patch1 绕第一个簇（右侧绕行），patch2 绕第二个簇（左侧绕行）
        let patch1 = vec![ein1.unwrap(), [116.9, 38.7], eout1.unwrap()];
        let mid1 = stitch(&skel, &patch1, ein1.unwrap(), eout1.unwrap());
        let patch2 = vec![ein2.unwrap(), [116.1, 39.7], eout2.unwrap()];
        let mid2 = stitch(&mid1, &patch2, ein2.unwrap(), eout2.unwrap());
        // 串接后：起点保留、两个绕行点都在、终点保留
        assert_eq!(mid2[0], skel[0]);
        assert_eq!(*mid2.last().unwrap(), skel[4]);
        assert!(mid2.contains(&[116.9, 38.7]));
        assert!(mid2.contains(&[116.1, 39.7]));
        // 无重复接缝点
        for w in mid2.windows(2) {
            assert!(w[0] != w[1], "duplicate seam: {:?}", w);
        }
    }

    #[test]
    fn radar_edge_weight_detours_high_cost() {
        use crate::config::{Radar, RadarType};
        use crate::threat::{SphericalRadarThreat, ThreatParams};
        let radar = Radar {
            id: "r".into(),
            lon: 116.5,
            lat: 39.5,
            radar_type: RadarType::Tracking,
            radius_km: 40.0,
            alt_m: 10.0,
            suppression_post_range_km: None,
            suppression_factor: None,
        };
        let params = ThreatParams::default();
        let threat = SphericalRadarThreat::new(std::slice::from_ref(&radar), params);
        // 穿雷达中心线段（中点距雷达 0）→ 代价显著高于远离线
        let w_center = edge_weight([116.0, 39.5], [117.0, 39.5], Some((&threat, 200.0)), 3000.0);
        let w_far = edge_weight([116.0, 38.0], [117.0, 38.0], Some((&threat, 200.0)), 3000.0);
        assert!(w_center > w_far, "center pass should cost more: {w_center} vs {w_far}");
        // 无雷达 → 纯几何
        let w_plain = edge_weight([116.0, 39.5], [117.0, 39.5], None, 3000.0);
        assert_eq!(w_plain.to_bits(), (crate::path::haversine_m(116.0, 39.5, 117.0, 39.5) / 1000.0).to_bits());
    }
}

// ==================== P2：patch 矩形 + 多簇 + 锚点 + 拼接（docs/12 §3.2/3.3/3.5） ====================

/// C7：接缝重试硬上限（次）——不写死绝对耗时（大输入误报 degraded_timeout），
/// 预算 = 重试次数上限 × 固定扩张步长（契约 2 终止性 + 确定性）。
pub const PATCH_RETRY_MAX: usize = 2;
/// 接缝重试固定扩张步长：每次 patch 半径 × 1.5。
pub const PATCH_RETRY_EXPAND: f64 = 1.5;

/// patch 矩形（§3.2：以触发点为中心的矩形走廊段，尺寸 PATCH_R=30km）。
#[derive(Debug, Clone, Copy, PartialEq)]
pub struct PatchRect {
    pub c: [f64; 2],
    pub half_deg: f64,
}

impl PatchRect {
    pub fn from_center(c: [f64; 2], r_deg: f64) -> Self {
        PatchRect { c, half_deg: r_deg }
    }
    pub fn contains(&self, p: [f64; 2]) -> bool {
        (p[0] - self.c[0]).abs() <= self.half_deg && (p[1] - self.c[1]).abs() <= self.half_deg
    }
}

/// 多簇聚类：verify 硬闸失败点按 `patch_r_deg` 合并（§3.2：距离 < PATCH_R 合并）。
///
/// 确定性（§3.2/§7）：先按坐标字典序（`total_cmp`）排序，再贪心合并——簇心取
/// 字典序首点，合并顺序与平局全部显式固定。
pub fn cluster_failures(issues: &[String], patch_r_deg: f64) -> Vec<[f64; 2]> {
    let mut pts: Vec<[f64; 2]> = Vec::new();
    for s in issues {
        if let Some((lon, lat)) = extract_issue_coord(s) {
            pts.push([lon, lat]);
        }
    }
    if pts.is_empty() {
        return Vec::new();
    }
    pts.sort_by(|a, b| a[0].total_cmp(&b[0]).then_with(|| a[1].total_cmp(&b[1])));
    let mut clusters: Vec<[f64; 2]> = Vec::new();
    for p in pts {
        let d_min = clusters
            .iter()
            .map(|c| crate::path::haversine_m(c[0], c[1], p[0], p[1]) / 1000.0)
            .fold(f64::INFINITY, f64::min);
        if d_min <= patch_r_deg * 111.32 + 1.0 {
            // 并入已有簇（簇心保持字典序首点——确定性）
        } else {
            clusters.push(p);
        }
    }
    clusters
}

/// 线段与矩形边界交点（参数 t ∈ [0,1] 排序）；矩形用 4 条边半平面表示。
fn seg_rect_crossings(a: [f64; 2], b: [f64; 2], rect: &PatchRect) -> Vec<(f64, [f64; 2])> {
    let mut out = Vec::new();
    let corners = [
        [rect.c[0] - rect.half_deg, rect.c[1] - rect.half_deg],
        [rect.c[0] + rect.half_deg, rect.c[1] - rect.half_deg],
        [rect.c[0] + rect.half_deg, rect.c[1] + rect.half_deg],
        [rect.c[0] - rect.half_deg, rect.c[1] + rect.half_deg],
    ];
    let edges = [
        (corners[0], corners[1]),
        (corners[1], corners[2]),
        (corners[2], corners[3]),
        (corners[3], corners[0]),
    ];
    for (p1, p2) in edges {
        // 两线段求交（参数法）
        let d1 = [b[0] - a[0], b[1] - a[1]];
        let d2 = [p2[0] - p1[0], p2[1] - p1[1]];
        let denom = d1[0] * d2[1] - d1[1] * d2[0];
        if denom.abs() < 1e-15 {
            continue; // 平行/共线（沿矩形边行走由 seg_intersects_convex 兜底）
        }
        let t1 = ((p1[0] - a[0]) * d2[1] - (p1[1] - a[1]) * d2[0]) / denom;
        let t2 = ((p1[0] - a[0]) * d1[1] - (p1[1] - a[1]) * d1[0]) / denom;
        if (0.0..=1.0).contains(&t1) && (0.0..=1.0).contains(&t2) {
            out.push((t1, [a[0] + t1 * d1[0], a[1] + t1 * d1[1]]));
        }
    }
    out.sort_by(|x, y| x.0.total_cmp(&y.0));
    out
}

/// 边界锚点（§3.2/§7）：骨架与 patch 矩形边界的交点——进入锚点（外→内首个）与
/// 离开锚点（内→外最后一个）。平局按骨架点索引序（遍历序即索引序，显式确定）。
pub fn boundary_anchors(skeleton: &[[f64; 2]], rect: &PatchRect) -> (Option<[f64; 2]>, Option<[f64; 2]>) {
    let mut in_anchor: Option<[f64; 2]> = None;
    let mut out_anchor: Option<[f64; 2]> = None;
    for w in skeleton.windows(2) {
        let a = w[0];
        let b = w[1];
        let a_in = rect.contains(a);
        let b_in = rect.contains(b);
        if !a_in && b_in {
            // 进入：取段与边界最小 t 交点
            let cr = seg_rect_crossings(a, b, rect);
            if let Some((_, p)) = cr.first() {
                if in_anchor.is_none() {
                    in_anchor = Some(*p);
                }
            }
        } else if a_in && !b_in {
            // 离开：取最大 t 交点（最后离开）
            let cr = seg_rect_crossings(a, b, rect);
            if let Some((_, p)) = cr.last() {
                out_anchor = Some(*p);
            }
        }
    }
    (in_anchor, out_anchor)
}

/// P2 多障碍可见图规划（docs/12 §3.3）：
/// - 顶点 = start/target 锚点 + 各硬墙凸化（C1）后矢量膨胀（C4）顶点；
/// - 边合法性：不穿任何凸化障碍（C5 epsilon 保守）+ 限飞区弦判据（高度判定 +
///   净距，restricted 不凸化）+ 雷达同源代价不进合法性（只进边权）；
/// - 边权（P2 口径，§3.3/§12.2）：几何长度 × (1 + radar_cost_coef × (p + geom))，
///   与 FMM 同源（×/同款 static_penetration），避免接缝处两套口径冲突。
///
/// `radar` = Some((threat, radar_cost_coef)) 时启用雷达同源边权；None → 纯几何（P1）。
pub fn plan_patch_multi(
    start: [f64; 2],
    target: [f64; 2],
    obstacles: &[Vec<[f64; 2]>],
    restricted: &[&crate::config::Zone],
    inflation_m: f64,
    alt_m: f64,
    radar: Option<(&crate::threat::SphericalRadarThreat, f64)>,
) -> PatchOutcome {
    // 各障碍凸化 + 膨胀
    let mut hulls: Vec<Vec<[f64; 2]>> = Vec::new();
    let mut inflated: Vec<Vec<[f64; 2]>> = Vec::new();
    for obs in obstacles {
        let hull = convex_hull(obs);
        if hull.len() < 3 {
            continue; // 退化（共线/单点）不构成障碍
        }
        let inf = inflate_convex(&hull, inflation_m / 111_320.0);
        if inf.len() < 3 {
            continue;
        }
        hulls.push(hull);
        inflated.push(inf);
    }

    // 无有效障碍 → 直线（仍过限飞区弦判据与雷达代价）
    if inflated.is_empty() {
        if restricted_edge_ok(restricted, alt_m, start, target).is_err() {
            return PatchOutcome::GeometricImpossible;
        }
        let w = edge_weight(start, target, radar, alt_m);
        return PatchOutcome::Path(vec![start, target], w);
    }

    // 节点：0=start，1=target，之后各障碍凸顶点（按障碍序 + 顶点序，确定性）
    let mut nodes: Vec<[f64; 2]> = Vec::new();
    nodes.push(start);
    nodes.push(target);
    for inf in &inflated {
        nodes.extend(inf.iter().copied());
    }
    let n_nodes = nodes.len();
    if n_nodes > MAX_VIS_VERTICES {
        return PatchOutcome::SearchTruncated;
    }

    // 起点/终点被任一膨胀障碍吞入 → 无机动空间，几何无解
    for inf in &inflated {
        if point_in_convex(start, inf, GEOM_EPS_DEG) || point_in_convex(target, inf, GEOM_EPS_DEG) {
            return PatchOutcome::GeometricImpossible;
        }
    }

    // 邻接表：边不穿任何障碍 + 不违限飞区弦判据
    let mut adj: Vec<Vec<(usize, f64)>> = vec![Vec::new(); n_nodes];
    for i in 0..n_nodes {
        for j in (i + 1)..n_nodes {
            let pi = nodes[i];
            let pj = nodes[j];
            let mut legal = true;
            for inf in &inflated {
                if seg_intersects_convex(pi, pj, inf, GEOM_EPS_DEG) {
                    legal = false;
                    break;
                }
            }
            if !legal {
                continue;
            }
            // 限飞区弦判据（§3.3：不凸化；高度判定 + 直线穿行带净距）
            if restricted_edge_ok(restricted, alt_m, pi, pj).is_err() {
                continue;
            }
            let w = edge_weight(pi, pj, radar, alt_m);
            adj[i].push((j, w));
            adj[j].push((i, w));
        }
    }

    dijkstra_path(n_nodes, &adj, &nodes)
}

/// P2 边权：几何长度 × 雷达同源代价——与 FMM 完全同款（solver.rs:454-465 口径）：
/// 只在探测概率 p > 0 时应用 ×(1 + coef×(p + geom))；geom = 深穿惩罚（u<1 时 1-u）。
fn edge_weight(
    a: [f64; 2],
    b: [f64; 2],
    radar: Option<(&crate::threat::SphericalRadarThreat, f64)>,
    alt_m: f64,
) -> f64 {
    let base = crate::path::haversine_m(a[0], a[1], b[0], b[1]) / 1000.0;
    if let Some((threat, coef)) = radar {
        // 中点采样（与 FMM 格点采样同口径；P2 段长 ≤ 60km 采样密度足够）
        let mid = [(a[0] + b[0]) / 2.0, (a[1] + b[1]) / 2.0];
        let p = threat.static_union_probability(mid[0], mid[1]);
        if p > 0.0 {
            let u = threat.static_penetration(mid[0], mid[1], alt_m);
            let geom = if u < 1.0 { 1.0 - u } else { 0.0 };
            base * (1.0 + coef * (p + geom))
        } else {
            base
        }
    } else {
        base
    }
}

/// 限飞区弦判据边检查（docs/12 §3.3）：restricted 不进入可见图，作为段合法性检查。
///
/// - 高度在禁行区间外（底部穿行/顶部绕飞语义，契约 7/8）→ 直穿合法；
/// - 高度在禁行区间内 → 段净距 ≤ 0（穿入/贴边）→ 拒绝。
fn restricted_edge_ok(
    restricted: &[&crate::config::Zone],
    alt_m: f64,
    a: [f64; 2],
    b: [f64; 2],
) -> Result<(), ()> {
    for z in restricted {
        if !crate::solver::restricted_blocks_alt(z, alt_m) {
            continue; // 高度在禁行带外 → 直穿合法
        }
        let cl = crate::config::zone_segment_clearance_km(a[0], a[1], b[0], b[1], z);
        if cl <= 0.0 {
            return Err(());
        }
    }
    Ok(())
}

/// Dijkstra 最短路径（确定性：total_cmp + 整数索引 tie-break）。
fn dijkstra_path(n_nodes: usize, adj: &[Vec<(usize, f64)>], nodes: &[[f64; 2]]) -> PatchOutcome {
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
    let mut path = Vec::new();
    let mut cur = 1;
    while cur != usize::MAX {
        path.push(nodes[cur]);
        if cur == 0 {
            break;
        }
        cur = prev[cur];
    }
    path.reverse();
    PatchOutcome::Path(path, dist[1])
}

/// 拼接（§3.5）：patch 路径在边界锚点处并入走廊骨架。
///
/// 骨架在进入锚点之前的点 + patch 路径 + 离开锚点之后的点；接缝重复点去重
/// （patch 首尾 = 锚点）。`in_anchor`/`out_anchor` 必须位于 patch 路径首尾。
pub fn stitch(
    skeleton: &[[f64; 2]],
    patch: &[[f64; 2]],
    in_anchor: [f64; 2],
    out_anchor: [f64; 2],
) -> Vec<[f64; 2]> {
    let dist2 = |p: [f64; 2], q: [f64; 2]| (p[0] - q[0]).powi(2) + (p[1] - q[1]).powi(2);
    let i_idx = skeleton
        .iter()
        .enumerate()
        .min_by(|(_, p), (_, q)| dist2(**p, in_anchor).total_cmp(&dist2(**q, in_anchor)))
        .map(|(i, _)| i)
        .unwrap_or(0);
    let j_idx = skeleton
        .iter()
        .enumerate()
        .min_by(|(_, p), (_, q)| dist2(**p, out_anchor).total_cmp(&dist2(**q, out_anchor)))
        .map(|(i, _)| i)
        .unwrap_or(skeleton.len().saturating_sub(1));
    let mut out: Vec<[f64; 2]> = Vec::new();
    if i_idx < skeleton.len() {
        out.extend_from_slice(&skeleton[..=i_idx]);
    }
    // 去重接缝：patch 首点若与骨架尾点重合则跳过
    let start_skip = patch.first().map_or(0, |p| {
        if out.last().map_or(false, |q| q[0].to_bits() == p[0].to_bits() && q[1].to_bits() == p[1].to_bits()) {
            1
        } else {
            0
        }
    });
    out.extend_from_slice(&patch[start_skip..]);
    let end_skip = patch.last().map_or(0, |p| {
        if skeleton.get(j_idx).map_or(false, |q| q[0].to_bits() == p[0].to_bits() && q[1].to_bits() == p[1].to_bits()) {
            1
        } else {
            0
        }
    });
    if end_skip == 1 {
        out.pop();
    }
    if j_idx < skeleton.len() {
        out.extend_from_slice(&skeleton[j_idx..]);
    }
    out
}

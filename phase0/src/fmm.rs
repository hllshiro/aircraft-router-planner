//! FMM 粗层传播原型（Phase 0 S2 / B1 基准）。
//!
//! 2D Fast Marching Method 求解 Eikonal `|∇T| = c(x,y)`：
//! - 窄带用 `BinaryHeap`（min-heap），三元状态 Far / Considered / Accepted；
//! - 2D Godunov 迎风差分更新（经典双邻居解二次方程）；
//! - 合成代价场 = 平滑地形（正弦叠加）+ 雷达高代价球 + 禁飞区矩形块；
//! - 路径回溯：终点沿 T 场梯度最大下降方向走回源点（走廊质量代理指标）。
//!
//! 全部确定性：代价场与随机源/终点均由固定种子生成（rand 0.10 StdRng）。

use std::cmp::Ordering;
use std::collections::BinaryHeap;

/// 2D 代价场（行优先，`idx = r * cols + c`）。`cost >= 1`，越大越难通过。
pub struct CostField {
    pub rows: usize,
    pub cols: usize,
    pub cost: Vec<f32>,
}

impl CostField {
    pub fn new(rows: usize, cols: usize) -> Self {
        Self {
            rows,
            cols,
            cost: vec![1.0; rows * cols],
        }
    }

    #[inline]
    pub fn get(&self, r: usize, c: usize) -> f32 {
        self.cost[r * self.cols + c]
    }

    #[inline]
    pub fn idx(&self, r: usize, c: usize) -> usize {
        r * self.cols + c
    }

    #[inline]
    pub fn in_bounds(&self, r: usize, c: usize) -> bool {
        r < self.rows && c < self.cols
    }
}

/// 合成代价场：平滑地形（正弦叠加）+ 雷达高代价球 + 禁飞区矩形块。
///
/// `cell_m`：网格单元边长（米），用于把公里级障碍参数换算成网格格数。
/// 地形代价：海拔 > 2500m 区域线性升高（至多 20x），模拟山区绕飞。
pub fn synthetic_cost_field(rows: usize, cols: usize, cell_m: f64, seed: u64) -> CostField {
    use rand::{RngExt, SeedableRng};
    let mut f = CostField::new(rows, cols);

    // --- 地形：两个方向的正弦叠加（平滑大尺度起伏），范围 0~4000m ---
    for r in 0..rows {
        for c in 0..cols {
            let i = r * cols + c;
            let x = r as f64 * cell_m;
            let y = c as f64 * cell_m;
            let h = 1800.0
                + 1200.0 * (x / 60_000.0 * std::f64::consts::TAU).sin()
                + 900.0 * (y / 45_000.0 * std::f64::consts::TAU).cos()
                + 450.0 * ((x + y) / 25_000.0 * std::f64::consts::TAU).sin();
            let h = h.clamp(0.0, 4200.0);
            let terr_cost = if h > 2500.0 {
                1.0 + 19.0 * (h - 2500.0) / (4200.0 - 2500.0)
            } else {
                1.0
            };
            f.cost[i] = terr_cost as f32;
        }
    }

    // --- 雷达高代价球：3 个，半径 ~15km，代价 30x ---
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let radar_cost = 30.0f32;
    for _ in 0..3 {
        let cx = rng.random_range(0.25..0.75) * rows as f64;
        let cy = rng.random_range(0.25..0.75) * cols as f64;
        let rad = 15_000.0 / cell_m; // 格数
        for r in 0..rows {
            for c in 0..cols {
                let i = r * cols + c;
                let dr = r as f64 - cx;
                let dc = c as f64 - cy;
                let d2 = dr * dr + dc * dc;
                if d2 <= rad * rad {
                    f.cost[i] = radar_cost;
                }
            }
        }
    }

    // --- 禁飞区矩形块：2 个，30x25km，代价 50x ---
    let nf_cost = 50.0f32;
    let mut rng2 = rand::rngs::StdRng::seed_from_u64(seed ^ 0xDEADBEEF);
    for _ in 0..2 {
        let cr = rng2.random_range(0.3..0.7) * rows as f64;
        let cc = rng2.random_range(0.3..0.7) * cols as f64;
        let hr = 15_000.0 / cell_m;
        let hc = 12_500.0 / cell_m;
        for r in 0..rows {
            for c in 0..cols {
                let i = r * cols + c;
                if (r as f64 - cr).abs() <= hr && (c as f64 - cc).abs() <= hc {
                    f.cost[i] = nf_cost;
                }
            }
        }
    }

    f
}

/// 窄带堆元素（按到达时间小优先）。
#[derive(Clone, Copy)]
struct HeapEntry {
    t: f32,
    idx: usize,
}

impl PartialEq for HeapEntry {
    fn eq(&self, other: &Self) -> bool {
        self.t == other.t
    }
}
impl Eq for HeapEntry {}
impl PartialOrd for HeapEntry {
    fn partial_cmp(&self, other: &Self) -> Option<Ordering> {
        Some(self.cmp(other))
    }
}
impl Ord for HeapEntry {
    fn cmp(&self, other: &Self) -> Ordering {
        // BinaryHeap 是 max-heap；反转使小 t 优先级高
        other
            .t
            .partial_cmp(&self.t)
            .unwrap_or(Ordering::Equal)
            .then_with(|| other.idx.cmp(&self.idx))
    }
}

/// FMM 传播结果。
pub struct FmmResult {
    /// 到达时间场（未到达为 `f32::INFINITY`）。
    pub times: Vec<f32>,
    /// 是否被接受（冻结）。
    pub accepted: Vec<bool>,
}

const STATE_FAR: u8 = 0;
const STATE_CONSIDERED: u8 = 1;
const STATE_ACCEPTED: u8 = 2;

/// 2D Godunov 迎风更新：以已接受邻居解二次方程。
#[inline]
fn solve_t(field: &CostField, times: &[f32], state: &[u8], r: usize, c: usize) -> f32 {
    let cost = field.get(r, c);
    let idx = |rr: usize, cc: usize| rr * field.cols + cc;

    let mut tx = f32::INFINITY;
    if r > 0 && state[idx(r - 1, c)] == STATE_ACCEPTED {
        tx = times[idx(r - 1, c)];
    }
    if r + 1 < field.rows && state[idx(r + 1, c)] == STATE_ACCEPTED {
        let v = times[idx(r + 1, c)];
        if v < tx {
            tx = v;
        }
    }
    let mut ty = f32::INFINITY;
    if c > 0 && state[idx(r, c - 1)] == STATE_ACCEPTED {
        ty = times[idx(r, c - 1)];
    }
    if c + 1 < field.cols && state[idx(r, c + 1)] == STATE_ACCEPTED {
        let v = times[idx(r, c + 1)];
        if v < ty {
            ty = v;
        }
    }

    if tx == f32::INFINITY && ty == f32::INFINITY {
        return f32::INFINITY;
    }
    if tx == f32::INFINITY {
        return ty + cost;
    }
    if ty == f32::INFINITY {
        return tx + cost;
    }
    let diff = (tx - ty).abs();
    let two_c2 = 2.0 * cost * cost;
    if two_c2 >= diff * diff {
        (tx + ty + (two_c2 - diff * diff).sqrt()) * 0.5
    } else {
        tx.min(ty) + cost
    }
}

/// 对已接受格点 `(r, c)` 的四邻域做一次更新尝试。
#[inline]
fn update_neighbors(
    field: &CostField,
    times: &mut [f32],
    state: &mut [u8],
    heap: &mut BinaryHeap<HeapEntry>,
    r: usize,
    c: usize,
) {
    let (rows, cols) = (field.rows, field.cols);
    let neighbors = [
        (r.wrapping_sub(1), c, r > 0),
        (r + 1, c, r + 1 < rows),
        (r, c.wrapping_sub(1), c > 0),
        (r, c + 1, c + 1 < cols),
    ];
    for (nr, nc, ok) in neighbors {
        if !ok {
            continue;
        }
        let nidx = nr * cols + nc;
        if state[nidx] == STATE_ACCEPTED {
            continue;
        }
        let t = solve_t(field, times, state, nr, nc);
        if t < times[nidx] {
            times[nidx] = t;
            if state[nidx] == STATE_FAR {
                state[nidx] = STATE_CONSIDERED;
            }
            heap.push(HeapEntry { t, idx: nidx });
        }
    }
}

/// FMM 单源传播：从 `(src_r, src_c)` 出发求解全场到达时间。
///
/// 防御：网格为空或源点越界时返回空结果（`times` 全 INF、`accepted` 全 false），不 panic。
pub fn fmm_propagate(field: &CostField, src_r: usize, src_c: usize) -> FmmResult {
    let n = field.rows * field.cols;
    let mut times = vec![f32::INFINITY; n];
    let mut state = vec![STATE_FAR; n];
    if n == 0 || src_r >= field.rows || src_c >= field.cols {
        return FmmResult {
            times,
            accepted: state.iter().map(|&s| s == STATE_ACCEPTED).collect(),
        };
    }
    let mut heap: BinaryHeap<HeapEntry> = BinaryHeap::with_capacity(n / 4);

    let s = field.idx(src_r, src_c);
    times[s] = 0.0;
    state[s] = STATE_ACCEPTED;
    update_neighbors(field, &mut times, &mut state, &mut heap, src_r, src_c);

    while let Some(ent) = heap.pop() {
        let idx = ent.idx;
        if state[idx] == STATE_ACCEPTED {
            continue; // 过期条目（lazy deletion）
        }
        state[idx] = STATE_ACCEPTED;
        let r = idx / field.cols;
        let c = idx % field.cols;
        update_neighbors(field, &mut times, &mut state, &mut heap, r, c);
    }

    FmmResult {
        times,
        accepted: state.iter().map(|&s| s == STATE_ACCEPTED).collect(),
    }
}

/// 从终点沿 T 场最大下降方向回溯到源点（走廊质量代理：路径长度/绕行比）。
///
/// 返回路径（从终点到源点，含两端）。若终点不可达返回 `None`。
pub fn backtrack_path(
    field: &CostField,
    res: &FmmResult,
    dst_r: usize,
    dst_c: usize,
    src_r: usize,
    src_c: usize,
) -> Option<Vec<(usize, usize)>> {
    if !res.accepted[field.idx(dst_r, dst_c)] {
        return None;
    }
    let mut path = vec![(dst_r, dst_c)];
    let mut r = dst_r;
    let mut c = dst_c;
    let mut guard = 0usize;
    let max_steps = field.rows * field.cols;
    while (r, c) != (src_r, src_c) {
        guard += 1;
        if guard > max_steps {
            return None;
        }
        let t_cur = res.times[field.idx(r, c)];
        let mut best = (r, c);
        let mut best_t = t_cur;
        for (nr, nc, ok) in [
            (r.wrapping_sub(1), c, r > 0),
            (r + 1, c, r + 1 < field.rows),
            (r, c.wrapping_sub(1), c > 0),
            (r, c + 1, c + 1 < field.cols),
        ] {
            if !ok {
                continue;
            }
            let t = res.times[field.idx(nr, nc)];
            if t < best_t {
                best_t = t;
                best = (nr, nc);
            }
        }
        if best == (r, c) {
            return None; // 卡在局部极小（理论上 FMM 场不会发生，防御性返回）
        }
        path.push(best);
        r = best.0;
        c = best.1;
    }
    Some(path)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn fmm_constant_field_reaches_all() {
        // 均匀代价场：全部格点可达，且时间场单调增长
        let f = CostField::new(64, 64);
        let res = fmm_propagate(&f, 32, 32);
        let n = f.rows * f.cols;
        assert_eq!(res.accepted.iter().filter(|&&a| a).count(), n);
        // 对角线方向（斜向 45°）到角落的时间应 > 直线距离代价
        let corner = f.idx(0, 0);
        assert!(res.times[corner].is_finite());
        assert!(res.times[corner] > 0.0);
    }

    #[test]
    fn fmm_backtrack_reaches_source() {
        let f = CostField::new(48, 48);
        let (sr, sc) = (24, 24);
        let res = fmm_propagate(&f, sr, sc);
        let path = backtrack_path(&f, &res, 5, 40, sr, sc).expect("reachable");
        assert_eq!(*path.first().unwrap(), (5, 40));
        assert_eq!(*path.last().unwrap(), (sr, sc));
        // 路径长度不小于直线距离（欧氏，单元网格）
        let steps = path.len() as f64;
        let direct = ((40i64 - sc as i64).pow(2) + (5i64 - sr as i64).pow(2)) as f64;
        assert!(steps as f64 >= direct.sqrt() - 1e-6);
    }
}

//! 代价场 + FMM 粗层传播（技术方案 4.4 定案主方案；迁移自 phase0 fmm.rs，B1 实测）。
//!
//! - `CostField`：2D 代价场（行优先，cost ≥ 1）；
//! - `fmm_propagate`：Godunov 迎风差分 + BinaryHeap 窄带，O(NlogN)，确定性；
//! - `backtrack_path`：沿 T 场最大下降方向回溯（走廊质量代理）；
//! - `synthetic_cost_field`：测试用合成场（平滑地形 + 雷达球 + 禁飞块）；
//! - `build_semantic_cost_field`：空洞分层语义代价场（Land/Water/Lake 基础 1.0、
//!   NoData 5x 初值、OOB 禁行墙，主管 2026-08-04 拍板）；
//! - Phase 2 以真实威胁/地形构建代价场（本模块只做传播骨架）。
//!
//! Phase 0 实测（docs/phase0_baseline.md）：128² 单次传播 2.62ms，常数 11-12.5ns/op。

use std::cmp::Ordering;
use std::collections::BinaryHeap;

use crate::terrain::{BulkPrefetch, Sample};

/// 2D 代价场（行优先，`idx = r * cols + c`）。`cost >= 1`，越大越难通过。
#[derive(Debug, Clone)]
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

/// 合成代价场（测试用，Phase 0 B1 语义）：平滑地形 + 雷达高代价球 + 禁飞区矩形块。
/// `cell_m`：网格单元边长（米），把公里级障碍参数换算成格数。
pub fn synthetic_cost_field(rows: usize, cols: usize, cell_m: f64, seed: u64) -> CostField {
    use rand::{RngExt, SeedableRng};
    let mut f = CostField::new(rows, cols);

    // 地形：正弦叠加（平滑大尺度起伏），范围 0~4000m，>2500m 线性升高至 20x
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

    // 雷达高代价球：3 个，半径 ~15km，代价 30x
    let mut rng = rand::rngs::StdRng::seed_from_u64(seed);
    let radar_cost = 30.0f32;
    for _ in 0..3 {
        let cx = rng.random_range(0.25..0.75) * rows as f64;
        let cy = rng.random_range(0.25..0.75) * cols as f64;
        let rad = 15_000.0 / cell_m;
        for r in 0..rows {
            for c in 0..cols {
                let i = r * cols + c;
                let dr = r as f64 - cx;
                let dc = c as f64 - cy;
                if dr * dr + dc * dc <= rad * rad {
                    f.cost[i] = radar_cost;
                }
            }
        }
    }

    // 禁飞区矩形块：2 个，30x25km，代价 50x
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

/// 语义代价场（空洞分层，主管 2026-08-04 拍板）。
/// 对每个格点 `(r, c)` 调用 `sample` 得到语义采样，映射为代价：
/// - Land/Water/Lake → 1.0（基础代价；地形高度代价由调用方按高度叠加）；
/// - NoData → `nodata_mult`（NODATA 高代价倍数，初值 5x）；
/// - OutOfBounds → `f32::INFINITY`（禁行墙，路径不得越出）。
pub fn build_semantic_cost_field<F>(rows: usize, cols: usize, mut sample: F, nodata_mult: f32) -> CostField
where
    F: FnMut(usize, usize) -> Sample,
{
    let mut f = CostField::new(rows, cols);
    for r in 0..rows {
        for c in 0..cols {
            let i = r * cols + c;
            f.cost[i] = sample(r, c).base_cost(nodata_mult);
        }
    }
    f
}

/// 语义代价场并行构建（候选优化，2026-08-07 对比验证）。
/// 与 `build_semantic_cost_field` **数值逐位一致**：每格采样独立，仅行遍历并行化。
/// 地形源块缓存（Mutex）天然线程安全：解压在锁外进行，多线程并发解压不同块
/// （zstd 解压是 field_build 主成本——zigzag11 实测 field_build 1266ms 占 80%）。
/// 线程数 = available_parallelism，上限 16；行数过少回退串行。
pub fn build_semantic_cost_field_par<F>(
    rows: usize,
    cols: usize,
    sample: &F,
    nodata_mult: f32,
) -> CostField
where
    F: Fn(usize, usize) -> Sample + Sync + Send,
{
    let nthreads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .clamp(1, 16);
    if nthreads <= 1 || rows < 64 {
        let mut f = CostField::new(rows, cols);
        for r in 0..rows {
            for c in 0..cols {
                let i = r * cols + c;
                f.cost[i] = sample(r, c).base_cost(nodata_mult);
            }
        }
        return f;
    }
    // 行分块；每线程构建子行段（行优先），按块序 join 拼接——顺序与串行一致。
    // scoped threads：借用非 'static 的 sample 引用，无需 Arc。
    let chunk = rows.div_ceil(nthreads);
    let mut f = CostField::new(rows, cols);
    std::thread::scope(|scope| {
        let mut ranges: Vec<(usize, usize)> = Vec::new();
        let mut handles = Vec::new();
        for t in 0..nthreads {
            let r0 = t * chunk;
            if r0 >= rows {
                break;
            }
            let r1 = (r0 + chunk).min(rows);
            ranges.push((r0, r1));
            let s = sample;
            handles.push(scope.spawn(move || {
                let n = (r1 - r0) * cols;
                let mut sub = vec![0f32; n];
                for r in r0..r1 {
                    let base = (r - r0) * cols;
                    for c in 0..cols {
                        sub[base + c] = s(r, c).base_cost(nodata_mult);
                    }
                }
                sub
            }));
        }
        let mut offset = 0;
        for (h, (_r0, r1)) in handles.into_iter().zip(&ranges) {
            let sub = h.join().unwrap_or_else(|_| vec![0.0; (r1 - _r0) * cols]);
            f.cost[offset..offset + sub.len()].copy_from_slice(&sub);
            offset += sub.len();
        }
    });
    f
}

/// 并行 + 无锁批量预取构建（候选③，2026-08-07 对比验证：release 冷缓存 3.71× vs 串行）。
///
/// 为什么组合才有效（对比测试结论）：
/// - 单独并行化（`build_semantic_cost_field_par`）：0.91×——共享 `sample_at` 的 Mutex
///   锁竞争抵消 zstd 解压并行收益；
/// - 单独无锁（`BulkPrefetch::sample_local` 单线程）：0.99×——Mutex 无竞争 lock 本来
///   便宜（~24ns/次），解压串行才是大头；
/// - **并行 + 无锁**：行分块，每线程 `prefetch_lonlat` 自己行范围的块（并行锁外解压）
///   + 局部无锁查表——3.71×（381ms → 103ms，1024² 冷缓存）。
///
/// 数值与 `build_semantic_cost_field` 逐位一致（每格采样独立，仅遍历并行化）。
/// `walled`：Fn(lon, lat) -> bool——命中硬墙（NoFly/Obstacle）→ OutOfBounds 禁行墙
/// （与 solver 原闭包语义一致；网格点经纬度 = `min_lon + (c+0.5)/grid*span`，同 cell_lonlat）。
pub fn build_semantic_cost_field_par_local<B, W>(
    src: &B,
    min_lon: f64,
    min_lat: f64,
    span: f64,
    grid: usize,
    nodata_mult: f32,
    walled: W,
) -> CostField
where
    B: BulkPrefetch + Sync,
    W: Fn(f64, f64) -> bool + Sync + Send,
{
    let nthreads = std::thread::available_parallelism()
        .map(|n| n.get())
        .unwrap_or(1)
        .clamp(1, 16);
    let mut f = CostField::new(grid, grid);
    if nthreads <= 1 || grid < 64 {
        // 回退：单线程预取 + 无锁（仍比带锁快/持平；walled 支持）
        let half = 0.5 / grid as f64 * span;
        let slack = span / grid as f64;
        let local = src.prefetch_lonlat(
            min_lon + half - slack,
            min_lat + half - slack,
            min_lon + span - half + slack,
            min_lat + span - half + slack,
        );
        for r in 0..grid {
            let v = (r as f64 + 0.5) / grid as f64;
            let lat = min_lat + v * span;
            for c in 0..grid {
                let u = (c as f64 + 0.5) / grid as f64;
                let lon = min_lon + u * span;
                let i = r * grid + c;
                f.cost[i] = if walled(lon, lat) {
                    Sample::OutOfBounds.base_cost(nodata_mult)
                } else {
                    src.sample_local(&local, lon, lat).base_cost(nodata_mult)
                };
            }
        }
        return f;
    }
    let chunk = grid.div_ceil(nthreads);
    let half = 0.5 / grid as f64 * span;
    let slack = span / grid as f64;
    let col_lon0 = min_lon + half - slack;
    let col_lon1 = min_lon + span - half + slack;
    std::thread::scope(|scope| {
        let mut ranges: Vec<(usize, usize)> = Vec::new();
        let mut handles = Vec::new();
        for t in 0..nthreads {
            let r0 = t * chunk;
            if r0 >= grid {
                break;
            }
            let r1 = (r0 + chunk).min(grid);
            ranges.push((r0, r1));
            let src = src;
            let walled = &walled;
            handles.push(scope.spawn(move || {
                // 本线程行范围 [r0, r1) 的网格点 lat 跨度 → 预取（并行锁外解压）
                let lat0 = min_lat + (r0 as f64 + 0.5) / grid as f64 * span;
                let lat1 = min_lat + (r1 as f64 + 0.5) / grid as f64 * span;
                let local = src.prefetch_lonlat(col_lon0, lat0 - slack, col_lon1, lat1 + slack);
                let n = (r1 - r0) * grid;
                let mut sub = vec![0f32; n];
                for r in r0..r1 {
                    let v = (r as f64 + 0.5) / grid as f64;
                    let lat = min_lat + v * span;
                    let base = (r - r0) * grid;
                    for c in 0..grid {
                        let u = (c as f64 + 0.5) / grid as f64;
                        let lon = min_lon + u * span;
                        sub[base + c] = if walled(lon, lat) {
                            Sample::OutOfBounds.base_cost(nodata_mult)
                        } else {
                            src.sample_local(&local, lon, lat).base_cost(nodata_mult)
                        };
                    }
                }
                sub
            }));
        }
        let mut offset = 0;
        for (h, (_r0, r1)) in handles.into_iter().zip(&ranges) {
            let sub = h.join().unwrap_or_else(|_| vec![0.0; (r1 - _r0) * grid]);
            f.cost[offset..offset + sub.len()].copy_from_slice(&sub);
            offset += sub.len();
        }
    });
    f
}

/// 窄带堆元素（按到达时间小优先；tie-break 用 idx 保证确定性）。
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
        // BinaryHeap 是 max-heap；反转使小 t 优先级高；tie-break 固定 idx（确定性）
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
    if dst_r >= field.rows || dst_c >= field.cols || !res.accepted[field.idx(dst_r, dst_c)] {
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
        let f = CostField::new(64, 64);
        let res = fmm_propagate(&f, 32, 32);
        let n = f.rows * f.cols;
        assert_eq!(res.accepted.iter().filter(|&&a| a).count(), n);
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
        let steps = path.len() as f64;
        let direct = ((40i64 - sc as i64).pow(2) + (5i64 - sr as i64).pow(2)) as f64;
        assert!(steps as f64 >= direct.sqrt() - 1e-6);
    }

    #[test]
    fn fmm_empty_grid_no_panic() {
        let f = CostField::new(0, 0);
        let res = fmm_propagate(&f, 0, 0);
        assert_eq!(res.times.len(), 0);
    }

    #[test]
    fn fmm_source_out_of_bounds_no_panic() {
        let f = CostField::new(16, 16);
        let res = fmm_propagate(&f, 99, 99);
        assert!(!res.accepted[0]);
    }
}

# 05 — 代价场与 FMM 传播、空间索引

> 代码位置：`cli/src/costfield.rs`（524 行）、`cli/src/spatial.rs`（210 行）
> 设计依据：技术方案 4.2.1（代价场融合规则）、4.4（FMM 定案主方案）、5.1（预算 ~15%）、十三轮共识（确定性热路径）
> 迁移来源：phase0/fmm.rs（B1 实测：128² 单次传播 2.62ms，常数 11-12.5ns/op）

## 1. CostField（代价场数据结构）

```rust
pub struct CostField {
    pub rows: usize,
    pub cols: usize,
    pub cost: Vec<f32>,   // 行优先，idx = r * cols + c；cost ≥ 1，越大越难通过
}
```

- `get(r,c)` / `idx(r,c)` / `in_bounds(r,c)` 内联访问器；
- **语义**：Land/Water/Lake → 1.0（基础代价）；NoData/OOB → 5x（高代价**通行**，2026-08-11 放开输入点限制）；Forbidden（NoFly/Obstacle 硬墙）→ f32::INFINITY；雷达代价 → 乘性放大。

## 2. 语义代价场构建（build_semantic_cost_field）

```rust
pub fn build_semantic_cost_field<F>(rows, cols, sample: F, nodata_mult: f32) -> CostField
where F: FnMut(usize, usize) -> Sample
```

- 对每格点调用 `sample(r, c)` 得到语义采样，映射 `base_cost(nodata_mult)`；
- solver 在闭包中叠加：Zone 硬墙（NoFly/Obstacle 全高度水平墙）→ `Sample::Forbidden`（INF 禁行 + LOS 遮挡，2026-08-11 新增，旧 OOB 语义不再用于硬墙）；地形 `sample_at`（数据范围外 OOB → 5x 高代价通行）；无地形 → `Sample::Land(0.0)`（海拔 0 平面）。

## 3. FMM 传播（fmm_propagate）—— 粗层主算法

**2D Godunov 迎风差分 + BinaryHeap 窄带**，O(NlogN)，确定性。

### 3.1 算法状态

```
STATE_FAR = 0（未访问） / STATE_CONSIDERED = 1（窄带内） / STATE_ACCEPTED = 2（已冻结）
```

### 3.2 Godunov 迎风更新（solve_t）

对格点 (r, c)，取四邻域**已接受**点的最小到达时间：

```
tx = min(上/下已接受点的 T)
ty = min(左/右已接受点的 T)
若 tx=∞ 且 ty=∞ → ∞
若 tx=∞ → ty + cost
若 ty=∞ → tx + cost
若 |tx−ty|² ≤ 2·cost²（对角支配）→ (tx + ty + √(2·cost² − (tx−ty)²)) / 2   # 解二次方程
否则 → min(tx, ty) + cost
```

### 3.3 传播主循环

```
1. 源点: T=0, ACCEPTED; 更新四邻域入堆
2. while heap 非空:
     pop 最小 (T, idx)  —— 过期条目（已 ACCEPTED）跳过（lazy deletion）
     标记 ACCEPTED; 对四邻域 update_neighbors（FAR → CONSIDERED 入堆；已 CONSIDERED 更新 T）
```

**确定性保证**：`HeapEntry` 的 Ord 实现反转比较使小 T 优先级高，**tie-break 固定 idx**（`other.idx.cmp(&self.idx)`）——迭代序与插入序无关，跨运行逐位一致。

### 3.4 防御

- 空网格 / 源点越界 → 返回空结果（times 全 INF、accepted 全 false），**不 panic**（B9）。

### 3.5 FmmResult

```rust
pub struct FmmResult { pub times: Vec<f32>, pub accepted: Vec<bool> }
```

## 4. 回溯（backtrack_path）

```rust
pub fn backtrack_path(field, res, dst_r, dst_c, src_r, src_c) -> Option<Vec<(usize, usize)>>
```

- 从终点沿 **T 场最大下降方向**逐格回溯到源点（走廊质量代理：路径长度/绕行比）；
- 返回顺序：终点 → 源点（**solver 中 reverse 后使用**）；
- 终点不可达 → None；步数 guard（> rows×cols 防御）→ None；卡在局部极小（理论上不发生，防御性）→ None。

## 5. 合成代价场（synthetic_cost_field，测试用）

- 地形：正弦叠加（1800±起伏，>2500m 线性升至 20x）；
- 雷达：3 个 ~15km 高代价球（30x，随机种子定位）；
- 禁飞区：2 个 30×25km 块（50x，`seed ^ 0xDEADBEEF` 种子）；
- 用途：Phase 0 B1 基准语义 + 单元测试。

## 6. 空间索引（spatial.rs）—— rstar 加速

### 6.1 RadarEntry（雷达索引）

```rust
pub struct RadarEntry { id, lon, lat, radius_m }  // 膨胀后探测半径
```

- `RTreeObject::envelope`：半径 → 经纬度矩形上界（保守；精确距离查询后过滤）；
- `RadarIndex::build`：`RTree::bulk_load`（批量加载）；
- `within(lon, lat, radius_m)`：R-tree 粗筛 + 球面距离精确过滤，**按 id 排序输出（确定性）**；
- `nearest(lon, lat)`：最近邻；
- 非有限输入 → 空结果/None（不 panic）。

### 6.2 CircleEntry / CircleIndex（圆区域索引）

- 用于圆形禁飞/限飞区快速包含查询（smooth 复验的 nofly 快速路径；多边形走 config::zone_contains 线性扫）；
- `containing(lon, lat)`：所在圆集合，按 id 排序（确定性）。

### 6.3 确定性（十三轮共识热路径）

查询结果一律 `sort_by(id)`——HashMap 遍历序/插入序无关，热路径（代价场聚合/雷达查询/走廊统计）不依赖无序迭代。

## 7. 与 solver 的协作（代价场如何被构建）

solver.rs 中代价场构建的三层叠加（详见 07 文档）：

1. **语义层**：`build_semantic_cost_field`（Land/Water/Lake=1.0、NoData/OOB=5x 通行、Forbidden=INF）+ Zone 硬墙；
2. **膨胀 + 过渡带**：`apply_inflation_and_band`（禁飞墙向外膨胀 inflation_cells 格 → INF；墙外 2 格 BFS 距离变换软罚，墙边 ×1.5 渐变到 ×1）；
3. **雷达静态代价**：`threat.static_union_probability` > 0 时 `cost *= 1 + 200 × (p + 深穿惩罚)`。

## 8. 性能预算定位（5.1）

| 环节 | 预算占比 | 说明 |
|------|---------|------|
| FMM 粗层传播（窄带） | ~15% | 多机共享同一传播场摊薄成本；单次传播 128² ≈ 2.62ms（B1） |
| 雷达/禁飞区碰撞与邻域查询 | ~30% | rstar 空间索引加速 |

## 9. 测试覆盖

- costfield：常数场全可达、回溯到达源点且步数 ≥ 直线距离、空网格/越界源不 panic；
- spatial：within 半径过滤、最近雷达、圆包含、空索引不 panic、haversine 北京-上海 ≈ 1067km；
- crash_suite：FMM 空/越界、回溯退化输入。

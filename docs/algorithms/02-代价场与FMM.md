# 02 — 代价场与 FMM 传播、空间索引

> 本节描述代价场构建与 FMM 传播算法。

## 1. CostField（代价场数据结构）

```rust
pub struct CostField {
    pub rows: usize,
    pub cols: usize,
    pub cost: Vec<f32>,   // 行优先，cost ≥ 1，越大越难通过
}
```

**语义**：Land/Water/Lake → 1.0（基础代价）；NoData/OOB → 5x（高代价通行）；Forbidden（NoFly/Obstacle 硬墙）→ f32::INFINITY；雷达代价 → 乘性放大。

## 2. 语义代价场构建流程

```mermaid
flowchart TD
    START([开始]) --> GRID[遍历网格<br>每格点 r,c]
    GRID --> SAMPLE[采样<br>sample_at]
    SAMPLE --> ZONE{在 Zone 内?}
    ZONE -->|是| WALL[Forbidden<br>INF 禁行]
    ZONE -->|否| TERRAIN{有地形?}
    TERRAIN -->|是| DEM[地形采样<br>Land/Water/NoData]
    TERRAIN -->|否| FLAT[Land 0.0<br>海拔 0 平面]

    WALL --> RADAR{雷达覆盖?}
    DEM --> RADAR
    FLAT --> RADAR

    RADAR -->|是| AMP[代价放大<br>× 1+200×p]
    RADAR -->|否| BASE[基础代价]

    AMP --> NEXT[下一格点]
    BASE --> NEXT
    NEXT --> GRID

    style START fill:#e1f5fe
    style WALL fill:#ffcdd2
    style AMP fill:#fff3e0
```

## 3. FMM 传播—— 粗层主算法

**2D Godunov 迎风差分 + BinaryHeap 窄带**，O(NlogN)，确定性。

### 3.1 FMM 传播流程

```mermaid
flowchart TD
    START([开始]) --> INIT[初始化<br>源点 T=0, ACCEPTED<br>四邻域入堆]

    INIT --> HEAP{堆非空?}
    HEAP -->|否| END([结束])
    HEAP -->|是| POP[弹出最小 T<br>过期条目跳过]

    POP --> ACCEPT[标记 ACCEPTED]
    ACCEPT --> NEIGH[遍历四邻域]

    NEIGH --> STATE{邻域状态?}
    STATE -->|FAR| ADD[入堆<br>CONSIDERED]
    STATE -->|CONSIDERED| UPDATE[更新 T]
    STATE -->|ACCEPTED| SKIP[跳过]

    ADD --> NEXT[下一邻域]
    UPDATE --> NEXT
    SKIP --> NEXT

    NEXT --> NEIGH
    NEIGH --> HEAP

    style START fill:#e1f5fe
    style END fill:#e8f5e8
    style POP fill:#fff3e0
    style ACCEPT fill:#c8e6c9
```

### 3.2 Godunov 迎风更新

对格点 (r, c)，取四邻域**已接受**点的最小到达时间：

```
tx = min(上/下已接受点的 T)
ty = min(左/右已接受点的 T)
若 tx=∞ 且 ty=∞ → ∞
若 tx=∞ → ty + cost
若 ty=∞ → tx + cost
若 |tx−ty|² ≤ 2·cost²（对角支配）→ (tx + ty + √(2·cost² − (tx−ty)²)) / 2
否则 → min(tx, ty) + cost
```

### 3.3 确定性保证

`HeapEntry` 的 Ord 实现反转比较使小 T 优先级高，**tie-break 固定 idx**——迭代序与插入序无关，跨运行逐位一致。

### 3.4 防御

空网格 / 源点越界 → 返回空结果（times 全 INF、accepted 全 false），**不 panic**（B9）。

## 4. 回溯流程

```mermaid
flowchart TD
    START([从终点开始]) --> CHECK{终点可达?}
    CHECK -->|否| NONE([返回 None])
    CHECK -->|是| NEIGH[遍历四邻域]

    NEIGH --> FIND[找 T 最小<br>已接受点]
    FIND --> MOVE[移动到该点]
    MOVE --> REACH{到达源点?}

    REACH -->|是| DONE([返回路径])
    REACH -->|否| GUARD{步数超限?}
    GUARD -->|是| NONE
    GUARD -->|否| NEIGH

    style START fill:#e1f5fe
    style DONE fill:#e8f5e8
    style NONE fill:#ffcdd2
```

从终点沿 **T 场最大下降方向**逐格回溯到源点。返回顺序：终点 → 源点（solver 中 reverse 后使用）。

## 5. 空间索引—— rstar 加速

### 5.1 RadarEntry（雷达索引）

- `RTreeObject::envelope`：半径 → 经纬度矩形上界
- `within(lon, lat, radius_m)`：R-tree 粗筛 + 球面距离精确过滤，**按 id 排序输出（确定性）**
- 非有限输入 → 空结果/None（不 panic）

### 5.2 CircleIndex（圆区域索引）

用于圆形禁飞/限飞区快速包含查询。`containing(lon, lat)`：所在圆集合，按 id 排序（确定性）。

### 5.3 确定性

查询结果一律 `sort_by(id)`——热路径不依赖无序迭代。

## 6. 代价场三层叠加

```mermaid
graph TB
    subgraph 第1层: 语义层
        A[Land/Water/Lake<br>1.0] --> D[语义代价场]
        B[NoData/OOB<br>5x] --> D
        C[Forbidden<br>INF] --> D
    end

    subgraph 第2层: 膨胀+过渡带
        D --> E[禁飞墙膨胀<br>INF 向外扩散]
        E --> F[过渡带软罚<br>BFS 距离变换]
    end

    subgraph 第3层: 雷达代价
        F --> G[雷达概率 p]
        G --> H[深穿惩罚]
        H --> I[最终代价<br>× 1+200×p]
    end

    style D fill:#bbdefb
    style F fill:#c8e6c9
    style I fill:#fff3e0
```

## 7. 测试覆盖

- costfield：常数场全可达、回溯到达源点、空网格/越界源不 panic
- spatial：within 半径过滤、最近雷达、圆包含、空索引不 panic
- crash_suite：FMM 空/越界、回溯退化输入
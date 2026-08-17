# Phase 0 — 性能原型与基准 crate

> **定位**：AircraftRouterPlanner 的 Phase 0 性能原型与基准工程，用于在正式实现
> （`cli/`）前验证关键算法与性能预算。现为**历史基准工程**，保留在 workspace 中以便
> 复跑 b1–b5 benchmark 与崩溃套件。

## 为什么存在

Phase 0（技术方案 §5.2）承担"实测预算回填 + 回归基线 + 标定值冻结"的前置验证：
FMM 粗层传播、代价场复用、rstar 空间索引、射线-地形求交、Dubins 细层基元五类
性能预算的实测。算法实现已全部并入/重写进正式工程 `cli/`（对应关系见下表），
本 crate 仅保留 benchmark 与最小原型代码。

## 与 cli/ 的对应关系

| phase0 原型 | 正式实现（cli/） |
|-------------|------------------|
| `src/fmm.rs` | `src/costfield.rs`（代价场 + FMM 传播 + 回溯） |
| `src/dubins.rs` | `src/dubins.rs`（CSC + CCC 完整解） |
| `src/terrain.rs` | `src/terrain/`（TerrainSource trait + 多数据源） |
| `benches/b1_fmm.rs` 等 | `cli/benches/b_load_decompress.rs`（加载基准）+ 内嵌单测 |

## 目录内容

- `src/` —— 早期原型代码（dubins / fmm / terrain / lib），仅被 benchmark 引用
- `benches/` —— b1–b5 五类性能预算实测（criterion）
- `tests/crash_suite.rs` —— B9 崩溃/退化输入套件（Phase 0 版，13 用例）
- `scripts/` —— 数据预处理 Python 脚本（DEM 分析 / ARPK1 压缩 / GSHHG 掩膜生成），
  一次性数据准备工具，非运行时依赖

## 常用命令

```bash
cargo bench -p phase0        # 跑 b1–b5 基准
cargo test -p phase0         # 跑 Phase 0 崩溃套件 + 单元测试
```

> 注：数据（`phase0/data/` 与根 `data/`）已 gitignore，需按 `docs/04-地形数据源.md`
> 自行准备后基准/测试才能覆盖真实地形分支。

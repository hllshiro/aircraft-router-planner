# AircraftRouterPlanner

飞机航路规划器——基于 FMM 快速行进法 + 语义代价场的低空/突防航路求解，支持地形、
禁飞区、限飞区、雷达威胁与多机/武器语义，端到端每百公里 ≤ 3s，确定性可复现。

## 仓库结构

```
├── cli/            # ★ 核心 CLI（lib + bin）——正式工程
│   ├── src/        # 契约 / 坐标 / 地形数据源 / 代价场 / FMM / 平滑 / 求解器
│   ├── tests/      # crash_suite / determinism / field_build_compare / 回归集
│   ├── benches/    # 内置格式加载基准
│   └── examples/   # 开发期调试工具
├── phase0/         # Phase 0 性能原型与基准 crate（历史，b1–b5 可复跑，见 phase0/README.md）
├── demo/           # 开发期可视化工具（server: Axum 后端 / web: React+Three.js 前端）
├── docs/           # 技术文档集（技术方案.md 权威设计 + 01–11 实现现状）
├── scripts/        # check.sh / release.sh / perf_regress.sh 等门禁与发布脚本
└── data/ install/  # gitignore：地形/掩膜数据与发布包，需另行准备
```

## 快速开始

```bash
# 构建核心 CLI（静态编译红线：零第三方 C/DLL 依赖）
cargo build --release -p aircraft-router-planner-cli

# 运行（从 stdin 读任务 JSON，输出路径 JSON）
cat mission.json | target/release/aircraft-router-planner-cli

# 测试与全量门禁
cargo test --lib
scripts/check.sh            # 构建 + 全套回归 + 静态红线 + 性能预算
```

## 文档

- 技术方案（权威设计蓝图）：[docs/技术方案.md](docs/技术方案.md)
- 实现现状文档集入口：[docs/README.md](docs/README.md)
- 输入/输出 JSON 契约：[docs/02-输入输出契约.md](docs/02-输入输出契约.md)
- 构建/依赖/发布：[docs/11-工程化与构建.md](docs/11-工程化与构建.md)
- Demo 可视化：[docs/09-演示应用.md](docs/09-演示应用.md) / [demo/README.md](demo/README.md)

## 数据说明

默认地形（`east_asia_7p5as.arpack`，GMTED2010 东亚 7.5 弧秒）与掩膜
（`mask_7p5as.mask`，GSHHG 全球 V2 3 态）体积较大，**不入库**，需按
[docs/04-地形数据源.md](docs/04-地形数据源.md) 自行放置到 `data/`（solver 会自动探测）。

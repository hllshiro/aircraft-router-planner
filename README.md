# AircraftRouterPlanner

飞机航路规划器——基于 FMM 快速行进法 + 语义代价场的低空/突防航路求解，支持地形、
禁飞区、限飞区、雷达威胁与多机/武器语义，端到端每百公里 ≤ 3s，确定性可复现。

## 仓库结构

```
├── src/
│   ├── cli/        # ★ 核心 CLI（lib + bin）——正式工程，核心功能只有路径规划
│   │   ├── src/    # 契约 / 坐标 / 地形数据源 / 代价场 / FMM / 平滑 / 求解器
│   │   └── Cargo.toml
│   ├── convert/    # 内部地形转换工具（arp-convert：外部格式 → ARPK1；不随核心 CLI 发布，随用随编）
│   └── demo/       # 开发期可视化工具（server: Axum 后端 / web: React+Three.js 前端）
├── docs/           # 技术文档集
├── scripts/        # release.js / start-demo.js 等发布脚本
└── data/ install/  # gitignore：地形/掩膜数据与发布包，需另行准备
```

## 快速开始

```bash
# 构建核心 CLI（静态编译红线：零第三方 C/DLL 依赖）
pnpm cli:build

# 运行（plan 子命令：从文件读任务 JSON，输出路径 JSON）
target/release/arpcli plan --file mission.json --out result.json

# 查看帮助（help 风格：arpcli / arpcli help，不使用 --help）
target/release/arpcli          # 顶层 help

# 测试
pnpm test                      # 单元测试
pnpm test:all                  # 全量测试
```

## 文档

- 实现现状文档集：[docs/README.md](docs/README.md)
- 输入/输出 JSON 契约：[docs/02-输入输出契约.md](docs/02-输入输出契约.md)
- 构建/依赖/发布：[docs/11-工程化与构建.md](docs/11-工程化与构建.md)
- Demo 可视化：[docs/09-演示应用.md](docs/09-演示应用.md) / [src/demo/README.md](src/demo/README.md)

## 版本与发布

- 版本号唯一事实来源：`Cargo.toml` 的 `[workspace.package] version`；发布 tag 必须为 `v<version>`。
- 变更记录：[CHANGELOG.md](CHANGELOG.md)（Keep a Changelog 格式）。
- 发版流程：[docs/发版流程.md](docs/发版流程.md)。
- 发布：触发 `release-prepare` workflow（GitHub Actions → Run workflow → 输入版本号），
  自动更新版本号 + CHANGELOG → commit → tag → push，
  tag 触发 [release.yml](.github/workflows/release.yml) 交叉编译 `windows/linux × amd64/arm64` 四个 CLI 产物并创建 GitHub Release。

## 数据说明

默认地形（`global_7p5as.arpack`，GMTED2010 全球 7.5 弧秒）与掩膜
（`global_7p5as.mask`，GSHHG 全球 V2 3 态）体积较大，**不入库**，需按
[docs/04-地形数据源.md](docs/04-地形数据源.md) 自行放置到 `data/`（solver 会自动探测）。

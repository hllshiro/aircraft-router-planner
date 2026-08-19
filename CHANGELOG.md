# Changelog

本文件记录项目的所有重要变更。格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本遵循 [SemVer](https://semver.org/lang/zh-CN/)。

> **版本号唯一事实来源**：`Cargo.toml` 的 `[workspace.package] version`。
> 发布时 tag 必须为 `v<version>`，由 `.github/workflows/release.yml` 校验并交叉编译四个平台产物。
> 升级流程见 `scripts/bump_version.sh`。

## [Unreleased]

### Changed
- 优化 help 展示：首行显示可执行文件名与版本，Usage/示例/错误信息动态使用当前执行文件名。
- 移除 `--version` 标志与 `plan` 未启用的保留参数 `--seed`/`--config`（**破坏性变更**：传入即报错退出）。

## [0.2.0] - 2026-08-18

### Added
- `arp-cli schema` 子命令：用 schemars 动态生成输入/输出 JSON Schema（代码即事实，零漂移）。

### Changed
- help 风格改为 `arp-cli` / `arp-cli help` / `arp-cli help <command>`，移除 `--help` 标志。
- 规划动作显式化为 `arp-cli plan` 子命令（裸 `arp-cli` 现显示顶层 help；**破坏性变更**，原 `arp-cli < mission.json` 管道改为 `arp-cli plan < mission.json`）。
- 地形转换/重压缩从核心 CLI 剥离为独立内部工具 `arp-convert`（`convert/` crate，**不随核心 CLI 发布**，随用随编）。

## [0.1.0] - 2026-08-17

首个可交付版本。

### Added
- FMM 快速行进法 + 语义代价场的低空/突防航路求解，端到端每百公里 ≤ 3s（确定性可复现）。
- JSON 输入输出契约（`schema_version 0.20`，`status` 四态：`success` / `degraded_timeout` / `no_solution` / `input_invalid`）。
- 坐标系统：WGS84/CGCS2000/GRS80 椭球、TM/UTM/GK3/WebMercator、近场 ENU。
- 地形数据源：ARPK1 内置格式 + SRTM/GeoTIFF/DTED 外置直读 + GSHHG 海陆掩膜（LOS 语义）。
- 威胁模型：球形雷达、探测概率衰减（Swerling I / 线性 / 指数）、LOS 遮挡、多雷达概率并集。
- 路径平滑：Theta\* / 样条 / Dubins（CSC + CCC）/ 贪心抽稀 + 全链复验。
- 多机共享代价场、禁飞/限飞区剖面决策、必经点、武器语义、多机路径交叉检测。
- 开发期可视化工具 `demo/`（Axum 后端 + React/Three.js 前端，不随发布版分发）。
- 工程化：CI 分层门禁（静态检查 + 手动全量测试）、release 流水线（`v*` tag 交叉编译 4 平台）、`CHANGELOG.md`、`scripts/bump_version.sh`。

### Changed
- `.cargo/config.toml`：确定性 `-fma` flag 由全局收敛到 x86_64 目标；新增 arm64 目标配置。

### Fixed
- CI 工具链由 1.85 升至 1.89（依赖 MSRV 提升）。
- 静态依赖红线 grep `proj` 误匹配 `pin-project-lite` 的问题。

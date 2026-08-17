# Changelog

本文件记录项目的所有重要变更。格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本遵循 [SemVer](https://semver.org/lang/zh-CN/)。

> **版本号唯一事实来源**：`Cargo.toml` 的 `[workspace.package] version`。
> 发布时 tag 必须为 `v<version>`，由 `.github/workflows/release.yml` 校验并交叉编译四个平台产物。
> 升级流程见 `scripts/bump_version.sh`。

## [Unreleased]

### Added
- 发布流水线：`.github/workflows/release.yml`——`v*` tag 触发，交叉编译
  `windows/linux × amd64/arm64` 四个 CLI 产物并创建 GitHub Release（含 SHA256SUMS）。
- `scripts/bump_version.sh`：版本号升级辅助脚本。
- `CHANGELOG.md`：本变更记录。
- `.cargo/config.toml`：新增 `aarch64-pc-windows-msvc` / `aarch64-unknown-linux-musl`
  目标配置（arm64 静态编译支持）。

### Changed
- `.cargo/config.toml`：确定性 `-fma` flag 由全局收敛到 x86_64 目标（arm64 的 FMA
  为 ARMv8 基线特性，不适用）。

## [0.1.0] - 2026-08-14

首个可交付版本。核心能力：

### Added
- FMM 快速行进法 + 语义代价场的低空/突防航路求解，端到端每百公里 ≤ 3s（确定性可复现）。
- JSON 输入输出契约（`schema_version 0.20`，`status` 四态：`success` / `degraded_timeout` / `no_solution` / `input_invalid`）。
- 坐标系统：WGS84/CGCS2000/GRS80 椭球、TM/UTM/GK3/WebMercator、近场 ENU。
- 地形数据源：ARPK1 内置格式 + SRTM/GeoTIFF/DTED 外置直读 + GSHHG 海陆掩膜（LOS 语义）。
- 威胁模型：球形雷达、探测概率衰减（Swerling I / 线性 / 指数）、LOS 遮挡、多雷达概率并集。
- 路径平滑：Theta\* / 样条 / Dubins（CSC + CCC）/ 贪心抽稀 + 全链复验。
- 多机共享代价场、禁飞/限飞区剖面决策、必经点、武器语义、多机路径交叉检测。
- 确定性构建红线（`-fma` 禁用）与崩溃/回归/确定性测试门禁（`.github/workflows/ci.yml`）。
- 开发期可视化工具 `demo/`（Axum 后端 + React/Three.js 前端，不随发布版分发）。

# Changelog

本文件记录项目的所有重要变更。格式遵循 [Keep a Changelog](https://keepachangelog.com/zh-CN/1.1.0/)，
版本遵循 [SemVer](https://semver.org/lang/zh-CN/)。

> **版本号唯一事实来源**：`Cargo.toml` 的 `[workspace.package] version`。
> 发布时 tag 必须为 `v<version>`，由 `.github/workflows/release.yml` 校验并交叉编译四个平台产物。
> 升级流程见 `scripts/bump-version.js`。

## [Unreleased]

## [0.7.0] - 2026-09-17

### Changed
- demo 移除 GeoTIFF（tiff）底图选项，arpcli 不支持该格式
- demo 掩膜选择改为 index id，与地形统一使用 index.yaml 索引机制
- 移除 index.yaml 依赖：地形索引改为扫描 data 目录，id 为文件名（不含扩展名）
- 移除 serde_yaml_neo 依赖（纯 Rust YAML 解析库）
- 数据文件重命名：`gmted2010_7p5as_global.z19.arpack` → `global_7p5as.arpack`，`mask_7p5as.mask` → `global_7p5as.mask`

### Fixed
- 修复 demo-server 瓦片接口不支持 index id 解析导致掩膜/地形文件找不到的问题

## [0.6.0] - 2026-09-17

### Changed
- 源码目录重构：`cli/`、`convert/`、`demo/` 迁移至 `src/` 下，统一为 `src/cli/`、`src/convert/`、`src/demo/`
- 新增 pnpm monorepo 工作区配置，根目录统一命令入口（`pnpm cli:dev`、`pnpm demo:build` 等）
- 构建脚本从 bash 迁移至 Node.js（`scripts/release.js`、`scripts/bump-version.js`），支持 Windows 开发环境
- 删除 `start_demo.bat`、`scripts/preview.sh`、`src/demo/start.sh`，功能由 pnpm 命令替代
- 新增 `scripts/start-demo.js` 和 `scripts/check.js`，注册到 package.json（`pnpm demo:start`、`pnpm check`）
- 更新所有 README 文档，启动命令统一使用 pnpm scripts
- 修复 release.yml 中版本升级命令提示

## [0.5.0] - 2026-09-16

### Changed
- **Breaking**: 移除 `ZoneType` 枚举和 `zone_type` 字段，zone 行为由 `alt_min_m`/`alt_max_m` 决定：两者都不存在 → 全高度墙（禁飞/障碍）；任一存在 → 高度层禁入（限飞区）。
- 移除所有测试用例（src/cli/src 内联测试、src/cli/tests/、phase0/），测试覆盖范围分析文档：`docs/测试覆盖范围分析.md`
- 移除 phase0 历史原型目录
- 移除 scripts/check.sh、perf_regress.sh、gen_overview_test.py

## [0.4.0] - 2026-09-15

### Changed
- **Breaking**: 三类 Zone（`no_fly_zones`/`restricted_zones`/`obstacles`）合并为统一 `zones` 数组，每个 zone 必须提供 `zone_type` 字段（no_fly / restricted / obstacle）。输入 JSON 中旧三数组改为 `"zones": [...]`。
- **Breaking**: 飞机性能参数重构——删除 `cruise_speed_mps`/`speed_range_mps`/`min_turn_radius_m`/`max_climb_angle_deg`/`max_bank_deg`/`ceiling_m`，新增 `maximum_speed_mps`/`maximum_turn_rate_dps`/`maximum_climb_rate_mps`/`maximum_altitude_m`（全部可选，有现代战机默认值）。
- **Breaking**: terrain 配置从 `source/path/mask_path`（文件路径）改为 `arpack/mask`（索引 id），对应 `data/index.yaml` 新增索引文件机制。输入 JSON 中 `"terrain": {"source": "path", "path": "data/xxx.arpack"}` 改为 `"terrain": {"arpack": "east_asia"}`。
- help 输出动态展示可用地形候选项（从 index.yaml 加载）。
- demo-server `GET /api/data-files` 从扫描文件改为返回 index.yaml 内容。

## [0.3.0] - 2026-09-15

### Changed
- **Breaking**: CLI 编译产物从 `aircraft-router-planner-cli` 重命名为 `arpcli`，包名同步更新。
- **Breaking**: CLI 接口精简为 `arpcli plan --file <file> --out <path>`（两个必填选项），移除 stdin/stdout 管道模式。
- 移除 `arpcli schema` 子命令。
- `grid_resolution` 迁移至 Input JSON 的 `parameters.grid_resolution`（8..1024，默认 256），CLI 不再提供 `--grid` 选项。
- help 输出从 clap 自动生成改为自定义格式（纯文本 + 定宽对齐，三层递进 Root → Category → Endpoint）。

### Fixed
- 修复 Windows 控制台中文乱码：启动时把控制台输出/输入代码页切到 UTF-8（kernel32 `SetConsoleOutputCP`/`SetConsoleCP`，零 C 依赖），help/告警/错误/结果 JSON 中的中文在 cmd/PowerShell 不再乱码；重定向到文件/管道时字节仍为 UTF-8 不受影响。

## [0.2.1] - 2026-08-20

### Added
- demo 场景新增鼠标悬停经纬高实时显示（右下角浮层）。
- demo-server 新增 `GET /api/data-files`：扫描数据目录（`DATA_DIR` 环境变量覆盖，默认 exe 同级 `data/`），按扩展名分类返回地形/掩膜文件列表，供前端下拉选择。

### Changed
- 优化 help 展示：首行显示可执行文件名与版本，Usage/示例/错误信息动态使用当前执行文件名。
- 移除 `--version` 标志与 `plan` 未启用的保留参数 `--seed`/`--config`（**破坏性变更**：传入即报错退出）。
- demo 地形显示与计算配置解耦（跟随/无/外部文件）。
- **移除 `builtin` 地形选项（破坏性变更）**：`TerrainSourceType` 仅保留 `none`/`path`；CLI `source=path` 未提供路径或加载失败时自动降级为无地形（不再报错中止），并在输出 `degradations` 中记录原因。
- 掩膜不再自动探测：仅显式指定 `--mask`/`terrain.mask_path` 时启用（文件缺失仍报错）。
- demo 前端自动扫描数据目录：启动时拉取 `/api/data-files`，有地形/掩膜文件时默认选中第一个，路径输入改为文件下拉。
- demo 地形面板重构：「地形（CLI计算）」更名为「地形显示」；原「地形显示」更名为「CLI计算数据源」（none/follow_view，默认跟随视图）。
- **输入/输出契约精简（破坏性变更，v0.21）**：删除全部未消费字段——顶层 `schema_version`（输入输出全删）、`crs`/`output_crs`、`red_forces.sams`、`radar.radar_type`、`terrain.resolution_m`/`vertical_datum`、`zone.height_semantics`（高度一律 MSL）、`weapon.fuze_type`/`target_ref`、`start_pose.heading_deg`、`profile.detection_probability`、`parameters` 的 `main_budget_ms`/`degrade_budget_ms`/`z_resolution_m`/`fine_success_threshold`/`coarse_cell_m`/`default_weapon_radius_km`/`weapon_map`；`DefaultParams` 同名死字段与零调用点的 `default_weapon_map()` 一并删除。携带旧字段的输入将被拒绝（`input_invalid: malformed_json`）。
- 契约版本追溯改由 CHANGELOG/技术方案版本承担（版本号不再是输入输出的重要参数）。
- 输出 JSON 不再携带 `schema_version`。

### 输入契约统一与飞行器化（v0.21 第二波）
- 删除 mission 包裹层与顶层 start/target：起终点逐机显式（aircraft[].start/target 必填）；aircraft 空数组 → input_invalid: missing_aircraft（新原因码）
- vehicle → aircraft 全量改名（契约 JSON 键 vehicles→aircraft、公开类型、内部标识符与注释）
- 删除 start_pose（VehiclePose 与 Waypoint 重复类型）与 target_ref 魔法字符串（"mission.target"/"lon,lat,alt"），目标结构化
- 武器移入飞行器：删除顶层 weapons 数组与 weapon_id="<id>_w1" 拼接约定；aircraft[].weapon{weapon_type 必填, range_km?, envelope?}（出现即启用）
- Zone 删除 zone_type 键：三数组为唯一类型标记（解析层按数组注入内部字段；旧键 → malformed_json）
- demo-server 修复：调用 CLI 补充 plan 子命令（v0.2.0 起需显式 plan，修复 /api/plan 失效）
- crash_suite 新增 6 个第二波护栏用例

### Fixed
- 修复 pnpm 构建时 hls.js 触发最小发布年龄检查报错。

## [0.2.0] - 2026-08-18

### Added
- `arpcli schema` 子命令：用 schemars 动态生成输入/输出 JSON Schema（代码即事实，零漂移）。

### Changed
- help 风格改为 `arpcli` / `arpcli help` / `arpcli help <command>`，移除 `--help` 标志。
- 规划动作显式化为 `arpcli plan` 子命令（裸 `arpcli` 现显示顶层 help；**破坏性变更**，原 `arpcli < mission.json` 管道改为 `arpcli plan < mission.json`）。
- 地形转换/重压缩从核心 CLI 剥离为独立内部工具 `arp-convert`（`src/convert/` crate，**不随核心 CLI 发布**，随用随编）。

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
- 开发期可视化工具 `src/demo/`（Axum 后端 + React/Three.js 前端，不随发布版分发）。
- 工程化：CI 分层门禁（静态检查 + 手动全量测试）、release 流水线（`v*` tag 交叉编译 4 平台）、`CHANGELOG.md`、`scripts/bump_version.sh`。

### Changed
- `.cargo/config.toml`：确定性 `-fma` flag 由全局收敛到 x86_64 目标；新增 arm64 目标配置。

### Fixed
- CI 工具链由 1.85 升至 1.89（依赖 MSRV 提升）。
- 静态依赖红线 grep `proj` 误匹配 `pin-project-lite` 的问题。

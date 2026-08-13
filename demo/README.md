# Demo（开发期工具 + 独立可运行测试包）

按技术方案八章裁决：Demo 层 = **工具可视化参数输入 + 航路验证**，非纯演示；
发布版交付物 = 单二进制 + 默认地形文件，**不含 Demo**。

2026-08-05 主管要求：按现有重构结果（schema 0.20 契约：多机 `vehicles`、红方雷达、
禁飞/限飞/障碍区、必经点 `mid_waypoints`、地形源、输出复验清单）移植旧项目
`simple-router-planner` 的 Demo 到本仓库。

2026-08-10 主管要求：打包**独立可运行 Demo** 到 `install/demo/`（供其他计算机无
工具链测试）——demo-server 增加静态文件服务（单进程 :3001 同时提供 API 与前端
页面）+ 可移植地形路径解析。

## 结构

```
demo/
├── server/    # Rust + Axum 后端：POST /api/plan → stdin/stdout 管道调核心 CLI
│              # （aircraft-router-planner-cli），透传 Output JSON；
│              # POST /api/terrain 直接采样 ARPK1 供 3D 地形渲染；
│              # 其余路径 serve 前端生产构建（web-dist/，环境变量 DEMO_WEB_DIR 覆盖）
│              # 端口环境变量 DEMO_PORT，默认 3001
└── web/       # React 18 + TS + Vite + Three.js 前端（npm）
    └── src/
        ├── types.ts        # 输入/输出契约类型 + 经纬高↔局部平面坐标工具
        ├── api.ts          # /api/plan 调用（相对路径，同源）
        ├── App.tsx         # 主界面（配置 + 场景 + 结果）
        └── components/     # Scene3D / ControlPanel / 雷达球 / 禁飞区棱柱 / 路径线
```

## 独立运行打包（install/demo/）

发布时把以下内容放入 `install/demo/`（复用 `install/` 根的 CLI 与默认地形/掩膜）：

```
install/demo/
├── demo-server.exe     # release 构建（含静态文件服务）
├── web-dist/           # npm run build 产物（vite build → dist → 复制为 web-dist）
├── start-demo.bat      # Windows 一键（cd install 根 + ARP_CLI + 开浏览器）
├── start-demo.sh       # Linux/macOS 一键
└── DEMO_README.md      # 分发给使用者的说明
```

**无需** cargo / npm / node——浏览器打开 http://localhost:3001 即用（demo-server
单进程提供 API + 页面）。核心逻辑与开发模式完全一致（同一 CLI 可执行文件）。

### 重新打包步骤（开发机）

```bash
cargo build --release -p demo-server
cd demo/web && npm run build          # 产出 demo/web/dist
copy demo/web/dist  → install/demo/web-dist
copy target/release/demo-server.exe → install/demo/
```

## 启动

### 开发模式（需工具链）

### Windows 一键启动（推荐）

项目根目录双击 **`start_demo.bat`**（或命令行 `start_demo.bat`）：

- 自动检查/构建 CLI 与 demo-server（产物已存在则跳过）
- 启动后端 :3001 与前端 :5173，并自动打开浏览器 http://localhost:5173
- 参数 `start_demo.bat rebuild` 强制重新构建两个二进制
- 停止：关闭 `arp-demo-server` / `arp-demo-web` 两个窗口，或 `taskkill /IM demo-server.exe /F`

### 手动分步（任意平台）

```bash
cargo build --release -p aircraft-router-planner-cli
cargo build --release -p demo-server
cargo run --release -p demo-server &        # :3001
cd demo/web && npm install && npm run dev   # :5173
```

Linux/macOS 可用 `bash demo/start.sh` 一键（脚本按 Git Bash/pnpm 编写）。

### 独立运行模式（无工具链）

见 `install/demo/DEMO_README.md`：Windows 双击 `start-demo.bat`，或
`./start-demo.sh`（Linux/macOS），浏览器打开 http://localhost:3001。

## 功能

- **可视化参数输入**：起点/终点（经纬高，可点图设置）、飞行器（固定翼/旋翼机、
  速度/转弯半径/爬升角）、必经点（经纬高——高度 2026-08-13 P8 M2 起生效，垂直剖面
  多锚点分段插值）、雷达（经纬/半径/类型）、禁飞区（圆形：经纬/
  半径/高度带/类型：禁飞/限飞/障碍）、地形源（none/builtin/path）、探测概率参数
  （探测曲线：Swerling I 默认——2026-08-13 base_p 标定，R_eff 处探测概率 0.9；指数 / 线性）；
- **航路验证**：Three.js 场景叠加路径（多车独立颜色）、雷达探测球+地面盘、禁飞区
  棱柱、必经点；结果面板展示每车路径长度/路点数/**复验警告（warnings）**、全局
  **降级记录（stats.degradations）**、FMM 耗时与 LOS 检查次数。

## 说明

- CLI 二进制默认位于 workspace 根 `target/release/aircraft-router-planner-cli`；
  可用环境变量 `ARP_CLI` 覆盖（例如指定交叉编译产物；**设置但不存在时回退候选**）。
- 默认场景为北京近郊（115.9°E, 39.8°N → 116.8°E, 40.3°N），默认地形
  `data/east_asia_7p5as.arpack`（开发模式 cwd=workspace 根命中）；独立模式下
  demo-server 按「cwd → exe 同目录 → exe 上级 → 去首段」多候选解析地形路径，
  与 install/ 根布局兼容。
- Web 只支持圆形禁飞区编辑；多边形（`polygon`）在契约中保留，前端仅预览。

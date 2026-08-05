# Demo（开发期工具，不随发布版分发）

按技术方案八章裁决：Demo 层 = **工具可视化参数输入 + 航路验证**，非纯演示；
发布版交付物 = 单二进制 + 默认地形文件，**不含 Demo**。

2026-08-05 主管要求：按现有重构结果（schema 0.20 契约：多机 `vehicles`、红方雷达、
禁飞/限飞/障碍区、必经点 `mid_waypoints`、地形源、输出复验清单）移植旧项目
`simple-router-planner` 的 Demo 到本仓库。

## 结构

```
demo/
├── server/    # Rust + Axum 后端：POST /api/plan → stdin/stdout 管道调核心 CLI
│              # （aircraft-router-planner-cli），透传 Output JSON
└── web/       # React 18 + TS + Vite + Three.js 前端（pnpm）
    └── src/
        ├── types.ts        # 输入/输出契约类型 + 经纬高↔局部平面坐标工具
        ├── api.ts          # /api/plan 调用
        ├── App.tsx         # 主界面（配置 + 场景 + 结果）
        └── components/     # Scene3D / ControlPanel / 雷达球 / 禁飞区棱柱 / 路径线
```

## 启动（需 Rust + pnpm）

```bash
# 方式一：一键（构建 CLI + server，启动 :3001 与 :5173）
bash demo/start.sh

# 方式二：手动
cargo build --release -p aircraft-router-planner-cli
cargo build --release -p demo-server
cargo run --release -p demo-server &        # :3001
cd demo/web && pnpm install && pnpm dev     # :5173
```

浏览器打开 http://localhost:5173 。

## 功能

- **可视化参数输入**：起点/终点（经纬高，可点图设置）、飞行器（固定翼/旋翼机、
  速度/转弯半径/爬升角）、必经点、雷达（经纬/半径/类型）、禁飞区（圆形：经纬/
  半径/高度带/类型：禁飞/限飞/障碍）、地形源（none/builtin/path）、探测概率参数；
- **航路验证**：Three.js 场景叠加路径（多车独立颜色）、雷达探测球+地面盘、禁飞区
  棱柱、必经点；结果面板展示每车路径长度/路点数/**复验警告（warnings）**、全局
  **降级记录（stats.degradations）**、FMM 耗时与 LOS 检查次数。

## 说明

- CLI 二进制默认位于 workspace 根 `target/release/aircraft-router-planner-cli`；
  可用环境变量 `ARP_CLI` 覆盖（例如指定交叉编译产物）。
- 默认场景为北京近郊（115.9°E, 39.8°N → 116.8°E, 40.3°N），无地形（海拔 0 平面）；
  切换 `builtin` 需准备 ARPK1 地形文件并通过 `ARP_CLI --terrain` 或输入 `terrain.path` 提供。
- Web 只支持圆形禁飞区编辑；多边形（`polygon`）在契约中保留，前端仅预览。

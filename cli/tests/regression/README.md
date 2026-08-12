# Phase 0 反馈错误输入回归集（regress_phase0）

来源：`phase0_out/`（Phase 0 调试产物目录，已 gitignore）中**输入型 mission JSON**——
历史上主管反馈 bug 的原始输入。复制至此作为正式回归用例，防止错误复现。

## 用例清单（21 个）

| 用例文件 | 对应历史 bug（memory 锚点） |
|---|---|
| `real_bad.json` | 主管真实反馈：no_fly 圆+多边形 + 真实地形 |
| `zigzag_rz.json` / `zigzag2.json` / `zigzag3.json` / `zigzag_nf2.json` / `zigzag4.json` | 353 点锯齿 + 3000m 直穿 restricted（FIX_ZIGZAG_NOFLY，commit 37667fe） |
| `real_rz.json` / `real_rz_1500.json` / `real_rz_low.json` | restricted 高度层剖面（底部/顶部判据） |
| `new_rz.json` / `moved_rz.json` / `top_vs_bottom_rz.json` | restricted 圆心变更场景（阶段四判据修正） |
| `rz_a_mountain.json` / `rz_b_plain.json` | 圆角落山误判穿行带修复（commit 1b1331b） |
| `real_rz_polygon.json` | restricted 多边形形态 |
| `test_trap.json` | 禁飞区陷阱（多边形） |
| `test_twin.json` / `test_twin7.json` | 双禁飞区（圆+多边形） |
| `zigzag11.json` | 段边界入口航向约束（commit 44aad43） |
| `zigzag12.json` | 剖面过渡直线穿硬墙 → need_wall 画墙兜底（2026-08-07） |
| `rz_poly_wall_detour.json` | 多边形受限区顶部/底部都不可行 → 画墙水平绕行；theta_star check 长弦短穿带漏检（N=16 等距采样错失）→ 解析求交修复（2026-08-12） |
| `rz_poly_wall_region_expand.json` | 多边形受限区画墙绕行但 region 未外扩 → 多边形尖角顶点超出 region → 绕行走廊被 region 边界截断 → coarse FMM no path 误报 geometrically_impossible（同形状禁飞区因 is_wall 纳入外扩能绕行）；修复 expand_region_for_walls 纳入会画墙的 restricted（2026-08-12） |

## 回归语义

1. 每个输入 parse + validate + solve 全过（不 panic、不 Err）；
2. 输出路径逐点**不穿任何 zone**（水平包含 + 高度带，与 `zone_contains_at` 同语义）——
   no_fly/obstacle 全高度禁入，restricted 仅禁入高度带（底部/顶部剖面穿越合法）。

## 地形依赖（主管决策 2026-08-06 / 2026-08-10）

- cases 中 `terrain.path` 指向 `data/east_asia_7p5as.arpack`（发布版默认地形，
  数据已 gitignore；2026-08-10 起 china_dem_l12 已退出测试流程）。
- 运行期检测：**数据存在** → 改写 path 为绝对路径使用真实地形；**数据缺失** →
  `terrain.source=none` 合成平面（覆盖不到真实地形 bug，但保证用例在无数据环境仍可跑）。

## 新增用例

把输入 JSON 放入本目录（`tests/regression/cases/`）即被 `tests/regress_phase0.rs`
自动发现（遍历 *.json），无需改测试代码。

# 🔒 发布版数据定案（LOCKED — 不可更改）

> **主管最终拍板，已再三强调（2026-08-08 定案，2026-08-10 再次确认）。**
> 此文件是项目级锁定记录：**任何任务执行前必须确认以下事实，且不得再讨论/推翻/提议替换。**

---

## 定案内容

| 项 | 定案值 | 说明 |
|----|--------|------|
| **默认地形** | `east_asia_7p5as.arpack` | GMTED2010 东亚 7.5as，537MB，70–135°E × 15–55°N，int16，zstd，vd=EGM96 |
| **默认掩膜** | `mask_7p5as.mask` | GSHHG 全球 V2 3 态（0 海洋/1 陆地/2 内陆湖），7.5as，86400×172800，30.8MB |

## 存放与查找

- 项目根 `data/`（gitignore）：`data/east_asia_7p5as.arpack` + `data/mask_7p5as.mask`
- 发布版 `install/data/`：同构目录（install/ 整个目录即安装包，gitignore）
- solver 默认自动探测：exe 同目录 / 工作目录 / phase0/data 下的
  `east_asia_7p5as.arpack` 与 `mask_7p5as.mask`（`default_mask_candidates()`）

## 作废/降级项（不得重新启用为默认）

- ❌ `mask_10as.mask`：不再作默认掩膜（仅可显式指定使用；V2 3 态、含南极内陆补全）
- ❌ `china_dem_l12.arpack`：仅回归测试用，不作发布默认
- ❌ `gmted2010_7p5as_global.z19.arpack`：2.31GB 超 800MB 目标，不作为发布默认
  （保留 data/ 可选）

## 验证记录（发布前数据规格验证，docs/10）

- east_asia_7p5as.arpack：默认规划 94.84km 基线一致；>10 弧秒契约满足；≤800MB 目标 ✓
- mask_7p5as.mask：10 个已知点语义与 10as 版全一致（生成 30.9s）；solver 默认自动探测

## 关联文档

- `docs/phase0_baseline.md` →「主管决策 2026-08-08（三项，A1_TERRAIN_PACKAGE）」
- `docs/04-地形数据源.md`、`docs/10-测试与验证.md`、`docs/11-工程化与构建.md`
- 工作区 `MEMORY.md` →「项目关键定案」、工作区 `AGENTS.md` →「第零步：确认发布版数据定案」

# Phase 0 回归基线与标定值回填（Phase 1 验收基准）

> 生成：2026-08-04（S9 完成）。本文件为 Phase 1 实施的性能验收基准：
> 已实测数值直接引用；数据依赖项待正式地形数据到位后回填（标记「待数据」）。

## 硬件规格（基准平台）

| 项 | 值 |
|----|-----|
| CPU | Intel Core i7-7700 @ 3.60GHz（4C/8T，2017 年老平台） |
| 内存 | 16 GB |
| 存储 | SSD（CT500MX500SSD1）+ HDD（ST1000DM003） |
| 构建 | release（opt-level=3, lto=thin, codegen-units=1, `-C target-feature=-fma`） |

## 性能预算实测（5.1 预算表 → 端到端 3s 口径）

| 环节 | 预算占比（初始） | 预算（3s） | 实测（100km 场景） | 判定 |
|------|-----------------|-----------|--------------------|------|
| B1 FMM 粗层传播（~15%） | 100 源×128² | 450ms | **248.7ms** | PASS |
| B2 代价场复用（多机摊薄） | — | — | 同源 **10.2x** / 异源 1.14x | 契约落定 |
| B3 rstar 索引/碰撞（~30%） | 1000 条线段 | 900ms | **≈1ms**（含候选精确检查） | PASS |
| B4 射线-地形求交（~40%） | 1000 条 LOS | 1200ms | **13.4ms** | PASS |
| B5 细层基元拟合（~10%） | 100 段 | 300ms | **0.14ms** | PASS |
| t_load_decompress 内置格式加载（Phase 1 实测，cli benches/b_load_decompress.rs） | — | ≤300ms | parse 1024² **8.89ms** / 1000 次采样 **171.75µs** | PASS |

## 标定值（已定 / 待数据）

### 已定（合成场景实测，可直接入契约）
- FMM O(NlogN) 实际常数：**11.0–12.5 ns/op**（64²–512² 四档稳定）
- 多机摊薄：同源 **10.2x（必共享）**、异源 1.14x（仅构建共享）
- rstar 候选剪枝：N=100 时 30.7/条（暴力 100）；**N=100 临界，N 大时 rstar 优势显著**
- 细层基元成功率：**95%（≥90% 判据）**，失败 100% 归因 d<2R（CCC 补全后预期 ~100%）
- 射线-地形：**LOD 网格降采样不省时**（采样点主导），省时靠采样点/射线数
- 单射线 13.4µs（1000 采样）→ 3s 预算可容纳 ≈9 万次 LOS 检查
- 走廊质量：100/100 可达，平均绕行比 2.02（合成障碍密度高于典型任务）

### 空洞语义（2026-08-04 Beijing_DEM 实测发现，决策输入——2026-08-08 beijing 数据/基准已清理，发现保留）
- **空洞语义两项重要发现**：
  1. LOS：空洞段视为不遮挡 → blocked 率 5.5% vs 全有效 19.6% → **低估遮挡 = 探测概率偏高（非保守侧）**，Phase 1 空洞需插值/排除，不能静默放行
  2. 代价场：空洞墙（保守禁行）把有效区切成多岛 → **可达率仅 33/100**；空洞处理策略（填充/禁行/代价权衡）直接影响路径规划可用性

### 待数据（2026-08-05 真实数据标定——GMTED2010 全球 + China_Dem_L12 两份 ARPK1 已实测回填）
- **P_cross 阈值、衰减模型形态**：`p_cross` 默认 0.1 维持占位（需真实雷达/地形模型，属雷达领域标定，非数据可定）
- **LOS mask 系数 0.05–0.1 → 定值 0.08（维持，保守口径）**：北京平原实测 LOS blocked 5.2%（GMTED2010 7.5as）/ 4.8%（China 9.888as）——分辨率差异对平原影响小；四川山地 blocked 28.5%（遮蔽>0 A7 用例达标）——低遮蔽区用区间下限附近保守值 0.08 合理
- **LOS 预计算耗时 ≤0.5s → 实测达标**：单射线 109–132µs（北京）/ ~180µs（四川，1000 采样点）；1000 条 ≤0.18s < 0.5s 预算
- **A6 速度-转弯半径公式 → 数值验证通过**：r_min = v²/(g·tan φ_max)，30° 坡度下 50→442m / 100→1766m / 150→3974m / 200→7065m / 250→11039m；默认 turn_radius 5000m 自洽 v_max ≈ 168 m/s（超速需配置更大 min_turn_radius，config 校验已强制）
- **细层步长/平滑阈值 → 实测**：北京区 100 组随机起止（256² FMM 粗层走廊 + smooth_path_chain 全链）**100% 拟合成功**（≥90% 判据远超）、平均 12.19ms/组（含 FMM+平滑，3s 预算内 100 源 ≈1.2s）；弦高容差 100m 初值维持（正式数据到位后可按需复标）
- **3s 预算按机线性分配 → 实测共享摊薄**：同源 n=16 共享 FMM 11.7ms + 16 回溯 ≈ 11.7ms vs 每机独立 FMM 167.9ms——**14.6x 摊薄**；线性分配（n×单机）为保守上限，实际远优
- **FNR-FPR 统计 → 测量基建建立**：真值 = 细网格 1024² 重算；北京平原 & 四川山地各 100 组粗(256²)/细对比——**FNR 0.0%**（粗层 256² 对 1°×1° 区域无漏检）、FPR 无样本（粗层 0 不可达）；正式验收需人工标注小样本集（十三轮真值分层：人工层待建）
- **152.87m 几何口径 → 二选一定案：等经纬度网格（弧秒制，equiangular）**——ARPK1 元数据 `resolution_semantics=equiangular` 已写入；GMTED2010 7.5 弧秒 = 231.9m（赤道）、China 9.888 弧秒 = 305.75m（赤道）——**实际数据分辨率粗于方案主档 152.87m（SRTM 5as 语义）**，等经纬度口径下按实际弧秒声明，纬向随 cosφ 收缩
- **确定性黄金基线 → 实测通过**：真实数据全链路（FMM → 回溯 → smooth 全链）两遍运行 path-sha256 逐位一致 `ff2d3a229de3a7a8392901f97f6ebc7e3b931de11011adf0fd9c07c44dbb8903`（确定性构建 `-C target-feature=-fma` + 固定 target-cpu 生效）
- **数据规格**：分辨率 ≤10 弧秒契约——GMTED2010 7.5as ✓ / China 9.888as 踩线 ✓；压缩体积——GMTED2010 2.311GB 超 800MB 目标（**全球主档口径，主管裁决推迟——发布版数据待以后再行决定**，2026-08-05）、China 76MB ✓
- **NODATA 5x 复核 → 维持**：南海区 NoData 55%——5x 高代价通行可达率 100%（路径绕行）vs inf 禁行 45%（切断连通）——**保守高代价保持可用性，设计合理**
- **ARPK1 采样性能修复（新发现）**：随机访问 748–1172µs/op（每点解压新块，zstd 固有成本）；**路径连续采样 0.15–2.05µs/pt**（缓存命中，真实访问模式）——`BuiltinSource` 缓存由无界 HashMap 改为 FIFO 有界 2048 块（≈256MB，原随机采样可膨胀 13GB+），解压移出 Mutex（多线程不串行）；**open 一次性 12.6s（GMTED2010 2.31GB read+SHA）**——属启动初始化，不在 3s 路径规划预算内（预算口径修订：t_load ≤300ms 对小文件有效，大文件一次性初始化单独计时）
- 掩膜语义验证（真实数据）：GMTED2010 + mask_10as 全球采样 Land 29.5% / Water 70.1% / Lake 0.5%（陆地占比吻合理论 ~29.2%）；定点北京 Land(52.1m)/太平洋 Water/青海湖 Lake(3194m)——掩膜 3 态集成正确；掩膜加速海洋采样（跳过 DEM 解压：masked 247µs vs 无掩膜 1172µs 随机）

- **待定数据（发布前，2026-08-05 转换完成；2026-08-08 迁移到项目根 `data/`，gitignore）**：
  - `data/gmted2010_7p5as_global.z19.arpack`：**2.311GB**、7.5 弧秒（≈231.9m 赤道）、全球 84°N..56°S 67200×172800、int16、zstd-19、vd=EGM96、空洞率≈0（海洋=0m）；Rust 采样验证：北京 52.1m/青藏 4928.7m/太平洋 0m ✓；>10 弧秒契约满足；体积超 800MB 目标（**已裁决（2026-08-08）：不作为发布默认，默认 = 东亚 7.5as 537MB，全球档保留 data/ 可选**）
  - `data/china_dem_l12.arpack`：**76MB**、9.888 弧秒（≈305.75m 赤道，踩线 ≤10 弧秒）、中国区 73.5-135.1°E / 3.6-53.6°N 18194×22429、有效 29%（NaN=海洋/境外→no_data -32768）、北京 100% 有效；Rust 采样验证：北京 49m/云南 1459.8m/海南 -7.8m ✓
  - 转换工具：`phase0/scripts/convert_to_arpk1.py`（GeoTIFF 直读 / JP2 gdal_translate -srcwin 并行解码；opj -t 与后台 start /b 子进程存在挂起问题，已弃用）+ `cli/examples/arpk1_probe.rs`（Rust 交叉验证）
- **掩膜分辨率定案：10 弧秒**（2026-08-04 主管拍板；`mask_10as.mask` V2 3 态：0 海洋/1 陆地/2 内陆湖，含南极内陆补全；2026-08-08 生成 7.5as 全球版 `data/mask_7p5as.mask` 并定为默认掩膜；掩膜暂不入 git，开发完成后决定）
- **NODATA 高代价倍数初值：5x**（2026-08-04 主管拍板；正式数据到位后按待数据 10 项标定复核）

## 主管决策 2026-08-05（六项，commit 39c59c4）

1. **默认低精度地形 = 量化中国数据（china_dem_l12.arpack）**：terrain source=path/builtin 且未给路径时，solver 依次尝试 exe 同目录 / exe 上溯 workspace 根 / 工作目录相对路径下的 `china_dem_l12.arpack`；全找不到 → input_invalid（data_error）。发布版 = 单二进制 + 同目录 `china_dem_l12.arpack`。北京场景无 --terrain 默认地形回归 163.06km 与显式路径一致。
2. **Windows MSVC 静态编译验证通过**：`.cargo/config.toml` 的 `x86_64-pc-windows-msvc` 加 `-C target-feature=+crt-static`；pefile 验证 exe 仅依赖系统 DLL（VCRUNTIME140.dll 消除，无第三方 DLL）；验证脚本 `cli/check_pe_deps.py`（`python check_pe_deps.py <exe>`）。Phase 5 交叉编译暂停。
3. **雷达参数外部输入，无效回落默认**：`mission.parameters` 支持 radar_inflation / detection_curve / p_cross / suppression_delta / los_mask_coef 外部覆盖；合法域与旧契约一致（radar_inflation>1、p_cross/los_mask_coef∈[0,1]、suppression_delta∈[0,1)、detection_curve∈{exponential,linear} 不区分大小写）；出域/非有限/非法字符串 → 回落默认，事实记入 `stats.degradations`（"parameter X invalid -> default Y"）；validate 不再对参数域 fail-fast。
4. **压制维持现状**：压制按雷达自身字段（suppression_post_range_km/suppression_factor）配置；独立干扰机（jammer）实体为后续需求，暂不使用（无代码改动）。
5. **GMTED2010 后台 zstd-19 重压缩**：`phase0/scripts/recompress_arpk1.py`（读头部 blocks_x/y → 逐块解压 → zstd-19 重压 → 重写索引+SHA）验证体积是否减小；结果待回填（预计减 15–25%）。
6. **坐标点均为经纬高定义**：输入/输出坐标点统一 WGS84 经纬度（度）+ MSL 高度（米）；输出 PathPoint 的 x=经度、y=纬度、alt_m=MSL（注释已更新）；无投影/ENU 混合语义。


## 主管决策 2026-08-08（三项，A1_TERRAIN_PACKAGE）

1. **默认地形 = GMTED2010 东亚 7.5as 压缩版（east_asia_7p5as.arpack 537MB）**：取代 china_dem_l12 默认地位；terrain source=path/builtin 且未给路径时，solver 依次尝试 exe 同目录 / 工作目录 / phase0/data / pending/east_asia_crop 下的 `east_asia_7p5as.arpack`（commit 29adf33 接线；2026-08-10 数据迁移到项目根 data/ 与 install/data/，实际文件 537.2MB）。
2. **海岸掩膜随默认地形提供——默认提供和使用的掩膜为全球 7.5as 版本（mask_7p5as.mask）**：GSHHG 全球 V2 3 态（覆盖 360°×180°，86400×172800，30.8MB，生成 30.9s；10 个已知点语义与 10as 版全一致）；solver `default_mask_candidates()` 自动探测（候选名 mask_7p5as.mask，exe 同目录 / 工作目录 / phase0/data），`TerrainConfig.mask_path` 可显式覆盖；区域窗口掩膜（east_asia_7p5as.mask 等）不自动探测，需显式指定；mask_10as.mask 保留（可显式使用）。
3. **内置纯 Rust 压缩编码器（COMPRESSION_DEFLATE=2，commit 8780d4e）**：成熟纯 Rust zstd 编码器不存在（ruzstd 仅解码；zstd-pure-rs immature 有数据损坏风险），经主管确认采用 miniz_oxide deflate（flate2 官方 rust 后端，零 C 红线）；POC 实测真实地形差分块压缩比 4.06:1 ≥ zstd 3.98:1。convert 输出自动压缩（索引动态记录 + finish 回填 + 流式重读算 SHA，与 Python convert 语义一致）。

## 崩溃测试套件（B9，CI 一票否决）
- Phase 0：`tests/crash_suite.rs` 13 用例 + 单元 9 = **22 测试全过**；FMM 越界源/空网格、Dubins NaN/零半径/极端坐标、Terrain 出界/NaN、rstar 空树均不 panic
- Phase 1（cli）：`cli/tests/crash_suite.rs` **20 用例**（config 畸形/非法坐标/退化、coord NaN/极端反算、terrain 垃圾字节/截断/零维度、spatial 空树+NaN 查询、costfield 退化网格/不可达回溯、输出序列化）+ 单元 **42** = **62 测试全过**
- 内置格式 fail-fast 三用例：哈希不符 / 版本不符 / 文件截断（+ 魔数不符）均 fail-fast 返回错误

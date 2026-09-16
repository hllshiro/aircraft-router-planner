# Input Schema 重构实施计划

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 重构 CLI 输入 Schema——合并三类 Zone、简化飞机性能参数、引入 Terrain 数据索引机制。

**Architecture:** 三项变更相互独立，按 Zone → Aircraft → Terrain 顺序实施，每项独立可测试。Zone 合并改 `serde` 注解 + `from_json_str` 注入逻辑；Aircraft 改结构体字段 + `smooth_options_for` 派生；Terrain 新增 `serde_yaml_neo` 依赖 + 索引加载 + help 动态候选项。

**Tech Stack:** Rust (serde, serde_json, serde_yaml_neo, clap)

**Spec:** 本计划基于用户确认的设计方案（2026-09-15 会话）

## Global Constraints

- 零 C 依赖：`cargo tree -e normal` 不得出现 openblas/zlib/curl/proj/gdal/pcre/ssl
- 不 panic：畸形/退化输入返回 error/status，不 panic
- 确定性：BTreeMap/fixed-order，不移除 `-fma`/`+crt-static` rustflags
- 所有 `cargo test --lib` / `cargo test --test` 必须通过
- `scripts/check.sh --quick` 必须通过

---

## 变更范围总览

| 分类 | 文件 | 变更类型 |
|------|------|---------|
| **依赖** | `cli/Cargo.toml` | 新增 `serde_yaml_neo` |
| **核心结构** | `cli/src/config.rs` | AircraftProfile/Zone/TerrainConfig/Input/DefaultParams/validate |
| **平滑派生** | `cli/src/smooth.rs` | `smooth_options_for()` 重写 |
| **求解器** | `cli/src/solver.rs` | Zone 合并、terrain 加载、profile 字段引用 |
| **帮助输出** | `cli/src/help.rs` | profile/zone/terrain 参数树 |
| **CLI 入口** | `cli/src/main.rs` | terrain 索引加载 |
| **测试** | `cli/tests/crash_suite.rs` | JSON 输入更新 |
| **测试** | `cli/tests/determinism.rs` | JSON 输入更新 |
| **测试** | `cli/tests/regress_phase0.rs` | JSON 输入 + zone 访问更新 |
| **测试** | `cli/tests/regression/cases/*.json` | 全部 34 个 JSON 文件 |
| **测试** | `cli/src/config.rs` (单元测试) | JSON 输入更新 |
| **测试** | `cli/src/solver.rs` (单元测试) | JSON 输入更新 |
| **demo** | `demo/server/src/main.rs` | terrain 处理更新 |
| **文档** | `docs/06-输入输出契约.md` | 契约更新 |
| **文档** | `docs/08-地形数据源.md` | terrain 文档更新 |
| **文档** | `CHANGELOG.md` | 变更记录 |
| **数据** | `data/index.yaml` | 新建索引文件 |

---

## Task 1: 合并三类 Zone

**目标**: 将 `no_fly_zones`、`restricted_zones`、`obstacles` 三个数组合并为一个 `zones[]`，每个 zone 增加必填 `zone_type` 字段。

**Files:**
- Modify: `cli/src/config.rs` — Input 结构体、Zone 结构体、from_json_str、validate
- Modify: `cli/src/solver.rs` — all_zones 合并逻辑
- Modify: `cli/src/help.rs` — 参数树
- Modify: `cli/tests/crash_suite.rs` — JSON 输入
- Modify: `cli/tests/determinism.rs` — JSON 输入
- Modify: `cli/tests/regress_phase0.rs` — JSON 输入 + zone 合并逻辑
- Modify: `cli/tests/regression/cases/*.json` — 全部 34 个文件
- Modify: `cli/src/config.rs` (单元测试) — JSON 输入
- Modify: `cli/src/solver.rs` (单元测试) — JSON 输入
- Modify: `docs/06-输入输出契约.md`

**Interfaces:**
- Produces: `Input.zones: Vec<Zone>`（替代三个独立数组）
- Produces: `Zone.zone_type: ZoneType`（从 `#[serde(skip)]` 改为正常反序列化）

### Step 1.1: 修改 Zone 结构体 — zone_type 从 skip 改为必填

`cli/src/config.rs:153-171`:

```rust
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Zone {
    pub id: String,
    // 删除 #[serde(skip)]，让 zone_type 从 JSON 正常反序列化
    pub zone_type: ZoneType,
    #[serde(flatten)]
    pub shape: ZoneShape,
    #[serde(default)]
    pub alt_min_m: Option<f64>,
    #[serde(default)]
    pub alt_max_m: Option<f64>,
}
```

同时删除 `Zone::is_wall()` 方法中的 `zone_type` 匹配（保留逻辑，但不再依赖注入）。

### Step 1.2: 修改 Input 结构体 — 三数组合一

`cli/src/config.rs:18-35`:

```rust
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct Input {
    pub aircraft: Vec<AircraftInput>,
    #[serde(default)]
    pub red_forces: RedForces,
    #[serde(default)]
    pub zones: Vec<Zone>,              // 替代三个独立数组
    #[serde(default)]
    pub terrain: TerrainConfig,
    #[serde(default)]
    pub parameters: ParamsOverride,
}
```

删除 `no_fly_zones`、`restricted_zones`、`obstacles` 三个字段。

### Step 1.3: 修改 from_json_str — 删除 zone_type 注入逻辑

`cli/src/config.rs:506-522`:

```rust
impl Input {
    pub fn from_json_str(s: &str) -> Result<Self, AppError> {
        let input: Self = serde_json::from_str(s).map_err(AppError::Json)?;
        // zone_type 现在直接从 JSON 反序列化，不再需要手动注入
        Ok(input)
    }
}
```

### Step 1.4: 修改 validate — 更新 zone 校验逻辑

`cli/src/config.rs:525-601`:

- 更新 `validate_zone` 函数：`restricted` 必须有 `alt_min_m < alt_max_m`（有限值），`no_fly`/`obstacle` 不要求
- 更新起点/目标/必经点不在禁飞区的校验：从 `input.no_fly_zones` 改为 `input.zones.iter().filter(|z| z.zone_type == ZoneType::NoFly)`
- 更新雷达位置校验：同上

### Step 1.5: 修改 solver.rs — 更新 zone 合并逻辑

`solver.rs:336-346`:

```rust
// 之前：三个数组 chain 合并
// 之后：直接使用 input.zones
let all_zones: &Vec<Zone> = &input.zones;
```

同时更新：
- `restricted_wall_zs` 构建（行 263-289）：从 `input.restricted_zones` 改为 `input.zones.iter().filter(|z| z.zone_type == ZoneType::Restricted)`
- `wall_zones` 构建：从 `input.no_fly_zones.chain(input.obstacles)` 改为 `input.zones.iter().filter(|z| z.is_wall())`

### Step 1.6: 修改 help.rs — 更新参数树

`cli/src/help.rs`:

将三个独立的 zone 节点合并为一个 `zones` 节点：

```rust
ParamNode {
    name: "zones",
    type_label: "array<object>",
    required: false,
    description: "区域数组",
    children: &[
        ParamNode { name: "id", type_label: "string", required: true, description: "区域 ID", children: &[] },
        ParamNode { name: "zone_type", type_label: "string", required: true, description: "no_fly / restricted / obstacle", children: &[] },
        ParamNode { name: "shape", type_label: "tagged-union", required: true, description: "circle / polygon", children: &[
            ParamNode { name: "circle", type_label: "object", required: false, description: "", children: &[
                ParamNode { name: "center", type_label: "array<f64,2>", required: true, description: "圆心 [lon, lat]", children: &[] },
                ParamNode { name: "radius_km", type_label: "f64", required: true, description: "半径 km", children: &[] },
            ]},
            ParamNode { name: "polygon", type_label: "object", required: false, description: "", children: &[
                ParamNode { name: "vertices", type_label: "array<array<f64,2>>", required: true, description: "顶点列表", children: &[] },
            ]},
        ]},
        ParamNode { name: "alt_min_m", type_label: "f64", required: false, description: "高度下界（restricted 必填）", children: &[] },
        ParamNode { name: "alt_max_m", type_label: "f64", required: false, description: "高度上界（restricted 必填）", children: &[] },
    ],
},
```

### Step 1.7: 更新所有测试 JSON 文件

**所有 34 个 `cli/tests/regression/cases/*.json` 文件** + 测试代码中的内联 JSON：

转换规则：
```json
// 旧格式
{
  "no_fly_zones": [{"id": "nf1", "shape": "circle", "geometry": {"center": [116.4, 39.9], "radius_km": 10}}],
  "restricted_zones": [{"id": "rz1", "shape": "circle", "geometry": {"center": [117.0, 40.0], "radius_km": 5}, "alt_min_m": 500, "alt_max_m": 3000}],
  "obstacles": []
}

// 新格式
{
  "zones": [
    {"id": "nf1", "zone_type": "no_fly", "shape": "circle", "geometry": {"center": [116.4, 39.9], "radius_km": 10}},
    {"id": "rz1", "zone_type": "restricted", "shape": "circle", "geometry": {"center": [117.0, 40.0], "radius_km": 5}, "alt_min_m": 500, "alt_max_m": 3000}
  ]
}
```

### Step 1.8: 更新 regress_phase0.rs — zone 合并逻辑

`cli/tests/regress_phase0.rs:63-104`:

```rust
// 之前：三个数组 clone + extend
// 之后：直接使用 input.zones
let zones = &input.zones;
```

### Step 1.9: 验证

```bash
cargo test --lib
cargo test --test crash_suite
cargo test --test determinism
cargo test --test regress_phase0
cargo check --workspace --all-targets
```

### Step 1.10: 提交

```bash
git add -A
git commit -m "refactor(cli): 合并三类 Zone 为统一 zones 数组

BREAKING CHANGE: no_fly_zones/restricted_zones/obstacles 三个数组合并为 zones[]，
每个 zone 需提供必填字段 zone_type: no_fly/restricted/obstacle。"
```

---

## Task 2: 重构飞机性能参数

**目标**: 将 AircraftProfile 的 6 个旧字段替换为 4 个新字段，更新派生逻辑和默认值。

**Files:**
- Modify: `cli/src/config.rs` — AircraftProfile 结构体、DefaultParams、validate_aircraft
- Modify: `cli/src/smooth.rs` — smooth_options_for() 重写
- Modify: `cli/src/solver.rs` — profile 字段引用（ceiling_m → maximum_altitude_m 等）
- Modify: `cli/src/help.rs` — 参数树
- Modify: `cli/tests/*.rs` — JSON 输入更新
- Modify: `cli/tests/regression/cases/*.json` — 全部 JSON 文件
- Modify: `cli/src/config.rs` (单元测试) — JSON 输入更新
- Modify: `cli/src/solver.rs` (单元测试) — JSON 输入更新

**Interfaces:**
- Produces: `AircraftProfile { aircraft_type, maximum_speed_mps, maximum_turn_rate_dps, maximum_climb_rate_mps, maximum_altitude_m }`
- Produces: `smooth_options_for()` 新派生逻辑
- Consumes: `DefaultParams` 新默认值字段

### Step 2.1: 修改 AircraftProfile 结构体

`cli/src/config.rs:75-97`:

```rust
#[derive(Debug, Clone, Deserialize, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct AircraftProfile {
    pub aircraft_type: AircraftType,
    /// 最大速度 m/s
    #[serde(default)]
    pub maximum_speed_mps: Option<f64>,
    /// 最大转弯角速率 °/s
    #[serde(default)]
    pub maximum_turn_rate_dps: Option<f64>,
    /// 最大爬升率 m/s
    #[serde(default)]
    pub maximum_climb_rate_mps: Option<f64>,
    /// 最大飞行高度 m
    #[serde(default)]
    pub maximum_altitude_m: Option<f64>,
}
```

更新 `AircraftProfile::default()` 实现：`aircraft_type = FixedWing`，其余全 `None`。

### Step 2.2: 修改 DefaultParams — 新增默认值字段

`cli/src/config.rs:324-383`:

```rust
pub struct DefaultParams {
    // ... 保留雷达/武器相关字段 ...
    pub default_max_bank_deg: f64,                    // 保留，不可派生
    pub default_fixed_wing_turn_radius_m: f64,        // 保留，兜底
    pub default_rotorcraft_turn_radius_m: f64,        // 保留，兜底
    // 新增
    pub default_fixed_wing_maximum_speed_mps: f64,    // 600.0
    pub default_rotorcraft_maximum_speed_mps: f64,    // 80.0
    pub default_fixed_wing_maximum_turn_rate_dps: f64, // 20.0
    pub default_rotorcraft_maximum_turn_rate_dps: f64, // 60.0
    pub default_fixed_wing_maximum_climb_rate_mps: f64, // 250.0
    pub default_rotorcraft_maximum_climb_rate_mps: f64, // 12.0
    pub default_fixed_wing_maximum_altitude_m: f64,   // 15000.0
    pub default_rotorcraft_maximum_altitude_m: f64,   // 6000.0
    pub default_fixed_wing_cruise_speed_mps: f64,     // 200.0（内部派生用）
    pub default_rotorcraft_cruise_speed_mps: f64,     // 60.0（内部派生用）
    // 删除旧字段
    // pub default_max_climb_angle_deg: f64,          // 删除
    // pub default_fixed_wing_speed_mps: f64,         // 删除
    // pub default_rotorcraft_speed_mps: f64,         // 删除
    pub default_grid_resolution: usize,
}
```

更新 `DefaultParams::default()` 实现，使用新默认值。

### Step 2.3: 重写 smooth_options_for()

`cli/src/smooth.rs:76-111`:

```rust
pub fn smooth_options_for(
    profile: &AircraftProfile,
    params: &DefaultParams,
) -> (SmoothOptions, f64) {
    // 1. 最大速度（用户输入或默认）
    let max_speed = profile.maximum_speed_mps.unwrap_or(match profile.aircraft_type {
        AircraftType::FixedWing => params.default_fixed_wing_maximum_speed_mps,
        AircraftType::Rotorcraft => params.default_rotorcraft_maximum_speed_mps,
    });

    // 2. 巡航速度 = min(默认巡航, 最大速度)
    let cruise_default = match profile.aircraft_type {
        AircraftType::FixedWing => params.default_fixed_wing_cruise_speed_mps,
        AircraftType::Rotorcraft => params.default_rotorcraft_cruise_speed_mps,
    };
    let v = cruise_default.min(max_speed);

    // 3. 最大转弯角速率
    let turn_rate_dps = profile.maximum_turn_rate_dps.unwrap_or(match profile.aircraft_type {
        AircraftType::FixedWing => params.default_fixed_wing_maximum_turn_rate_dps,
        AircraftType::Rotorcraft => params.default_rotorcraft_maximum_turn_rate_dps,
    });

    // 4. 转弯半径 = v / tan(ω)
    let turn_radius_m = if turn_rate_dps > 0.1 {
        v / turn_rate_dps.to_radians().tan()
    } else {
        match profile.aircraft_type {
            AircraftType::FixedWing => params.default_fixed_wing_turn_radius_m,
            AircraftType::Rotorcraft => params.default_rotorcraft_turn_radius_m,
        }
    };

    // 5. 最大爬升率
    let climb_rate = profile.maximum_climb_rate_mps.unwrap_or(match profile.aircraft_type {
        AircraftType::FixedWing => params.default_fixed_wing_maximum_climb_rate_mps,
        AircraftType::Rotorcraft => params.default_rotorcraft_maximum_climb_rate_mps,
    });

    // 6. 爬升角 = asin(climb_rate / v)
    let max_climb_deg = if v > 0.1 {
        (climb_rate / v).asin().to_degrees().clamp(1.0, 60.0)
    } else {
        15.0 // fallback
    };

    // 7. 坡度（不可派生，使用默认）
    let bank_deg = params.default_max_bank_deg;
    let phys_min_radius_m = v * v / (9.81 * bank_deg.to_radians().tan());

    let opts = SmoothOptions {
        aircraft_type: profile.aircraft_type,
        turn_radius_m,
        max_climb_deg,
        ..Default::default()
    };
    (opts, phys_min_radius_m.min(turn_radius_m))
}
```

### Step 2.4: 修改 solver.rs — 更新 profile 字段引用

需要更新所有引用旧字段的位置：

| 旧引用 | 新引用 | 位置 |
|--------|--------|------|
| `v.profile.ceiling_m` | `v.profile.maximum_altitude_m` | solver.rs:260,588,844,869,1353 |
| `v.profile.max_bank_deg` | 删除，直接用 `params_merged.default_max_bank_deg` | solver.rs:1760-1761 |
| `v.profile.cruise_speed_mps` | 删除，用新派生逻辑 | solver.rs:1765,1939 |
| `v.profile.speed_range_mps` | 删除 | solver.rs:1766 |
| `v.profile.max_climb_angle_deg` | 删除，用 `opts.max_climb_deg` | solver.rs:1874,1882 |

关键修改：
- `spec_climb` 构建（行 256-261）：`s.profile.ceiling_m` → `s.profile.maximum_altitude_m`
- 降速提示（行 1755-1781）：重写，使用新派生逻辑
- 终点段爬升（行 1867-1883）：使用 `opts.max_climb_deg` 替代 `profile.max_climb_angle_deg`

### Step 2.5: 修改 validate_aircraft

`cli/src/config.rs:619-658`:

```rust
fn validate_aircraft(a: &AircraftInput) -> Result<(), AppError> {
    let p = &a.profile;
    // 最大速度校验
    if let Some(s) = p.maximum_speed_mps {
        if !(1.0..=2000.0).contains(&s) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    // 最大转弯角速率校验
    if let Some(r) = p.maximum_turn_rate_dps {
        if !(0.1..=360.0).contains(&r) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    // 最大爬升率校验
    if let Some(c) = p.maximum_climb_rate_mps {
        if !(0.1..=500.0).contains(&c) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    // 最大高度校验
    if let Some(a) = p.maximum_altitude_m {
        if !(100.0..=30000.0).contains(&a) {
            return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
        }
    }
    // 武器射程校验（保留）
    if let Some(w) = &a.weapon {
        if let Some([lo, hi]) = w.range_km {
            if !(lo.is_finite() && hi.is_finite() && lo < hi) {
                return Err(AppError::InputInvalid(InputInvalidReason::OutOfBounds));
            }
        }
    }
    Ok(())
}
```

### Step 2.6: 修改 help.rs — 更新参数树

更新 `build_request_body()` 中 profile 节点：

```rust
ParamNode {
    name: "profile",
    type_label: "object",
    required: false,
    description: "机型性能参数",
    children: &[
        ParamNode { name: "aircraft_type", type_label: "string", required: true, description: "FIXED_WING / ROTORCRAFT", children: &[] },
        ParamNode { name: "maximum_speed_mps", type_label: "f64", required: false, description: "最大速度 m/s", children: &[] },
        ParamNode { name: "maximum_turn_rate_dps", type_label: "f64", required: false, description: "最大转弯角速率 °/s", children: &[] },
        ParamNode { name: "maximum_climb_rate_mps", type_label: "f64", required: false, description: "最大爬升率 m/s", children: &[] },
        ParamNode { name: "maximum_altitude_m", type_label: "f64", required: false, description: "最大飞行高度 m", children: &[] },
    ],
},
```

### Step 2.7: 更新所有测试 JSON 文件

转换规则：
```json
// 旧格式
"profile": {
    "aircraft_type": "FIXED_WING",
    "cruise_speed_mps": 250,
    "min_turn_radius_m": 442,
    "max_climb_angle_deg": 15,
    "max_bank_deg": 30,
    "ceiling_m": 15000
}

// 新格式（所有字段可选，可全部省略走默认）
"profile": {
    "aircraft_type": "FIXED_WING"
}
// 或指定部分参数
"profile": {
    "aircraft_type": "FIXED_WING",
    "maximum_speed_mps": 600,
    "maximum_turn_rate_dps": 20
}
```

### Step 2.8: 更新 smooth.rs 单元测试

`cli/src/smooth.rs:2230-2290` — 更新测试用的 AircraftProfile 构造：

```rust
// 旧
AircraftProfile {
    aircraft_type: AircraftType::FixedWing,
    cruise_speed_mps: Some(250.0),
    min_turn_radius_m: Some(442.0),
    ..Default::default()
}
// 新
AircraftProfile {
    aircraft_type: AircraftType::FixedWing,
    maximum_speed_mps: Some(600.0),
    maximum_turn_rate_dps: Some(20.0),
    ..Default::default()
}
```

### Step 2.9: 验证

```bash
cargo test --lib
cargo test --test crash_suite
cargo test --test determinism
cargo test --test regress_phase0
cargo check --workspace --all-targets
```

### Step 2.10: 提交

```bash
git add -A
git commit -m "refactor(cli): 飞机性能参数重构为 maximum_speed/turn_rate/climb_rate/altitude

BREAKING CHANGE: AircraftProfile 字段全面替换——删除 cruise_speed_mps/speed_range_mps/
min_turn_radius_m/max_climb_angle_deg/max_bank_deg/ceiling_m，新增 maximum_speed_mps/
maximum_turn_rate_dps/maximum_climb_rate_mps/maximum_altitude_m（全部可选，有默认值）。"
```

---

## Task 3: Terrain 数据索引

**目标**: 引入 `data/index.yaml` 索引文件，terrain 输入从文件路径改为索引 id，help 动态展示候选项。

**Files:**
- Modify: `cli/Cargo.toml` — 新增 `serde_yaml_neo` 依赖
- Create: `data/index.yaml` — 数据索引文件（不提交 git）
- Modify: `cli/src/config.rs` — TerrainConfig 结构体
- Modify: `cli/src/solver.rs` — terrain 加载逻辑
- Modify: `cli/src/help.rs` — terrain 参数树（动态候选项）
- Modify: `cli/src/main.rs` — 索引加载 + 传递给 help/solver
- Modify: `cli/tests/*.rs` — JSON 输入更新
- Modify: `cli/tests/regression/cases/*.json` — 全部 JSON 文件
- Modify: `demo/server/src/main.rs` — terrain 处理更新
- Modify: `docs/08-地形数据源.md` — 文档更新

**Interfaces:**
- Produces: `TerrainConfig { arpack: Option<String>, mask: Option<String> }`（索引 id）
- Produces: `TerrainIndex` 结构体（加载 index.yaml）
- Produces: `help::print_help()` 新增 terrain 候选参数

### Step 3.1: 新增 serde_yaml_neo 依赖

`cli/Cargo.toml`:

```toml
serde_yaml_neo = "0.11"
```

验证 `cargo tree -e normal` 无 C 依赖。

### Step 3.2: 创建 data/index.yaml

```yaml
arpacks:
  - id: east_asia
    desc: 东亚 7.5角秒地形
    file: east_asia_7p5as.arpack
  - id: global
    desc: GMTED2010 全球 7.5角秒
    file: gmted2010_7p5as_global.z19.arpack

masks:
  - id: east_asia
    desc: 东亚海陆掩膜
    file: east_asia_7p5as.mask
  - id: global
    desc: 全球海陆掩膜
    file: mask_7p5as.mask
```

确保 `data/` 在 `.gitignore` 中（已存在）。

### Step 3.3: 新增 TerrainIndex 结构体

`cli/src/config.rs`（或新建 `cli/src/terrain_index.rs`）:

```rust
use serde::Deserialize;

#[derive(Debug, Clone, Deserialize)]
pub struct TerrainIndexEntry {
    pub id: String,
    pub desc: String,
    pub file: String,
}

#[derive(Debug, Clone, Deserialize)]
pub struct TerrainIndex {
    #[serde(default)]
    pub arpacks: Vec<TerrainIndexEntry>,
    #[serde(default)]
    pub masks: Vec<TerrainIndexEntry>,
}

impl TerrainIndex {
    /// 从 data/index.yaml 加载索引
    pub fn load(data_dir: &std::path::Path) -> Option<Self> {
        let index_path = data_dir.join("index.yaml");
        let content = std::fs::read_to_string(&index_path).ok()?;
        serde_yaml_neo::from_str(&content).ok()
    }

    /// 根据 id 查找 arpack 文件路径
    pub fn find_arpack(&self, id: &str) -> Option<std::path::PathBuf> {
        self.arpacks.iter().find(|e| e.id == id).map(|e| std::path::PathBuf::from(&e.file))
    }

    /// 根据 id 查找 mask 文件路径
    pub fn find_mask(&self, id: &str) -> Option<std::path::PathBuf> {
        self.masks.iter().find(|e| e.id == id).map(|e| std::path::PathBuf::from(&e.file))
    }
}
```

### Step 3.4: 修改 TerrainConfig 结构体

`cli/src/config.rs:203-222`:

```rust
#[derive(Debug, Clone, Deserialize, Default, JsonSchema)]
#[serde(deny_unknown_fields)]
pub struct TerrainConfig {
    /// arpack 数据索引 id（可选）
    #[serde(default)]
    pub arpack: Option<String>,
    /// mask 数据索引 id（可选）
    #[serde(default)]
    pub mask: Option<String>,
}
```

删除 `TerrainSourceType` 枚举、`source`、`path`、`mask_path` 字段。

### Step 3.5: 修改 solver.rs — terrain 加载逻辑

`solver.rs:138-206`:

重写 terrain 加载逻辑，从 `TerrainIndex` 解析路径：

```rust
let terrain: TerrainHandle = {
    // 1. 尝试加载索引
    let data_dir = find_data_dir(); // 从 exe 同级或 workspace 查找 data/
    let index = data_dir.as_deref().and_then(TerrainIndex::load);

    // 2. 解析 arpack 路径
    let arpack_path = match (&input.terrain.arpack, &index) {
        (Some(id), Some(idx)) => idx.find_arpack(id).map(|p| resolve_data_path(&data_dir, &p)),
        (Some(_), None) => {
            terrain_warnings.push("terrain 索引文件不存在（data/index.yaml），无法加载地形".into());
            None
        }
        (None, _) => None,
    };

    // 3. 解析 mask 路径
    let mask_path = match (&input.terrain.mask, &index) {
        (Some(id), Some(idx)) => idx.find_mask(id).map(|p| resolve_data_path(&data_dir, &p)),
        (Some(_), None) => {
            terrain_warnings.push("terrain 索引文件不存在（data/index.yaml），无法加载掩膜".into());
            None
        }
        (None, _) => None,
    };

    // 4. 加载地形（复用现有逻辑）
    match arpack_path {
        Some(p) => {
            let ext = p.extension().and_then(|e| e.to_str()).unwrap_or("").to_ascii_lowercase();
            let inner = match ext.as_str() {
                "arpack" | "zstd" => BuiltinSource::open(&p).map(InnerSource::Builtin),
                _ => crate::terrain::open_source(&p).map(InnerSource::Dyn),
            };
            // ... mask 包装逻辑（同现有）...
        }
        None => TerrainHandle::None,
    }
};
```

### Step 3.6: 修改 help.rs — 动态候选项

`cli/src/help.rs`:

修改 `print_help()` 签名，接收 `TerrainIndex` 参数：

```rust
pub fn print_help(bin: &str, version: &str, index: Option<&TerrainIndex>) {
    // ... 现有逻辑 ...
    // terrain 节点根据 index 动态生成
    let terrain_node = build_terrain_node(index);
    // ...
}
```

`build_terrain_node()` 实现：

```rust
fn build_terrain_node(index: Option<&TerrainIndex>) -> ParamNode {
    match index {
        Some(idx) => {
            // 动态生成候选项描述
            let arpack_desc = if idx.arpacks.is_empty() {
                "无可用 arpack".to_string()
            } else {
                let items: Vec<String> = idx.arpacks.iter()
                    .map(|e| format!("{}({})", e.id, e.desc))
                    .collect();
                items.join(" / ")
            };
            // 类似生成 mask_desc
            // 返回带候选项描述的 ParamNode
        }
        None => {
            // 索引文件不存在
            ParamNode {
                name: "terrain",
                type_label: "object",
                required: false,
                description: "地形配置（当前无数据文件，无法提供支持）",
                children: &[],
            }
        }
    }
}
```

### Step 3.7: 修改 main.rs — 索引加载

`cli/src/main.rs`:

```rust
fn main() {
    // ... 现有逻辑 ...

    // 加载 terrain 索引（用于 help 输出）
    let data_dir = find_data_dir();
    let terrain_index = data_dir.as_deref().and_then(TerrainIndex::load);

    // help 拦截
    if args.len() <= 1 || args.get(1).map(|s| s.as_str()) == Some("help") {
        help::print_help(&bin, &version, terrain_index.as_ref());
        return;
    }

    // plan 执行时，TerrainIndex 传递给 solver（通过 Input 或 SolveParams）
}
```

### Step 3.8: 更新所有测试 JSON 文件

转换规则：
```json
// 旧格式
"terrain": {
    "source": "path",
    "path": "data/east_asia_7p5as.arpack",
    "mask_path": "data/east_asia_7p5as.mask"
}

// 新格式
"terrain": {
    "arpack": "east_asia",
    "mask": "east_asia"
}

// 无地形：删除 terrain 字段或设为默认
"terrain": {}
```

注意：测试中如果 data 目录不可用，需要 mock 或跳过 terrain 相关测试。

### Step 3.9: 修改 demo-server

`demo/server/src/main.rs`:

- 更新 `/api/data-files` 端点：返回 index.yaml 内容（如果存在）
- 更新 `/api/plan` 端点：terrain 路径解析改为从 index 查找
- 更新地形加载逻辑：从 `TerrainIndex` 解析路径

### Step 3.10: 更新文档

- `docs/06-输入输出契约.md` — 更新 terrain 契约
- `docs/08-地形数据源.md` — 更新 terrain 文档
- `CHANGELOG.md` — 记录变更

### Step 3.11: 验证

```bash
cargo test --lib
cargo test --test crash_suite
cargo test --test determinism
cargo test --test regress_phase0
cargo check --workspace --all-targets
scripts/check.sh --quick
```

### Step 3.12: 提交

```bash
git add -A
git commit -m "refactor(cli): terrain 输入从文件路径改为索引 id

BREAKING CHANGE: terrain 配置从 source/path/mask_path 改为 arpack/mask（索引 id）。
新增 data/index.yaml 索引文件机制，help 动态展示可用地形候选项。"
```

---

## Task 4: 最终验证与发版准备

**目标**: 全量测试、文档更新、CHANGELOG 更新。

### Step 4.1: 全量测试

```bash
cargo test --lib
cargo test --test crash_suite
cargo test --test determinism
cargo test --test regress_phase0
cargo test --test regress_phase0 -- --ignored  # 如果有忽略的测试
scripts/check.sh --quick
```

### Step 4.2: 更新 CHANGELOG.md

```markdown
## [Unreleased]

### Changed
- **Breaking**: 三类 Zone（no_fly_zones/restricted_zones/obstacles）合并为统一 zones 数组，需指定 zone_type。
- **Breaking**: 飞机性能参数重构——删除 cruise_speed_mps/speed_range_mps/min_turn_radius_m/max_climb_angle_deg/max_bank_deg/ceiling_m，新增 maximum_speed_mps/maximum_turn_rate_dps/maximum_climb_rate_mps/maximum_altitude_m（全部可选，有现代战机默认值）。
- **Breaking**: terrain 配置从 source/path/mask_path 改为 arpack/mask（索引 id），新增 data/index.yaml 索引机制。
- help 输出动态展示可用地形候选项（基于索引文件）。
```

### Step 4.3: 更新文档

- `docs/06-输入输出契约.md` — 完整更新
- `docs/08-地形数据源.md` — 完整更新
- `docs/设计与实现差异对照表.md` — 更新差异

### Step 4.4: 提交

```bash
git add -A
git commit -m "docs: 更新输入契约和地形文档，同步 Schema 变更"
```

---

## 依赖变更检查

新增依赖：`serde_yaml_neo = "0.11"`

```bash
cargo tree -e normal | grep -iE "openblas|zlib|curl|proj|gdal|pcre|ssl"
# 应无输出
```

## 回归风险

| 风险 | 影响 | 缓解 |
|------|------|------|
| 34 个 JSON 文件手动转换出错 | 测试失败 | 脚本化批量转换 + 逐个验证 |
| solver 中 profile 字引遗漏 | 编译失败 | 编译器会捕获所有未更新的字段引用 |
| terrain 索引路径解析 | 测试降级 | 测试中 data 不存在时降级为无地形 |
| smooth_options_for 派生逻辑 | 路径质量变化 | 保留默认值表，确保向后兼容 |
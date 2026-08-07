//! 端到端 solver（Phase 4 M1）：把 Phase 0-3 库层组件串成主流程。
//!
//! parse → validate（main）→ TerrainSource（ARPK1/无地形）→ 语义代价场
//! （Land/Water/NoData/NODATA 5x + Zone 硬墙 INF）→ FMM → 回溯 → 平滑链
//! （Theta* 去锯齿 + 抽稀 + Dubins 拟合 + 复验门）→ VehicleOutput 契约。
//!
//! M1 范围（主管拍板 2026-08-05）：单机/多机（共享代价场，每机独立 FMM）；
//! Zone 水平几何禁入（高度层 M2）；SmoothOptions 默认值（VehicleProfile 派生 M4）；
//! 无雷达代价（M3）；target_ref 暂按 mission.target（M5 每机 waypoints）。

use std::path::PathBuf;
use std::time::Instant;

use crate::config::{
    zone_contains, zone_contains_at, Input, Output, PathPoint, Stats, TerrainSourceType,
    VehicleOutput, Zone, ZoneShape,
};
use crate::coord::Geo;
use crate::costfield::{backtrack_path, build_semantic_cost_field, fmm_propagate};
use crate::error::{AppError, InputInvalidReason};
use crate::path::{Path, PathPoint as RouterPoint};
use crate::smooth::{default_chain, smooth_path_chain, VerifyContext};
use crate::spatial::{CircleEntry, CircleIndex};
use crate::terrain::builtin::BuiltinSource;
use crate::terrain::{Sample, TerrainSource};
use crate::threat::{SphericalRadarThreat, ThreatModel, ThreatParams};

/// 解算参数（M1：地形路径 CLI/输入指定；grid 粗网格分辨率）。
#[derive(Debug, Clone)]
pub struct SolveParams {
    pub terrain_path: Option<PathBuf>,
    pub grid: usize,
}

impl Default for SolveParams {
    fn default() -> Self {
        Self {
            terrain_path: None,
            grid: 256,
        }
    }
}

/// 解算任务区域（方形经纬度包围盒 + 缓冲）。
#[derive(Debug, Clone, Copy)]
struct Region {
    min_lon: f64,
    min_lat: f64,
    span_deg: f64,
}

/// 待解算的车辆规格（多机或默认单机）。
struct VehicleSpec {
    id: String,
    start: Geo,
    alt_m: f64,
    /// 机型配置（Phase 4 M4：平滑参数派生输入）。
    profile: crate::config::VehicleProfile,
    /// 中途必经点（Phase 4 M5：start → mid[0..] → target 分段拼接）。
    mid_waypoints: Vec<Geo>,
}

/// 端到端解算。elapsed_ms 为端到端耗时（main 计时传入）。
pub fn solve(input: &Input, params: &SolveParams, elapsed_ms: u64) -> Result<Output, AppError> {
    // 1. 地形源（none = 无地形平面；path/builtin = ARPK1 文件）
    let terrain: Option<Box<dyn TerrainSource>> = match input.mission.terrain.source {
        TerrainSourceType::None => None,
        _ => {
            let p = params
                .terrain_path
                .clone()
                .or_else(|| input.mission.terrain.path.clone().map(PathBuf::from));
            match p {
                Some(p) => Some(Box::new(BuiltinSource::open(&p)?)),
                None => {
                    // 主管决策 2026-08-05：默认低精度地形 = 量化中国数据（china_dem_l12.arpack）。
                    // 候选：exe 同目录 / exe 上溯 workspace 根 / 工作目录相对路径。
                    let mut candidates = vec![
                        PathBuf::from("china_dem_l12.arpack"),
                        PathBuf::from("phase0/data/pending/china_dem_l12.arpack"),
                        PathBuf::from("../phase0/data/pending/china_dem_l12.arpack"),
                    ];
                    if let Ok(exe) = std::env::current_exe() {
                        if let Some(dir) = exe.parent() {
                            candidates.insert(0, dir.join("china_dem_l12.arpack"));
                            // 上溯 3 层（target/release → target → workspace 根），逐层试开发路径
                            for anc in dir.ancestors().skip(1).take(3) {
                                candidates.push(anc.join("phase0/data/pending/china_dem_l12.arpack"));
                            }
                        }
                    }
                    if let Some(c) = candidates.iter().find(|c| c.exists()) {
                        Some(Box::new(BuiltinSource::open(c)?))
                    } else {
                        return Err(AppError::Data(
                            "terrain.source=path/builtin 但未提供地形文件，且默认低精度地形 \
                             (china_dem_l12.arpack) 未找到（--terrain / terrain.path / exe 同目录 / phase0/data/pending）"
                                .into(),
                        ));
                    }
                }
            }
        }
    };

    // 2. 车辆规格（vehicles 空 → 默认单机：mission.start → mission.target）
    let specs: Vec<VehicleSpec> = if input.mission.vehicles.is_empty() {
        vec![VehicleSpec {
            id: "v1".into(),
            start: input.mission.start.to_geo()?,
            alt_m: input.mission.start.alt_m,
            profile: crate::config::VehicleProfile::default(),
            mid_waypoints: Vec::new(),
        }]
    } else {
        input
            .mission
            .vehicles
            .iter()
            .map(|v| {
                let mid = v
                    .mid_waypoints
                    .iter()
                    .map(|w| {
                        Geo::new(w.lon, w.lat)
                            .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))
                    })
                    .collect::<Result<Vec<_>, AppError>>()?;
                Ok(VehicleSpec {
                    id: v.id.clone(),
                    start: Geo::new(v.start_pose.lon, v.start_pose.lat)
                        .map_err(|_| AppError::InputInvalid(InputInvalidReason::IllegalCoordinate))?,
                    alt_m: v.start_pose.alt_m,
                    profile: v.profile.clone(),
                    mid_waypoints: mid,
                })
            })
            .collect::<Result<Vec<_>, AppError>>()?
    };

    // 3. 任务区域（所有起点 + target 包围盒 + 缓冲）
    let target = input.mission.target.to_geo()?;
    let region = region_of(&specs, &target);
    // 3b. 网格自适应（主管 2026-08-06 双大雷达/多边形场景）：**仅大区域**（span > 2.5°）时
    // 固定 256 格 → 格距粗 → FMM 绕行弧锯齿曲率 < 物理转弯半径 → 平滑链转弯半径
    // verify 拒 → 回退锯齿。
    // 自适应格距：**含多边形墙（NoFly/Obstacle 多边形）→ ≤600m**（绕多边形尖角曲率
    // 敏感：顶点处绕弧曲率 ≈ 膨胀距离，锯齿误差 2% 即不足，实测 zigzag7 grid512
    // cell 0.72km 失败 / grid600 cell 0.64km 成功）；纯圆墙 → ≤1100m（绕圆弧曲率 =
    // 墙半径，不敏感，zigzag5 grid300 即通过）。
    // 小区域（≤2.5°）保持默认 grid——细网格会让 5c2 软罚带物理宽度变窄 → FMM 贴墙
    // 更近 → 绕行 clearance 余量不足（双禁飞区 real_bad 256 成功、301 失败）。
    // 上限 1024 防 OOM。
    let span_km = region.span_deg * 111.32;
    let has_poly_wall = input
        .mission
        .no_fly_zones
        .iter()
        .chain(input.mission.restricted_zones.iter())
        .chain(input.mission.obstacles.iter())
        .any(|z| z.is_wall() && matches!(z.shape, ZoneShape::Polygon { .. }));
    let target_cell = if has_poly_wall { 600.0 } else { 1100.0 };
    let auto_grid = if region.span_deg > 2.5 {
        ((span_km * 1000.0) / target_cell).ceil() as usize
    } else {
        0
    };
    let grid = params.grid.max(8).max(auto_grid).min(1024);
    eprintln!("[debug] region span={:.2}deg grid={} cell_m={:.0}", region.span_deg, grid, region.span_deg * 111_320.0 / grid as f64);

    // 4. Zone 集合（no_fly + restricted + obstacles）
    //    代价场墙策略（M2 高度层）：
    //    - NoFly/Obstacle → 全高度水平墙（代价场 INF）——保守禁入；
    //    - Restricted → 不画墙（高度区间外可穿越），由 Theta* check + verify 高度判定。
    let all_zones: Vec<Zone> = input
        .mission
        .no_fly_zones
        .iter()
        .chain(input.mission.restricted_zones.iter())
        .chain(input.mission.obstacles.iter())
        .cloned()
        .collect();
    let nofly = circle_index(&all_zones.iter().collect::<Vec<_>>());

    // 4b. 参数合并 + 禁飞区膨胀距离（主管 2026-08-06：绕飞太贴边→考虑飞机机动）。
    //     物理转弯半径 r = v²/(g·tanφ)（与 smooth_options_for 同式）；绕行弧需要 ≥r 的
    //     转弯空间，把 NoFly/Obstacle 硬墙向外膨胀 max(0.5×r)（clamp [2km, 10km]）——
    //     FMM 绕行自然远离边界，Dubins 转弯弧留足空间（不再因贴边急弯被物理复验拒绝）。
    let params_merged = crate::config::DefaultParams::default().merge(&input.mission.parameters);
    let mut degradations = Vec::new();
    radar_param_degradations(input, &mut degradations);
    let inflation_m = specs
        .iter()
        .map(|v| {
            let (_opts, phys) = crate::smooth::smooth_options_for(&v.profile, &params_merged);
            (phys * 0.5).clamp(2_000.0, 10_000.0)
        })
        .fold(0.0f64, f64::max);
    let inflation_km = inflation_m / 1000.0;

    // 5. 语义代价场（Land=1 / Water=1 / Lake=1 / NoData=5 / OOB=INF；NoFly/Obstacle 墙）
    let mut field = build_semantic_cost_field(grid, grid, |r, c| {
        let (lon, lat) = cell_lonlat(r, c, &region, grid);
        if let Ok(g) = Geo::new(lon, lat) {
            if all_zones
                .iter()
                .any(|z| z.is_wall() && zone_contains(z, &g))
            {
                return Sample::OutOfBounds;
            }
        }
        match &terrain {
            Some(t) => t.sample_at(lon, lat),
            None => Sample::Land(0.0),
        }
    }, 5.0);

    // 5c. 禁飞区墙向外膨胀 + 过渡带软罚（见 apply_inflation_and_band）
    let cell_m = region.span_deg * 111_320.0 / grid as f64;
    let inflation_cells = (inflation_m / cell_m.max(1.0)).ceil() as usize;
    apply_inflation_and_band(&mut field, inflation_cells, cell_m);

    // 5b. 雷达静态代价（Phase 4 M3）：膨胀半径内 cost ×(1+coef·(几何并集概率 + 深穿惩罚))
    //     ——FMM 倾向绕行；LOS 动态遮挡由 verify 威胁评估判定（不进静态代价场）。
    //     coef = radar_cost_coef（默认 200）。几何深穿惩罚（u<1 时 ×(1+coef·(1-u))，
    //     探测区外 u≥1 无几何项）：确保穿探测区明确绕行——主管 2026-08-06：
    //     并排双雷达不得直穿探测区（即使 P_cross 调高，几何绕行与验收阈值解耦）。
    let threat_params = radar_threat_params(&params_merged);
    let threat = SphericalRadarThreat::new(&input.mission.red_forces.radars, threat_params.clone());
    if !input.mission.red_forces.radars.is_empty() {
        for r in 0..grid {
            for c in 0..grid {
                let (lon, lat) = cell_lonlat(r, c, &region, grid);
                let p = threat.static_union_probability(lon, lat);
                if p > 0.0 {
                    let idx = r * grid + c;
                    if field.cost[idx].is_finite() {
                        let u = threat.static_penetration(lon, lat, 0.0);
                        let geom = if u < 1.0 { 1.0 - u } else { 0.0 };
                        field.cost[idx] *= (1.0 + params_merged.radar_cost_coef * (p + geom)) as f32;
                    }
                }
            }
        }
    }

    // 6. 每机：分段 FMM（start → mid[0..] → target，共享代价场）→ 拼接 → 平滑 → 输出
    let mut out_vehicles = Vec::new();
    let mut fmm_ms = 0.0f64;
    'veh: for v in &specs {
        // 段序列：起点 + 必经点 + 目标
        let mut seg_ends: Vec<Geo> = Vec::with_capacity(v.mid_waypoints.len() + 2);
        seg_ends.push(v.start);
        seg_ends.extend(v.mid_waypoints.iter().copied());
        seg_ends.push(target);
        // 机型平滑参数提前（受限区剖面需要 max_climb：决定下降/爬升距离）
        let (opts, phys_min_radius_m) = crate::smooth::smooth_options_for(&v.profile, &params_merged);
        // 受限区墙（剖面直穿语义，主管 2026-08-06 二轮+三轮）：飞行高度落在 restricted
        // 高度区间内 → 比较底部穿行 / 顶部绕飞（底部可行恒更优，否则顶部）→ 可行则
        // 不画墙，FMM 直穿后由 build_restricted_profiles 沿 raw 路径生成剖面；两者都
        // 不可行（地形过高且超升限 / 太贴边 / 多边形）→ 画墙水平绕行（fallback 保底）；
        // 高度在区间外（如低于 alt_min_m 的"底部通道"）→ 不画墙直穿（可通行）。
        // 三轮架构增强（主管 2026-08-06 指定实现）：不再因"start→target 直线穿硬墙"
        // 而画 restricted 墙——FMM 只绕硬墙（no_fly/obstacle），restricted 直穿后在
        // build 内沿 raw 轨迹剖面（绕硬墙弧 + 圆内弦，即"先绕 no_fly 再剖面直穿"的
        // 组合机动）；若剖面段（desc→in→out→climb）仍穿硬墙 → need_wall → 第二轮
        // FMM 画 restricted 墙水平绕行兜底。
        let mut smooth_src: Vec<Path> = Vec::new();
        let mut profile_mask: Vec<bool> = Vec::new();
        let mut raw_joined: Path = Path::new(Vec::new());
        let mut force_restricted_wall = false;
        'fmm_attempt: for _attempt in 0..2 {
            let restricted_wall_for = |z: &Zone| {
                force_restricted_wall
                    || restricted_detour_required(
                        z,
                        v.alt_m,
                        v.profile.ceiling_m,
                        terrain.as_deref(),
                        &v.start,
                        &target,
                        opts.max_climb_deg,
                    )
            };
            let veh_field: Option<crate::costfield::CostField> = if all_zones
                .iter()
                .any(|z| restricted_wall_for(z))
            {
                let mut f = field.clone();
                let g = f.rows;
                for r in 0..g {
                    for c in 0..g {
                        let (lon, lat) = cell_lonlat(r, c, &region, g);
                        if let Ok(gg) = Geo::new(lon, lat) {
                            if all_zones
                                .iter()
                                .any(|z| restricted_wall_for(z) && zone_contains(z, &gg))
                            {
                                f.cost[r * g + c] = f32::INFINITY;
                            }
                        }
                    }
                }
                apply_inflation_and_band(&mut f, inflation_cells, cell_m);
                Some(f)
            } else {
                None
            };
            let field_ref = veh_field.as_ref().unwrap_or(&field);
            eprintln!("[debug] fmm attempt {} field ready (veh={})", _attempt, veh_field.is_some());
            // 逐段 FMM → 回溯 → 拼接（去重段端点）
            let mut raw_segs: Vec<Path> = Vec::new();
            let mut no_solution = false;
            for seg in seg_ends.windows(2) {
                let (s, e) = (seg[0], seg[1]);
                let (sr, sc) = lonlat_cell(s.lon, s.lat, &region, grid);
                let (dr, dc) = lonlat_cell(e.lon, e.lat, &region, grid);
                let t0 = Instant::now();
                let res = fmm_propagate(field_ref, sr, sc);
                fmm_ms += t0.elapsed().as_secs_f64() * 1000.0;
                let Some(mut cells) = backtrack_path(field_ref, &res, dr, dc, sr, sc) else {
                    no_solution = true;
                    break;
                };
                // backtrack 返回 dst→src 顺序 → 反转为 src→dst（路径语义）
                cells.reverse();
                raw_segs.push(Path::new(
                    cells
                        .iter()
                        .map(|&(r, c)| {
                            let (lon, lat) = cell_lonlat(r, c, &region, grid);
                            RouterPoint::new(lon, lat, v.alt_m)
                        })
                        .collect(),
                ));
            }
            if no_solution || raw_segs.is_empty() {
                out_vehicles.push(VehicleOutput {
                    id: v.id.clone(),
                    status: "no_solution".into(),
                    path: Vec::new(),
                    distance_m: 0.0,
                    warnings: vec!["coarse FMM no path".into()],
                });
                continue 'veh;
            }
            // 段端点（必经点/目标）是硬约束：任何平滑不得移除
            raw_joined = join_paths(&raw_segs);

            // 受限区底部/顶部剖面切分（沿 raw 路径；剖面段跳过平滑链）
            smooth_src.clear();
            profile_mask.clear();
            let mut need_wall = false;
            for seg in &raw_segs {
                let (sub, mask, nw) = build_restricted_profiles(
                    seg,
                    &all_zones,
                    v.alt_m,
                    opts.max_climb_deg,
                    v.profile.ceiling_m,
                    terrain.as_deref(),
                    &v.start,
                    &target,
                    inflation_m / 1000.0,
                );
                need_wall |= nw;
                smooth_src.extend(sub);
                profile_mask.extend(mask);
            }
            if need_wall {
                // 剖面段穿硬墙（如 no_fly 圆）→ 该 restricted 必须画墙水平绕行 → 第二轮重算
                force_restricted_wall = true;
                continue 'fmm_attempt;
            }
            break;
        }
        let mut warnings = Vec::new();
        let mut pts = raw_joined.points.clone();
        if pts.len() >= 2 {
            let check = make_segment_check(
                &all_zones,
                Some(&threat as &dyn crate::threat::ThreatModel),
                inflation_km,
            );
            let ctx = VerifyContext {
                terrain: terrain.as_deref(),
                nofly: Some(&nofly),
                zones: Some(&all_zones),
                threat: Some(&threat),
                zone_inflation_m: inflation_m,
            };
            // 每段独立平滑（首尾段端点保留——Theta* 截直不得移除必经点）。
            // 入口航向：前一段输出方向，约束当前段首跳（段边界转角，否则拼接后
            // 终检暴露——2026-08-07 主管 1755 点场景 seg3 out→climb 与 seg4
            // climb→A 夹角 61.94° > 60°，climb 是段首点单段 verify 无法发现）。
            let mut smooth_segs = Vec::new();
            let mut seg_warnings = Vec::new();
            let mut entry_heading: Option<f64> = None;
            for (idx, seg) in smooth_src.iter().enumerate() {
                if profile_mask[idx] {
                    // 受限区剖面段：已按 max_climb 生成下降/平飞/爬升，直接采用
                    smooth_segs.push(seg.clone());
                    entry_heading = seg.last_segment_heading();
                    continue;
                }
                let chain = default_chain(&opts, &check, entry_heading);
                let result = smooth_path_chain(seg, &chain, &opts, &ctx, Some(phys_min_radius_m));
                if let Some(w) = &result.warning {
                    seg_warnings.push(w.clone());
                }
                seg_warnings.extend(result.verify.warnings.iter().cloned());
                smooth_segs.push(result.path.clone());
                entry_heading = result.path.last_segment_heading();
            }
            // 拼接 + 全路径终检（段间转角/整路径威胁在拼接后才可见）
            let joined = join_paths(&smooth_segs);
            // 段端点 = 起点 + 必经点 + 目标（直线替代用；必经点硬约束，任何平滑不得移除）
            let mut straight_pts: Vec<crate::path::PathPoint> = Vec::new();
            for g in &seg_ends {
                let p = crate::path::PathPoint::new(g.lon, g.lat, v.alt_m);
                let dup = straight_pts.last().map_or(false, |q| {
                    (q.lon - p.lon).abs() < 1e-12 && (q.lat - p.lat).abs() < 1e-12
                });
                if !dup {
                    straight_pts.push(p);
                }
            }
            let straight = Path::new(straight_pts);
            let final_rep = crate::smooth::verify_path(
                &joined,
                None,
                &opts,
                &ctx,
                Some(phys_min_radius_m),
            );
            if final_rep.ok {
                pts = joined.points;
                warnings = seg_warnings.clone();
            } else {
                // 终检失败 → 回退未平滑拼接（必经点保留，宁丑勿违）
                pts = raw_joined.points;
                let msg = "smoothing_failed: no smoothed stage passed full verification";
                warnings.push(msg.into());
                degradations.push(msg.into());
                warnings.extend(final_rep.warnings.iter().cloned());
                // 空洞/代价场网格伪影兜底（空洞策略 2026-08-04：可用结果 + 降级警告进
                // stats.degradations）：FMM 对 NoData 5x 代价区域绕行 → raw 是密集网格
                // 楼梯（本场景绕渤海 NoData 44km 侧偏，1138km vs 直线 784km）→ Theta*
                // 拉直被弦高门（相对 raw 100m）拒绝 → 全链失败回退楼梯。若 raw 显著长于
                // 分段直线（网格伪影而非真实绕障）且直线通过完整几何复验（不穿硬墙/不超
                // 机动/净空满足，NoData 已降级为警告）→ 交付直线（必经点保留）+ 降级警告。
                let cur_dist = Path::new(pts.clone()).length_m();
                // 不穿雷达深区（≥0.7×有效半径）才走通用兜底——穿雷达由下方雷达专用
                // 直线直穿替代处理（主管 2026-08-05 拍板语义，保持雷达行为不变）。
                let threat_ok = straight.points.iter().all(|p| {
                    threat.static_penetration(p.lon, p.lat, p.alt_m) >= 0.7
                });
                if straight.points.len() >= 2
                    && cur_dist > straight.length_m() * 1.05 + 1_000.0
                    && threat_ok
                {
                    let rep_s = crate::smooth::verify_path(
                        &straight,
                        None,
                        &opts,
                        &ctx,
                        Some(phys_min_radius_m),
                    );
                    if rep_s.ok {
                        pts = straight.points.clone();
                        // 直线替代成功 → 最终交付已平滑，撤销 smoothing_failed 误报
                        warnings.retain(|w| !w.starts_with("smoothing_failed"));
                        degradations.retain(|d| !d.starts_with("smoothing_failed"));
                        warnings.extend(rep_s.warnings.iter().cloned());
                        let msg2 = format!(
                            "raw FMM grid artifact: straight-line transit adopted (terrain NoData degraded)"
                        );
                        if !degradations.contains(&msg2) {
                            degradations.push(msg2.clone());
                        }
                        warnings.push(msg2);
                    }
                }
            }
            // 雷达 degradation：从终检 issues + 终检 warnings + 段警告提取（雷达软约束，去重）
            for i in final_rep
                .issues
                .iter()
                .chain(final_rep.warnings.iter())
                .chain(seg_warnings.iter())
            {
                if i.contains("radar") && !degradations.contains(i) {
                    degradations.push(i.clone());
                }
            }
            // 雷达避不开 → 直线直穿替代（主管 2026-08-05 锯齿问题修复）：
            // FMM 直穿雷达区时 Theta* 拒绝拉直（check 穿雷达=false）→ 交付网格锯齿；
            // 若整路径探测概率仍超阈值 且 距离显著大于直线（锯齿是网格伪影而非真实绕行）
            // → 用分段直线直穿（必经点保留，最短暴露时长）。直线需过几何复验（防穿山/超机动）。
            if let Some(tm) = ctx.threat {
                let rep_now = tm.evaluate(&Path::new(pts.clone()), ctx.terrain);
                // 直穿判定：路径某点深入任一雷达有效半径 70% 以内（与 Theta* 深探测
                // DEEP_RATIO 一致）才视为"避不开的直穿"；完全绕出（最近点 ≥ 0.7×半径）
                // 保持绕行，不替代。（P_cross 是验收阈值，不参与直穿判定——主管
                // 2026-08-06：航路必须绕开雷达探测区域，不得因调高 P_cross 而直穿。）
                let mut penetrates = false;
                for p in &pts {
                    if threat.static_penetration(p.lon, p.lat, p.alt_m) < 0.7 {
                        penetrates = true;
                        break;
                    }
                }
                if rep_now.over_threshold && penetrates {
                    let cur_dist = Path::new(pts.clone()).length_m();
                    if straight.points.len() >= 2
                        && cur_dist > straight.length_m() * 1.05 + 1_000.0
                    {
                        let rep_s = crate::smooth::verify_path(
                            &straight,
                            None,
                            &opts,
                            &ctx,
                            Some(phys_min_radius_m),
                        );
                        if rep_s.ok {
                            pts = straight.points.clone();
                            // 直线替代成功 → 最终交付已平滑，撤销 smoothing_failed 误报
                            warnings.retain(|w| !w.starts_with("smoothing_failed"));
                            degradations.retain(|d| !d.starts_with("smoothing_failed"));
                            let msg = format!(
                                "radar: unavoidable crossing -> straight-line transit (p {:.4})",
                                rep_now.cumulative_p
                            );
                            if !degradations.contains(&msg) {
                                degradations.push(msg.clone());
                            }
                            warnings.push(msg);
                        }
                    }
                }
            }
        }
        // NoData 退化汇总（空洞策略 2026-08-04：最坏降级警告进 stats.degradations）：
        // verify 对空洞只降级警告（不阻断），此处把沿途 NoData 采样汇总为一条 degradation。
        let nodata_n = warnings
            .iter()
            .filter(|w| w.contains("NoData terrain"))
            .count();
        if nodata_n > 0 {
            let msg = format!(
                "terrain: {nodata_n} NoData sample(s) along route, clearance unknown (degraded)"
            );
            if !degradations.contains(&msg) {
                degradations.push(msg);
            }
        }
        let dist = Path::new(pts.clone()).length_m();
        out_vehicles.push(VehicleOutput {
            id: v.id.clone(),
            status: "planned".into(),
            path: pts
                .iter()
                .map(|p| PathPoint {
                    x: p.lon,
                    y: p.lat,
                    alt_m: p.alt_m,
                })
                .collect(),
            distance_m: dist,
            warnings,
        });
    }

    Ok(Output {
        schema_version: crate::config::SCHEMA_VERSION.into(),
        status: "success".into(),
        error: None,
        elapsed_ms: Some(elapsed_ms),
        vehicles: out_vehicles,
        stats: Stats {
            fmm_ms,
            los_checks: 0,
            degradations,
        },
    })
}

// ==================== 辅助 ====================

/// 任务区域：所有起点 + target 的方形包围盒 + 0.15° 缓冲（保证源/目标不贴边）。
fn region_of(specs: &[VehicleSpec], target: &Geo) -> Region {
    let mut min_lon = f64::INFINITY;
    let mut max_lon = f64::NEG_INFINITY;
    let mut min_lat = f64::INFINITY;
    let mut max_lat = f64::NEG_INFINITY;
    for s in specs {
        min_lon = min_lon.min(s.start.lon);
        max_lon = max_lon.max(s.start.lon);
        min_lat = min_lat.min(s.start.lat);
        max_lat = max_lat.max(s.start.lat);
    }
    min_lon = min_lon.min(target.lon);
    max_lon = max_lon.max(target.lon);
    min_lat = min_lat.min(target.lat);
    max_lat = max_lat.max(target.lat);
    let pad = 0.15;
    min_lon -= pad;
    min_lat -= pad;
    let span = (max_lon - min_lon).max(max_lat - min_lat) + 2.0 * pad;
    Region {
        min_lon,
        min_lat,
        span_deg: span,
    }
}

fn cell_lonlat(r: usize, c: usize, region: &Region, grid: usize) -> (f64, f64) {
    let u = (c as f64 + 0.5) / grid as f64;
    let v = (r as f64 + 0.5) / grid as f64;
    (
        region.min_lon + u * region.span_deg,
        region.min_lat + v * region.span_deg,
    )
}

fn lonlat_cell(lon: f64, lat: f64, region: &Region, grid: usize) -> (usize, usize) {
    let c = (((lon - region.min_lon) / region.span_deg) * grid as f64) as usize;
    let r = (((lat - region.min_lat) / region.span_deg) * grid as f64) as usize;
    (r.min(grid - 1), c.min(grid - 1))
}

/// 圆形 zone → CircleIndex（smooth 复验禁飞包含用；zones 提供时 verify 不再用它）。
fn circle_index(zones: &[&Zone]) -> CircleIndex {
    let entries: Vec<CircleEntry> = zones
        .iter()
        .filter_map(|z| match &z.shape {
            ZoneShape::Circle { center, radius_km } => Some(CircleEntry {
                id: z.id.clone(),
                lon: center[0],
                lat: center[1],
                radius_m: radius_km * 1000.0,
            }),
            ZoneShape::Polygon { .. } => None,
        })
        .collect();
    CircleIndex::build(entries)
}

/// 雷达威胁参数（默认参数表落默认值；输入覆盖合并）。
/// base_p（中心探测概率）固定 0.1 占位，**与 p_cross 解耦**（主管 2026-08-05 反馈：
/// P_cross 是验收阈值——调高只放宽"容忍探测"的评估/拉直判定，不应把物理探测概率
/// 一起抬高导致代价爆炸 + 强绕行 + 锯齿；真实雷达参数标定（A2）后接入独立 base_p）。
fn radar_threat_params(d: &crate::config::DefaultParams) -> ThreatParams {
    ThreatParams {
        radar_inflation: d.radar_inflation,
        detection_curve: d.detection_curve,
        p_cross: d.p_cross,
        suppression_delta: d.suppression_delta,
        base_p: 0.1, // 解耦：固定占位（A2 标定后接入独立 base_p）
    }
}

/// 无效参数回落默认的降级报告（主管决策 2026-08-05：无外部参数或参数无效使用默认值）。
/// merge 已回落；此处把"输入无效"事实记入 stats.degradations 供验收可见。
fn radar_param_degradations(input: &Input, out: &mut Vec<String>) {
    let p = &input.mission.parameters;
    if let Some(v) = p.radar_inflation
        && !(v.is_finite() && v > 1.0)
    {
        out.push(format!("parameter radar_inflation={v} invalid -> default 1.2"));
    }
    if let Some(v) = p.p_cross
        && !(v.is_finite() && v >= 0.0 && v <= 1.0)
    {
        out.push(format!("parameter p_cross={v} invalid -> default 0.1"));
    }
    if let Some(v) = p.suppression_delta
        && !(v.is_finite() && v >= 0.0 && v < 1.0)
    {
        out.push(format!("parameter suppression_delta={v} invalid -> default 0.5"));
    }
    if let Some(v) = p.radar_cost_coef
        && !(v.is_finite() && v > 0.0)
    {
        out.push(format!("parameter radar_cost_coef={v} invalid -> default 200"));
    }
    if let Some(v) = p.los_mask_coef
        && !(v.is_finite() && v >= 0.0 && v <= 1.0)
    {
        out.push(format!("parameter los_mask_coef={v} invalid -> default 0.08"));
    }
    if let Some(s) = &p.detection_curve
        && !matches!(s.to_ascii_lowercase().as_str(), "exponential" | "linear")
    {
        out.push(format!("parameter detection_curve={s} invalid -> default exponential"));
    }
}

/// 拼接多段路径（去重相邻重复点；段端点保留——必经点/目标硬约束）。
fn join_paths(segs: &[Path]) -> Path {
    let mut pts: Vec<RouterPoint> = Vec::new();
    for seg in segs {
        for p in &seg.points {
            let dup = pts
                .last()
                .map(|q: &RouterPoint| (q.lon - p.lon).abs() < 1e-12 && (q.lat - p.lat).abs() < 1e-12)
                .unwrap_or(false);
            if !dup {
                pts.push(*p);
            }
        }
    }
    Path::new(pts)
}

/// Theta* 去锯齿段检查：直连 (a)→(b) 不穿任何 Zone（几何精确判定——
/// 多边形：线段与任一边相交或端点在内；圆形：段到圆心最近距离 ≤ 半径。
/// 含边界接触（保守拒绝）。此前为等距 16 点采样，斜切多边形的线段采样点
/// 可能恰好全部落在外部 → 拉直穿过禁飞区内部（主管 2026-08-06 梯形禁飞区
/// 航路从边缘穿过）；几何判定无采样漏判。
/// 高度层（M2）：NoFly/Obstacle 全高度水平墙（相交即拒）；Restricted 相交后
/// 按段高度采样判定（高度沿线段线性插值，区间外可穿越）。
/// 雷达威胁：直连"深穿"任一雷达（归一化深度 < 0.7，即深入有效半径 70% 以内）
/// → 拒绝拉直（保住 FMM 绕行决策——P_cross 只是验收阈值，不得因调高 P_cross
/// 而把绕行弧拉直成穿雷达区的直线；主管 2026-08-06：航路必须绕开雷达探测区域）。
/// **例外**：段两端点任一已在深区（目标/必经点本身在雷达探测区内，绕不开）→
/// 允许拉直（雷达软约束由 verify 记录；无条件拒绝会让最后接近段无法拉直 →
/// 交付 FMM 网格伪影，主管 2026-08-06 37 点场景）。
/// 低概率边缘（≥0.7，即有效半径外）允许拉直 → 绕行路径可平滑。
/// 线段合法性检查（Theta* 去锯齿拉直用）。
/// Zone 水平判定：NoFly/Obstacle 全高度墙——段到 Zone 水平净距 < inflation_km 即拒绝
/// （主管 2026-08-06：绕飞太贴边→考虑飞机机动；膨胀距离按物理转弯半径 v²/(g·tanφ)
/// 的 0.5 倍（clamp [2,10]km），拉直不得贴进膨胀带，FMM 绕行留转弯空间）；
/// Restricted 保持"水平相交 + 段高度采样"（M2 高度层语义，不膨胀）。
/// 雷达威胁：直连"深穿"任一雷达（归一化深度 < 0.7，即深入有效半径 70% 以内）
/// → 拒绝拉直（保住 FMM 绕行决策——P_cross 只是验收阈值，不得因调高 P_cross
/// 而把绕行弧拉直成穿雷达区的直线；主管 2026-08-06：航路必须绕开雷达探测区域）。
/// **例外**：段两端点任一已在深区（目标/必经点本身在雷达探测区内，绕不开）→
/// 允许拉直（雷达软约束由 verify 记录；无条件拒绝会让最后接近段无法拉直 →
/// 交付 FMM 网格伪影，主管 2026-08-06 37 点场景）。
/// 低概率边缘（≥0.7，即有效半径外）允许拉直 → 绕行路径可平滑。
fn make_segment_check<'a>(
    zones: &'a [Zone],
    threat: Option<&'a dyn crate::threat::ThreatModel>,
    inflation_km: f64,
) -> impl Fn(f64, f64, f64, f64, f64, f64) -> bool + 'a {
    move |lon1, lat1, alt1, lon2, lat2, alt2| {
        const N: usize = 16;
        const DEEP_RATIO: f64 = 0.7;
        for z in zones {
            let clr = crate::config::zone_segment_clearance_km(lon1, lat1, lon2, lat2, z);
            if z.is_wall() {
                if clr <= 1e-9 || clr < inflation_km {
                    return false;
                }
            } else if let crate::config::ZoneShape::Circle { center, radius_km } = &z.shape {
                // restricted 圆：与 verify 完全同口径——解析二次方程得到穿圆参数区间
                // [t1,t2]，**区间内**采样高度（0..N 等距采样会漏掉浅穿/短弦：段擦圆
                // 边缘穿入仅 0.03 宽，16 个等距点可能全在圆外 → check 放行 verify
                // 会拒的穿区段，2026-08-06 zigzag9 theta_star 拉直段擦过 restricted 圆）。
                if let Some((t1, t2)) = crate::smooth::segment_circle_intersect_t(
                    lon1,
                    lat1,
                    lon2,
                    lat2,
                    center[0],
                    center[1],
                    *radius_km,
                ) {
                    for i in 0..=N {
                        let t = t1 + (t2 - t1) * i as f64 / N as f64;
                        let lon = lon1 + (lon2 - lon1) * t;
                        let lat = lat1 + (lat2 - lat1) * t;
                        let alt = alt1 + (alt2 - alt1) * t;
                        if let Ok(g) = Geo::new(lon, lat) {
                            if zone_contains_at(z, &g, alt, None) {
                                return false;
                            }
                        }
                    }
                }
            } else if clr <= 1e-9 {
                // restricted 多边形：净距相交（段-边相交=0）→ 高度层采样
                for i in 0..=N {
                    let t = i as f64 / N as f64;
                    let lon = lon1 + (lon2 - lon1) * t;
                    let lat = lat1 + (lat2 - lat1) * t;
                    let alt = alt1 + (alt2 - alt1) * t;
                    if let Ok(g) = Geo::new(lon, lat) {
                        if zone_contains_at(z, &g, alt, None) {
                            return false;
                        }
                    }
                }
            }
        }
        if let Some(tm) = threat {
            // 雷达：仅当**两端点都在深区外**时，直连穿深区 = 破坏 FMM 绕行决策 → 拒绝。
            // 任一端点已在深区（目标/必经点落在雷达探测区内，无法绕开，如 2026-08-06
            // 主管 37 点场景 target 距雷达 61km < 0.7×100km）→ 允许拉直——该端点深穿
            // 不可避免，拉直只简化 FMM 网格伪影，不引入新的"绕行决策破坏"；雷达是软
            // 约束，最终由 verify 记录 P_cross（此前无条件拒绝深穿 → Theta* 无法拉直
            // 最后接近段 → 交付密集网格点伪影）。
            let deep_a = tm.static_penetration(lon1, lat1, alt1) < DEEP_RATIO;
            let deep_b = tm.static_penetration(lon2, lat2, alt2) < DEEP_RATIO;
            if !deep_a && !deep_b {
                for i in 0..=N {
                    let t = i as f64 / N as f64;
                    let lon = lon1 + (lon2 - lon1) * t;
                    let lat = lat1 + (lat2 - lat1) * t;
                    let alt = alt1 + (alt2 - alt1) * t;
                    if tm.static_penetration(lon, lat, alt) < DEEP_RATIO {
                        return false;
                    }
                }
            }
        }
        true
    }
}

/// Restricted 是否按该飞行高度视为禁行墙（底部可通行语义，主管 2026-08-06）：
/// 飞行高度落在 restricted 高度区间内 → 该机 FMM 画墙绕行（否则直穿）。
fn restricted_blocks_alt(z: &Zone, alt_m: f64) -> bool {
    matches!(z.zone_type, crate::config::ZoneType::Restricted)
        && alt_m >= z.alt_min_m
        && alt_m <= z.alt_max_m
}

/// 度制近似平面距离（km）。短距离（<100km）内精度足够（剖面可行性采样/锚点用）。
/// 纬度 111.32 km/°；经度按中纬度 cos 收缩（度单位直接换算，勿再 to_radians）。
fn dist_km(lon1: f64, lat1: f64, lon2: f64, lat2: f64) -> f64 {
    let dlat = lat2 - lat1; // 度
    let dlon = lon2 - lon1; // 度
    let mlat = ((lat1 + lat2) / 2.0).to_radians();
    let x = dlon * mlat.cos() * 111.32;
    let y = dlat * 111.32;
    (x * x + y * y).sqrt()
}

/// 受限区穿行剖面高度决策（主管 2026-08-06 二轮：比较顶部绕飞与底部穿行的代价，选更优）。
/// 仅对高度区间内的 restricted 调用；返回：
/// - Some(pass_alt)：可剖面穿行——底部（pass=alt_min−500m）与顶部（pass=alt_max+500m）
///   都评估后选更优：两者水平路径同为直线（总水平距离相同），仅垂直机动总量不同
///   （底部 2×|alt−alt_min−500|，顶部 2×|alt_max+500−alt|）→ 垂直机动更少的底部恒优；
///   底部不可行（穿行区地形挡住底部 / 爬升距离不足）→ 顶部绕飞（高于任何地形，且须
///   ≤ 升限 ceiling_m）；
/// - None：底部与顶部都不可行 → 需画墙水平绕行（fallback 保底）。
/// 多边形 restricted 的剖面暂不支持（MVP：仅圆形直穿剖面），返回 None（水平绕行）。
fn restricted_pass_alt(
    z: &Zone,
    alt_m: f64,
    ceiling_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    max_climb_deg: f64,
) -> Option<f64> {
    let ZoneShape::Circle { center, radius_km } = z.shape else {
        return None;
    };
    let bottom = z.alt_min_m - 500.0;
    let top = z.alt_max_m + 500.0;
    let climb_dist = |pass: f64| -> f64 {
        if max_climb_deg > 0.1 {
            (alt_m - pass).abs() / max_climb_deg.to_radians().tan() * 1.25
        } else {
            f64::INFINITY
        }
    };
    // start/target 到圆边界留有爬升距离（d 单位 km，climb_dist 单位 m → 统一换算）
    let d_in = dist_km(center[0], center[1], start.lon, start.lat) - radius_km;
    let d_out = dist_km(center[0], center[1], target.lon, target.lat) - radius_km;
    let fit = |pass: f64| d_in * 1000.0 >= climb_dist(pass) && d_out * 1000.0 >= climb_dist(pass);
    // 顶部绕飞可行性：爬升距离 + 升限（alt_max + 500 ≤ ceiling）
    let top_ok = fit(top) && ceiling_m.map_or(true, |c| top <= c);
    // 底部穿行可行性：爬升距离 + **底部严格低于 alt_min**（穿行高度必须在 restricted
    // 高度区间外；alt_min=0 时 bottom=-500 负高不可行——0m 仍在 [0,alt_max] 区间内且
    // 撞地形）+ 穿行带（直线穿圆段）地形 ≤ 底部 − 净空
    let bottom_ok = bottom >= 0.0
        && bottom < z.alt_min_m
        && fit(bottom)
        && bottom_terrain_ok(z, terrain, bottom, start, target);
    match (bottom_ok, top_ok) {
        (true, _) => Some(bottom), // 底部垂直机动总量更小 → 恒更优（显式代价比较结论）
        (false, true) => Some(top), // 底部不可行 → 顶部绕飞（优于水平绕行：水平距离不增加）
        (false, false) => None,
    }
}

/// 底部通道地形可行性（主管 2026-08-06 三轮纠偏）：判据 = **直线穿行带**，不是整个圆面。
/// 穿行剖面沿 start→target 直线穿圆，飞机只经过圆内的一条**线**（穿行段），因此地形
/// 检查只采样该直线在圆内的穿行段沿线（步长 ~2.2km，取最高点）——圆面角落的高山
/// （飞机不经过）不应把底部判为不可行（圆面判据过度保守，会把"穿行带 46m、圆角落
/// 685m"的场景错误导向顶部绕飞）。
/// 无地形（平面 0m）→ 恒可行。顶部绕飞不查地形（高于任何地形）。
fn bottom_terrain_ok(
    z: &Zone,
    terrain: Option<&dyn TerrainSource>,
    bottom: f64,
    start: &Geo,
    target: &Geo,
) -> bool {
    let Some(t) = terrain else {
        return true;
    };
    let ZoneShape::Circle { center, radius_km } = z.shape else {
        return false;
    };
    // 平面近似：start→target 直线与圆交点（同 build_restricted_profiles 的解析二次方程）
    let mlat = ((start.lat + target.lat) / 2.0).to_radians();
    let kx = mlat.cos() * 111.32;
    let ky = 111.32;
    let dx = (target.lon - start.lon) * kx;
    let dy = (target.lat - start.lat) * ky;
    let cx = (center[0] - start.lon) * kx;
    let cy = (center[1] - start.lat) * ky;
    let a = dx * dx + dy * dy;
    if a < 1e-9 {
        return true; // start/target 重合（调用方已过滤）
    }
    let b = -2.0 * (dx * cx + dy * cy);
    let c = cx * cx + cy * cy - radius_km * radius_km;
    let disc = b * b - 4.0 * a * c;
    if disc <= 0.0 {
        return true; // 直线不穿圆
    }
    let sq = disc.sqrt();
    let (u_in, u_out) = ((-b - sq) / (2.0 * a), (-b + sq) / (2.0 * a));
    let (u_in, u_out) = (u_in.max(0.0), u_out.min(1.0));
    if u_out <= u_in {
        return true; // 圆在线段端点之外（FMM 直穿不经过）
    }
    // 沿穿行段采样地形：穿行段长 ≈ 圆直径（40km），步长 ~2.2km → n ≈ 18；clamp [8,64]
    let seg_km = (u_out - u_in) * (dx * dx + dy * dy).sqrt();
    let n = ((seg_km / 2.2).round() as usize).clamp(8, 64);
    let mut max_terr: Option<f64> = None;
    for k in 0..=n {
        let u = u_in + (u_out - u_in) * (k as f64 / n as f64);
        let lon = start.lon + (target.lon - start.lon) * u;
        let lat = start.lat + (target.lat - start.lat) * u;
        if let Sample::Land(h) = t.sample_at(lon, lat) {
            max_terr = Some(max_terr.map_or(h, |m: f64| m.max(h)));
        }
    }
    match max_terr {
        Some(h) => h + 100.0 <= bottom, // 净空满足 → 底部可行
        None => true,                    // 穿行段无陆地（水面/无数据）→ 直穿
    }
}

/// 该机飞行高度落在 restricted 高度区间内时，是否必须在 FMM 层画墙**水平绕行**：
/// 高度在区间外 → 不拦截直穿；区间内 → `restricted_pass_alt` 决策：底部穿行 / 顶部
/// 绕飞（均可行时底部更优，不画墙，FMM 直穿后由 `build_restricted_profiles` 生成
/// 对应剖面）；底部与顶部都不可行 → 画墙水平绕行（fallback 保底，不产生失败路径）。
fn restricted_detour_required(
    z: &Zone,
    alt_m: f64,
    ceiling_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    max_climb_deg: f64,
) -> bool {
    if !restricted_blocks_alt(z, alt_m) {
        return false;
    }
    restricted_pass_alt(z, alt_m, ceiling_m, terrain, start, target, max_climb_deg).is_none()
}

/// 直线段是否穿/贴任一硬墙（NoFly/Obstacle，全高度墙）：净距 ≤ 0 或 < inflation。
/// 用于直线剖面 fallback（raw 未穿圆时用 start→target 直线参数化剖面，需直线避开硬墙）
/// 与 build 剖面段防御。与 verify 的墙判定口径一致。
fn line_hits_wall_km(
    lon1: f64,
    lat1: f64,
    lon2: f64,
    lat2: f64,
    zones: &[Zone],
    inflation_km: f64,
) -> bool {
    zones.iter().any(|z| {
        if !z.is_wall() {
            return false;
        }
        let clr = crate::config::zone_segment_clearance_km(lon1, lat1, lon2, lat2, z);
        clr <= 1e-9 || clr < inflation_km
    })
}

/// 受限区穿行剖面（主管 2026-08-06 二轮+三轮架构增强）：FMM 直穿圆形 restricted
/// （只绕硬墙）后，沿 raw 路径找穿行区间（进入点 in / 穿出点 out）：
/// [首段 raw[0..=i_desc]@alt_m, desc→in 过渡直线(mask=true 跳过平滑,15°),
///  in→out raw 子段@pass_alt（绕行弧平飞，走平滑链）,
///  out→climb 过渡直线(mask=true), 尾段 raw[i_climb..]@alt_m]。
/// 过渡段跳过平滑（拉直会缩短水平距离→爬升角超 15°）；in→out 平飞段（含 no_fly
/// 与 restricted 重叠时深入圆内的绕行弧）以 pass_alt 飞行，即"先绕 no_fly 再剖面
/// 直穿 restricted"组合机动。in→out 平滑前做硬墙外扩（FMM 贴墙 clearance≈inflation，
/// 平滑内切后不足 verify 阈值）。穿行高度由 `restricted_pass_alt` 决策：
/// 底部可行恒选底部；底部被穿行带地形挡住 → 顶部绕飞；都不可行 → 已画墙绕行
/// （`restricted_detour_required`），raw 不穿它，此处自动跳过。
///
/// 返回 (切分段, 段掩码 true=跳过平滑, need_wall_fallback)。
/// 剖面段 = raw 子段（FMM 已避硬墙）→ need_wall 恒 false（保留签名供调用方兜底）。
#[allow(clippy::too_many_arguments)]
fn build_restricted_profiles(
    seg: &Path,
    zones: &[Zone],
    alt_m: f64,
    max_climb_deg: f64,
    ceiling_m: Option<f64>,
    terrain: Option<&dyn TerrainSource>,
    start: &Geo,
    target: &Geo,
    inflation_km: f64,
) -> (Vec<Path>, Vec<bool>, bool) {
    let n = seg.points.len();
    if n < 2 {
        return (vec![seg.clone()], vec![false], false);
    }
    // 该机高度拦截的圆形 restricted（底部/顶部剖面穿行类型）
    let hits: Vec<&Zone> = zones
        .iter()
        .filter(|z| matches!(z.shape, ZoneShape::Circle { .. }) && restricted_blocks_alt(z, alt_m))
        .collect();
    if hits.is_empty() {
        return (vec![seg.clone()], vec![false], false);
    }
    let (p0, p1) = (seg.points[0], *seg.points.last().unwrap());
    let mut out_segs: Vec<Path> = vec![seg.clone()];
    let mut out_mask: Vec<bool> = vec![false];
    let need_wall = false;
    // 逐个 hit：在当前的尾段上找穿行区间 → 切 [首段, 剖面段, 尾段]（多 restricted 顺序处理）
    for z in hits {
        let tail = out_segs.last().unwrap();
        let ZoneShape::Circle { center, radius_km } = z.shape else {
            continue;
        };
        let (cx, cy, r) = (center[0], center[1], radius_km);
        // 找进入索引（第一个与圆相交段的起点）与最后圆内点（穿出点）：
        // 逐格点 dist<r 判定会漏掉浅穿（锯齿格点全在圆外但线段穿入，如 new_rz
        // 垂距 19.5km 仅穿入 0.5km）→ 改用"段与圆相交"（最近距离 < r）判定。
        let mut in_idx: Option<usize> = None;
        let mut out_idx: Option<usize> = None;
        let mut in_circle = false;
        for i in 0..tail.points.len().saturating_sub(1) {
            let pa = &tail.points[i];
            let pb = &tail.points[i + 1];
            let d = crate::config::zone_segment_clearance_km(pa.lon, pa.lat, pb.lon, pb.lat, z);
            // 端点 inside 判定与 verify（zone_contains_at）同口径：Geo::distance_m
            // （haversine）≤ r——dist_km（平面近似）在圆边界处有 ~0.1% 偏差，
            // 边界点（如距圆心 20.02 vs 19.99km）会漏判进入 → 首段含圆内 3000m 违规。
            let a_in = Geo::new(pa.lon, pa.lat)
                .map_or(false, |g| crate::config::zone_contains(z, &g));
            let b_in = Geo::new(pb.lon, pb.lat)
                .map_or(false, |g| crate::config::zone_contains(z, &g));
            let crossing = d <= 1e-9 || a_in || b_in; // 段与圆相交/端点在内
            if crossing && !in_circle {
                // 圆凸：in_idx 只取第一次进入（raw 锯齿在圆边界摆动时可能"出圆→再进"，
                // 覆盖 in_idx 会把剖面起点推迟到最后一个进入点 → 首段含圆内 3000m 违规）
                if in_idx.is_none() {
                    in_idx = Some(i);
                }
                in_circle = true;
            }
            if in_circle {
                if b_in {
                    out_idx = Some(i + 1); // 圆内区间持续（终点更新）
                } else if !crossing {
                    // 段已完全在圆外 → 圆凸，区间结束（out = 段起点，圆外）
                    out_idx = Some(i);
                    in_circle = false;
                } else {
                    // crossing && !b_in：段从圆内穿出 → out = 段终点（圆外点）。
                    // out 必须是圆外点——否则 out→climb 爬升过渡从圆内开始，
                    // 高度 500→1000+ 进入 restricted 区间违规（verify alt band 拒）。
                    out_idx = Some(i + 1);
                }
            }
        }
        let (Some(i_in), Some(i_out)) = (in_idx, out_idx) else {
            // raw 未穿该圆：FMM 网格离散擦边（浅穿深度 < 格距，格点全在圆外）时
            // 真实几何仍穿圆 → 平滑拉直会穿圆违规（verify 几何精确拦截）→ 回退锯齿。
            // fallback：start→target 直线穿圆（且直线避开全部硬墙）→ 直线参数化剖面
            // （1b1331b 旧方案；仅 raw 未穿圆时启用，主管 2026-08-06 三轮架构保留）。
            let Some(pass_alt) =
                restricted_pass_alt(z, alt_m, ceiling_m, terrain, start, target, max_climb_deg)
            else {
                continue;
            };
            if line_hits_wall_km(p0.lon, p0.lat, p1.lon, p1.lat, zones, inflation_km) {
                continue; // 直线穿硬墙（如 no_fly）→ 直线剖面不可用
            }
            let mlat = ((p0.lat + p1.lat) / 2.0).to_radians();
            let kx = mlat.cos() * 111.32;
            let ky = 111.32;
            let ddx = (p1.lon - p0.lon) * kx;
            let ddy = (p1.lat - p0.lat) * ky;
            let oox = (cx - p0.lon) * kx;
            let ooy = (cy - p0.lat) * ky;
            let aa = ddx * ddx + ddy * ddy;
            let bb = -2.0 * (ddx * oox + ddy * ooy);
            let cc = oox * oox + ooy * ooy - r * r;
            let disc = bb * bb - 4.0 * aa * cc;
            if disc <= 0.0 {
                continue;
            }
            let sq = disc.sqrt();
            let u1 = ((-bb - sq) / (2.0 * aa)).clamp(0.0, 1.0);
            let u2 = ((-bb + sq) / (2.0 * aa)).clamp(0.0, 1.0);
            if u2 <= 0.0 || u1 >= 1.0 {
                continue;
            }
            let climb_base_km = if max_climb_deg > 0.1 {
                (alt_m - pass_alt).abs() / max_climb_deg.to_radians().tan() / 1000.0 * 1.1
            } else {
                f64::INFINITY
            };
            let line_len_km = aa.sqrt();
            let u_desc = (u1 - climb_base_km / line_len_km).max(0.0);
            let u_climb = (u2 + climb_base_km / line_len_km).min(1.0);
            let pt_at = |u: f64, h: f64| {
                RouterPoint::new(
                    p0.lon + (p1.lon - p0.lon) * u,
                    p0.lat + (p1.lat - p0.lat) * u,
                    h,
                )
            };
            let desc_p = pt_at(u_desc, alt_m);
            let in_p = pt_at(u1, pass_alt);
            let out_p = pt_at(u2, pass_alt);
            let climb_p = pt_at(u_climb, alt_m);
            // 全直线 5 段（跳过平滑——本身是 max_climb 直线 + 平飞弦）
            let head_l = Path::new(vec![tail.points[0], desc_p]);
            let desc_in_l = Path::new(vec![desc_p, in_p]);
            let in_out_l = Path::new(vec![in_p, out_p]);
            let out_climb_l = Path::new(vec![out_p, climb_p]);
            let tail2_l = Path::new(vec![climb_p, *tail.points.last().unwrap()]);
            out_segs.pop();
            out_mask.pop();
            for (s, m) in [
                (head_l, true),
                (desc_in_l, true),
                (in_out_l, true),
                (out_climb_l, true),
                (tail2_l, true),
            ] {
                out_segs.push(s);
                out_mask.push(m);
            }
            break; // 直线剖面覆盖整段，后续 hit 不再处理
        };
        if i_out <= i_in {
            continue;
        }
        // 穿行高度决策（底部优先 / 顶部备选），不可行 → 已画墙绕行（raw 不穿它）
        let Some(pass_alt) =
            restricted_pass_alt(z, alt_m, ceiling_m, terrain, start, target, max_climb_deg)
        else {
            continue; // 底部/顶部都不可行 → 已画墙绕行（raw 不穿它）
        };
        // 过渡段基线水平距离（无裕量）：高差 → 15° 爬升角所需最小水平距离。
        // desc/climb 锚点判定用"直线距离 ≥ 1.1×基线"（保证过渡直线爬升角 ≤ 13.6°），
        // 且锚点沿 raw 路径前移（直线段上，距硬墙远，避免拉直后爬升角超/穿墙）。
        let climb_base_km = if max_climb_deg > 0.1 {
            (alt_m - pass_alt).abs() / max_climb_deg.to_radians().tan() / 1000.0 * 1.1
        } else {
            f64::INFINITY
        };
        // in→out 平飞段 = raw[i_in..=i_out] @pass_alt（含绕行弧），平滑前硬墙外扩
        //（FMM 贴墙 clearance≈inflation，平滑内切后不足 verify 阈值 → 外扩到
        // radius+inflation+margin；margin=0.5km 吸收平滑内切与网格离散误差）。
        let in_out_raw: Vec<RouterPoint> = tail.points[i_in..=i_out]
            .iter()
            .map(|p| RouterPoint::new(p.lon, p.lat, pass_alt))
            .collect();
        let in_out = push_out_of_walls(&in_out_raw, zones, inflation_km, 0.5);
        let pin2 = in_out[0];
        let pout2 = *in_out.last().unwrap();
        // 方向辅助（局部等距投影 heading；相邻过渡段转角 ≤ 60°——固定翼最大转角，
        // 否则 join 后 final verify turn 拒 → 回退锯齿，如 zigzag6 vertex5 64.55°）。
        let heading = |lon1: f64, lat1: f64, lon2: f64, lat2: f64| {
            let mlat = ((lat1 + lat2) / 2.0).to_radians();
            let kx = mlat.cos() * 111.32;
            let dx = (lon2 - lon1) * kx;
            let dy = (lat2 - lat1) * 111.32;
            dy.atan2(dx).to_degrees().rem_euclid(360.0)
        };
        let angle_between = |h1: f64, h2: f64| {
            let d = (h1 - h2).abs() % 360.0;
            if d > 180.0 { 360.0 - d } else { d }
        };
        let p0pt = tail.points[0];
        let plast = *tail.points.last().unwrap();
        // desc：沿路径从 i_in 向前，找直线距离 ≥ climb_base 且 start→desc 与 desc→in
        // 转角 ≤ 55° 的点（首段平滑为 start→desc 直线后连接 desc→in 过渡；55° 留 5°
        // 余量——检查用局部投影近似，verify 用精确投影，边界会差 ~0.3°）
        let mut i_desc = i_in;
        for i in (0..i_in).rev() {
            if dist_km(tail.points[i].lon, tail.points[i].lat, pin2.lon, pin2.lat) >= climb_base_km {
                let h1 = heading(p0pt.lon, p0pt.lat, tail.points[i].lon, tail.points[i].lat);
                let h2 = heading(tail.points[i].lon, tail.points[i].lat, pin2.lon, pin2.lat);
                if angle_between(h1, h2) <= 55.0 {
                    i_desc = i;
                    break;
                }
            }
        }
        // climb：沿路径从 i_out 向后，找直线距离 ≥ climb_base 且 out→climb 与
        // climb→target 转角 ≤ 55° 的点（尾段平滑为 climb→target 直线）
        let mut i_climb = i_out;
        for i in (i_out + 1)..tail.points.len() {
            if dist_km(pout2.lon, pout2.lat, tail.points[i].lon, tail.points[i].lat) >= climb_base_km {
                let h1 = heading(pout2.lon, pout2.lat, tail.points[i].lon, tail.points[i].lat);
                let h2 = heading(tail.points[i].lon, tail.points[i].lat, plast.lon, plast.lat);
                if angle_between(h1, h2) <= 55.0 {
                    i_climb = i;
                    break;
                }
            }
        }
        // 切分 5 段：
        //  [首段 raw[0..=i_desc]@alt_m, 过渡直线 desc→in(mask=true 跳过平滑,15°),
        //   in→out raw 子段@pass_alt（绕行弧平飞，走平滑链）,
        //   过渡直线 out→climb(mask=true), 尾段 raw[i_climb..]@alt_m]
        let head = Path::new(tail.points[..=i_desc].to_vec());
        let desc_in = Path::new(vec![
            RouterPoint::new(tail.points[i_desc].lon, tail.points[i_desc].lat, alt_m),
            pin2,
        ]);
        let out_climb = Path::new(vec![
            pout2,
            RouterPoint::new(tail.points[i_climb].lon, tail.points[i_climb].lat, alt_m),
        ]);
        let tail2 = Path::new(tail.points[i_climb..].to_vec());
        // 替换尾段为 [首段, desc→in, in→out, out→climb, 新尾段]（继续处理下一个 hit）
        out_segs.pop();
        out_mask.pop();
        out_segs.push(head);
        out_mask.push(false);
        out_segs.push(desc_in);
        out_mask.push(true);
        out_segs.push(Path::new(in_out));
        out_mask.push(false);
        out_segs.push(out_climb);
        out_mask.push(true);
        out_segs.push(tail2);
        out_mask.push(false);
    }
    (out_segs, out_mask, need_wall)
}

/// 逐点把路径外扩到所有硬墙（NoFly/Obstacle）安全距离外：
/// 距圆墙圆心 < radius+inflation+margin → 沿径向（远离圆心）外移到该距离。
/// 用于 in→out 平飞段（FMM 贴墙绕行 clearance≈inflation，平滑内切后不足 verify
/// 阈值；margin 吸收平滑内切与 FMM 网格离散误差）。多边形墙当前不处理（沿用 raw）。
fn push_out_of_walls(
    pts: &[RouterPoint],
    zones: &[Zone],
    inflation_km: f64,
    margin_km: f64,
) -> Vec<RouterPoint> {
    pts.iter()
        .map(|p| {
            let (mut lon, mut lat) = (p.lon, p.lat);
            for z in zones {
                if !z.is_wall() {
                    continue;
                }
                if let ZoneShape::Circle { center, radius_km } = &z.shape {
                    let (cx, cy) = (center[0], center[1]);
                    let d = dist_km(lon, lat, cx, cy);
                    let target = radius_km + inflation_km + margin_km;
                    if d < target {
                        if d < 1e-6 {
                            // 极端：点与圆心重合 → 沿经度方向外移
                            lon = cx + target / 111.32;
                            continue;
                        }
                        let f = target / d;
                        lon = cx + (lon - cx) * f;
                        lat = cy + (lat - cy) * f;
                    }
                }
            }
            RouterPoint::new(lon, lat, p.alt_m)
        })
        .collect()
}


/// 禁飞区墙膨胀 + 过渡带软罚（5c + 5c2）：
/// - 5c：NoFly/Obstacle 硬墙向外膨胀 inflation_cells 格（考虑飞机机动留转弯空间，
///   主管 2026-08-06：绕飞太贴边→考虑飞机机动——绕行需留物理转弯空间）；
/// - 5c2：膨胀墙外过渡带内代价渐变递增（墙边 ×1.5，带外 ×1），FMM 权衡代价后自然
///   走离墙更远的栅格，拉直后 clearance 余量充足（防贴墙锯齿，主管 2026-08-05
///   双禁飞区场景实测）。**软罚带按物理距离 1.5km（格数随 cell_m 自适应）**——
///   固定 2 格在细网格（自适应大 region，cell<0.75km）时物理宽度变窄 → FMM 贴墙
///   更近 → 绕行 clearance 余量不足 → 平滑内切后 verify 拒（主管 2026-08-06
///   双大雷达+多边形 no_fly 场景实测 clearance 0.00 < inflation）。窄缝（7.8km）
///   仍 > 2×1.5km，不挤缝（band=3 格/0.75km=2.25km 物理才挤，已弃）。
fn apply_inflation_and_band(
    field: &mut crate::costfield::CostField,
    inflation_cells: usize,
    cell_m: f64,
) {
    let grid = field.rows.max(field.cols);
    // 5c. 膨胀（栅格级多轮 8 邻域扩散）
    if inflation_cells > 0 {
        let mut expanded: Vec<bool> = (0..grid * grid)
            .map(|i| !field.cost[i].is_finite())
            .collect();
        for _ in 0..inflation_cells {
            let cur = expanded.clone();
            for r in 0..grid {
                for c in 0..grid {
                    let idx = r * grid + c;
                    if cur[idx] {
                        continue;
                    }
                    let mut near = false;
                    for dr in -1i32..=1 {
                        for dc in -1i32..=1 {
                            if dr == 0 && dc == 0 {
                                continue;
                            }
                            let nr = r as i32 + dr;
                            let nc = c as i32 + dc;
                            if nr >= 0
                                && nr < grid as i32
                                && nc >= 0
                                && nc < grid as i32
                                && cur[nr as usize * grid + nc as usize]
                            {
                                near = true;
                                break;
                            }
                        }
                        if near {
                            break;
                        }
                    }
                    if near {
                        expanded[idx] = true;
                    }
                }
            }
        }
        for i in 0..grid * grid {
            if expanded[i] {
                field.cost[i] = f32::INFINITY;
            }
        }
    }
    // 5c2. 过渡带软罚（BFS 距离变换，8 邻域，源 = 当前 INF 墙；物理带 1.5km）
    {
        use std::collections::VecDeque;
        const BAND_M: f64 = 1500.0;
        const BAND_COEF: f32 = 0.5;
        let band_cells = (BAND_M / cell_m.max(1.0)).ceil() as u32;
        let mut dist = vec![u32::MAX; grid * grid];
        let mut q = VecDeque::new();
        for i in 0..grid * grid {
            if !field.cost[i].is_finite() {
                dist[i] = 0;
                q.push_back(i);
            }
        }
        while let Some(idx) = q.pop_front() {
            let r = idx / grid;
            let c = idx % grid;
            let nd = dist[idx] + 1;
            for (dr, dc) in [
                (0i32, 1i32),
                (0, -1),
                (1, 0),
                (-1, 0),
                (1, 1),
                (1, -1),
                (-1, 1),
                (-1, -1),
            ] {
                let nr = r as i32 + dr;
                let nc = c as i32 + dc;
                if nr >= 0 && nr < grid as i32 && nc >= 0 && nc < grid as i32 {
                    let ni = nr as usize * grid + nc as usize;
                    if dist[ni] > nd {
                        dist[ni] = nd;
                        q.push_back(ni);
                    }
                }
            }
        }
        for i in 0..grid * grid {
            if !field.cost[i].is_finite() {
                continue;
            }
            let d = dist[i];
            if d >= 1 && d <= band_cells {
                let t = 1.0 - (d as f32 - 1.0) / band_cells as f32; // d=1 → t=1.0；d=band → t≈0.33
                field.cost[i] *= 1.0 + BAND_COEF * t;
            }
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use crate::config::Input;

    fn parse(s: &str) -> Input {
        Input::from_json_str(s).unwrap()
    }

    const BASE: &str = r#"{
        "schema_version":"0.20",
        "mission":{
            "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
            "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
            "terrain":{"source":"none"}
        }
    }"#;

    #[test]
    fn m1_end_to_end_plain() {
        let input = parse(BASE);
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        assert_eq!(out.status, "success");
        assert_eq!(out.vehicles.len(), 1);
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(v.path.len() >= 2, "path 应 ≥2 点，实际 {}", v.path.len());
        assert!(v.distance_m > 100_000.0, "北京 115E,39N → 116.5E,39.9N ≈ 150km，实际 {}", v.distance_m);
        // 起终点在路径内（容差 0.02°≈2km——网格 256 格 1.5°≈210m/格）
        let (x0, y0) = (v.path.first().unwrap().x, v.path.first().unwrap().y);
        let (x1, y1) = (v.path.last().unwrap().x, v.path.last().unwrap().y);
        assert!((x0 - 115.0).abs() < 0.02 && (y0 - 39.0).abs() < 0.02);
        assert!((x1 - 116.5).abs() < 0.02 && (y1 - 39.9).abs() < 0.02);
    }

    #[test]
    fn m1_no_solution_when_target_blocked() {
        // 目标被巨型禁飞区完全覆盖 → 回溯失败 → no_solution
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[{"id":"wall","zone_type":"no_fly","shape":"circle",
                    "geometry":{"center":[116.5,39.9],"radius_km":30},
                    "alt_min_m":0,"alt_max_m":10000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "no_solution");
    }

    #[test]
    fn twin_zone_smooth_after_band_penalty() {
        // 双禁飞区并列（20km 圆 + 6 边形，主管 2026-08-05 真实场景无地形版）：
        // FMM 贴膨胀墙走 → Theta* 拉直 clearance 差 ~1 格 → 全链失败回退 362 点锯齿。
        // 5c2 过渡带软罚后必须平滑（≤10 点且无 smoothing_failed）。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":116.7188493150615,"lat":40.20313810412108,"alt_m":3000},
                "target":{"lon":115.44547741980215,"lat":38.63800678240428,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[
                    {"id":"c1","zone_type":"no_fly","shape":"circle",
                     "geometry":{"center":[115.90510756361434,39.1546051011457],"radius_km":20},
                     "alt_min_m":0,"alt_max_m":20000},
                    {"id":"p1","zone_type":"no_fly","shape":"polygon",
                     "geometry":{"vertices":[
                         [116.10919779988855,39.40629934778061],
                         [116.66748158411933,39.2812425371292],
                         [116.63856029155627,39.72638339128002],
                         [116.21277919960414,39.78606996932337],
                         [115.82845032806537,39.89839427431206],
                         [115.56298632103817,39.63798773661437]]},
                     "alt_min_m":0,"alt_max_m":20000}
                ]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 10,
            "双禁飞区并列应平滑（过渡带软罚），实际 {} 点（锯齿回退）",
            v.path.len()
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "不应有 smoothing_failed，实际 {:?}",
            v.warnings
        );
    }

    #[test]
    fn bottom_pass_band_ignores_corner_mountain() {
        // 主管 2026-08-06 三轮纠偏：判据 = 直线穿行带（飞机实际走的线），不是整个圆面。
        // 圆西侧角落 700m 高山（飞机穿行不经过），穿行带地形 50m → 底部 1500m 应可行。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.14959340327005, 39.597263409766285],
                radius_km: 20.0,
            },
            alt_min_m: 2000.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target = Geo::new(115.28680713092322, 39.04668499383146).unwrap();
        // 地形网格：覆盖 start→target 走廊；lon < 115.95（圆西侧角落）700m，其余 50m
        let rows = 20usize;
        let cols = 24usize;
        let mut h = vec![0.0f32; rows * cols];
        for r in 0..rows {
            for c in 0..cols {
                let lon = 115.55 + 0.05 * c as f64;
                h[r * cols + c] = if lon < 115.95 { 700.0 } else { 50.0 };
            }
        }
        let terr = crate::terrain::memory::Terrain {
            rows,
            cols,
            origin_lon: 115.55,
            origin_lat: 39.30,
            cell_lon_deg: 0.05,
            cell_lat_deg: 0.05,
            h,
        };
        let pass = restricted_pass_alt(&z, 3000.0, None, Some(&terr), &start, &target, 15.0);
        assert_eq!(
            pass,
            Some(1500.0),
            "穿行带 50m → 底部 1500m 可行；圆角落 700m 山不应参与判据"
        );
    }

    #[test]
    fn restricted_pass_alt_top_flyover_vs_bottom_descent() {
        // 主管 2026-08-06 二轮：比较顶部绕飞与底部穿行代价选更优。
        // 圆 [2000,5000]msl，巡航 3000m；start/target 距圆 > 爬升距离。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.14959340327005, 39.597263409766285],
                radius_km: 20.0,
            },
            alt_min_m: 2000.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target = Geo::new(115.28680713092322, 39.04668499383146).unwrap();
        // 无地形 → 底部 1500m 可行（垂直机动少 → 恒优）
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, None, None, &start, &target, 15.0),
            Some(1500.0)
        );
        // 穿行区地形 1450m：底部 1500 − 净空100 = 1400 < 1450 → 底部不可行 → 顶部绕飞 5500m
        let terr = crate::terrain::memory::Terrain {
            rows: 10,
            cols: 10,
            origin_lon: 115.9,
            origin_lat: 39.3,
            cell_lon_deg: 0.05,
            cell_lat_deg: 0.05,
            h: vec![1450.0f32; 100],
        };
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, None, Some(&terr), &start, &target, 15.0),
            Some(5500.0)
        );
        // 升限 5000m → 顶部 5500 超升限且底部被地形挡 → 两者不可行 → None（画墙绕行）
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, Some(5000.0), Some(&terr), &start, &target, 15.0),
            None
        );
        // 巡航 1500m（区间外底部通道）→ 不拦截（restricted_blocks_alt false）
        assert!(!restricted_blocks_alt(&z, 1500.0));
    }

    #[test]
    fn restricted_pass_alt_zero_alt_min_forces_top() {
        // 主管 2026-08-06 场景：restricted [0,5000]msl → bottom = -500（负高）不可行
        //（0m 仍在 [0,5000] 区间内且撞地形）→ 必须顶部绕飞 5500m（而非 0m 剖面——
        // 0m 穿行被 verify alt band + 地形净空拒绝 → 回退锯齿）。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [115.1103270025858, 39.570299948815645],
                radius_km: 20.0,
            },
            alt_min_m: 0.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target = Geo::new(113.93832638409175, 38.5937625849369).unwrap();
        // 无地形：底部 -500 负高 → 不可行 → 顶部 5500（不能返回 0m）
        assert_eq!(
            restricted_pass_alt(&z, 3000.0, None, None, &start, &target, 15.0),
            Some(5500.0)
        );
    }

    #[test]
    fn build_restricted_profiles_top_flyover_profile() {
        // 主管 2026-08-06 二轮：底部被 1450m 地形挡住 → 剖面应为顶部绕飞（5500m 平飞穿行）。
        let z = Zone {
            id: "rz".into(),
            zone_type: crate::config::ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.14959340327005, 39.597263409766285],
                radius_km: 20.0,
            },
            alt_min_m: 2000.0,
            alt_max_m: 5000.0,
            height_semantics: crate::config::HeightSemantics::Msl,
        };
        let start_geo = Geo::new(116.82168446499925, 40.23810827713887).unwrap();
        let target_geo = Geo::new(115.28680713092322, 39.04668499383146).unwrap();
        let start = RouterPoint::new(start_geo.lon, start_geo.lat, 3000.0);
        let target = RouterPoint::new(target_geo.lon, target_geo.lat, 3000.0);
        let pts: Vec<RouterPoint> = (0..=20)
            .map(|i| {
                let t = i as f64 / 20.0;
                RouterPoint::new(
                    start.lon + (target.lon - start.lon) * t,
                    start.lat + (target.lat - start.lat) * t,
                    3000.0,
                )
            })
            .collect();
        let seg = Path::new(pts);
        let terr = crate::terrain::memory::Terrain {
            rows: 10,
            cols: 10,
            origin_lon: 115.9,
            origin_lat: 39.3,
            cell_lon_deg: 0.05,
            cell_lat_deg: 0.05,
            h: vec![1450.0f32; 100],
        };
        let (segs, mask, _nw) = build_restricted_profiles(
            &seg,
            &[z.clone()],
            3000.0,
            15.0,
            None,
            Some(&terr),
            &start_geo,
            &target_geo,
            0.0, // 无硬墙 → 外扩不生效
        );
        assert_eq!(segs.len(), 5, "应切 [首段, desc→in, in→out, out→climb, 尾段]");
        assert_eq!(mask, vec![false, true, false, true, false]);
        // 平飞段 = segs[2]（in→out）：穿行高度 = alt_max + 500 = 5500
        let prof = &segs[2].points;
        assert!(
            prof.iter().any(|p| (p.alt_m - 5500.0).abs() < 1.0),
            "顶部平飞高度应为 5500m，实际 {:?}",
            prof.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
        assert!(
            prof.iter().all(|p| (p.alt_m - 5500.0).abs() < 1.0),
            "平飞段应全程 5500m，实际 {:?}",
            prof.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
        // 过渡段（segs[1] desc→in / segs[3] out→climb）：2 点直线，端点高度 3000/5500
        assert_eq!(segs[1].points.len(), 2, "desc→in 过渡应为 2 点直线");
        assert_eq!(segs[3].points.len(), 2, "out→climb 过渡应为 2 点直线");
        assert!(
            segs[1].points.iter().all(|p| p.alt_m == 3000.0 || (p.alt_m - 5500.0).abs() < 1.0),
            "desc→in 端点高度应为 3000/5500，实际 {:?}",
            segs[1].points.iter().map(|p| p.alt_m.round() as i64).collect::<Vec<_>>()
        );
    }

    #[test]
    fn restricted_zone_height_layer_blocks_or_passes() {
        // 底部可通行的限飞区（主管 2026-08-06）：restricted 圆 [2000,5000]msl 挡在
        // start→target 直线上。3000m 飞行高度在区间内、底部通道地形可行 → **降高剖面
        // 直穿**（下降→底部平飞→爬升，不水平绕路——航路允许变高度）；
        // 1500m 在区间外（底部通道）→ 直穿（不绕行）。
        let mk = |alt: f64| {
            format!(
                r#"{{
                    "schema_version":"0.20",
                    "mission":{{
                        "start":{{"lon":116.82168446499925,"lat":40.23810827713887,"alt_m":{alt}}},
                        "target":{{"lon":115.28680713092322,"lat":39.04668499383146,"alt_m":{alt}}},
                        "terrain":{{"source":"none"}},
                        "restricted_zones":[{{"id":"rz","zone_type":"restricted","shape":"circle",
                            "geometry":{{"center":[116.14959340327005,39.597263409766285],"radius_km":20}},
                            "alt_min_m":2000,"alt_max_m":5000}}]
                    }}
                }}"#
            )
        };
        // 3000m：区间内 + 底部可行 → 降高剖面直穿（不再水平绕行；不得回退密集锯齿）
        let input = parse(&mk(3000.0));
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 10,
            "3000m 剖面直穿应 ≈6 点（desc/in/out/climb + 端点），实际 {} 点",
            v.path.len()
        );
        assert!(
            v.distance_m < 190_000.0,
            "3000m 底部剖面直穿距离应 ≈187km（不绕行），实际 {}km",
            v.distance_m / 1000.0
        );
        assert!(
            v.path.iter().any(|p| p.alt_m < 2000.0),
            "3000m 剖面应在 restricted 穿行区降高到 <2000m，实际最小高度 {}m",
            v.path.iter().map(|p| p.alt_m).fold(f64::MAX, f64::min)
        );
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "3000m 不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        // 1500m：区间外（底部通道）→ 直穿，≤3 点且距离 ≈ 直线
        let input = parse(&mk(1500.0));
        let out = solve(&input, &SolveParams::default(), 42).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.path.len() <= 3,
            "1500m 底部可通行应直穿，实际 {} 点",
            v.path.len()
        );
        assert!(
            v.distance_m < 192_000.0,
            "1500m 直穿距离应 ≈187km，实际 {}km",
            v.distance_m / 1000.0
        );
    }

    #[test]
    fn segment_check_geometry_catches_diagonal_polygon_crossing() {
        // 梯形禁飞区（主管 2026-08-06 场景）：直线斜切穿内部（16 点采样会漏——
        // 几何判定必须拒绝）；绕行折线（先下后右）必须放行。
        use crate::config::{HeightSemantics, ZoneShape, ZoneType};
        let z = Zone {
            id: "trap".into(),
            zone_type: ZoneType::NoFly,
            shape: ZoneShape::Polygon {
                vertices: vec![
                    [116.2, 39.9],
                    [116.5, 39.9],
                    [116.5, 40.2],
                    [116.35, 40.2],
                ],
            },
            alt_min_m: 0.0,
            alt_max_m: 12000.0,
            height_semantics: HeightSemantics::Msl,
        };
        let zones = vec![z];
        let check = make_segment_check(&zones, None, 0.0);
        // 斜切直线：start → 接近 target 的直线，穿过梯形内部 → 拒绝拉直
        assert!(!check(115.9, 39.8, 3000.0, 116.48, 40.3, 3000.0));
        // 绕行折线两段：先向下绕过梯形下边（y<39.9），再从右侧上行（x>116.5）→ 放行
        assert!(check(115.9, 39.8, 3000.0, 116.55, 39.85, 3000.0));
        assert!(check(116.55, 39.85, 3000.0, 116.8, 40.3, 3000.0));
        // 机动膨胀（主管 2026-08-06：绕飞太贴边→考虑飞机机动）：贴边绕行段
        // （距下边 ~5.5km < 膨胀 6km）被拒；远离段放行。
        let check_infl = make_segment_check(&zones, None, 6.0);
        assert!(
            !check_infl(115.9, 39.8, 3000.0, 116.55, 39.85, 3000.0),
            "绕行段距梯形下边 < 膨胀 6km 应被拒绝（留转弯空间）"
        );
        assert!(
            check_infl(115.0, 39.0, 3000.0, 117.0, 39.5, 3000.0),
            "远离段应放行"
        );
    }

    #[test]
    fn segment_check_restricted_shallow_graze_rejected() {
        // zigzag9 根因（2026-08-06）：theta_star 拉直段"擦过" restricted 圆边缘——
        // 段-圆相交区间仅 ~0.03 宽（t∈[0.592,0.622]），16 点等距采样可能全在圆外
        // → 旧 check（净距 clr≤1e-9 或等距采样）放行 verify 会拒的穿区段。
        // 修复：check 与 verify 同口径（解析二次方程 + [t1,t2] 区间内采样）。
        use crate::config::{HeightSemantics, ZoneShape, ZoneType};
        let z = Zone {
            id: "rz".into(),
            zone_type: ZoneType::Restricted,
            shape: ZoneShape::Circle {
                center: [116.27050736818683, 41.08978345198258],
                radius_km: 50.0,
            },
            alt_min_m: 0.0,
            alt_max_m: 5000.0,
            height_semantics: HeightSemantics::Msl,
        };
        let zones = vec![z];
        let check = make_segment_check(&zones, None, 0.0);
        // zigzag9 theta_star 拉直段 start→(114.2076,42.3648) 擦过 rz1 圆边缘
        // （浅穿）→ 高度 3000 在带内 → 必须拒绝拉直
        assert!(
            !check(
                118.28982699875671,
                38.42208802408725,
                3000.0,
                114.2076,
                42.3648,
                3000.0
            ),
            "擦边浅穿 restricted 圆必须拒绝（与 verify 同口径）"
        );
        // 同段但高度 6000（带外）→ 放行
        assert!(check(
            118.28982699875671,
            38.42208802408725,
            6000.0,
            114.2076,
            42.3648,
            6000.0
        ));
    }

    #[test]
    fn m1_detours_around_zone() {        // 挡路禁飞区（圆心在中点）→ 路径绕行（折线长度 > 直线）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[{"id":"mid","zone_type":"no_fly","shape":"circle",
                    "geometry":{"center":[115.75,39.45],"radius_km":25},
                    "alt_min_m":0,"alt_max_m":10000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let v = &out.vehicles[0];
        // 直线 ≈ 153km；绕行应更长（25km 半径圆挡在中点）
        assert!(v.distance_m > 155_000.0, "应绕行，距离 {}", v.distance_m);
    }

    #[test]
    fn m2_restricted_band_does_not_wall() {
        // Restricted 高度层 [0, 2000]m：巡航 3000m 在区间外 → 可穿越（直达，不绕行）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "restricted_zones":[{"id":"band","zone_type":"restricted","shape":"circle",
                    "geometry":{"center":[115.75,39.45],"radius_km":25},
                    "alt_min_m":0,"alt_max_m":2000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 高度区间外 → 路径可直穿（Theta* 截直）→ 距离 ≈ 直线（≈164km），远小于绕行
        let d = out.vehicles[0].distance_m;
        assert!(d < 167_000.0, "应直穿 restricted 高度层，距离 {}", d);
    }

    #[test]
    fn m2_nofly_wall_blocks_regardless_of_altitude() {
        // NoFly 同位置：全高度墙 → 绕行（距离显著大于直线）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "no_fly_zones":[{"id":"wall","zone_type":"no_fly","shape":"circle",
                    "geometry":{"center":[115.75,39.45],"radius_km":25},
                    "alt_min_m":0,"alt_max_m":2000}]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let d = out.vehicles[0].distance_m;
        assert!(d > 160_000.0, "NoFly 应绕行，距离 {}", d);
    }

    #[test]
    fn m3_radar_threat_detour_and_warning() {
        // 场景 A：大雷达（40km）挡在中点。radar_cost_coef=50（占位）→ 高概率区代价显著
        // （中心 ×6）→ FMM 明确绕行躲避（主管 2026-08-05：探测概率应明显影响航路规划）。
        // 绕行后最近点应完全绕出有效半径（48km），累计探测概率应低于阈值。
        let big = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "red_forces":{"radars":[{"id":"r1","lon":115.75,"lat":39.45,"radar_type":"tracking","radius_km":40}]}
            }
        }"#;
        let out = solve(&parse(big), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 绕行：显著大于直线 164km（不得回到直穿 <170km）
        let d = out.vehicles[0].distance_m;
        assert!(d > 170_000.0, "40km 雷达应绕行躲避，距离 {}", d);
        // 绕行平滑（Theta* 转角受限 + Catmull-Rom）：不得交付网格锯齿（>50 点）
        assert!(out.vehicles[0].path.len() <= 50, "绕行应平滑，点数 {}", out.vehicles[0].path.len());
        // 绕行成功 → 累计探测概率低于阈值（不超 P_cross=0.1）
        let over = out.vehicles[0]
            .warnings
            .iter()
            .chain(out.stats.degradations.iter())
            .any(|s| s.contains("radar: cumulative detection p") && s.contains("> threshold"));
        assert!(!over, "绕行后探测概率应 <0.1: warnings={:?} degradations={:?}",
            out.vehicles[0].warnings, out.stats.degradations);
    }

    #[test]
    fn m3_target_in_radar_deep_zone_straightens_approach() {
        // 主管 2026-08-06 37 点场景：target 落在雷达深区（距圆心 61km < 0.7×100km），
        // Theta* check 无条件拒绝"直连穿深区"→ 最后接近段无法拉直 → 交付 FMM 网格
        // 伪影（33 点共线等距，总路径 37 点）。修复：段两端点任一已在深区（目标/必经
        // 点本身在雷达探测区内，绕不开）→ 允许拉直；雷达软约束由 verify 记录 P_cross。
        // 本用例：target 在 100km 雷达深区内，start 在深区外 → 路径应平滑（无网格伪影）。
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":118.0,"lat":42.0,"alt_m":3000},
                "target":{"lon":111.26,"lat":43.83,"alt_m":3000},
                "terrain":{"source":"none"},
                "red_forces":{"radars":[{"id":"r1","lon":111.0,"lat":44.0,"radar_type":"early_warning","radius_km":100}]}
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 平滑：接近段必须拉直（修复前 Theta* 拒深区直连 → 数十~上百点网格伪影）
        assert!(
            out.vehicles[0].path.len() <= 12,
            "深区 target 接近段应拉直平滑，点数 {}",
            out.vehicles[0].path.len()
        );
        assert!(
            !out.vehicles[0].warnings.iter().any(|w| w.contains("smoothing_failed")),
            "不应 smoothing_failed: {:?}",
            out.vehicles[0].warnings
        );
    }

    #[test]
    fn m3_small_radar_crossing_reports_probability() {
        // 场景 B：小雷达（5km）代价极小 → FMM 微绕（≈直线，<170km）或直穿。
        // 微绕时探测概率≈0 可不报告；直穿时必须报告。A2 未标定（见上）。
        let small = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "red_forces":{"radars":[{"id":"r1","lon":115.75,"lat":39.45,"radar_type":"tracking","radius_km":5}]}
            }
        }"#;
        let out = solve(&parse(small), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let d = out.vehicles[0].distance_m;
        assert!(d < 170_000.0, "小雷达应直穿或微绕，距离 {}", d);
        let radar_reported = out.vehicles[0]
            .warnings
            .iter()
            .chain(out.stats.degradations.iter())
            .any(|s| s.contains("radar"));
        // 若接近直线（直穿）则必须报告；微绕（>+1.5km）可免
        if d < 165_500.0 {
            assert!(radar_reported, "直穿应报告雷达概率: warnings={:?} degradations={:?}",
                out.vehicles[0].warnings, out.stats.degradations);
        }
    }

    #[test]
    fn m5_mid_waypoint_passes_through() {
        // 单机 mid_waypoints：路径应经过必经点附近（分段 FMM 拼接 + 段端点保留）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[{"lon":115.3,"lat":39.8,"alt_m":3000}]}
                ]
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        // 经过必经点（格距内 < 0.05°≈5km）
        let near_mid = out.vehicles[0].path.iter().any(|p| {
            (p.x - 115.3).abs() < 0.05 && (p.y - 39.8).abs() < 0.05
        });
        assert!(near_mid, "应经过必经点 (115.3,39.8): {:?}",
            out.vehicles[0].path.iter().map(|p| format!("({:.2},{:.2})", p.x, p.y)).take(6).collect::<Vec<_>>());
        // 北侧绕行 → 距离显著大于直线 164km
        assert!(out.vehicles[0].distance_m > 200_000.0, "dist {}", out.vehicles[0].distance_m);
    }

    #[test]
    fn m5_per_vehicle_independent_waypoints() {
        // 多机各自 mid_waypoints（主管拍板：每机独立序列）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[{"lon":115.3,"lat":39.8,"alt_m":3000}]},
                    {"id":"uav2","profile":{"aircraft_type":"ROTORCRAFT","cruise_speed_mps":60},
                     "start_pose":{"lon":115.2,"lat":39.1,"alt_m":2000,"heading_deg":90},
                     "mid_waypoints":[{"lon":116.1,"lat":39.2,"alt_m":2000}]}
                ]
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles.len(), 2);
        for v in &out.vehicles {
            assert_eq!(v.status, "planned");
        }
        // uav1 经 (115.3,39.8)；uav2 经 (116.1,39.2)（各自必经点独立）
        let u1_near = out.vehicles[0].path.iter().any(|p| (p.x - 115.3).abs() < 0.05 && (p.y - 39.8).abs() < 0.05);
        let u2_near = out.vehicles[1].path.iter().any(|p| (p.x - 116.1).abs() < 0.05 && (p.y - 39.2).abs() < 0.05);
        assert!(u1_near, "uav1 应经过 (115.3,39.8)");
        assert!(u2_near, "uav2 应经过 (116.1,39.2)");
        // 旋翼机无 smoothing 告警
        assert!(out.vehicles[1].warnings.is_empty(), "{:?}", out.vehicles[1].warnings);
    }

    #[test]
    fn m5_multiple_mid_waypoints_sequence() {
        // 多个必经点顺序经过（三段时间拼接）
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45},
                     "mid_waypoints":[
                        {"lon":115.3,"lat":39.8,"alt_m":3000},
                        {"lon":116.0,"lat":39.7,"alt_m":3000}
                     ]}
                ]
            }
        }"#;
        let out = solve(&parse(s), &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles[0].status, "planned");
        let p = &out.vehicles[0].path;
        // 两个必经点都经过（格距内）
        let m1 = p.iter().position(|q| (q.x - 115.3).abs() < 0.05 && (q.y - 39.8).abs() < 0.05);
        let m2 = p.iter().position(|q| (q.x - 116.0).abs() < 0.05 && (q.y - 39.7).abs() < 0.05);
        assert!(m1.is_some() && m2.is_some(), "必经点应都经过: m1={m1:?} m2={m2:?}");
        // 顺序：m1 在 m2 前
        assert!(m1.unwrap() < m2.unwrap(), "顺序应 m1 < m2");
    }

    #[test]
    fn m1_multi_vehicle() {
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":115.0,"lat":39.0,"alt_m":3000},
                "target":{"lon":116.5,"lat":39.9,"alt_m":3000},
                "terrain":{"source":"none"},
                "vehicles":[
                    {"id":"uav1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":100},
                     "start_pose":{"lon":115.0,"lat":39.0,"alt_m":3000,"heading_deg":45}},
                    {"id":"uav2","profile":{"aircraft_type":"ROTORCRAFT","cruise_speed_mps":50},
                     "start_pose":{"lon":115.5,"lat":39.2,"alt_m":2000,"heading_deg":90}}
                ]
            }
        }"#;
        let input = parse(s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        assert_eq!(out.vehicles.len(), 2);
        assert_eq!(out.vehicles[0].id, "uav1");
        assert_eq!(out.vehicles[1].id, "uav2");
        for v in &out.vehicles {
            assert_eq!(v.status, "planned");
            assert!(v.path.len() >= 2);
        }
        // 旋翼机（uav2）无 Dubins 链（急转合法）——路径点应不因拟合失败回退
        assert!(out.vehicles[1].warnings.is_empty(), "旋翼机不应有 smoothing 告警");
    }

    #[test]
    fn zigzag8_nodata_endpoints_do_not_staircase() {
        // 主管 2026-08-06 输入：无 zone/雷达，China DEM L12 真实地形。
        // start (118.10,38.57) 近渤海、target (111.78,43.79) 内蒙——两点均落在
        // china_dem_l12 的 NoData 空洞（起点带 t∈[0,0.062]、终点带 t∈[0.983,1.0]）。
        // 空洞策略（2026-08-04 拍板）：不设数据合格判断，对任意空洞给出可用结果，
        // 最坏降级警告进 stats.degradations。
        // 修复前：verify 对 NoData 保守硬拒 → 全链失败 → 回退 1196 点网格楼梯
        // 1138km + smoothing_failed（密集锯齿）。
        // 修复后：NoData 降级警告 + 网格伪影直线兜底 → 平滑直线 ~784km。
        let root = PathBuf::from(env!("CARGO_MANIFEST_DIR")).join("..");
        let cand = root.join("phase0/data/pending/china_dem_l12.arpack");
        if !cand.exists() {
            eprintln!("skip zigzag8: real terrain missing ({})", cand.display());
            return;
        }
        let s = r#"{
            "schema_version":"0.20",
            "mission":{
                "start":{"lon":118.09890161274431,"lat":38.5694180755746,"alt_m":1000},
                "target":{"lon":111.78290034358717,"lat":43.78646838602853,"alt_m":3000},
                "vehicles":[{"id":"v1","profile":{"aircraft_type":"FIXED_WING","cruise_speed_mps":250,
                    "min_turn_radius_m":442,"max_climb_angle_deg":15},
                    "start_pose":{"lon":118.09890161274431,"lat":38.5694180755746,"alt_m":3000,"heading_deg":45},
                    "mid_waypoints":[]}],
                "red_forces":{"radars":[]},
                "no_fly_zones":[],"restricted_zones":[],"obstacles":[],
                "terrain":{"source":"path","path":"__P__"},
                "parameters":{}
            }
        }"#;
        let s = s.replace("__P__", &cand.to_string_lossy().replace('\\', "\\\\"));
        let input = parse(&s);
        let out = solve(&input, &SolveParams::default(), 0).unwrap();
        let v = &out.vehicles[0];
        assert_eq!(v.status, "planned");
        assert!(
            v.warnings.iter().all(|w| !w.contains("smoothing_failed")),
            "NoData 空洞不应 smoothing_failed，实际 {:?}",
            v.warnings
        );
        assert!(
            v.path.len() <= 10,
            "应交付平滑直线（≈2 点），实际 {} 点",
            v.path.len()
        );
        // 直线 783.9km：交付距离应接近直线，远小于 raw 楼梯 1138km
        assert!(
            v.distance_m < 800_000.0,
            "应 ≈ 直线 784km，实际 {}km",
            v.distance_m / 1000.0
        );
        // 空洞策略：NoData 降级警告进 stats.degradations
        assert!(
            out.stats.degradations.iter().any(|d| d.contains("NoData")),
            "degradations 应含 NoData 降级，实际 {:?}",
            out.stats.degradations
        );
    }
}


// AircraftRouterPlanner Demo 前端类型 —— 匹配 cli/src/config.rs 输入/输出契约（schema 0.21；
// 2026-08-19 第二波：mission 包裹层已拍平、逐机显式 start/target、武器移入飞行器）。
// 坐标统一：经纬度（度，WGS84）+ MSL 高度（米）。三维场景采用局部等距投影
// （geoToLocal，见下方）：x=东（经度→米）、y=北（纬度→米）、z=高（米）。

// === 基础 ===
export type Vec3 = [number, number, number];
export type Vec2 = [number, number];

// 经纬高航路点（输入/输出统一语义；主管决策 2026-08-05）
export interface Waypoint {
  lon: number;
  lat: number;
  alt_m: number;
}

// === 输入类型（匹配 Input JSON） ===
export type AircraftType = 'FIXED_WING' | 'ROTORCRAFT';

export interface AircraftProfile {
  aircraft_type: AircraftType;
  cruise_speed_mps?: number;
  speed_range_mps?: [number, number];
  min_turn_radius_m?: number;
  max_climb_angle_deg?: number;
  max_bank_deg?: number;
  ceiling_m?: number;
}

export interface AircraftInput {
  id: string;
  /** 机型性能参数（整段省略 → 缺省固定翼占位） */
  profile?: AircraftProfile;
  /** 起点（必填） */
  start: Waypoint;
  /** 目标点（必填） */
  target: Waypoint;
  /** 中途必经点：start → mid[0..] → target（alt_m 为垂直剖面分段锚点） */
  mid_waypoints?: Waypoint[];
  /** 武器（出现即启用；缺省 = 点目标语义） */
  weapon?: WeaponInput;
}

export interface Radar {
  id: string;
  lon: number;
  lat: number;
  radius_km: number;
  alt_m?: number;
  suppression_post_range_km?: number;
  suppression_factor?: number;
}

export type ZoneType = 'no_fly' | 'restricted' | 'obstacle';

export interface CircleGeometry {
  center: [number, number]; // [lon, lat]
  radius_km: number;
}
export interface PolygonGeometry {
  vertices: [number, number][]; // [[lon, lat], ...]
}
export type ZoneGeometry = CircleGeometry | PolygonGeometry;

export interface Zone {
  id: string;
  /** zone_type 不入 JSON（后端按所属数组注入：no_fly_zones/restricted_zones/obstacles）；
   *  前端渲染时由 App 按数组打标（Scene3D 的 VisualZone） */
  shape: 'circle' | 'polygon';
  geometry: ZoneGeometry;
  /** 仅限飞区（restricted）需要高度区间；禁飞/障碍全高度禁入，省略（2026-08-12） */
  alt_min_m?: number;
  alt_max_m?: number;
}

export interface TerrainConfig {
  source: 'none' | 'builtin' | 'path';
  path?: string;
}

/** 场景地形显示配置（demo 显示配置；与 CLI 计算配置 Input.terrain 解耦，2026-08-20） */
export interface SceneTerrainConfig {
  /** follow = 跟随 CLI 计算配置（默认）；none = 无地形表面；path = 独立外部文件 */
  mode: 'follow' | 'none' | 'path';
  /** mode='path' 时的文件路径 */
  path?: string;
}

/** 显示配置地形解析：follow → 复用 CLI 计算配置（只读）；none/path → demo 显示配置独立。
 *  注意：follow 且 CLI 计算 source='builtin' 时，解析结果为 builtin —— 显示采样
 *  只认 path（tiles.ts wantTerrain），builtin 无表面（与现状一致）。 */
export function resolveSceneTerrain(
  scene: SceneTerrainConfig,
  planning: TerrainConfig,
): TerrainConfig {
  switch (scene.mode) {
    case 'follow': return planning;
    case 'none':   return { source: 'none' };
    case 'path':   return { source: 'path', path: scene.path };
  }
}

export interface ParamsOverride {
  radar_inflation?: number;
  /** 探测曲线形态：swerling1（默认，2026-08-13 base_p 标定——Swerling I 典型监视雷达模型，R_eff 处探测概率 0.9）/ exponential / linear */
  detection_curve?: string;
  p_cross?: number;
  suppression_delta?: number;
  los_mask_coef?: number;
}

// === 武器（匹配 cli/src/config.rs Weapon；2026-08-19 移入飞行器：weapon 出现即启用、weapon_type 必填） ===
export type WeaponType = 'aam' | 'agm' | 'bomb';

export interface LaunchEnvelope {
  /** 航向窗 [min, max]°（硬校验） */
  heading_deg?: [number, number];
  /** 高度窗 [min, max] m（硬校验） */
  alt_m?: [number, number];
  /** 速度窗 [min, max] m/s（软校验） */
  speed_mps?: [number, number];
}

export interface WeaponInput {
  /** 武器类型（aam / agm / bomb）。出现即启用，类型必填。 */
  weapon_type: WeaponType;
  /** 射程 [Rmin, Rmax] km；缺省 = 按类型默认（aam [5,40] / agm [3,120] / bomb [1,15]） */
  range_km?: [number, number];
  /** 发射包线（航向/高度/速度窗） */
  envelope?: LaunchEnvelope;
}

/** 武器类型默认射程 [Rmin, Rmax] km（与 cli/config.rs WeaponType::default_range_km 对齐） */
export const WEAPON_DEFAULT_RANGE_KM: Record<WeaponType, [number, number]> = {
  aam: [5, 40],
  agm: [3, 120],
  bomb: [1, 15],
};

/** 顶层输入（2026-08-19 第二波：mission 包裹层已拍平；逐机显式 start/target） */
export interface Input {
  /** 飞行器数组（必填非空；空数组 → missing_aircraft） */
  aircraft: AircraftInput[];
  red_forces: { radars: Radar[] };
  no_fly_zones: Zone[];
  restricted_zones: Zone[];
  obstacles: Zone[];
  terrain: TerrainConfig;
  parameters: ParamsOverride;
}

// === 输出类型（匹配 Output JSON） ===
export interface PathPoint {
  x: number; // 经度（度）
  y: number; // 纬度（度）
  alt_m: number; // MSL 米
}

export interface AircraftOutput {
  id: string;
  status: 'planned' | 'no_solution' | 'degraded';
  path: PathPoint[];
  distance_m: number;
  warnings: string[];
}

export interface OutputStats {
  fmm_ms: number;
  los_checks: number;
  degradations: string[];
}

export interface ErrorDetail {
  code: string;
  message: string;
}

export interface PlanResult {
  status: 'success' | 'degraded_timeout' | 'no_solution' | 'input_invalid';
  error?: ErrorDetail | null;
  elapsed_ms?: number;
  aircraft: AircraftOutput[];
  stats: OutputStats;
}

// === 地形网格（POST /api/terrain 响应） ===
export interface TerrainInfo {
  nx: number;
  ny: number;
  min_lon: number;
  min_lat: number;
  max_lon: number;
  max_lat: number;
  resolution: string;
  source_bounds: [number, number, number, number] | null;
  heights: (number | null)[];
}

// === 坐标工具：经纬高 → 局部平面（等距投影，场景以 ref 为原点） ===
export interface GeoRef {
  lon: number;
  lat: number;
}

export function geoToLocal(wp: Waypoint, ref: GeoRef, zScale = 1): Vec3 {
  const lat0 = (ref.lat * Math.PI) / 180;
  const x = (wp.lon - ref.lon) * 111320 * Math.cos(lat0);
  const y = (wp.lat - ref.lat) * 110574;
  // zScale：地形夸张系数统一作用于所有含高度对象（航路/标记/zone），
  // 保证航路与地形表面同一尺度（否则航路绝对高度 vs 夸张地形 → 视觉钻入山中）
  return [x, y, wp.alt_m * zScale];
}

export function geoPointToLocal(
  lon: number,
  lat: number,
  alt_m: number,
  ref: GeoRef,
  zScale = 1,
): Vec3 {
  return geoToLocal({ lon, lat, alt_m }, ref, zScale);
}

export function localToGeo(v: Vec3, ref: GeoRef): Waypoint {
  const lat0 = (ref.lat * Math.PI) / 180;
  const lon = ref.lon + v[0] / (111320 * Math.cos(lat0));
  const lat = ref.lat + v[1] / 110574;
  return { lon, lat, alt_m: v[2] };
}

// === 默认配置（北京近郊演示场景） ===
export function buildDefaultInput(): Input {
  return {
    aircraft: [
      {
        id: 'a1',
        profile: { aircraft_type: 'FIXED_WING' },
        start: { lon: 115.9, lat: 39.8, alt_m: 3000 },
        target: { lon: 116.4, lat: 40.0, alt_m: 1000 },
        mid_waypoints: [],
      },
    ],
    red_forces: { radars: [] },
    no_fly_zones: [],
    restricted_zones: [],
    obstacles: [],
    terrain: { source: 'none', path: undefined },
    parameters: {},
  };
}


// === 底图层（2026-08-13：掩膜 / GeoTIFF / WMS 三选一，主管定稿） ===
export type BaseMapSource = 'mask' | 'tiff' | 'wms' | 'none';
export type TiffProjection = 'auto' | '4326' | '3857';

export interface BaseMapConfig {
  source: BaseMapSource;
  /** mask/tiff 共用文件路径（mask 默认 data/mask_7p5as.mask，与默认地形同目录） */
  path?: string;
  /** tiff 投影：auto = 后端读 GeoKey 自动识别 */
  tiffProjection?: TiffProjection;
  /** wms：GeoServer WMS 端点（如 http://127.0.0.1:8080/geoserver/wms） */
  wmsUrl?: string;
  wmsLayers?: string;
  wmsCrs?: 'EPSG:4326' | 'EPSG:3857';
}

export interface BaseMapInfo {
  nx: number;
  ny: number;
  min_lon: number;
  min_lat: number;
  max_lon: number;
  max_lat: number;
  resolution: string;
  source: 'mask' | 'tiff';
  projection: 'mask' | '4326' | '3857';
  /** nx*ny*4，0..255；越界区域为透明 [0,0,0,0] */
  rgba: number[];
}

export function defaultBaseMapConfig(): BaseMapConfig {
  return {
    source: 'mask',
    path: 'data/mask_7p5as.mask',
    tiffProjection: 'auto',
    wmsUrl: 'http://127.0.0.1:8080/geoserver/wms',
    wmsLayers: 'workspace:layer',
    wmsCrs: 'EPSG:4326',
  };
}

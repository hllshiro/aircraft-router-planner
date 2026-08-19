// AircraftRouterPlanner Demo 前端类型 —— 匹配 cli/src/config.rs 输入/输出契约（schema 0.20）。
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

export interface VehicleProfile {
  aircraft_type: AircraftType;
  cruise_speed_mps?: number;
  speed_range_mps?: [number, number];
  min_turn_radius_m?: number;
  max_climb_angle_deg?: number;
  max_bank_deg?: number;
  ceiling_m?: number;
}

export interface VehiclePose {
  lon: number;
  lat: number;
  alt_m: number;
}

export interface VehicleInput {
  id: string;
  profile: VehicleProfile;
  start_pose: VehiclePose;
  /** 目标引用：缺省 / "mission.target" = mission.target；"lon,lat[,alt]" = 自定义坐标（每机独立终点） */
  target_ref?: string;
  mid_waypoints?: Waypoint[];
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
  zone_type: ZoneType;
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

export interface ParamsOverride {
  radar_inflation?: number;
  /** 探测曲线形态：swerling1（默认，2026-08-13 base_p 标定——Swerling I 典型监视雷达模型，R_eff 处探测概率 0.9）/ exponential / linear */
  detection_curve?: string;
  p_cross?: number;
  suppression_delta?: number;
  los_mask_coef?: number;
}

// === 武器（匹配 cli/src/config.rs WeaponEntry；P6-D 2026-08-12） ===
export type WeaponType = 'aam' | 'agm' | 'bomb';

export interface WeaponInput {
  weapon_id: string;
  /** 缺省 = 该武器默认不启用（不参与规划计算） */
  weapon_type?: WeaponType;
  /** 射程 [Rmin, Rmax] km；缺省 = 按类型默认（aam [5,40] / agm [3,120] / bomb [1,15]） */
  range_km?: [number, number];
}

/** 武器类型默认射程 [Rmin, Rmax] km（与 cli/config.rs WeaponType::default_range_km 对齐） */
export const WEAPON_DEFAULT_RANGE_KM: Record<WeaponType, [number, number]> = {
  aam: [5, 40],
  agm: [3, 120],
  bomb: [1, 15],
};

export interface Mission {
  start: Waypoint;
  target: Waypoint;
  vehicles: VehicleInput[];
  red_forces: { radars: Radar[] };
  no_fly_zones: Zone[];
  restricted_zones: Zone[];
  obstacles: Zone[];
  terrain: TerrainConfig;
  weapons: WeaponInput[];
  parameters: ParamsOverride;
}

export interface InputConfig {
  mission: Mission;
}

// === 输出类型（匹配 Output JSON） ===
export interface PathPoint {
  x: number; // 经度（度）
  y: number; // 纬度（度）
  alt_m: number; // MSL 米
}

export interface VehicleOutput {
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
  vehicles: VehicleOutput[];
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

/** 每机自定义目标（target_ref = "lon,lat[,alt]"）；缺省 / "mission.target" → null */
export function parseVehicleTargetRef(
  v: VehicleInput,
  missionTarget: Waypoint,
): Waypoint | null {
  const r = (v.target_ref ?? '').trim();
  if (!r || r === 'mission.target') return null;
  const parts = r.split(',').map((s) => s.trim());
  if (parts.length < 2) return null;
  const lon = +parts[0];
  const lat = +parts[1];
  if (!Number.isFinite(lon) || !Number.isFinite(lat)) return null;
  const alt =
    parts.length >= 3 && Number.isFinite(+parts[2])
      ? +parts[2]
      : missionTarget.alt_m;
  return { lon, lat, alt_m: alt };
}

export function localToGeo(v: Vec3, ref: GeoRef): Waypoint {
  const lat0 = (ref.lat * Math.PI) / 180;
  const lon = ref.lon + v[0] / (111320 * Math.cos(lat0));
  const lat = ref.lat + v[1] / 110574;
  return { lon, lat, alt_m: v[2] };
}

// === 默认配置（北京近郊演示场景） ===
export function defaultInputConfig(): InputConfig {
  return {
    mission: {
      start: { lon: 115.9, lat: 39.8, alt_m: 3000 },
      target: { lon: 116.8, lat: 40.3, alt_m: 3000 },
      vehicles: [
        {
          id: 'v1',
          profile: {
            aircraft_type: 'FIXED_WING',
            cruise_speed_mps: 250,
            min_turn_radius_m: 442,
            max_climb_angle_deg: 15,
          },
          start_pose: { lon: 115.9, lat: 39.8, alt_m: 3000 },
          mid_waypoints: [],
        },
      ],
      red_forces: { radars: [] },
      no_fly_zones: [],
      restricted_zones: [],
      obstacles: [],
      weapons: [],
      // 默认真实地形：east_asia_7p5as ARPK1（~537MB，GMTED2010 东亚 7.5as，70-135E, 15-55N）
      // 与发布版（install/）默认地形对齐；路径相对 workspace 根（demo-server 与 CLI 的 cwd）
      terrain: { source: 'path', path: 'data/east_asia_7p5as.arpack' },
      parameters: {},
    },
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

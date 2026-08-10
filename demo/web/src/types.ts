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
  detection_probability?: number;
}

export interface VehiclePose {
  lon: number;
  lat: number;
  alt_m: number;
  heading_deg?: number;
}

export interface VehicleInput {
  id: string;
  profile: VehicleProfile;
  start_pose: VehiclePose;
  mid_waypoints?: Waypoint[];
}

export type RadarType = 'early_warning' | 'tracking' | 'fire_control';

export interface Radar {
  id: string;
  lon: number;
  lat: number;
  radar_type: RadarType;
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
  alt_min_m: number;
  alt_max_m: number;
  height_semantics?: 'msl' | 'agl';
}

export interface TerrainConfig {
  source: 'none' | 'builtin' | 'path';
  path?: string;
  resolution_m?: number;
}

export interface ParamsOverride {
  radar_inflation?: number;
  detection_curve?: string;
  p_cross?: number;
  suppression_delta?: number;
  los_mask_coef?: number;
}

export interface Mission {
  start: Waypoint;
  target: Waypoint;
  vehicles: VehicleInput[];
  red_forces: { radars: Radar[] };
  no_fly_zones: Zone[];
  restricted_zones: Zone[];
  obstacles: Zone[];
  terrain: TerrainConfig;
  parameters: ParamsOverride;
}

export interface InputConfig {
  schema_version: string;
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
  schema_version: string;
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

export function geoToLocal(wp: Waypoint, ref: GeoRef): Vec3 {
  const lat0 = (ref.lat * Math.PI) / 180;
  const x = (wp.lon - ref.lon) * 111320 * Math.cos(lat0);
  const y = (wp.lat - ref.lat) * 110574;
  return [x, y, wp.alt_m];
}

export function geoPointToLocal(lon: number, lat: number, alt_m: number, ref: GeoRef): Vec3 {
  return geoToLocal({ lon, lat, alt_m }, ref);
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
    schema_version: '0.20',
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
          start_pose: { lon: 115.9, lat: 39.8, alt_m: 3000, heading_deg: 45 },
          mid_waypoints: [],
        },
      ],
      red_forces: { radars: [] },
      no_fly_zones: [],
      restricted_zones: [],
      obstacles: [],
      // 默认真实地形：east_asia_7p5as ARPK1（~537MB，GMTED2010 东亚 7.5as，70-135E, 15-55N）
      // 与发布版（install/）默认地形对齐；路径相对 workspace 根（demo-server 与 CLI 的 cwd）
      terrain: { source: 'path', path: 'data/east_asia_7p5as.arpack' },
      parameters: {},
    },
  };
}

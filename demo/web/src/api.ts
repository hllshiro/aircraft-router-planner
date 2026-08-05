import type { InputConfig, PlanResult, TerrainInfo } from './types';

export async function planRoute(config: InputConfig): Promise<PlanResult> {
  const resp = await fetch('/api/plan', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(config),
  });
  return resp.json();
}

export async function fetchTerrain(
  path: string,
  bbox: [number, number, number, number],
  grid: [number, number] = [64, 64],
): Promise<TerrainInfo> {
  const resp = await fetch('/api/terrain', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({ path, bbox, grid }),
  });
  const data = await resp.json();
  if (data.error) {
    throw new Error(data.error);
  }
  return data as TerrainInfo;
}

/** 场景包围盒：start/target 中心，跨度至少覆盖初始相机视野（~130km），避免地形网格只覆盖路径附近 */
export function sceneBounds(
  config: InputConfig,
): [number, number, number, number] {
  const { start, target } = config.mission;
  const minLon = Math.min(start.lon, target.lon);
  const maxLon = Math.max(start.lon, target.lon);
  const minLat = Math.min(start.lat, target.lat);
  const maxLat = Math.max(start.lat, target.lat);
  // 最小跨度：1.6° 经度（≈ 北纬40° 136km）× 1.4° 纬度（≈ 155km），中心 ±60% 覆盖
  const spanLon = Math.max(maxLon - minLon, 1.6);
  const spanLat = Math.max(maxLat - minLat, 1.4);
  const cLon = (minLon + maxLon) / 2;
  const cLat = (minLat + maxLat) / 2;
  return [
    cLon - spanLon * 0.6,
    cLat - spanLat * 0.6,
    cLon + spanLon * 0.6,
    cLat + spanLat * 0.6,
  ];
}

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

/** 场景包围盒：start/target 中心，跨度按 start→target 距离自适应（1.4 倍 + 最小 2.5°×2.2°），
 *  覆盖全场景 + 四周余量，避免地形网格只覆盖路径附近（主管 2026-08-06：地形块太小）。 */
export function sceneBounds(
  config: InputConfig,
): [number, number, number, number] {
  const { start, target } = config.mission;
  const minLon = Math.min(start.lon, target.lon);
  const maxLon = Math.max(start.lon, target.lon);
  const minLat = Math.min(start.lat, target.lat);
  const maxLat = Math.max(start.lat, target.lat);
  // 跨度 = max(实际距离 × 1.4, 最小 2.5°lon ≈ 220km / 2.2°lat ≈ 245km)
  const spanLon = Math.max((maxLon - minLon) * 1.4, 2.5);
  const spanLat = Math.max((maxLat - minLat) * 1.4, 2.2);
  const cLon = (minLon + maxLon) / 2;
  const cLat = (minLat + maxLat) / 2;
  return [
    cLon - spanLon / 2,
    cLat - spanLat / 2,
    cLon + spanLon / 2,
    cLat + spanLat / 2,
  ];
}

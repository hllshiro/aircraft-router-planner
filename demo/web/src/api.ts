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

/** 场景包围盒：start/target 范围 + 15% 扩展（用于地形采样） */
export function sceneBounds(
  config: InputConfig,
): [number, number, number, number] {
  const { start, target } = config.mission;
  const minLon = Math.min(start.lon, target.lon);
  const maxLon = Math.max(start.lon, target.lon);
  const minLat = Math.min(start.lat, target.lat);
  const maxLat = Math.max(start.lat, target.lat);
  const dLon = Math.max(maxLon - minLon, 0.05);
  const dLat = Math.max(maxLat - minLat, 0.05);
  return [
    minLon - dLon * 0.15,
    minLat - dLat * 0.15,
    maxLon + dLon * 0.15,
    maxLat + dLat * 0.15,
  ];
}

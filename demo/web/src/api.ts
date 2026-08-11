import type { InputConfig, PlanResult, TerrainInfo } from './types';
import { parseVehicleTargetRef } from './types';

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
  bbox?: [number, number, number, number] | null,
  grid?: [number, number] | null,
): Promise<TerrainInfo> {
  const resp = await fetch('/api/terrain', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    // bbox/grid 缺省（null/undefined）→ server 用数据范围 + 按跨度自适应精度
    body: JSON.stringify({ path, bbox: bbox ?? undefined, grid: grid ?? undefined }),
  });
  const data = await resp.json();
  if (data.error) {
    throw new Error(data.error);
  }
  return data as TerrainInfo;
}

/** 场景包围盒：start/target + 各机起点（vehicles[].start_pose）包围，跨度按
 *  start→target 距离自适应（1.4 倍 + 最小 2.5°×2.2°），覆盖全场景 + 四周余量，
 *  避免地形网格只覆盖路径附近（主管 2026-08-06：地形块太小；2026-08-08：多机起点）。 */
export function sceneBounds(
  config: InputConfig,
): [number, number, number, number] {
  const { start, target, vehicles } = config.mission;
  let minLon = Math.min(start.lon, target.lon);
  let maxLon = Math.max(start.lon, target.lon);
  let minLat = Math.min(start.lat, target.lat);
  let maxLat = Math.max(start.lat, target.lat);
  for (const v of vehicles) {
    minLon = Math.min(minLon, v.start_pose.lon);
    maxLon = Math.max(maxLon, v.start_pose.lon);
    minLat = Math.min(minLat, v.start_pose.lat);
    maxLat = Math.max(maxLat, v.start_pose.lat);
    // 每机自定义目标（target_ref）纳入包围盒 → 点击设置终点在场景外时 bbox 自动扩展
    const vt = parseVehicleTargetRef(v, target);
    if (vt) {
      minLon = Math.min(minLon, vt.lon);
      maxLon = Math.max(maxLon, vt.lon);
      minLat = Math.min(minLat, vt.lat);
      maxLat = Math.max(maxLat, vt.lat);
    }
  }
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

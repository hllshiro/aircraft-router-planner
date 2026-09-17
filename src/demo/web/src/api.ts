import type { Input, PlanResult, TerrainInfo, BaseMapInfo, TiffProjection, DataFilesResponse } from './types';

/** 解析后端响应并兜底：空 body / 非 JSON / 非 2xx → 明确错误，
 *  避免 resp.json() 裸调用抛 "Unexpected end of JSON input"（2026-08-13 修复）。 */
async function readJsonResponse(resp: Response, what: string): Promise<any> {
  const text = await resp.text();
  if (!text) {
    throw new Error(`${what}: 后端返回空响应（HTTP ${resp.status}）`);
  }
  let data: any;
  try {
    data = JSON.parse(text);
  } catch {
    throw new Error(
      `${what}: 后端返回非 JSON 响应（HTTP ${resp.status}）: ${text.slice(0, 160)}`,
    );
  }
  if (!resp.ok && !data.error) {
    throw new Error(`${what}: HTTP ${resp.status}`);
  }
  return data;
}

export async function planRoute(config: Input): Promise<PlanResult> {
  const resp = await fetch('/api/plan', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(config),
  });
  return readJsonResponse(resp, '规划');
}

export async function fetchTerrain(
  path: string,
  bbox?: [number, number, number, number] | null,
  grid?: [number, number] | null,
): Promise<TerrainInfo> {
  const resp = await fetch('/api/terrain', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    // bbox/grid 显式 null → server 用数据范围 + 按跨度自适应精度
    // （注意：不能省略字段——旧 server 的 bbox 非 Option 时缺字段会 400 plain text）
    body: JSON.stringify({
      path,
      bbox: bbox ?? null,
      grid: grid ?? null,
    }),
  });
  const data = await readJsonResponse(resp, '地形');
  if (data.error) {
    throw new Error(data.error);
  }
  return data as TerrainInfo;
}

/** 查询单点地面海拔（MSL 米）；范围外/无数据/失败 → null。
 *  2026-08-14：起终点高度输入框最小高度 = 该点地面海拔。 */
export async function fetchElevation(
  path: string,
  lon: number,
  lat: number,
): Promise<number | null> {
  try {
    const resp = await fetch('/api/elevation', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({ path, lon, lat }),
    });
    const data = await readJsonResponse(resp, '海拔');
    if (data.error) {
      return null;
    }
    const e = data.elevation_m;
    return typeof e === 'number' && Number.isFinite(e) ? e : null;
  } catch {
    return null; // 查询失败静默：输入框无 min，不影响使用
  }
}

/** 场景包围盒：逐机 start/target 包围，跨度按 start→target 距离自适应（1.4 倍 +
 *  最小 2.5°×2.2°），覆盖全场景 + 四周余量，避免地形网格只覆盖路径附近
 * （主管 2026-08-06：地形块太小；2026-08-19：mission 拍平，逐机显式 start/target）。 */
export function sceneBounds(config: Input): [number, number, number, number] {
  const a = config.aircraft[0];
  const start = a.start;
  const target = a.target;
  let minLon = Math.min(start.lon, target.lon);
  let maxLon = Math.max(start.lon, target.lon);
  let minLat = Math.min(start.lat, target.lat);
  let maxLat = Math.max(start.lat, target.lat);
  for (const ac of config.aircraft) {
    minLon = Math.min(minLon, ac.start.lon, ac.target.lon);
    maxLon = Math.max(maxLon, ac.start.lon, ac.target.lon);
    minLat = Math.min(minLat, ac.start.lat, ac.target.lat);
    maxLat = Math.max(maxLat, ac.start.lat, ac.target.lat);
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


// === 底图层（2026-08-13：掩膜 / GeoTIFF / WMS 三选一） ===

/** mask/tiff：后端统一输出「经纬度 bbox 对齐的 RGBA 网格」，前端按 bbox 贴图（投影无感） */
export async function fetchBaseMap(
  path: string,
  source: 'mask' | 'tiff',
  bbox?: [number, number, number, number] | null,
  grid?: [number, number] | null,
  projection?: TiffProjection,
): Promise<BaseMapInfo> {
  const resp = await fetch('/api/basemap', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      source,
      path,
      bbox: bbox ?? null,
      grid: grid ?? null,
      projection: source === 'tiff' ? (projection ?? 'auto') : undefined,
    }),
  });
  const data = await readJsonResponse(resp, '底图');
  if (data.error) {
    throw new Error(data.error);
  }
  return data as BaseMapInfo;
}

// ===================== /api/tile（瓦片合并端点） =====================

/** 路径清洗：去除资源管理器复制路径时混入的不可见方向字符（U+202A 等）与首尾空白。
 *  2026-08-13：主管粘贴 `D:\workspace\...\HYP.tif` 时带 U+202A → 后端打不开文件 →
 *  整瓦片失败空白；输入与请求两处都清洗，双保险。 */
export function sanitizePath(p: string): string {
  return p.replace(/[\u202a\u202b\u202c\u200e\u200f\ufeff]/g, '').trim();
}

/** base64 RGBA → number[]（与 BaseMapInfo.rgba 兼容；atob 浏览器内置） */
function base64DecodeRgba(b64: string): number[] {
  const bin = atob(b64);
  const out = new Array<number>(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

/**
 * 瓦片请求：一次 /api/tile 返回地形 + 底图（同 bbox 同 grid）。
 * 2026-08-13 瓦片化：每瓦片 1 请求替代 2 请求；rgba 为 base64（JSON 体积减 ~60%）。
 */
export async function fetchTile(params: {
  terrainPath: string | null;
  basemap: { source: 'mask' | 'tiff'; path: string; projection?: TiffProjection } | null;
  bbox: [number, number, number, number];
  grid: [number, number];
}): Promise<{ terrain: TerrainInfo | null; basemap: BaseMapInfo | null; basemapError: string | null }> {
  const resp = await fetch('/api/tile', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify({
      terrain_path: params.terrainPath ? sanitizePath(params.terrainPath) : null,
      basemap: params.basemap
        ? {
            source: params.basemap.source,
            path: sanitizePath(params.basemap.path),
            projection: params.basemap.projection,
          }
        : null,
      bbox: params.bbox,
      grid: params.grid,
    }),
  });
  const data = await readJsonResponse(resp, '瓦片');
  if (data.error) {
    throw new Error(data.error);
  }
  const { basemap: bmRaw, ...terrain } = data;
  // 底图错误不整体抛（2026-08-13）：地形照常渲染，底图失败单独记录，
  // 避免"底图打不开 → 整瓦片（含地形）消失 → 画面空白无提示"
  if (bmRaw?.error) {
    return {
      terrain: params.terrainPath ? (terrain as TerrainInfo) : null,
      basemap: null,
      basemapError: `底图: ${bmRaw.error}`,
    };
  }
  let bm: BaseMapInfo | null = null;
  if (bmRaw) {
    const raw = bmRaw as BaseMapInfo & { rgba_b64?: string };
    if (raw.rgba_b64) {
      raw.rgba = base64DecodeRgba(raw.rgba_b64);
      delete raw.rgba_b64;
    }
    bm = raw as BaseMapInfo;
  }
  return {
    terrain: params.terrainPath ? (terrain as TerrainInfo) : null,
    basemap: bm,
    basemapError: null,
  };
}

/** WMS GetMap（后端代理，固定 WMS 1.1.1 + SRS）：返回 blob URL，前端 TextureLoader 加载 */export async function fetchWmsBlob(params: {
  base_url: string;
  bbox: [number, number, number, number];
  width: number;
  height: number;
  layers: string;
  crs: string;
}): Promise<string> {
  const q = new URLSearchParams({
    base_url: params.base_url,
    bbox: params.bbox.join(','),
    width: String(params.width),
    height: String(params.height),
    layers: params.layers,
    crs: params.crs,
    format: 'image/png',
  });
  const resp = await fetch(`/api/wms?${q.toString()}`);
  if (!resp.ok) {
    const text = await resp.text();
    let msg = text;
    try {
      msg = JSON.parse(text).error ?? text;
    } catch {
      /* 非 JSON（GeoServer 直接错误）保留原文 */
    }
    throw new Error(msg);
  }
  const blob = await resp.blob();
  return URL.createObjectURL(blob);
}

/** WMS 请求尺寸：按 bbox 纵横比（米）取 512² 基准，clamp [256, 1024] */
export function wmsSize(
  bbox: [number, number, number, number],
): [number, number] {
  const [minLon, minLat, maxLon, maxLat] = bbox;
  const kx = 111320 * Math.cos((((minLat + maxLat) / 2) * Math.PI) / 180);
  const wM = Math.max((maxLon - minLon) * kx, 1);
  const hM = Math.max((maxLat - minLat) * 110574, 1);
  const ratio = Math.min(Math.max(hM / wM, 0.25), 4);
  const target = 512;
  let w = Math.round(target * Math.sqrt(1 / ratio));
  let h = Math.round(target * Math.sqrt(ratio));
  w = Math.min(Math.max(w, 256), 1024);
  h = Math.min(Math.max(h, 256), 1024);
  return [w, h];
}

export async function fetchDataFiles(): Promise<DataFilesResponse> {
  const resp = await fetch('/api/data-files');
  return readJsonResponse(resp, '数据文件扫描');
}

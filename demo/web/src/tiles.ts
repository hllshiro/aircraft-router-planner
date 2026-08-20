// 视口瓦片加载（2026-08-13 主管：相机视口完全独立，漫游到哪加载到哪；
// 地形/底图按瓦片增量加载，视口外 LRU 缓存，WMS 按视口级单图覆盖）。
// 2026-08-13 修订：层级切换冷却 + 渲染视口内瓦片（旧层级兜底、新层级覆盖，
// 无空洞）+ LRU 只删视口外瓦片——修复缩放时频繁重载与空洞。
import { useCallback, useEffect, useRef, useState } from 'react';
import * as THREE from 'three';
import type {
  GeoRef,
  TerrainInfo,
  BaseMapInfo,
  BaseMapConfig,
  TerrainConfig,
} from './types';
import { fetchTile, fetchWmsBlob, wmsSize } from './api';

// ===== 类型 =====
export interface TileEntry {
  key: string;
  bbox: [number, number, number, number];
  terrain: TerrainInfo | null;
  terrainError: string | null;
  /** mask/tiff 瓦片级底图（WMS 不按瓦片，见 ViewportTilesState.wms） */
  baseMap: BaseMapInfo | null;
  baseMapError: string | null;
}

/** 视口级 WMS 图（按相机视口 bbox 请求一张，覆盖所有可见瓦片） */
export interface WmsTile {
  url: string;
  bbox: [number, number, number, number];
}

export interface ViewportTilesState {
  /** 渲染瓦片（视口内；旧层级兜底在前、活跃层级覆盖在后） */
  tiles: TileEntry[];
  /** WMS 视口图（仅 source=wms 时非空） */
  wms: WmsTile | null;
  loading: boolean;
  error: string | null;
}

export interface ViewportTilesOptions {
  terrainConfig: TerrainConfig;
  baseMapConfig: BaseMapConfig;
  cameraRef: React.RefObject<THREE.PerspectiveCamera | null>;
  /** drei OrbitControls 实例（EventDispatcher；change → 节流刷新视口） */
  controlsRef: React.RefObject<any>;
  geoRef: GeoRef;
  /** Canvas 挂载完成（camera/controls 就绪）后由 Scene3D 置 true */
  sceneReady: boolean;
}

// ===== 常量 =====
/** 视口外缓存瓦片上限（GPU 内存/顶点数控制；128² 网格 × 24 ≈ 393K 顶点） */
/** LRU 缓存上限：拉远加载大跨度瓦片 + 拉近局部瓦片混合，48 足够 */
const CACHE_LIMIT = 48;
/** 瓦片请求并发上限 */
const CONCURRENCY = 4;
/** 相机 change → 视口刷新节流（ms） */
const THROTTLE_MS = 400;
/** 层级切换冷却（ms）：缩放过程中保持当前层级，避免相邻层级来回抖动全量重载 */
const COOLDOWN_MS = 1200;
/** 瓦片跨度层级：0.05°×2^k（0.05..90）；视口内 ≤4 瓦片取最小层级。
 *  2026-08-13 扩展：主管要求 Google Earth 式缩放——拉远看全貌（最大 90° 全球 2×2 瓦片）、
 *  拉近看局部（0.05°）。原上限 6.4° 导致拉远后只能看一小块。 */
const TILE_SPANS = [0.05, 0.1, 0.2, 0.4, 0.8, 1.6, 3.2, 6.4, 12.8, 25.6, 51.2, 90];

// ===== 纯函数 =====

/** 相机视口 → 经纬 bbox（中心尺度法，2026-08-13 修订）：
 *  中心 = 视线与地面交点（或相机正下方）；可见范围 = 相机高度 × fov / 俯角
 *  （屏幕中心处的地面尺度），与视锥远角无关 → 倾斜视角下 bbox 平滑稳定，
 *  不会因远角在地面投影拉远而剧烈变化（修复远方频繁加载）。
 *  相机基本平视/朝上（看不到地面）→ null。 */
export function viewportBBox(
  camera: THREE.PerspectiveCamera,
  geoRef: GeoRef,
): [number, number, number, number] | null {
  camera.updateMatrixWorld();
  const camPos = new THREE.Vector3().setFromMatrixPosition(camera.matrixWorld);
  const H = camPos.y;
  if (!(H > 1)) return null;
  const dir = new THREE.Vector3(0, 0, -1).applyQuaternion(camera.quaternion);
  const down = -dir.y;
  if (down < 0.02) return null; // 基本平视/朝上
  // 俯角下限：低视角时限制可见范围膨胀（平视时远方不加载过多瓦片）
  const effDown = Math.max(down, 0.25);
  // 视线与地面交点（中心点）
  let cx: number;
  let cz: number;
  if (down > 1e-6) {
    const t = H / down;
    cx = camPos.x + dir.x * t;
    cz = camPos.z + dir.z * t;
  } else {
    cx = camPos.x;
    cz = camPos.z;
  }
  // 屏幕中心处地面可见范围（米）
  const fov = (camera.fov * Math.PI) / 180;
  const aspect = camera.aspect || 1;
  const visH = (2 * H * Math.tan(fov / 2)) / effDown;
  const visW = visH * aspect;
  // 跨度上限（米 ≈ 2 万 km，覆盖全球）：拉远可看全貌（Google Earth 式）。
  // 2026-08-13 扩展：原 300km 把视口 clamp 在 ~2.7°，相机拉再高也只能看一小块。
  const MAX_SPAN_M = 20000000;
  const halfH = Math.min(visH / 2, MAX_SPAN_M / 2);
  const halfW = Math.min(visW / 2, MAX_SPAN_M / 2);
  // 米 → 经纬（等距投影，geoRef 校正）
  // 场景坐标 2026-08-14 起：x=东、y=上、z=-北（物理右手系，修复朝北看镜像）
  // → 地面交点 z（米）= -北，纬度 = ref.lat - z / ky
  const lat0 = (geoRef.lat * Math.PI) / 180;
  const kx = 111320 * Math.cos(lat0);
  const ky = 110574;
  // clamp 到全球范围（拉远后 bbox 可能超出 ±180/±90）
  const clamp = (v: number, lo: number, hi: number) => Math.min(Math.max(v, lo), hi);
  return [
    clamp(geoRef.lon + (cx - halfW) / kx, -180, 180),
    clamp(geoRef.lat - (cz + halfH) / ky, -90, 90),
    clamp(geoRef.lon + (cx + halfW) / kx, -180, 180),
    clamp(geoRef.lat - (cz - halfH) / ky, -90, 90),
  ];
}

/** 视口 bbox → 期望瓦片跨度层级（视口内 ≤4 瓦片） */
export function pickSpan(bbox: [number, number, number, number]): number {
  const spanDeg = Math.max(bbox[2] - bbox[0], bbox[3] - bbox[1]);
  const minSpan = spanDeg / 4;
  for (const s of TILE_SPANS) {
    if (s >= minSpan) return s;
  }
  return TILE_SPANS[TILE_SPANS.length - 1];
}

/** 覆盖 bbox（外扩半瓦片余量）的瓦片集合；全球网格 0 原点对齐 → 相机移动时
 *  大部分瓦片 key 稳定，只有边缘瓦片变化（增量加载生效）。 */
export function coverTiles(
  bbox: [number, number, number, number],
  span: number,
): { key: string; bbox: [number, number, number, number] }[] {
  const minLon = bbox[0] - span / 2;
  const maxLon = bbox[2] + span / 2;
  const minLat = bbox[1] - span / 2;
  const maxLat = bbox[3] + span / 2;
  const c0 = Math.floor(minLon / span);
  const c1 = Math.floor(maxLon / span);
  const r0 = Math.floor(minLat / span);
  const r1 = Math.floor(maxLat / span);
  const out: { key: string; bbox: [number, number, number, number] }[] = [];
  for (let r = r0; r <= r1; r++) {
    // 跳过纬度越界瓦片（如 span=90 时 row=-1 覆盖 -180~-90，无意义）
    if ((r + 1) * span <= -90 || r * span >= 90) continue;
    for (let c = c0; c <= c1; c++) {
      // 跳过经度越界瓦片
      if ((c + 1) * span <= -180 || c * span >= 180) continue;
      out.push({
        key: `${span}|${c}|${r}`,
        bbox: [c * span, r * span, (c + 1) * span, (r + 1) * span],
      });
    }
  }
  return out;
}

function bboxIntersect(
  a: [number, number, number, number],
  b: [number, number, number, number],
): boolean {
  return a[0] < b[2] && a[2] > b[0] && a[1] < b[3] && a[3] > b[1];
}

// ===== 瓦片加载 =====

async function loadTile(
  t: { key: string; bbox: [number, number, number, number] },
  cache: Map<string, TileEntry>,
  terrainConfig: TerrainConfig,
  baseMapConfig: BaseMapConfig,
): Promise<TileEntry> {
  const entry: TileEntry = {
    key: t.key,
    bbox: t.bbox,
    terrain: null,
    terrainError: null,
    baseMap: null,
    baseMapError: null,
  };
  cache.set(t.key, entry);
  const wantTerrain = terrainConfig.source === 'path' && terrainConfig.path;
  const src = baseMapConfig.source;
  const wantBase = (src === 'mask' || src === 'tiff') && baseMapConfig.path;
  if (!wantTerrain && !wantBase) return entry;
  // 合并端点 /api/tile：每瓦片 1 请求返回地形+底图（rgba base64，2026-08-13）
  try {
    const res = await fetchTile(
      {
        terrainPath: wantTerrain ? (terrainConfig.path ?? null) : null,
        basemap: wantBase
          ? {
              source: src as 'mask' | 'tiff',
              path: baseMapConfig.path as string,
              projection: baseMapConfig.tiffProjection,
            }
          : null,
        bbox: t.bbox,
        grid: [128, 128],
      },
    );
    if (res.terrain) entry.terrain = res.terrain;
    if (res.basemap) entry.baseMap = res.basemap;
    if (res.basemapError) entry.baseMapError = res.basemapError;
    if (wantTerrain && !res.terrain) entry.terrainError = '瓦片: 后端未返回地形';
  } catch (e) {
    if (wantTerrain) entry.terrainError = String(e);
    if (wantBase) entry.baseMapError = String(e);
  }
  return entry;
}

// ===== Hook =====

export function useViewportTiles(opts: ViewportTilesOptions): ViewportTilesState {
  const { terrainConfig, baseMapConfig, cameraRef, controlsRef, geoRef, sceneReady } = opts;
  const cacheRef = useRef<Map<string, TileEntry>>(new Map());
  const pendingRef = useRef<Set<string>>(new Set());
  const activeRef = useRef(0);
  const timerRef = useRef<number | null>(null);
  const prevWmsUrlRef = useRef<string | null>(null);
  // 活跃层级 + 上次切换时间（缩放冷却，防层级抖动全量重载）
  const activeSpanRef = useRef<number | null>(null);
  const lastSwitchRef = useRef(0);
  const [tiles, setTiles] = useState<TileEntry[]>([]);
  const [wms, setWms] = useState<WmsTile | null>(null);
  const [loading, setLoading] = useState(false);
  const [error, setError] = useState<string | null>(null);

  // 渲染列表 = 缓存中与视口（外扩余量）相交的瓦片；
  // 排序：非活跃层级在前（兜底，新瓦片未就绪时显示），活跃层级在后（覆盖）。
  const updateRender = useCallback(() => {
    const cache = cacheRef.current;
    const cam = cameraRef.current;
    const span = activeSpanRef.current ?? 0;
    if (!cam) {
      setTiles([]);
      return;
    }
    const vp = viewportBBox(cam, geoRef);
    if (!vp) {
      setTiles([]);
      return;
    }
    const vpExpanded: [number, number, number, number] = [
      vp[0] - span,
      vp[1] - span,
      vp[2] + span,
      vp[3] + span,
    ];
    const render = [...cache.values()].filter(
      (e) =>
        (e.terrain || e.baseMap || baseMapConfig.source === 'wms') &&
        bboxIntersect(e.bbox, vpExpanded),
    );
    render.sort((a, b) => {
      const aActive = a.key.startsWith(`${span}|`);
      const bActive = b.key.startsWith(`${span}|`);
      if (aActive === bActive) return 0;
      return aActive ? 1 : -1;
    });
    setTiles(render);
  }, [cameraRef, geoRef, baseMapConfig.source]);

  // LRU trim：只删视口外瓦片（视口内旧层级兜底瓦片受保护，避免空洞）
  const trimCache = useCallback(() => {
    const cache = cacheRef.current;
    const cam = cameraRef.current;
    const span = activeSpanRef.current ?? 0;
    if (!cam) return;
    const vp = viewportBBox(cam, geoRef);
    if (!vp) return;
    const vpExpanded: [number, number, number, number] = [
      vp[0] - span,
      vp[1] - span,
      vp[2] + span,
      vp[3] + span,
    ];
    const keys = [...cache.keys()]; // Map 插入序 = LRU 序（旧→新）
    let i = 0;
    while (cache.size > CACHE_LIMIT && i < keys.length) {
      const k = keys[i++];
      const e = cache.get(k);
      if (!e) continue;
      if (bboxIntersect(e.bbox, vpExpanded)) continue; // 保护视口内（含兜底）
      cache.delete(k);
    }
  }, [cameraRef, geoRef]);

  const refresh = useCallback(() => {
    const cam = cameraRef.current;
    if (!cam) return;
    const vp = viewportBBox(cam, geoRef);
    if (!vp) return;
    const desired = pickSpan(vp);
    // 层级决策：冷却期内保持当前层级（防抖）；否则切换（配置变化后 activeSpan=null 强制重选）
    const now = Date.now();
    if (
      activeSpanRef.current === null ||
      now - lastSwitchRef.current >= COOLDOWN_MS
    ) {
      if (activeSpanRef.current !== desired) {
        activeSpanRef.current = desired;
        lastSwitchRef.current = now;
      }
    }
    const span = activeSpanRef.current as number;
    const wanted = coverTiles(vp, span);

    // WMS：视口级单图（相机移动 → 重新请求；旧 blob revoke）。
    // 注意：不 return——WMS 仅替代底图纹理，地形瓦片照常按视口加载。
    if (baseMapConfig.source === 'wms') {
      if (baseMapConfig.wmsUrl && baseMapConfig.wmsLayers) {
        const [w, h] = wmsSize(vp);
        fetchWmsBlob({
          base_url: baseMapConfig.wmsUrl,
          bbox: vp,
          width: w,
          height: h,
          layers: baseMapConfig.wmsLayers,
          crs: baseMapConfig.wmsCrs ?? 'EPSG:4326',
        })
          .then((url) => {
            if (prevWmsUrlRef.current) {
              URL.revokeObjectURL(prevWmsUrlRef.current);
            }
            prevWmsUrlRef.current = url;
            setWms({ url, bbox: vp });
          })
          .catch((e) => setError(String(e)));
      }
    }

    // mask/tiff / none：增量加载缺失瓦片
    const cache = cacheRef.current;
    const missing = wanted.filter(
      (t) => !cache.has(t.key) && !pendingRef.current.has(t.key),
    );
    if (missing.length > 0) {
      const queue = [...missing];
      const workers = Array.from(
        { length: Math.min(CONCURRENCY, queue.length) },
        async () => {
          while (queue.length) {
            const t = queue.shift()!;
            pendingRef.current.add(t.key);
            try {
              const e = await loadTile(t, cache, terrainConfig, baseMapConfig);
              // 瓦片级失败汇总显示（2026-08-13：避免"无提示空白"——路径打不开等
              // 原因直接暴露给用户）
              if (e.terrainError || e.baseMapError) {
                setError(
                  `瓦片加载失败 ${e.key}: ${e.terrainError || e.baseMapError}`,
                );
              }
              updateRender(); // 渐进渲染
            } finally {
              pendingRef.current.delete(t.key);
            }
          }
        },
      );
      activeRef.current += 1;
      setLoading(true);
      Promise.all(workers).then(() => {
        activeRef.current -= 1;
        trimCache();
        updateRender();
        if (activeRef.current === 0) {
          setLoading(false);
          // 视口内瓦片全部成功 → 清除历史错误（2026-08-13：避免切源时
          // 旧路径请求失败的错误残留遮挡正常加载）
          const hasErr = wanted.some((tw) => {
            const e = cache.get(tw.key);
            return e && (e.terrainError || e.baseMapError);
          });
          if (!hasErr) setError(null);
        }
      });
    }
    // 视口变化（即使无缺失瓦片）也刷新渲染列表（相机移动 → 视口相交集合变化）
    updateRender();
  }, [terrainConfig, baseMapConfig, cameraRef, geoRef, updateRender, trimCache]);

  // 配置变化 → 清空缓存重载（层级强制重选）
  useEffect(() => {
    cacheRef.current.clear();
    pendingRef.current.clear();
    activeSpanRef.current = null;
    lastSwitchRef.current = 0;
    setTiles([]);
    refresh();
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [
    terrainConfig.source,
    terrainConfig.path,
    baseMapConfig.source,
    baseMapConfig.path,
    baseMapConfig.tiffProjection,
    baseMapConfig.wmsUrl,
    baseMapConfig.wmsLayers,
    baseMapConfig.wmsCrs,
  ]);

  // 相机 change → 节流刷新视口；挂载就绪后立即加载首屏
  useEffect(() => {
    if (!sceneReady) return;
    refresh();
    const ctrl = controlsRef.current;
    if (!ctrl?.addEventListener) return;
    const onChange = () => {
      if (timerRef.current !== null) return;
      timerRef.current = window.setTimeout(() => {
        timerRef.current = null;
        refresh();
      }, THROTTLE_MS);
    };
    ctrl.addEventListener('change', onChange);
    return () => {
      ctrl.removeEventListener('change', onChange);
      if (timerRef.current !== null) {
        clearTimeout(timerRef.current);
        timerRef.current = null;
      }
    };
    // eslint-disable-next-line react-hooks/exhaustive-deps
  }, [sceneReady, refresh]);

  // 卸载时 revoke WMS blob
  useEffect(
    () => () => {
      if (prevWmsUrlRef.current) {
        URL.revokeObjectURL(prevWmsUrlRef.current);
      }
      if (timerRef.current !== null) {
        clearTimeout(timerRef.current);
      }
    },
    [],
  );

  return { tiles, wms, loading, error };
}

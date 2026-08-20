import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import * as THREE from 'three';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import type { ThreeEvent } from '@react-three/fiber';
import type {
  Waypoint,
  GeoRef,
  AircraftInput,
  Radar,
  Zone,
  ZoneType,
  AircraftOutput,
  Vec2,
  TerrainConfig,
  BaseMapConfig,
} from '../types';
import { geoToLocal, geoPointToLocal, localToGeo } from '../types';
import { useViewportTiles, type TileEntry } from '../tiles';
import { StartMarker } from './StartMarker';
import { TargetZone } from './TargetZone';
import { RadarSphere } from './RadarSphere';
import { NFZPrism } from './NFZPrism';
import { PathLine } from './PathLine';
import { TerrainMesh } from './TerrainMesh';
import { MidpointMarker } from './MidpointMarker';

interface Scene3DProps {
  geoRef: GeoRef;
  target: Waypoint;
  aircraft: AircraftInput[];
  radars: Radar[];
  /** zone 渲染视图：输入 zone 不带 zone_type，由 App 按所属数组打标（仅前端着色用） */
  zones: VisualZone[];
  results: AircraftOutput[] | null;
  /** 地形源配置（source=none/path；瓦片按相机视口加载，2026-08-13） */
  terrainConfig: TerrainConfig;
  /** 底图配置（mask/tiff 瓦片级纹理；wms 视口级单图） */
  baseMapConfig: BaseMapConfig;
  /** 场景高度范围（米；起终点/结果路径高度差，无地形时驱动 z 夸张） */
  sceneAltRange: number;
  /** 场景包围盒 [minLon, minLat, maxLon, maxLat]（sceneBounds；点击平面/无地形 z 夸张用） */
  bounds: [number, number, number, number];
  onGroundClick: (wp: Waypoint) => void;
  onRadarMove: (id: string, lon: number, lat: number) => void;
  onZoneMove: (id: string, dLon: number, dLat: number) => void;
  /** 必经点拖动：更新某车第 index 个必经点经纬（高度保留） */
  onMidpointMove: (
    vehicleId: string,
    index: number,
    lon: number,
    lat: number,
  ) => void;
  activeClickMode: 'start' | 'target' | 'midpoint' | 'polygon' | null;
  /** 瓦片加载状态回调（App 用于 ControlPanel 底图状态 / canvas overlay） */
  onTilesStatus?: (loading: boolean, error: string | null) => void;
}

/** zone 渲染视图：输入 Zone 无 zone_type（JSON 契约 2026-08-19），前端按所属数组打标着色 */
export type VisualZone = Zone & { zone_type: ZoneType };

/** 圆形 zone → 局部平面多边形（24 边近似） */
function circleToLocalPolygon(center: [number, number], radiusKm: number, ref: GeoRef): Vec2[] {
  const pts: Vec2[] = [];
  const cx = (center[0] - ref.lon) * 111320 * Math.cos((ref.lat * Math.PI) / 180);
  const cy = (center[1] - ref.lat) * 110574;
  const r = radiusKm * 1000;
  for (let i = 0; i < 24; i++) {
    const a = (i / 24) * Math.PI * 2;
    pts.push([cx + r * Math.cos(a), cy + r * Math.sin(a)]);
  }
  return pts;
}

function zoneBoundaryLocal(zone: Zone, ref: GeoRef): Vec2[] {
  if (zone.shape === 'circle') {
    const g = zone.geometry as { center: [number, number]; radius_km: number };
    return circleToLocalPolygon(g.center, g.radius_km, ref);
  }
  return (zone.geometry as { vertices: [number, number][] }).vertices.map(
    ([lon, lat]) => [
      (lon - ref.lon) * 111320 * Math.cos((ref.lat * Math.PI) / 180),
      (lat - ref.lat) * 110574,
    ],
  );
}

function GroundClickPlane({
  active,
  onClick,
  geoRef,
  suppressUntil,
  bounds,
}: {
  active: boolean;
  onClick: (wp: Waypoint) => void;
  geoRef: GeoRef;
  suppressUntil: number;
  bounds: [number, number, number, number];
}) {
  // 同位置防抖：r3f 事件重复触发/StrictMode 重绑定时，同一位置 400ms 内只生效一次
  const last = useRef<{ x: number; y: number; t: number } | null>(null);

  const handleClick = useCallback(
    (e: ThreeEvent<MouseEvent>) => {
      if (!active) return;
      e.stopPropagation();
      // 雷达拖动刚结束（400ms 内）→ 抑制，避免误设起点/目标/顶点
      const now = Date.now();
      if (now < suppressUntil) return;
      // 用事件自带的 world 交点（e.point），不依赖全局 pointer state
      const px = e.point.x;
      const pz = e.point.z;
      if (
        last.current &&
        now - last.current.t < 400 &&
        Math.abs(last.current.x - px) < 1 &&
        Math.abs(last.current.y - pz) < 1
      ) {
        return;
      }
      last.current = { x: px, y: pz, t: now };
      // 场景坐标 z=-北 → 纬度 = ref.lat - z / ky（2026-08-14）
      onClick(localToGeo([px, -pz, 0], geoRef));
    },
    [active, onClick, geoRef, suppressUntil],
  );

  // 点击平面 = sceneBounds 外扩 3 倍（宽高 ×3，中心不变）：
  // 起终点/顶点可点在已加载场景之外 → sceneBounds 自动扩展（相机/瓦片不随之调整，
  // 2026-08-13：相机视口完全独立，起终点变化只影响标记位置）。
  const kx = 111320 * Math.cos((geoRef.lat * Math.PI) / 180);
  const ky = 110574;
  const [minLon, minLat, maxLon, maxLat] = bounds;
  const width = (maxLon - minLon) * kx * 3;
  const height = (maxLat - minLat) * ky * 3;
  const cx = ((minLon + maxLon) / 2 - geoRef.lon) * kx;
  const cy = ((minLat + maxLat) / 2 - geoRef.lat) * ky;

  return (
    <mesh
      rotation={[-Math.PI / 2, 0, 0]}
      position={[cx, 0, -cy]}
      visible={active}
      onClick={handleClick}
    >
      <planeGeometry args={[width, height]} />
      <meshBasicMaterial visible={false} />
    </mesh>
  );
}

const ZONE_COLORS: Record<string, string> = {
  no_fly: '#ff8800',
  restricted: '#44aaff',
  obstacle: '#ff4455',
};

/** 禁飞/障碍全高度禁入的可视高度上限（米，乘 zScale；2026-08-12 起无高度区间） */
const WALL_VISUAL_TOP_M = 30000;

/** 地形 z 夸张系数（多瓦片合并高度范围；2026-08-13 瓦片化）：
 * 有地形瓦片 → 瓦片合并跨度 / 高度范围 × 0.08，clamp [3, 20]；
 * 无地形（source=none / 加载失败）→ 场景包围盒跨度 / 场景高度范围（起终点/结果路径
 * 高度差）× 0.08，clamp [3, 20]。 */
function computeZScaleFromTiles(
  tiles: TileEntry[],
  geoRef: GeoRef,
  sceneAltRange: number,
  bounds: [number, number, number, number],
): number {
  const lat0 = (geoRef.lat * Math.PI) / 180;
  const valid = tiles.filter((t) => t.terrain);
  let spanMeters: number;
  let range: number;
  if (valid.length) {
    let minH = Infinity;
    let maxH = -Infinity;
    let minLon = Infinity;
    let minLat = Infinity;
    let maxLon = -Infinity;
    let maxLat = -Infinity;
    for (const t of valid) {
      const b = t.bbox;
      minLon = Math.min(minLon, b[0]);
      minLat = Math.min(minLat, b[1]);
      maxLon = Math.max(maxLon, b[2]);
      maxLat = Math.max(maxLat, b[3]);
      for (const h of t.terrain!.heights) {
        if (h !== null) {
          minH = Math.min(minH, h);
          maxH = Math.max(maxH, h);
        }
      }
    }
    spanMeters = Math.hypot(
      (maxLon - minLon) * 111320 * Math.cos(lat0),
      (maxLat - minLat) * 110574,
    );
    range = Math.max(maxH - minH, sceneAltRange, 1);
  } else {
    spanMeters = Math.hypot(
      (bounds[2] - bounds[0]) * 111320 * Math.cos(lat0),
      (bounds[3] - bounds[1]) * 110574,
    );
    range = Math.max(sceneAltRange, 1);
  }
  if (!(spanMeters > 0)) return 3;
  return Math.min(Math.max((spanMeters / range) * 0.08, 3), 20);
}

/** 单瓦片地形网格 + 底图纹理（mask/tiff 瓦片纹理；wms 用视口级纹理） */
function TileMesh({
  entry,
  geoRef,
  zScale,
  wmsTexture,
  wmsBbox,
  onPick,
}: {
  entry: TileEntry;
  geoRef: GeoRef;
  zScale: number;
  wmsTexture: THREE.Texture | null;
  wmsBbox: [number, number, number, number] | null;
  onPick?: (wp: Waypoint) => void;
}) {
  // mask/tiff：瓦片 RGBA 网格 → CanvasTexture（卸载时 dispose）
  const tileTex = useMemo(() => {
    if (!entry.baseMap) return null;
    const { nx, ny, rgba } = entry.baseMap;
    const canvas = document.createElement('canvas');
    canvas.width = nx;
    canvas.height = ny;
    const ctx = canvas.getContext('2d');
    if (!ctx) return null;
    const img = ctx.createImageData(nx, ny);
    img.data.set(rgba);
    ctx.putImageData(img, 0, 0);
    const tex = new THREE.CanvasTexture(canvas);
    tex.colorSpace = THREE.SRGBColorSpace;
    tex.magFilter = THREE.LinearFilter;
    tex.minFilter = THREE.LinearFilter;
    return tex;
  }, [entry.baseMap]);
  useEffect(() => () => {
    tileTex?.dispose();
  }, [tileTex]);

  if (!entry.terrain) return null;
  const texture = entry.baseMap ? tileTex : wmsTexture;
  const textureBbox = entry.baseMap ? entry.bbox : wmsBbox;
  return (
    <TerrainMesh
      data={entry.terrain}
      geoRef={geoRef}
      zScale={zScale}
      texture={texture}
      textureBbox={textureBbox}
      onPick={onPick}
    />
  );
}

export function Scene3D({
  geoRef,
  target,
  aircraft,
  radars,
  zones,
  results,
  terrainConfig,
  baseMapConfig,
  sceneAltRange,
  bounds,
  onGroundClick,
  onRadarMove,
  onZoneMove,
  onMidpointMove,
  activeClickMode,
  onTilesStatus,
}: Scene3DProps) {
  // 相机/轨道控制器引用
  const cameraRef = useRef<any>(null);
  const controlsRef = useRef<any>(null);
  // Canvas 挂载完成（camera/controls 就绪）→ 瓦片系统启动
  const [sceneReady, setSceneReady] = useState(false);

  // geoRef 固定为首个 sceneBounds 中心：相机视口完全独立漫游（2026-08-13），
  // 起终点变化只影响标记位置——局部投影原点不再跟随 sceneBounds 变化（否则
  // 所有瓦片/标记局部坐标整体平移 → 视野跳动）。
  const geoRefRef = useRef<GeoRef>(geoRef);
  const stableGeoRef = geoRefRef.current;

  // 视口瓦片系统（相机 change 节流驱动；配置变化清缓存重载）
  const { tiles, wms, loading: tilesLoading, error: tilesError } = useViewportTiles({
    terrainConfig,
    baseMapConfig,
    cameraRef,
    controlsRef,
    geoRef: stableGeoRef,
    sceneReady,
  });

  // WMS 视口图纹理（blob URL → TextureLoader；替换时 dispose）
  const [wmsTexture, setWmsTexture] = useState<THREE.Texture | null>(null);
  useEffect(() => {
    if (!wms) {
      setWmsTexture(null);
      return;
    }
    const loader = new THREE.TextureLoader();
    const tex = loader.load(wms.url);
    tex.colorSpace = THREE.SRGBColorSpace;
    setWmsTexture(tex);
    return () => {
      THREE.Cache.remove(wms.url);
      tex.dispose();
    };
  }, [wms]);

  // 瓦片加载状态上报（ControlPanel 底图状态 / canvas overlay）
  useEffect(() => {
    onTilesStatus?.(tilesLoading, tilesError);
  }, [tilesLoading, tilesError, onTilesStatus]);

  // 统一 z 夸张系数（地形 + 航路 + 标记 + zone 高度共用，保证同一尺度）
  const zScale = useMemo(
    () => computeZScaleFromTiles(tiles, stableGeoRef, sceneAltRange, bounds),
    [tiles, stableGeoRef, sceneAltRange, bounds],
  );

  const targetPos = useMemo(
    () => geoToLocal(target, stableGeoRef, zScale),
    [target, stableGeoRef, zScale],
  );

  // 拖动物体（雷达/zone）状态：拖动期间禁用 OrbitControls，结束 400ms 内抑制地面点击
  const [dragActive, setDragActive] = useState(false);
  const [dragSuppressUntil, setDragSuppressUntil] = useState(0);
  const handleDragState = useCallback((dragging: boolean) => {
    setDragActive(dragging);
    if (!dragging) setDragSuppressUntil(Date.now() + 400);
  }, []);

  // 拾取统一入口（地形表面 / 地面平面共用）：拖动结束 400ms 内抑制 + 同位置 400ms 防抖
  const lastPick = useRef<{ x: number; y: number; t: number } | null>(null);
  const handlePick = useCallback(
    (wp: Waypoint) => {
      if (Date.now() < dragSuppressUntil) return;
      const now = Date.now();
      if (
        lastPick.current &&
        now - lastPick.current.t < 400 &&
        Math.abs(lastPick.current.x - wp.lon) < 1e-6 &&
        Math.abs(lastPick.current.y - wp.lat) < 1e-6
      ) {
        return;
      }
      lastPick.current = { x: wp.lon, y: wp.lat, t: now };
      onGroundClick(wp);
    },
    [dragSuppressUntil, onGroundClick],
  );

  const radarMeshes = radars.map((r) => ({
    id: r.id,
    center: geoPointToLocal(r.lon, r.lat, r.alt_m ?? 10, stableGeoRef, zScale),
    radiusM: r.radius_km * 1000,
  }));

  const zoneMeshes = zones.map((z) => {
    // 禁飞/障碍全高度禁入：无高度范围 → 从地面拉到可视顶部；
    // 限飞区用 [alt_min, alt_max]（缺省 0..12000 兜底）
    const wall = z.zone_type === 'no_fly' || z.zone_type === 'obstacle';
    return {
      id: z.id,
      color: ZONE_COLORS[z.zone_type] ?? '#ff8800',
      boundary: zoneBoundaryLocal(z, stableGeoRef),
      altMin: wall ? 0 : (z.alt_min_m ?? 0) * zScale,
      altMax: wall ? WALL_VISUAL_TOP_M * zScale : (z.alt_max_m ?? 12000) * zScale,
    };
  });

  // 多边形 zone 顶点（可视化编辑锚点）
  const polygonVerts = useMemo(
    () =>
      zones.flatMap((z) => {
        if (z.shape !== 'polygon') return [];
        return (z.geometry as { vertices: [number, number][] }).vertices.map(
          ([lon, lat]) => {
            const p = geoPointToLocal(lon, lat, z.alt_min_m ?? 0, stableGeoRef, zScale);
            return { id: `${z.id}_${lon}_${lat}`, pos: p, color: z.zone_type };
          },
        );
      }),
    [zones, stableGeoRef, zScale],
  );

  // 飞行器路径（输出，经纬高 → 局部平面；高度乘 zScale 贴合地形表面）
  const aircraftLines = useMemo(() => {
    if (!results) return [];
    return results.map((ao) => ({
      id: ao.id,
      status: ao.status,
      points: ao.path.map((p) =>
        geoPointToLocal(p.x, p.y, p.alt_m, stableGeoRef, zScale),
      ),
    }));
  }, [results, stableGeoRef, zScale]);

  // 必经点（输入；可拖动——MidpointMarker）
  const midPoints = useMemo(
    () =>
      aircraft.flatMap((a) =>
        (a.mid_waypoints ?? []).map((m, idx) => ({
          vehicleId: a.id,
          index: idx,
          pos: geoToLocal(m, stableGeoRef, zScale),
        })),
      ),
    [aircraft, stableGeoRef, zScale],
  );

  return (
    <Canvas
      // 场景坐标（2026-08-14 修复朝北镜像）：x=东、y=上、z=-北（物理右手系）
      // 默认相机 = 正南高空俯视（主管 2026-08-14）：正南 80000、高 60000
      // → 看向原点即朝北：北在上、南在下、东在右（地图式方位）
      camera={{
        position: [0, 60000, 80000],
        up: [0, 1, 0],
        fov: 50,
        near: 10,
        far: 10000000,
      }}
      onCreated={({ camera }) => {
        cameraRef.current = camera;
        // 首帧相机必须先看向场景中心：否则 quaternion 保持默认（看向 -z），
        // viewportBBox 视线交点偏南（黄海/连云港=海），首屏会加载错误瓦片
        // （初始渲染显示海色/错乱）。OrbitControls 挂载后 target 仍为 (0,0,0)，
        // 与这里 lookAt 一致，不会造成跳变。
        camera.lookAt(0, 0, 0);
        camera.updateProjectionMatrix();
        setSceneReady(true);
      }}
      style={{ background: '#1c2942' }}
    >
      <ambientLight intensity={0.85} />
      <hemisphereLight args={['#b8c8f0', '#4a5a78', 0.7]} />
      <directionalLight position={[20000, 30000, -10000]} intensity={1.1} />

      <OrbitControls
        ref={(c) => {
          controlsRef.current = c;
          if (c) {
            setSceneReady(true);
          }
        }}
        makeDefault
        maxPolarAngle={Math.PI / 2.1}
        enabled={!dragActive}
      />

      {/* 视口瓦片：地形网格 + 底图纹理（mask/tiff 瓦片级；wms 视口级单图） */}
      {tiles.map((t) => (
        <TileMesh
          key={t.key}
          entry={t}
          geoRef={stableGeoRef}
          zScale={zScale}
          wmsTexture={wmsTexture}
          wmsBbox={wms?.bbox ?? null}
          onPick={activeClickMode !== null ? handlePick : undefined}
        />
      ))}

      <Grid
        args={[100000, 100000, 20, 20]}
        position={[50000, 0, 50000]}
        cellSize={1000}
        cellThickness={0.5}
        cellColor="#4a5a78"
        sectionSize={5000}
        sectionThickness={1}
        sectionColor="#6a7a98"
        fadeDistance={400000}
        infiniteGrid
      />

      {/* 每机独立起点 marker（aircraft[].start；v0.21 起输入无 heading_deg，机头朝向为固定 45° 占位） */}
      {aircraft.map((a) => (
        <StartMarker
          key={a.id}
          position={geoPointToLocal(
            a.start.lon,
            a.start.lat,
            a.start.alt_m,
            stableGeoRef,
            zScale,
          )}
          heading={45}
        />
      ))}
      <TargetZone center={targetPos} />

      {radarMeshes.map((r) => (
        <RadarSphere
          key={r.id}
          id={r.id}
          center={r.center}
          radiusM={r.radiusM}
          geoRef={stableGeoRef}
          onRadarMove={onRadarMove}
          onDragStateChange={handleDragState}
        />
      ))}
      {zoneMeshes.map((z) => (
        <NFZPrism
          key={z.id}
          id={z.id}
          boundaryPoints={z.boundary}
          altMin={z.altMin}
          altMax={z.altMax}
          color={z.color}
          geoRef={stableGeoRef}
          onZoneMove={onZoneMove}
          onDragStateChange={handleDragState}
        />
      ))}

      {/* 多边形顶点锚点（底部高亮球） */}
      {polygonVerts.map((p, i) => (
        <mesh key={`pv_${i}`} position={[p.pos[0], p.pos[2] + 20, -p.pos[1]]}>
          <sphereGeometry args={[120, 16, 8]} />
          <meshBasicMaterial color="#ffcc00" />
        </mesh>
      ))}

      {/* 必经点（黄色可拖动小球；Alt+点击新增由地面拾取层处理） */}
      {midPoints.map((m) => (
        <MidpointMarker
          key={`mid_${m.vehicleId}_${m.index}`}
          vehicleId={m.vehicleId}
          index={m.index}
          center={m.pos}
          geoRef={stableGeoRef}
          onMidpointMove={onMidpointMove}
          onDragStateChange={handleDragState}
        />
      ))}

      {/* 飞行器路径 */}
      {aircraftLines.map((al) => (
        <PathLine
          key={al.id}
          waypoints={al.points}
          color={al.status === 'planned' ? '#ffdd00' : '#ff66aa'}
        />
      ))}

      <GroundClickPlane
        active={activeClickMode !== null}
        onClick={handlePick}
        geoRef={stableGeoRef}
        suppressUntil={dragSuppressUntil}
        bounds={bounds}
      />
    </Canvas>
  );
}

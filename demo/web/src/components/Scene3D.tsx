import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import type { ThreeEvent } from '@react-three/fiber';
import type { Waypoint, GeoRef, VehicleInput, Radar, Zone, VehicleOutput, Vec2, TerrainInfo } from '../types';
import { geoToLocal, geoPointToLocal, localToGeo, parseVehicleTargetRef } from '../types';
import { StartMarker } from './StartMarker';
import { TargetZone } from './TargetZone';
import { RadarSphere } from './RadarSphere';
import { NFZPrism } from './NFZPrism';
import { PathLine } from './PathLine';
import { TerrainMesh } from './TerrainMesh';
import { MidpointMarker } from './MidpointMarker';

interface Scene3DProps {
  geoRef: GeoRef;
  start: Waypoint;
  target: Waypoint;
  vehicles: VehicleInput[];
  radars: Radar[];
  zones: Zone[];
  results: VehicleOutput[] | null;
  terrainData: TerrainInfo | null;
  /** 场景包围盒 [minLon, minLat, maxLon, maxLat]（sceneBounds；与地形网格/相机视野一致） */
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
}

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
      onClick(localToGeo([px, pz, 0], geoRef));
    },
    [active, onClick, geoRef, suppressUntil],
  );

  // 点击平面 = sceneBounds 外扩 3 倍（宽高 ×3，中心不变）：
  // 起终点/顶点可点在已加载场景之外 → sceneBounds 自动扩展 → 地形重新采样，
  // 场景随点击逐步扩大（主管 2026-08-10：场景不应被限制在固定大小）。
  const kx = 111320 * Math.cos((geoRef.lat * Math.PI) / 180);
  const ky = 110574;
  const [minLon, minLat, maxLon, maxLat] = bounds;
  const width = (maxLon - minLon) * kx * 3;
  const height = (maxLat - minLat) * ky * 3;
  const cx = ((minLon + maxLon) / 2 - geoRef.lon) * kx;
  const cz = ((minLat + maxLat) / 2 - geoRef.lat) * ky;

  return (
    <mesh
      rotation={[-Math.PI / 2, 0, 0]}
      position={[cx, 0, cz]}
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

/** 地形 z 夸张系数（与 TerrainMesh 共用，Scene3D 统一计算后传给所有含高度对象）：
 * 无地形数据 → 1（航路绝对高度显示）；有 → 场景跨度/高度范围 × 0.08，clamp [3, 20]。
 * 主管 2026-08-10：原 ×0.25 clamp [10,60] 过大不协调 → 降为 ×0.08 clamp [3,20]。 */
function computeZScale(terrainData: TerrainInfo | null, geoRef: GeoRef): number {
  if (!terrainData) return 1;
  const hs = terrainData.heights.filter((h): h is number => h !== null);
  if (!hs.length) return 1;
  const minH = Math.min(...hs);
  const maxH = Math.max(...hs);
  const lat0 = (geoRef.lat * Math.PI) / 180;
  const spanMeters = Math.hypot(
    (terrainData.max_lon - terrainData.min_lon) * 111320 * Math.cos(lat0),
    (terrainData.max_lat - terrainData.min_lat) * 110574,
  );
  const range = Math.max(maxH - minH, 1);
  return Math.min(Math.max((spanMeters / range) * 0.08, 3), 20);
}

/** 每机自定义目标 → 局部坐标（parseVehicleTargetRef 定义在 types.ts，App/api 共享） */

export function Scene3D({
  geoRef,
  target,
  vehicles,
  radars,
  zones,
  results,
  terrainData,
  bounds,
  onGroundClick,
  onRadarMove,
  onZoneMove,
  onMidpointMove,
  activeClickMode,
}: Scene3DProps) {
  // 相机/轨道控制器引用（视野随场景包围盒自适应，见 fitCameraToBounds）
  const cameraRef = useRef<any>(null);
  const controlsRef = useRef<any>(null);

  // 相机视野随 sceneBounds 自适应（2026-08-11 主管：起终点更改 → 场景应扩展）：
  // App 传的 geoRef = sceneBounds 中心 → 场景中心恒为局部原点 (0,0,0)；
  // 相机保持当前方向（用户手动旋转/缩放后的姿态），仅按场景对角线调整距离——
  // fov 50° 垂直半角 25°，target 处视宽 ≈ 2·d·tan(25°) ≈ 0.93·d，
  // 取 d = diag/0.75 → 视宽 ≈ 1.24·diag（留 ~24% 余量，完整看到扩展后的场景）。
  // 跨度变化 < 20% 不调整（拖拽微调起终点时视野不抖动）。
  const fitCameraToBounds = useCallback(() => {
    const cam = cameraRef.current;
    const ctrl = controlsRef.current;
    if (!cam || !ctrl) return;
    const [minLon, minLat, maxLon, maxLat] = bounds;
    const kx = 111320 * Math.cos((geoRef.lat * Math.PI) / 180);
    const ky = 110574;
    const diag = Math.hypot((maxLon - minLon) * kx, (maxLat - minLat) * ky);
    if (!(diag > 0)) return;
    const desiredDist = diag / 0.75;
    ctrl.target.set(0, 0, 0);
    const dir = cam.position.clone().sub(ctrl.target);
    const len = dir.length();
    if (len < 1e-6) {
      cam.position.set(desiredDist, desiredDist * 0.75, desiredDist);
    } else if (Math.abs(len - desiredDist) > desiredDist * 0.2) {
      dir.normalize().multiplyScalar(desiredDist);
      cam.position.copy(ctrl.target).add(dir);
    }
    ctrl.update();
  }, [bounds, geoRef]);

  // 挂载 + 每次 bounds 变化 → 视野对齐（地形已随 bbox 重新采样，相机需跟上）
  useEffect(() => {
    fitCameraToBounds();
  }, [fitCameraToBounds]);

  // 统一 z 夸张系数（地形 + 航路 + 标记 + zone 高度共用，保证同一尺度）
  const zScale = useMemo(
    () => computeZScale(terrainData, geoRef),
    [terrainData, geoRef],
  );

  const targetPos = useMemo(
    () => geoToLocal(target, geoRef, zScale),
    [target, geoRef, zScale],
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
    center: geoPointToLocal(r.lon, r.lat, r.alt_m ?? 10, geoRef, zScale),
    radiusM: r.radius_km * 1000,
  }));

  const zoneMeshes = zones.map((z) => ({
    id: z.id,
    color: ZONE_COLORS[z.zone_type] ?? '#ff8800',
    boundary: zoneBoundaryLocal(z, geoRef),
    // zone 高度范围（相对海平面）乘同一 zScale，与地形/航路同尺度
    altMin: z.alt_min_m * zScale,
    altMax: z.alt_max_m * zScale,
  }));

  // 多边形 zone 顶点（可视化编辑锚点）
  const polygonVerts = useMemo(
    () =>
      zones.flatMap((z) => {
        if (z.shape !== 'polygon') return [];
        return (z.geometry as { vertices: [number, number][] }).vertices.map(
          ([lon, lat]) => {
            const p = geoPointToLocal(lon, lat, z.alt_min_m, geoRef, zScale);
            return { id: `${z.id}_${lon}_${lat}`, pos: p, color: z.zone_type };
          },
        );
      }),
    [zones, geoRef, zScale],
  );

  // 车辆路径（输出，经纬高 → 局部平面；高度乘 zScale 贴合地形表面）
  const vehicleLines = useMemo(() => {
    if (!results) return [];
    return results.map((v) => ({
      id: v.id,
      status: v.status,
      points: v.path.map((p) =>
        geoPointToLocal(p.x, p.y, p.alt_m, geoRef, zScale),
      ),
    }));
  }, [results, geoRef, zScale]);

  // 必经点（输入；可拖动——MidpointMarker）
  const midPoints = useMemo(
    () =>
      vehicles.flatMap((v) =>
        (v.mid_waypoints ?? []).map((m, idx) => ({
          vehicleId: v.id,
          index: idx,
          pos: geoToLocal(m, geoRef, zScale),
        })),
      ),
    [vehicles, geoRef, zScale],
  );

  // 每机自定义目标（非 mission.target 的 target_ref → 红色标记，与全局蓝色目标区分）
  const vehicleTargets = useMemo(
    () =>
      vehicles.flatMap((v) => {
        const t = parseVehicleTargetRef(v, target);
        return t ? [{ id: v.id, pos: geoToLocal(t, geoRef, zScale) }] : [];
      }),
    [vehicles, target, geoRef, zScale],
  );

  return (
    <Canvas
      camera={{
        position: [80000, 60000, 80000],
        fov: 50,
        near: 10,
        far: 10000000,
      }}
      onCreated={({ camera }) => {
        cameraRef.current = camera;
        // 挂载即 fit 一次（首帧相机可能未就绪，fitCameraToBounds 内已防御）
        fitCameraToBounds();
      }}
      style={{ background: '#1c2942' }}
    >
      <ambientLight intensity={0.85} />
      <hemisphereLight args={['#b8c8f0', '#4a5a78', 0.7]} />
      <directionalLight position={[20000, 30000, 10000]} intensity={1.1} />

      <OrbitControls
        ref={controlsRef}
        makeDefault
        maxPolarAngle={Math.PI / 2.1}
        enabled={!dragActive}
      />

      {terrainData && (
        <TerrainMesh
          data={terrainData}
          geoRef={geoRef}
          zScale={zScale}
          onPick={activeClickMode !== null ? handlePick : undefined}
        />
      )}

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

      {/* 每机独立起点 marker（vehicles[].start_pose；多机时各自位置/航向） */}
      {vehicles.map((v) => (
        <StartMarker
          key={v.id}
          position={geoPointToLocal(
            v.start_pose.lon,
            v.start_pose.lat,
            v.start_pose.alt_m,
            geoRef,
            zScale,
          )}
          heading={v.start_pose.heading_deg ?? 45}
        />
      ))}
      <TargetZone center={targetPos} />

      {/* 每机自定义目标（红色小标记；mission.target 缺省不显示，复用全局 TargetZone） */}
      {vehicleTargets.map((t) => (
        <mesh key={t.id} position={[t.pos[0], t.pos[2], t.pos[1]]}>
          <sphereGeometry args={[250, 32, 16]} />
          <meshStandardMaterial color="#ff4466" />
        </mesh>
      ))}

      {radarMeshes.map((r) => (
        <RadarSphere
          key={r.id}
          id={r.id}
          center={r.center}
          radiusM={r.radiusM}
          geoRef={geoRef}
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
          geoRef={geoRef}
          onZoneMove={onZoneMove}
          onDragStateChange={handleDragState}
        />
      ))}

      {/* 多边形顶点锚点（底部高亮球） */}
      {polygonVerts.map((p, i) => (
        <mesh key={`pv_${i}`} position={[p.pos[0], p.pos[2] + 20, p.pos[1]]}>
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
          geoRef={geoRef}
          onMidpointMove={onMidpointMove}
          onDragStateChange={handleDragState}
        />
      ))}

      {/* 车辆路径 */}
      {vehicleLines.map((v) => (
        <PathLine
          key={v.id}
          waypoints={v.points}
          color={v.status === 'planned' ? '#ffdd00' : '#ff66aa'}
        />
      ))}

      <GroundClickPlane
        active={activeClickMode !== null}
        onClick={handlePick}
        geoRef={geoRef}
        suppressUntil={dragSuppressUntil}
        bounds={bounds}
      />
    </Canvas>
  );
}

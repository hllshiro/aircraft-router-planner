import { useCallback, useMemo, useRef, useState } from 'react';
import { Canvas } from '@react-three/fiber';
import { OrbitControls, Grid } from '@react-three/drei';
import type { ThreeEvent } from '@react-three/fiber';
import type { Waypoint, GeoRef, VehicleInput, Radar, Zone, VehicleOutput, Vec2, TerrainInfo } from '../types';
import { geoToLocal, geoPointToLocal, localToGeo } from '../types';
import { StartMarker } from './StartMarker';
import { TargetZone } from './TargetZone';
import { RadarSphere } from './RadarSphere';
import { NFZPrism } from './NFZPrism';
import { PathLine } from './PathLine';
import { TerrainMesh } from './TerrainMesh';

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
  activeClickMode: 'start' | 'target' | 'polygon' | null;
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

  // 点击平面 = sceneBounds（与地形网格/相机视野一致的可见范围）：
  // 固定 ±600km 平面比场景显示范围大得多（近距场景最小 2.5°×2.2°≈±135km），
  // 多边形顶点可点在视野外/地形外 → 添加看不见的顶点（主管 2026-08-07：
  // 顶点点击范围应与场景范围一致）。以 geoRef(start) 为原点换算局部坐标。
  const kx = 111320 * Math.cos((geoRef.lat * Math.PI) / 180);
  const ky = 110574;
  const [minLon, minLat, maxLon, maxLat] = bounds;
  const width = (maxLon - minLon) * kx;
  const height = (maxLat - minLat) * ky;
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

export function Scene3D({
  geoRef,
  start,
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
  activeClickMode,
}: Scene3DProps) {
  const startPos = useMemo(() => geoToLocal(start, geoRef), [start, geoRef]);
  const targetPos = useMemo(() => geoToLocal(target, geoRef), [target, geoRef]);

  // 拖动物体（雷达/zone）状态：拖动期间禁用 OrbitControls，结束 400ms 内抑制地面点击
  const [dragActive, setDragActive] = useState(false);
  const [dragSuppressUntil, setDragSuppressUntil] = useState(0);
  const handleDragState = useCallback((dragging: boolean) => {
    setDragActive(dragging);
    if (!dragging) setDragSuppressUntil(Date.now() + 400);
  }, []);

  const radarMeshes = radars.map((r) => ({
    id: r.id,
    center: geoPointToLocal(r.lon, r.lat, r.alt_m ?? 10, geoRef),
    radiusM: r.radius_km * 1000,
  }));

  const zoneMeshes = zones.map((z) => ({
    id: z.id,
    color: ZONE_COLORS[z.zone_type] ?? '#ff8800',
    boundary: zoneBoundaryLocal(z, geoRef),
    altMin: z.alt_min_m,
    altMax: z.alt_max_m,
  }));

  // 多边形 zone 顶点（可视化编辑锚点）
  const polygonVerts = useMemo(
    () =>
      zones.flatMap((z) => {
        if (z.shape !== 'polygon') return [];
        return (z.geometry as { vertices: [number, number][] }).vertices.map(
          ([lon, lat]) => {
            const p = geoPointToLocal(lon, lat, z.alt_min_m, geoRef);
            return { id: `${z.id}_${lon}_${lat}`, pos: p, color: z.zone_type };
          },
        );
      }),
    [zones, geoRef],
  );

  // 车辆路径（输出，经纬高 → 局部平面）
  const vehicleLines = useMemo(() => {
    if (!results) return [];
    return results.map((v) => ({
      id: v.id,
      status: v.status,
      points: v.path.map((p) =>
        geoPointToLocal(p.x, p.y, p.alt_m, geoRef),
      ),
    }));
  }, [results, geoRef]);

  // 必经点（输入）
  const midPoints = useMemo(
    () =>
      vehicles.flatMap((v) =>
        (v.mid_waypoints ?? []).map((m) => ({
          id: v.id,
          pos: geoToLocal(m, geoRef),
        })),
      ),
    [vehicles, geoRef],
  );

  return (
    <Canvas
      camera={{
        position: [80000, 60000, 80000],
        fov: 50,
        near: 10,
        far: 2000000,
      }}
      style={{ background: '#1c2942' }}
    >
      <ambientLight intensity={0.85} />
      <hemisphereLight args={['#b8c8f0', '#4a5a78', 0.7]} />
      <directionalLight position={[20000, 30000, 10000]} intensity={1.1} />

      <OrbitControls
        makeDefault
        maxPolarAngle={Math.PI / 2.1}
        enabled={!dragActive}
      />

      {terrainData && <TerrainMesh data={terrainData} geoRef={geoRef} />}

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

      <StartMarker position={startPos} heading={vehicles[0]?.start_pose.heading_deg ?? 45} />
      <TargetZone center={targetPos} />

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

      {/* 必经点（黄色小球） */}
      {midPoints.map((m, i) => (
        <mesh key={`mid_${i}`} position={[m.pos[0], m.pos[2], m.pos[1]]}>
          <sphereGeometry args={[80, 16, 8]} />
          <meshBasicMaterial color="#ffee00" />
        </mesh>
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
        onClick={onGroundClick}
        geoRef={geoRef}
        suppressUntil={dragSuppressUntil}
        bounds={bounds}
      />
    </Canvas>
  );
}

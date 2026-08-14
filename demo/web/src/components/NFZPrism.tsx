import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import type { ThreeEvent } from '@react-three/fiber';
import type { Vec2, GeoRef } from '../types';

interface NFZPrismProps {
  id: string;
  boundaryPoints: Vec2[]; // 局部平面坐标 [x, y]（米）
  altMin: number;
  altMax: number;
  color?: string;
  geoRef: GeoRef;
  onZoneMove: (id: string, dLon: number, dLat: number) => void;
  onDragStateChange: (dragging: boolean) => void;
}

// 地面（Three y=0 平面）——拖动时把鼠标射线求交到地平面计算平移量
const Y0_PLANE = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const RAYCASTER = new THREE.Raycaster();
const NDC = new THREE.Vector2();
const HIT = new THREE.Vector3();

export function NFZPrism({
  id,
  boundaryPoints,
  altMin,
  altMax,
  color = '#ff8800',
  geoRef,
  onZoneMove,
  onDragStateChange,
}: NFZPrismProps) {
  // 本地拖动边界：拖动期间直接平移渲染，pointerup 一次性提交经纬度增量
  const [localBoundary, setLocalBoundary] = useState<Vec2[]>(boundaryPoints);
  const { camera, gl } = useThree();
  const dragRef = useRef(false);
  const startHitRef = useRef<{ x: number; z: number } | null>(null);
  const startBoundaryRef = useRef<Vec2[]>([]);
  const lastDeltaRef = useRef<{ x: number; z: number } | null>(null);

  // 外部边界变化（pointerup 提交后 config 更新）→ 同步本地边界
  useEffect(() => {
    setLocalBoundary(boundaryPoints);
  }, [boundaryPoints]);

  const shape = useMemo(() => {
    const s = new THREE.Shape();
    // THREE.Shape.closePath 需要 ≥2 点；少于 2 点时留空（渲染层另有 <3 保护）
    if (localBoundary.length < 2) return s;
    s.moveTo(localBoundary[0][0], localBoundary[0][1]);
    for (let i = 1; i < localBoundary.length; i++) {
      s.lineTo(localBoundary[i][0], localBoundary[i][1]);
    }
    s.closePath();
    return s;
  }, [localBoundary]);

  const handleMove = useCallback(
    (e: PointerEvent) => {
      if (!dragRef.current) return;
      const rect = gl.domElement.getBoundingClientRect();
      if (rect.width === 0 || rect.height === 0) return;
      NDC.set(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      );
      RAYCASTER.setFromCamera(NDC, camera);
      const sx = startHitRef.current;
      if (!sx || !RAYCASTER.ray.intersectPlane(Y0_PLANE, HIT)) return;
      const dx = HIT.x - sx.x;
      const dz = HIT.z - sx.z;
      lastDeltaRef.current = { x: dx, z: dz };
      // 场景坐标 z=-北（2026-08-14 右手系）：场景 +z 增量 = 向南移动，
      // Shape 的 y 轴是北 → 北坐标变化取反（by - dz），否则拖动方向与鼠标相反
      setLocalBoundary(
        startBoundaryRef.current.map(([bx, by]) => [bx + dx, by - dz]),
      );
    },
    [camera, gl],
  );

  const handleUp = useCallback(() => {
    if (!dragRef.current) return;
    dragRef.current = false;
    onDragStateChange(false);
    const d = lastDeltaRef.current;
    lastDeltaRef.current = null;
    startHitRef.current = null;
    if (d && (Math.abs(d.x) > 0.5 || Math.abs(d.z) > 0.5)) {
      const lat0 = (geoRef.lat * Math.PI) / 180;
      const dLon = d.x / (111320 * Math.cos(lat0));
      // 场景坐标 z=-北 → 纬度增量取反（2026-08-14）
      const dLat = -d.z / 110574;
      onZoneMove(id, dLon, dLat);
    }
  }, [id, geoRef, onDragStateChange, onZoneMove]);

  useEffect(() => {
    window.addEventListener('pointermove', handleMove);
    window.addEventListener('pointerup', handleUp);
    return () => {
      window.removeEventListener('pointermove', handleMove);
      window.removeEventListener('pointerup', handleUp);
    };
  }, [handleMove, handleUp]);

  const handleDown = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      // 记录拖动起点（鼠标射线 → 地面交点）与当前边界
      const rect = gl.domElement.getBoundingClientRect();
      if (rect.width === 0 || rect.height === 0) return;
      NDC.set(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      );
      RAYCASTER.setFromCamera(NDC, camera);
      if (RAYCASTER.ray.intersectPlane(Y0_PLANE, HIT)) {
        startHitRef.current = { x: HIT.x, z: HIT.z };
      } else {
        // 兜底：用命中点水平分量（极少出现射线平行地面）
        startHitRef.current = { x: e.point.x, z: e.point.z };
      }
      startBoundaryRef.current = localBoundary;
      lastDeltaRef.current = null;
      dragRef.current = true;
      onDragStateChange(true);
    },
    [camera, gl, localBoundary, onDragStateChange],
  );

  // 阻止 click 冒泡到地面拾取层（避免拖动结束后误设起点/目标/顶点）
  const stopClick = useCallback((e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
  }, []);

  const height = altMax - altMin;
  if (height <= 0 || localBoundary.length < 3) return null;

  // 局部平面 [东, 北] → Shape(x=东, y=北)；绕 x 轴 -90° 使 shape 北(y) → three z 负
  // （2026-08-14 场景坐标改为北=-z 修复朝北镜像；extrude 深度沿 shape z 轴：
  // -90° 后指向 +y（向上），故 mesh 底面放 altMin 顶面向上挤出 height 到 altMax）
  return (
    <mesh
      position={[0, altMin, 0]}
      rotation={[-Math.PI / 2, 0, 0]}
      onPointerDown={handleDown}
      onClick={stopClick}
    >
      <extrudeGeometry
        args={[shape, { steps: 1, depth: height, bevelEnabled: false }]}
      />
      <meshBasicMaterial
        color={color}
        transparent
        opacity={0.22}
        side={THREE.DoubleSide}
      />
    </mesh>
  );
}

import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import type { ThreeEvent } from '@react-three/fiber';
import type { Vec3, GeoRef } from '../types';
import { localToGeo } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, y];
}

interface RadarSphereProps {
  id: string;
  center: Vec3;
  radiusM: number;
  geoRef: GeoRef;
  onRadarMove: (id: string, lon: number, lat: number) => void;
  onDragStateChange: (dragging: boolean) => void;
}

// 地面（Three y=0 平面）——用于拖动时把鼠标射线求交到地平面
const Y0_PLANE = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const RAYCASTER = new THREE.Raycaster();
const NDC = new THREE.Vector2();
const HIT = new THREE.Vector3();

export function RadarSphere({
  id,
  center,
  radiusM,
  geoRef,
  onRadarMove,
  onDragStateChange,
}: RadarSphereProps) {
  const pos = useMemo(() => toThreePos(center), [center]);
  // 本地拖动位置（拖动期间直接改 mesh，不依赖外部 state 高频更新）
  const [localPos, setLocalPos] = useState<[number, number, number]>(pos);
  const { camera, gl } = useThree();
  const dragRef = useRef(false);
  const lastGeoRef = useRef<{ lon: number; lat: number } | null>(null);

  // 外部中心变化（如 pointerup 提交后 config 更新）→ 同步本地位置
  useEffect(() => {
    setLocalPos(pos);
  }, [pos]);

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
      if (RAYCASTER.ray.intersectPlane(Y0_PLANE, HIT)) {
        setLocalPos([HIT.x, pos[1], HIT.z]);
        const g = localToGeo([HIT.x, HIT.z, 0], geoRef);
        lastGeoRef.current = { lon: g.lon, lat: g.lat };
      }
    },
    [camera, gl, geoRef, pos],
  );

  const handleUp = useCallback(() => {
    if (!dragRef.current) return;
    dragRef.current = false;
    onDragStateChange(false);
    const g = lastGeoRef.current;
    lastGeoRef.current = null;
    if (g) onRadarMove(id, g.lon, g.lat);
  }, [id, onDragStateChange, onRadarMove]);

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
      dragRef.current = true;
      lastGeoRef.current = null;
      onDragStateChange(true);
    },
    [onDragStateChange],
  );

  // 阻止 click 冒泡到地面拾取层（避免拖动结束后误设起点/目标）
  const stopClick = useCallback((e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
  }, []);

  if (radiusM <= 0) return null;

  return (
    <group>
      {/* 探测体积（可点选拖动） */}
      <mesh
        position={localPos}
        onPointerDown={handleDown}
        onClick={stopClick}
      >
        <sphereGeometry args={[radiusM, 48, 24]} />
        <meshBasicMaterial color="#ff4444" transparent opacity={0.15} />
      </mesh>
      {/* 地面探测圆盘（可点选拖动） */}
      <mesh
        position={[localPos[0], 0, localPos[2]]}
        rotation={[-Math.PI / 2, 0, 0]}
        onPointerDown={handleDown}
        onClick={stopClick}
      >
        <circleGeometry args={[radiusM, 64]} />
        <meshBasicMaterial color="#ff4444" transparent opacity={0.25} />
      </mesh>
    </group>
  );
}

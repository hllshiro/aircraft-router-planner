import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import type { ThreeEvent } from '@react-three/fiber';
import type { Vec3, GeoRef } from '../types';
import { localToGeo } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, y];
}

interface MidpointMarkerProps {
  /** 所属飞机 id（提交时回传定位） */
  vehicleId: string;
  /** 在 mid_waypoints 数组中的下标 */
  index: number;
  center: Vec3;
  geoRef: GeoRef;
  onMidpointMove: (
    vehicleId: string,
    index: number,
    lon: number,
    lat: number,
  ) => void;
  onDragStateChange: (dragging: boolean) => void;
}

// 地面（Three y=0 平面）——用于拖动时把鼠标射线求交到地平面（同 RadarSphere）
const Y0_PLANE = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const RAYCASTER = new THREE.Raycaster();
const NDC = new THREE.Vector2();
const HIT = new THREE.Vector3();

/** 必经点标记：黄色小球，可在场景中直接拖动改经纬（高度保留） */
export function MidpointMarker({
  vehicleId,
  index,
  center,
  geoRef,
  onMidpointMove,
  onDragStateChange,
}: MidpointMarkerProps) {
  const pos = useMemo(() => toThreePos(center), [center]);
  const [localPos, setLocalPos] = useState<[number, number, number]>(pos);
  const { camera, gl } = useThree();
  const dragRef = useRef(false);
  const lastGeoRef = useRef<{ lon: number; lat: number } | null>(null);

  // 外部位置变化（config 更新后）→ 同步本地
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
    if (g) onMidpointMove(vehicleId, index, g.lon, g.lat);
  }, [vehicleId, index, onDragStateChange, onMidpointMove]);

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

  // 阻止 click 冒泡到地面拾取层（避免拖动结束后误设起点/目标/顶点）
  const stopClick = useCallback((e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
  }, []);

  return (
    <mesh position={localPos} onPointerDown={handleDown} onClick={stopClick}>
      <sphereGeometry args={[140, 16, 8]} />
      <meshBasicMaterial color="#ffee00" />
      {/* 地面小圆盘提示（可拖动落点） */}
      <mesh
        position={[0, -localPos[1], 0]}
        rotation={[-Math.PI / 2, 0, 0]}
      >
        <circleGeometry args={[140, 24]} />
        <meshBasicMaterial color="#ffee00" transparent opacity={0.35} />
      </mesh>
    </mesh>
  );
}

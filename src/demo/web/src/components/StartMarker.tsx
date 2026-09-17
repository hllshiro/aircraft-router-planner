import { useMemo } from 'react';
import * as THREE from 'three';
import type { Vec3, GeoRef } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  // 局部平面 [东, 北, 高] → 场景坐标 [x, z, -y]（Y 轴向上、北=-z，
  // 2026-08-14 修复朝北看镜像：物理右手系）
  return [x, z, -y];
}

interface StartMarkerProps {
  position: Vec3;
  heading: number;
}

export function StartMarker({ position, heading }: StartMarkerProps) {
  const pos = useMemo(() => toThreePos(position), [position]);
  // 航向箭头：cone 默认尖端朝 +y，用四元数把 +y 转到航向方向
  // （场景坐标：北=-z、东=+x；heading 0°=北、90°=东、顺时针）
  const arrowQuat = useMemo(() => {
    const h = (heading * Math.PI) / 180;
    const dir = new THREE.Vector3(Math.sin(h), 0, -Math.cos(h)).normalize();
    return new THREE.Quaternion().setFromUnitVectors(
      new THREE.Vector3(0, 1, 0),
      dir,
    );
  }, [heading]);

  return (
    <group>
      {/* Green sphere */}
      <mesh position={pos}>
        <sphereGeometry args={[150, 32, 16]} />
        <meshStandardMaterial color="#00ff88" />
      </mesh>
      {/* White heading arrow (cone pointing forward) */}
      <mesh position={pos} quaternion={arrowQuat}>
        <coneGeometry args={[60, 200, 8]} />
        <meshStandardMaterial color="#ffffff" />
      </mesh>
    </group>
  );
}

export type { GeoRef };

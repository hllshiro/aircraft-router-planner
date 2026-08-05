import { useMemo } from 'react';
import type { Vec3, GeoRef } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  // 局部平面 [东, 北, 高] → Three.js [x, z, y]（Y 轴向上）
  return [x, z, y];
}

interface StartMarkerProps {
  position: Vec3;
  heading: number;
}

export function StartMarker({ position, heading }: StartMarkerProps) {
  const pos = useMemo(() => toThreePos(position), [position]);
  const arrowRot = useMemo((): [number, number, number] => {
    // 默认圆锥朝 +z（北）；航向 0°=北、90°=东 → rotation.y 取负（顺时针）
    return [0, -(heading * Math.PI) / 180, 0];
  }, [heading]);

  return (
    <group>
      {/* Green sphere */}
      <mesh position={pos}>
        <sphereGeometry args={[150, 32, 16]} />
        <meshStandardMaterial color="#00ff88" />
      </mesh>
      {/* White heading arrow (cone pointing forward) */}
      <mesh position={pos} rotation={arrowRot}>
        <coneGeometry args={[60, 200, 8]} />
        <meshStandardMaterial color="#ffffff" />
      </mesh>
    </group>
  );
}

export type { GeoRef };

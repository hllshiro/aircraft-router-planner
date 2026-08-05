import { useMemo } from 'react';
import type { Vec3 } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, y];
}

interface RadarSphereProps {
  center: Vec3;
  radiusM: number;
}

export function RadarSphere({ center, radiusM }: RadarSphereProps) {
  const pos = useMemo(() => toThreePos(center), [center]);

  if (radiusM <= 0) return null;

  return (
    <group>
      <mesh position={pos}>
        <sphereGeometry args={[radiusM, 48, 24]} />
        <meshBasicMaterial color="#ff4444" transparent opacity={0.15} />
      </mesh>
      {/* 地面探测圆盘 */}
      <mesh position={[pos[0], 0, pos[2]]} rotation={[-Math.PI / 2, 0, 0]}>
        <circleGeometry args={[radiusM, 64]} />
        <meshBasicMaterial color="#ff4444" transparent opacity={0.25} />
      </mesh>
    </group>
  );
}

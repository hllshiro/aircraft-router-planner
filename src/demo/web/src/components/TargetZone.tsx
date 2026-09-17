import { useMemo } from 'react';
import type { Vec3 } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  // 局部平面 [东, 北, 高] → 场景坐标 [x, z, -y]（Y 轴向上、北=-z，2026-08-14）
  return [x, z, -y];
}

interface TargetZoneProps {
  center: Vec3;
}

export function TargetZone({ center }: TargetZoneProps) {
  const pos = useMemo(() => toThreePos(center), [center]);

  return (
    <group>
      <mesh position={pos}>
        <sphereGeometry args={[400, 48, 24]} />
        <meshBasicMaterial color="#4488ff" transparent opacity={0.25} />
      </mesh>
      <mesh position={pos}>
        <sphereGeometry args={[150, 32, 16]} />
        <meshStandardMaterial color="#4488ff" />
      </mesh>
    </group>
  );
}

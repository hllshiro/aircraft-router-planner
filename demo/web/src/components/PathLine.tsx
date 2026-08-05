import { useMemo } from 'react';
import { Line } from '@react-three/drei';
import type { Vec3 } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, y];
}

interface PathLineProps {
  waypoints: Vec3[];
  color?: string;
}

export function PathLine({ waypoints, color = '#ffdd00' }: PathLineProps) {
  const points = useMemo(
    () => waypoints.map((wp) => toThreePos(wp)),
    [waypoints],
  );

  if (points.length < 2) return null;

  return (
    <group>
      <Line points={points} color={color} lineWidth={3} />
      {/* Waypoint dots */}
      {points.map((p, i) => (
        <mesh key={i} position={p}>
          <sphereGeometry args={[60, 16, 8]} />
          <meshBasicMaterial
            color={
              i === 0
                ? '#00ff88'
                : i === points.length - 1
                  ? '#4488ff'
                  : color
            }
          />
        </mesh>
      ))}
    </group>
  );
}

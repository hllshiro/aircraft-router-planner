import { useMemo } from 'react';
import { Line } from '@react-three/drei';
import type { Vec3 } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  // 局部平面 [东, 北, 高] → 场景坐标 [x, z, -y]（Y 轴向上、北=-z，2026-08-14）
  return [x, z, -y];
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

  // 航路点颜色：起点绿 / 终点蓝（语义），中间点按索引 HSL 彩虹渐变——每个航路点
  // 不同色，便于目视区分多段路径的点序列（主管 2026-08-07：用不同颜色显示航路点）。
  const dotColor = (i: number): string => {
    if (i === 0) return '#00ff88';
    if (i === points.length - 1) return '#4488ff';
    const hue = ((i - 1) / Math.max(1, points.length - 2)) * 300;
    return `hsl(${hue}, 100%, 60%)`;
  };

  return (
    <group>
      <Line points={points} color={color} lineWidth={3} />
      {/* Waypoint dots */}
      {points.map((p, i) => (
        <mesh key={i} position={p}>
          <sphereGeometry args={[60, 16, 8]} />
          <meshBasicMaterial color={dotColor(i)} />
        </mesh>
      ))}
    </group>
  );
}

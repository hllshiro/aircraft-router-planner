import { useMemo } from 'react';
import * as THREE from 'three';
import type { Vec2 } from '../types';

interface NFZPrismProps {
  boundaryPoints: Vec2[]; // 局部平面坐标 [x, y]（米）
  altMin: number;
  altMax: number;
  color?: string;
}

export function NFZPrism({
  boundaryPoints,
  altMin,
  altMax,
  color = '#ff8800',
}: NFZPrismProps) {
  const shape = useMemo(() => {
    const s = new THREE.Shape();
    // THREE.Shape.closePath 需要 ≥2 点；少于 2 点时留空（渲染层另有 <3 保护）
    if (boundaryPoints.length < 2) return s;
    s.moveTo(boundaryPoints[0][0], boundaryPoints[0][1]);
    for (let i = 1; i < boundaryPoints.length; i++) {
      s.lineTo(boundaryPoints[i][0], boundaryPoints[i][1]);
    }
    s.closePath();
    return s;
  }, [boundaryPoints]);

  const height = altMax - altMin;
  if (height <= 0 || boundaryPoints.length < 3) return null;

  // 局部平面 [东, 北] → Shape(x=东, y=北)；绕 x 轴 +90° 使 shape 北(y) → three z 正
  // （与 StartMarker/锚点/路径的 toThreePos [东,高,北] 一致；旧 rotation [-π/2] 会把
  // 北映射到 three z 负 → 多边形渲染 z 镜像，顶点锚点与渲染形状南北对不上）。
  // extrude 深度沿 shape z 轴：+90° 后指向 -y（向下），故 mesh 底面放 altMax 顶面向下
  // 挤出 height 到 altMin（底面仍与锚点 altMin 水平对齐）。
  return (
    <mesh position={[0, altMax, 0]} rotation={[Math.PI / 2, 0, 0]}>
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

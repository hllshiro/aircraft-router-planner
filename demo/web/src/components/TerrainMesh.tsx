import { useMemo } from 'react';
import * as THREE from 'three';
import type { ThreeEvent } from '@react-three/fiber';
import type { GeoRef, TerrainInfo, Waypoint } from '../types';
import { localToGeo } from '../types';

interface TerrainMeshProps {
  data: TerrainInfo;
  geoRef: GeoRef;
  /** 统一 z 夸张系数（Scene3D computeZScale；与航路/标记/zone 同一尺度） */
  zScale: number;
  /** 拾取模式下的地形点击（返回地形表面交点经纬，避免夸张后射线打到 y=0 平面偏差） */
  onPick?: (wp: Waypoint) => void;
}

/** 高度 → 颜色（低绿 → 棕 → 高白雪线） */
function heightColor(h: number, minH: number, maxH: number): THREE.Color {
  const t = maxH > minH ? (h - minH) / (maxH - minH) : 0.5;
  const c = new THREE.Color();
  if (t < 0.5) {
    c.lerpColors(
      new THREE.Color(0x2e6b34),
      new THREE.Color(0x8a6b3f),
      t * 2,
    );
  } else {
    c.lerpColors(
      new THREE.Color(0x8a6b3f),
      new THREE.Color(0xdfe8e0),
      (t - 0.5) * 2,
    );
  }
  return c;
}

export function TerrainMesh({ data, geoRef, zScale, onPick }: TerrainMeshProps) {
  const geometry = useMemo(() => {
    const { nx, ny } = data;
    const lat0 = (geoRef.lat * Math.PI) / 180;
    const xScale = 111320 * Math.cos(lat0);
    const yScale = 110574;

    // 统计有效高度范围（无数据点用 0 占位）
    const hs = data.heights.filter((h): h is number => h !== null);
    const minH = hs.length ? Math.min(...hs) : 0;
    const maxH = hs.length ? Math.max(...hs) : 1;

    const geo = new THREE.BufferGeometry();
    const positions = new Float32Array(nx * ny * 3);
    const colors = new Float32Array(nx * ny * 3);
    const indices: number[] = [];

    for (let j = 0; j < ny; j++) {
      const lat = data.max_lat - (j * (data.max_lat - data.min_lat)) / (ny - 1);
      const y = (lat - geoRef.lat) * yScale;
      for (let i = 0; i < nx; i++) {
        const lon =
          data.min_lon + (i * (data.max_lon - data.min_lon)) / (nx - 1);
        const x = (lon - geoRef.lon) * xScale;
        const h = data.heights[j * nx + i];
        const z = h === null ? 0 : (h - minH) * zScale;
        const idx = j * nx + i;
        positions[idx * 3] = x;
        positions[idx * 3 + 1] = z;
        positions[idx * 3 + 2] = y;
        const col = heightColor(h ?? 0, minH, maxH);
        colors[idx * 3] = col.r;
        colors[idx * 3 + 1] = col.g;
        colors[idx * 3 + 2] = col.b;
        if (i < nx - 1 && j < ny - 1) {
          const a = idx;
          const b = idx + 1;
          const c = idx + nx;
          const d = idx + nx + 1;
          indices.push(a, c, b, b, c, d);
        }
      }
    }

    geo.setAttribute('position', new THREE.BufferAttribute(positions, 3));
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
  }, [data, geoRef]);

  return (
    <mesh
      geometry={geometry}
      receiveShadow
      onClick={
        onPick
          ? (e: ThreeEvent<MouseEvent>) => {
              e.stopPropagation();
              // 地形表面交点（x=东/米、z=南/米）→ 经纬；高度用 0（起终点高度由表单决定）
              onPick(localToGeo([e.point.x, e.point.z, 0], geoRef));
            }
          : undefined
      }
    >
      <meshStandardMaterial
        vertexColors
        side={THREE.DoubleSide}
        transparent
        opacity={0.9}
      />
    </mesh>
  );
}

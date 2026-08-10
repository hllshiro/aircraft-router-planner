import { useMemo } from 'react';
import * as THREE from 'three';
import type { GeoRef, TerrainInfo } from '../types';

interface TerrainMeshProps {
  data: TerrainInfo;
  geoRef: GeoRef;
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

export function TerrainMesh({ data, geoRef }: TerrainMeshProps) {
  const geometry = useMemo(() => {
    const { nx, ny } = data;
    const lat0 = (geoRef.lat * Math.PI) / 180;
    const xScale = 111320 * Math.cos(lat0);
    const yScale = 110574;

    // 统计有效高度范围（无数据点用 0 占位）
    const hs = data.heights.filter((h): h is number => h !== null);
    const minH = hs.length ? Math.min(...hs) : 0;
    const maxH = hs.length ? Math.max(...hs) : 1;

    // z 轴夸张：绝对高度在场景跨度（百 km 级）下不可见，按 (h - minH) × 夸张系数
    // 映射。系数取场景跨度 / 高度范围 × 0.25，clamp [10, 60]（主管 2026-08-08：
    // 7.5as 地形看不到特征 → 相对高度 + z 夸张，让山区起伏可辨）。
    const spanMeters = Math.hypot(
      (data.max_lon - data.min_lon) * xScale,
      (data.max_lat - data.min_lat) * yScale,
    );
    const range = Math.max(maxH - minH, 1);
    const zScale = Math.min(Math.max((spanMeters / range) * 0.25, 10), 60);

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
    <mesh geometry={geometry} receiveShadow>
      <meshStandardMaterial
        vertexColors
        side={THREE.DoubleSide}
        transparent
        opacity={0.9}
      />
    </mesh>
  );
}

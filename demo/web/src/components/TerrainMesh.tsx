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
  /** 底图纹理（mask/tiff 瓦片纹理或 WMS 视口纹理）；有纹理 → 替换顶点高度着色，
   *  地形起伏 + zScale 夸张作用在底图上（2026-08-13 主管：底图贴地形表面） */
  texture?: THREE.Texture | null;
  /** 纹理对应的经纬 bbox（mask/tiff = 瓦片 bbox；WMS = 视口 bbox） */
  textureBbox?: [number, number, number, number] | null;
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

export function TerrainMesh({
  data,
  geoRef,
  zScale,
  texture,
  textureBbox,
  onPick,
}: TerrainMeshProps) {
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
    const uvs = new Float32Array(nx * ny * 2);
    const hasUv = Boolean(textureBbox);
    const indices: number[] = [];

    // 纹理经纬 → uv（v 南→北 0→1，与 CanvasTexture flipY=true 匹配：canvas 顶行在北）
    let u0 = 0;
    let u1 = 1;
    let v0 = 0;
    let v1 = 1;
    if (textureBbox) {
      const [tMinLon, tMinLat, tMaxLon, tMaxLat] = textureBbox;
      u0 = tMinLon;
      u1 = tMaxLon - tMinLon || 1;
      v0 = tMinLat;
      v1 = tMaxLat - tMinLat || 1;
    }

    for (let j = 0; j < ny; j++) {
      const lat = data.max_lat - (j * (data.max_lat - data.min_lat)) / (ny - 1);
      const y = (lat - geoRef.lat) * yScale;
      for (let i = 0; i < nx; i++) {
        const lon =
          data.min_lon + (i * (data.max_lon - data.min_lon)) / (nx - 1);
        const x = (lon - geoRef.lon) * xScale;
        const h = data.heights[j * nx + i];
        // 顶点高度 = 真实海拔（米）：所有瓦片同一基准 → 边界无裂缝；
        // z 夸张由 mesh.scale.y 应用（2026-08-13 瓦片化后修正）
        const z = h === null ? 0 : h;
        const idx = j * nx + i;
        // 场景坐标（2026-08-14 修复朝北镜像）：x=东、y=高、z=-北（物理右手系）
        positions[idx * 3] = x;
        positions[idx * 3 + 1] = z;
        positions[idx * 3 + 2] = -y;
        const col = heightColor(h ?? 0, minH, maxH);
        colors[idx * 3] = col.r;
        colors[idx * 3 + 1] = col.g;
        colors[idx * 3 + 2] = col.b;
        if (hasUv) {
          uvs[idx * 2] = (lon - u0) / u1;
          uvs[idx * 2 + 1] = (lat - v0) / v1;
        }
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
    if (hasUv) {
      geo.setAttribute('uv', new THREE.BufferAttribute(uvs, 2));
    }
    geo.setAttribute('color', new THREE.BufferAttribute(colors, 3));
    geo.setIndex(indices);
    geo.computeVertexNormals();
    return geo;
  }, [data, geoRef, textureBbox]);

  return (
    <mesh
      geometry={geometry}
      scale={[1, zScale, 1]}
      receiveShadow
      onClick={
        onPick
          ? (e: ThreeEvent<MouseEvent>) => {
              e.stopPropagation();
              // 地形表面交点（x=东/米、z=-北/米，e.point 已含 scale）→ 经纬
              // （2026-08-14 北取反：纬度 = ref.lat - z / ky）
              onPick(localToGeo([e.point.x, -e.point.z, 0], geoRef));
            }
          : undefined
      }
    >
      {texture ? (
        <meshStandardMaterial
          map={texture}
          side={THREE.DoubleSide}
        />
      ) : (
        <meshStandardMaterial
          vertexColors
          side={THREE.DoubleSide}
          transparent
          opacity={0.9}
        />
      )}
    </mesh>
  );
}

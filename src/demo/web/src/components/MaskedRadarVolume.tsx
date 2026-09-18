import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import { useThree } from '@react-three/fiber';
import * as THREE from 'three';
import type { ThreeEvent } from '@react-three/fiber';
import type { Vec3, GeoRef } from '../types';
import { localToGeo } from '../types';

function toThreePos([x, y, z]: Vec3): [number, number, number] {
  return [x, z, -y];
}

interface MaskedRadarVolumeProps {
  id: string;
  center: Vec3;
  radiusM: number;
  radarAltM: number;
  geoRef: GeoRef;
  sampleHeight: (lon: number, lat: number) => number | null;
  onRadarMove: (id: string, lon: number, lat: number) => void;
  onDragStateChange: (dragging: boolean) => void;
}

const N_AZ = 24;
const N_EL = 12;
const RAY_SAMPLES = 24;
const EL_MIN_DEG = -5;
const EL_MAX_DEG = 80;

const Y0_PLANE = new THREE.Plane(new THREE.Vector3(0, 1, 0), 0);
const RAYCASTER = new THREE.Raycaster();
const NDC = new THREE.Vector2();
const HIT = new THREE.Vector3();

function buildMaskedGeometry(
  radiusM: number,
  radarAltM: number,
  geoRef: GeoRef,
  sampleHeight: (lon: number, lat: number) => number | null,
): THREE.BufferGeometry {
  const lat0Rad = (geoRef.lat * Math.PI) / 180;
  const kx = 111320 * Math.cos(lat0Rad);
  const ky = 110574;
  const stepM = radiusM / RAY_SAMPLES;

  const verts: number[] = [];
  const indices: number[] = [];

  const elRange = EL_MAX_DEG - EL_MIN_DEG;

  for (let ia = 0; ia <= N_AZ; ia++) {
    const az = (ia / N_AZ) * Math.PI * 2;
    const cosAz = Math.cos(az);
    const sinAz = Math.sin(az);

    for (let ie = 0; ie <= N_EL; ie++) {
      const elRad = ((EL_MIN_DEG + (ie / N_EL) * elRange) * Math.PI) / 180;
      const cosEl = Math.cos(elRad);
      const sinEl = Math.sin(elRad);

      const dx = cosEl * cosAz;
      const dy = sinEl;
      const dz = cosEl * sinAz;

      let visDist = radiusM;

      for (let s = 1; s <= RAY_SAMPLES; s++) {
        const dist = stepM * s;
        const lon = geoRef.lon + (dx * dist) / kx;
        const lat = geoRef.lat + (dz * dist) / ky;
        const h = sampleHeight(lon, lat);
        const losAlt = radarAltM + dy * dist;

        if (h === null) continue;
        if (h > losAlt) {
          visDist = dist;
          break;
        }
      }

      const scale = visDist / radiusM;
      verts.push(dx * radiusM * scale, dy * radiusM * scale, dz * radiusM * scale);
    }
  }

  const stride = N_EL + 1;
  for (let ia = 0; ia < N_AZ; ia++) {
    for (let ie = 0; ie < N_EL; ie++) {
      const a = ia * stride + ie;
      const b = (ia + 1) * stride + ie;
      const c = (ia + 1) * stride + ie + 1;
      const d = ia * stride + ie + 1;
      indices.push(a, b, d);
      indices.push(b, c, d);
    }
  }

  const geo = new THREE.BufferGeometry();
  geo.setAttribute('position', new THREE.Float32BufferAttribute(verts, 3));
  geo.setIndex(indices);
  geo.computeVertexNormals();
  return geo;
}

export function MaskedRadarVolume({
  id,
  center,
  radiusM,
  radarAltM,
  geoRef,
  sampleHeight,
  onRadarMove,
  onDragStateChange,
}: MaskedRadarVolumeProps) {
  const pos = useMemo(() => toThreePos(center), [center]);
  const [localPos, setLocalPos] = useState<[number, number, number]>(pos);
  const { camera, gl } = useThree();
  const dragRef = useRef(false);
  const lastGeoRef = useRef<{ lon: number; lat: number } | null>(null);

  useEffect(() => {
    setLocalPos(pos);
  }, [pos]);

  const geometry = useMemo(
    () => buildMaskedGeometry(radiusM, radarAltM, geoRef, sampleHeight),
    [radiusM, radarAltM, geoRef, sampleHeight],
  );

  const handleMove = useCallback(
    (e: PointerEvent) => {
      if (!dragRef.current) return;
      const rect = gl.domElement.getBoundingClientRect();
      if (rect.width === 0 || rect.height === 0) return;
      NDC.set(
        ((e.clientX - rect.left) / rect.width) * 2 - 1,
        -((e.clientY - rect.top) / rect.height) * 2 + 1,
      );
      RAYCASTER.setFromCamera(NDC, camera);
      if (RAYCASTER.ray.intersectPlane(Y0_PLANE, HIT)) {
        setLocalPos([HIT.x, pos[1], HIT.z]);
        const g = localToGeo([HIT.x, -HIT.z, 0], geoRef);
        lastGeoRef.current = { lon: g.lon, lat: g.lat };
      }
    },
    [camera, gl, geoRef, pos],
  );

  const handleUp = useCallback(() => {
    if (!dragRef.current) return;
    dragRef.current = false;
    onDragStateChange(false);
    const g = lastGeoRef.current;
    lastGeoRef.current = null;
    if (g) onRadarMove(id, g.lon, g.lat);
  }, [id, onDragStateChange, onRadarMove]);

  useEffect(() => {
    window.addEventListener('pointermove', handleMove);
    window.addEventListener('pointerup', handleUp);
    return () => {
      window.removeEventListener('pointermove', handleMove);
      window.removeEventListener('pointerup', handleUp);
    };
  }, [handleMove, handleUp]);

  const handleDown = useCallback(
    (e: ThreeEvent<PointerEvent>) => {
      e.stopPropagation();
      dragRef.current = true;
      lastGeoRef.current = null;
      onDragStateChange(true);
    },
    [onDragStateChange],
  );

  const stopClick = useCallback((e: ThreeEvent<MouseEvent>) => {
    e.stopPropagation();
  }, []);

  if (radiusM <= 0) return null;

  return (
    <group>
      <mesh
        position={localPos}
        geometry={geometry}
        onPointerDown={handleDown}
        onClick={stopClick}
      >
        <meshBasicMaterial
          color="#ff4444"
          transparent
          opacity={0.15}
          side={THREE.DoubleSide}
        />
      </mesh>
      <mesh
        position={[localPos[0], 0, localPos[2]]}
        rotation={[-Math.PI / 2, 0, 0]}
        onPointerDown={handleDown}
        onClick={stopClick}
      >
        <circleGeometry args={[radiusM, 64]} />
        <meshBasicMaterial color="#ff4444" transparent opacity={0.25} />
      </mesh>
    </group>
  );
}

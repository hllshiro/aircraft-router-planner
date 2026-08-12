import { useState, useCallback, useEffect } from 'react';
import { Scene3D } from './components/Scene3D';
import { ControlPanel } from './components/ControlPanel';
import type {
  InputConfig,
  PlanResult,
  TerrainInfo,
  Waypoint,
  Zone,
  CircleGeometry,
  PolygonGeometry,
} from './types';
import { defaultInputConfig } from './types';
import { planRoute, sceneBounds, fetchTerrain } from './api';

type ClickMode = 'start' | 'target' | 'midpoint' | 'polygon' | null;

/** 解析 target_ref 当前目标高度（自定义坐标第 3 段；缺省回落 mission.target.alt_m） */
function currentTargetAlt(v: { target_ref?: string } | undefined, fallback: number): number {
  if (v?.target_ref) {
    const parts = v.target_ref.split(',').map((s) => s.trim());
    const a = +parts[2];
    if (parts.length >= 3 && Number.isFinite(a)) return a;
  }
  return fallback;
}

export default function App() {
  const [config, setConfig] = useState<InputConfig>(defaultInputConfig);
  const [result, setResult] = useState<PlanResult | null>(null);
  const [loading, setLoading] = useState(false);
  const [clickMode, setClickMode] = useState<ClickMode>(null);
  // 每机拾取目标下标（clickMode === 'start' | 'target' 时生效；每机独立起终点）
  const [pickVehicleIdx, setPickVehicleIdx] = useState<number | null>(null);
  // 多边形拾取编辑目标 zone id（clickMode === 'polygon' 时生效）
  const [editingZoneId, setEditingZoneId] = useState<string | null>(null);
  // 地形网格（source=path 时按场景范围采样）
  const [terrainData, setTerrainData] = useState<TerrainInfo | null>(null);
  const [terrainError, setTerrainError] = useState<string | null>(null);

  useEffect(() => {
    const t = config.mission.terrain;
    if (t.source !== 'path' || !t.path) {
      setTerrainData(null);
      setTerrainError(null);
      return;
    }
    let cancelled = false;
    setTerrainError(null);
    // 外部格式（GeoTIFF/DTED/SRTM）：渲染范围=数据范围、精度=server 按跨度自适应
    // （2026-08-11 主管：载入外部地形时渲染应达到数据的范围与精度，而非场景包围盒）
    const isExternal = /\.(tif|tiff|hgt|dt0|dt1|dt2)$/i.test(t.path);
    if (isExternal) {
      fetchTerrain(t.path, null, null)
        .then((d) => {
          if (!cancelled) setTerrainData(d);
        })
        .catch((e) => {
          if (!cancelled) {
            setTerrainData(null);
            setTerrainError(String(e));
          }
        });
      return () => {
        cancelled = true;
      };
    }
    const bbox = sceneBounds(config);
    // 96×96：跨度变大（sceneBounds 1.4 倍自适应）后保持地形精细度（server 上限 128）
    fetchTerrain(t.path, bbox, [96, 96])
      .then((d) => {
        if (!cancelled) setTerrainData(d);
      })
      .catch((e) => {
        if (!cancelled) {
          setTerrainData(null);
          setTerrainError(String(e));
        }
      });
    return () => {
      cancelled = true;
    };
  }, [
    config.mission.terrain.source,
    config.mission.terrain.path,
    config.mission.start.lon,
    config.mission.start.lat,
    config.mission.target.lon,
    config.mission.target.lat,
    // 每机起点/目标变化 → bbox 变化 → 地形网格刷新（2026-08-10 每机独立起终点）
    JSON.stringify(
      config.mission.vehicles.map((v) => [
        v.start_pose.lon,
        v.start_pose.lat,
        v.target_ref ?? '',
      ]),
    ),
  ]);

  const handlePlan = async () => {
    setLoading(true);
    setResult(null);
    try {
      const res = await planRoute(config);
      setResult(res);
    } catch (err) {
      setResult({
        schema_version: '0.20',
        status: 'input_invalid',
        error: {
          code: 'NETWORK_ERROR',
          message: String(err),
        },
        elapsed_ms: 0,
        vehicles: [],
        stats: { fmm_ms: 0, los_checks: 0, degradations: [] },
      });
    } finally {
      setLoading(false);
    }
  };

  const handleRadarMove = useCallback((id: string, lon: number, lat: number) => {
    setConfig((prev) => ({
      ...prev,
      mission: {
        ...prev.mission,
        red_forces: {
          ...prev.mission.red_forces,
          radars: prev.mission.red_forces.radars.map((r) =>
            r.id === id ? { ...r, lon, lat } : r,
          ),
        },
      },
    }));
  }, []);

  const handleZoneMove = useCallback((id: string, dLon: number, dLat: number) => {
    setConfig((prev) => {
      const shift = (z: Zone): Zone => {
        if (z.id !== id) return z;
        if (z.shape === 'circle') {
          const g = z.geometry as CircleGeometry;
          return {
            ...z,
            geometry: {
              ...g,
              center: [g.center[0] + dLon, g.center[1] + dLat],
            },
          };
        }
        const g = z.geometry as PolygonGeometry;
        return {
          ...z,
          geometry: {
            vertices: g.vertices.map(
              ([lon, lat]) => [lon + dLon, lat + dLat] as [number, number],
            ),
          },
        };
      };
      const m = prev.mission;
      return {
        ...prev,
        mission: {
          ...m,
          no_fly_zones: m.no_fly_zones.map(shift),
          restricted_zones: m.restricted_zones.map(shift),
          obstacles: m.obstacles.map(shift),
        },
      };
    });
  }, []);

  const handleMidpointMove = useCallback(
    (vehicleId: string, index: number, lon: number, lat: number) => {
      setConfig((prev) => ({
        ...prev,
        mission: {
          ...prev.mission,
          vehicles: prev.mission.vehicles.map((v) => {
            if (v.id !== vehicleId) return v;
            const ms = [...(v.mid_waypoints ?? [])];
            if (!ms[index]) return v;
            ms[index] = { ...ms[index], lon, lat };
            return { ...v, mid_waypoints: ms };
          }),
        },
      }));
    },
    [],
  );

  const handleGroundClick = useCallback(
    (wp: Waypoint) => {
      // 每机独立起点/终点拾取（2026-08-10）：start → 该机 start_pose；target → 该机
      // target_ref（"lon,lat,alt" 自定义坐标，核心已支持每机独立目标）
      if ((clickMode === 'start' || clickMode === 'target') && pickVehicleIdx !== null) {
        setConfig((prev) => {
          const vehicles = prev.mission.vehicles.map((v, i) => {
            if (i !== pickVehicleIdx) return v;
            if (clickMode === 'start') {
              return {
                ...v,
                start_pose: { ...v.start_pose, lon: wp.lon, lat: wp.lat },
              };
            }
            const alt = currentTargetAlt(v, prev.mission.target.alt_m);
            return { ...v, target_ref: `${wp.lon},${wp.lat},${alt}` };
          });
          return { ...prev, mission: { ...prev.mission, vehicles } };
        });
        setClickMode(null);
        setPickVehicleIdx(null);
      } else if (clickMode === 'midpoint' && pickVehicleIdx !== null) {
        // 场景拾取添加必经点（2026-08-10）：高度取该机起点高度；保持拾取模式可连续添加
        setConfig((prev) => {
          const vehicles = prev.mission.vehicles.map((v, i) => {
            if (i !== pickVehicleIdx) return v;
            return {
              ...v,
              mid_waypoints: [
                ...(v.mid_waypoints ?? []),
                { lon: wp.lon, lat: wp.lat, alt_m: v.start_pose.alt_m },
              ],
            };
          });
          return { ...prev, mission: { ...prev.mission, vehicles } };
        });
      } else if (clickMode === 'polygon' && editingZoneId) {
        // 场景拾取追加多边形顶点（禁飞区或限飞区，2026-08-12 补限飞区支持）
        setConfig((prev) => {
          // 先查禁飞区数组（zone_ 前缀），未命中则查限飞区数组（rz_ 前缀）
          const inNoFly = prev.mission.no_fly_zones.some(
            (z) => z.id === editingZoneId,
          );
          if (inNoFly) {
            const zones = prev.mission.no_fly_zones.map((z) => {
              if (z.id !== editingZoneId || z.shape !== 'polygon') return z;
              return {
                ...z,
                geometry: {
                  vertices: [
                    ...(z.geometry as { vertices: [number, number][] }).vertices,
                    [wp.lon, wp.lat] as [number, number],
                  ],
                },
              };
            });
            return { ...prev, mission: { ...prev.mission, no_fly_zones: zones } };
          }
          const rzones = prev.mission.restricted_zones.map((z) => {
            if (z.id !== editingZoneId || z.shape !== 'polygon') return z;
            return {
              ...z,
              geometry: {
                vertices: [
                  ...(z.geometry as { vertices: [number, number][] }).vertices,
                  [wp.lon, wp.lat] as [number, number],
                ],
              },
            };
          });
          return {
            ...prev,
            mission: { ...prev.mission, restricted_zones: rzones },
          };
        });
      }
    },
    [clickMode, editingZoneId, pickVehicleIdx],
  );

  const bbox = sceneBounds(config);

  return (
    <div className="app-layout">
      <div className="panel">
        <ControlPanel
          config={config}
          onConfigChange={setConfig}
          onPlan={handlePlan}
          result={result}
          loading={loading}
          activeClickMode={clickMode}
          onSetClickMode={setClickMode}
          editingZoneId={editingZoneId}
          onEditingZoneId={setEditingZoneId}
          pickVehicleIdx={pickVehicleIdx}
          onPickVehicle={setPickVehicleIdx}
        />
      </div>
      <div className="canvas">
        <Scene3D
          geoRef={{
            lon: (bbox[0] + bbox[2]) / 2,
            lat: (bbox[1] + bbox[3]) / 2,
          }}
          start={config.mission.start}
          target={config.mission.target}
          vehicles={config.mission.vehicles}
          radars={config.mission.red_forces.radars}
          zones={[
            ...config.mission.no_fly_zones,
            ...config.mission.restricted_zones,
            ...config.mission.obstacles,
          ]}
          results={result?.vehicles ?? null}
          terrainData={terrainData}
          bounds={sceneBounds(config)}
          onGroundClick={handleGroundClick}
          onRadarMove={handleRadarMove}
          onZoneMove={handleZoneMove}
          onMidpointMove={handleMidpointMove}
          activeClickMode={clickMode}
        />
        {terrainError && (
          <div className="canvas-overlay-error">⚠ 地形加载失败: {terrainError}</div>
        )}
        {config.mission.terrain.source === 'path' && !terrainData && !terrainError && (
          <div className="canvas-overlay-loading">⏳ 地形采样中…</div>
        )}
      </div>
    </div>
  );
}

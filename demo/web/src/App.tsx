import { useState, useCallback, useMemo } from 'react';
import { Scene3D } from './components/Scene3D';
import { ControlPanel } from './components/ControlPanel';
import type {
  Input,
  PlanResult,
  Waypoint,
  Zone,
  CircleGeometry,
  PolygonGeometry,
  BaseMapConfig,
} from './types';
import { buildDefaultInput, defaultBaseMapConfig } from './types';
import { planRoute, sceneBounds } from './api';

type ClickMode = 'start' | 'target' | 'midpoint' | 'polygon' | null;

export default function App() {
  const [config, setConfig] = useState<Input>(buildDefaultInput);
  const [result, setResult] = useState<PlanResult | null>(null);
  const [loading, setLoading] = useState(false);
  const [clickMode, setClickMode] = useState<ClickMode>(null);
  // 拾取目标机下标（clickMode === 'start' | 'target' | 'midpoint' 时生效；逐机起终点/必经点）
  const [pickAircraftIdx, setPickAircraftIdx] = useState<number | null>(null);
  // 多边形拾取编辑目标 zone id（clickMode === 'polygon' 时生效）
  const [editingZoneId, setEditingZoneId] = useState<string | null>(null);
  // 底图层（2026-08-13：掩膜 / GeoTIFF / WMS 三选一；地形/底图瓦片由 Scene3D 按
  // 相机视口加载——2026-08-13 主管：视口完全独立，漫游到哪加载到哪）
  const [baseMapConfig, setBaseMapConfig] = useState<BaseMapConfig>(
    defaultBaseMapConfig,
  );
  const [baseMapLoading, setBaseMapLoading] = useState(false);
  const [baseMapError, setBaseMapError] = useState<string | null>(null);
  // 视口瓦片是否在加载（canvas overlay 用）
  const [tilesLoading, setTilesLoading] = useState(false);

  // Scene3D 瓦片加载状态回调（loading/error 汇总 → ControlPanel 底图区 + overlay）
  const handleTilesStatus = useCallback(
    (loading: boolean, error: string | null) => {
      setBaseMapLoading(loading);
      setBaseMapError(error);
      setTilesLoading(loading);
    },
    [],
  );

  const handlePlan = async () => {
    setLoading(true);
    setResult(null);
    try {
      const res = await planRoute(config);
      setResult(res);
    } catch (err) {
      setResult({
        status: 'input_invalid',
        error: {
          code: 'NETWORK_ERROR',
          message: String(err),
        },
        elapsed_ms: 0,
        aircraft: [],
        stats: { fmm_ms: 0, los_checks: 0, degradations: [] },
      });
    } finally {
      setLoading(false);
    }
  };

  const handleRadarMove = useCallback((id: string, lon: number, lat: number) => {
    setConfig((prev) => ({
      ...prev,
      red_forces: {
        ...prev.red_forces,
        radars: prev.red_forces.radars.map((r) =>
          r.id === id ? { ...r, lon, lat } : r,
        ),
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
      return {
        ...prev,
        no_fly_zones: prev.no_fly_zones.map(shift),
        restricted_zones: prev.restricted_zones.map(shift),
        obstacles: prev.obstacles.map(shift),
      };
    });
  }, []);

  const handleMidpointMove = useCallback(
    (aircraftId: string, index: number, lon: number, lat: number) => {
      setConfig((prev) => ({
        ...prev,
        aircraft: prev.aircraft.map((a) => {
          if (a.id !== aircraftId) return a;
          const ms = [...(a.mid_waypoints ?? [])];
          if (!ms[index]) return a;
          ms[index] = { ...ms[index], lon, lat };
          return { ...a, mid_waypoints: ms };
        }),
      }));
    },
    [],
  );

  const handleGroundClick = useCallback(
    (wp: Waypoint) => {
      // 逐机独立起点/终点拾取：start → 该机 start；target → 该机 target（2026-08-19：
      // 契约拍平后 target 为必填 Waypoint，直接改坐标，保留原高度）
      if ((clickMode === 'start' || clickMode === 'target') && pickAircraftIdx !== null) {
        setConfig((prev) => ({
          ...prev,
          aircraft: prev.aircraft.map((a, i) => {
            if (i !== pickAircraftIdx) return a;
            if (clickMode === 'start') {
              return { ...a, start: { ...a.start, lon: wp.lon, lat: wp.lat } };
            }
            return { ...a, target: { ...a.target, lon: wp.lon, lat: wp.lat } };
          }),
        }));
        setClickMode(null);
        setPickAircraftIdx(null);
      } else if (clickMode === 'midpoint' && pickAircraftIdx !== null) {
        // 场景拾取添加必经点：高度取该机起点高度；保持拾取模式可连续添加
        setConfig((prev) => ({
          ...prev,
          aircraft: prev.aircraft.map((a, i) => {
            if (i !== pickAircraftIdx) return a;
            return {
              ...a,
              mid_waypoints: [
                ...(a.mid_waypoints ?? []),
                { lon: wp.lon, lat: wp.lat, alt_m: a.start.alt_m },
              ],
            };
          }),
        }));
      } else if (clickMode === 'polygon' && editingZoneId) {
        // 场景拾取追加多边形顶点（禁飞区或限飞区）
        setConfig((prev) => {
          // 先查禁飞区数组（zone_ 前缀），未命中则查限飞区数组（rz_ 前缀）
          const inNoFly = prev.no_fly_zones.some(
            (z) => z.id === editingZoneId,
          );
          if (inNoFly) {
            const zones = prev.no_fly_zones.map((z) => {
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
            return { ...prev, no_fly_zones: zones };
          }
          const rzones = prev.restricted_zones.map((z) => {
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
          return { ...prev, restricted_zones: rzones };
        });
      }
    },
    [clickMode, editingZoneId, pickAircraftIdx],
  );

  const bbox = sceneBounds(config);

  // 场景高度范围（米）：逐机 start/target / 结果路径高度
  // —— 无地形数据时驱动 z 夸张（2026-08-12：起终点不同高度轨迹需呈现倾斜）
  const sceneAltRange = useMemo(() => {
    const alts: number[] = [];
    for (const a of config.aircraft) {
      alts.push(a.start.alt_m, a.target.alt_m);
    }
    result?.aircraft.forEach((ao) => ao.path.forEach((p) => alts.push(p.alt_m)));
    if (!alts.length) return 2000;
    return Math.max(Math.max(...alts) - Math.min(...alts), 1);
  }, [config, result]);

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
          pickAircraftIdx={pickAircraftIdx}
          onPickAircraft={setPickAircraftIdx}
          baseMapConfig={baseMapConfig}
          onBaseMapConfigChange={setBaseMapConfig}
          baseMapLoading={baseMapLoading}
          baseMapError={baseMapError}
        />
      </div>
      <div className="canvas">
        <Scene3D
          geoRef={{
            lon: (bbox[0] + bbox[2]) / 2,
            lat: (bbox[1] + bbox[3]) / 2,
          }}
          target={config.aircraft[0].target}
          aircraft={config.aircraft}
          radars={config.red_forces.radars}
          zones={[
            ...config.no_fly_zones.map((z) => ({
              ...z,
              zone_type: 'no_fly' as const,
            })),
            ...config.restricted_zones.map((z) => ({
              ...z,
              zone_type: 'restricted' as const,
            })),
            ...config.obstacles.map((z) => ({
              ...z,
              zone_type: 'obstacle' as const,
            })),
          ]}
          results={result?.aircraft ?? null}
          terrainConfig={config.terrain}
          baseMapConfig={baseMapConfig}
          sceneAltRange={sceneAltRange}
          bounds={sceneBounds(config)}
          onGroundClick={handleGroundClick}
          onRadarMove={handleRadarMove}
          onZoneMove={handleZoneMove}
          onMidpointMove={handleMidpointMove}
          activeClickMode={clickMode}
          onTilesStatus={handleTilesStatus}
        />
        {baseMapError && (
          <div className="canvas-overlay-error">⚠ 数据加载失败: {baseMapError}</div>
        )}
        {tilesLoading && (
          <div className="canvas-overlay-loading">⏳ 视口数据加载中…</div>
        )}
      </div>
    </div>
  );
}

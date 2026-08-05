import { useState, useCallback, useEffect } from 'react';
import { Scene3D } from './components/Scene3D';
import { ControlPanel } from './components/ControlPanel';
import type { InputConfig, PlanResult, TerrainInfo, Waypoint } from './types';
import { defaultInputConfig } from './types';
import { planRoute, sceneBounds, fetchTerrain } from './api';

type ClickMode = 'start' | 'target' | 'polygon' | null;

export default function App() {
  const [config, setConfig] = useState<InputConfig>(defaultInputConfig);
  const [result, setResult] = useState<PlanResult | null>(null);
  const [loading, setLoading] = useState(false);
  const [clickMode, setClickMode] = useState<ClickMode>(null);
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
    const bbox = sceneBounds(config);
    fetchTerrain(t.path, bbox, [64, 64])
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

  const handleGroundClick = useCallback(
    (wp: Waypoint) => {
      if (clickMode === 'start') {
        setConfig((prev) => ({
          ...prev,
          mission: {
            ...prev.mission,
            start: { ...prev.mission.start, lon: wp.lon, lat: wp.lat },
            vehicles: prev.mission.vehicles.map((v) => ({
              ...v,
              start_pose: {
                ...v.start_pose,
                lon: wp.lon,
                lat: wp.lat,
              },
            })),
          },
        }));
        // 起点选定后自动退出拾取模式（避免持续拾取重复触发）
        setClickMode(null);
      } else if (clickMode === 'target') {
        setConfig((prev) => ({
          ...prev,
          mission: {
            ...prev.mission,
            target: { ...prev.mission.target, lon: wp.lon, lat: wp.lat },
          },
        }));
        setClickMode(null);
      } else if (clickMode === 'polygon' && editingZoneId) {
        // 向多边形禁飞区追加顶点
        setConfig((prev) => {
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
        });
      }
    },
    [clickMode, editingZoneId],
  );

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
        />
      </div>
      <div className="canvas">
        <Scene3D
          geoRef={{
            lon: config.mission.start.lon,
            lat: config.mission.start.lat,
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
          onGroundClick={handleGroundClick}
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

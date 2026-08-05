import { useState, useCallback } from 'react';
import { Scene3D } from './components/Scene3D';
import { ControlPanel } from './components/ControlPanel';
import type { InputConfig, PlanResult, Waypoint } from './types';
import { defaultInputConfig } from './types';
import { planRoute } from './api';

export default function App() {
  const [config, setConfig] = useState<InputConfig>(defaultInputConfig);
  const [result, setResult] = useState<PlanResult | null>(null);
  const [loading, setLoading] = useState(false);
  const [clickMode, setClickMode] = useState<'start' | 'target' | null>(null);

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
      } else if (clickMode === 'target') {
        setConfig((prev) => ({
          ...prev,
          mission: {
            ...prev.mission,
            target: { ...prev.mission.target, lon: wp.lon, lat: wp.lat },
          },
        }));
      }
    },
    [clickMode],
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
        />
      </div>
      <div className="canvas">
        <Scene3D
          ref={{
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
          onGroundClick={handleGroundClick}
          activeClickMode={clickMode}
        />
      </div>
    </div>
  );
}

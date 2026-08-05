import { useCallback } from 'react';
import type {
  InputConfig,
  PlanResult,
  Radar,
  Zone,
  Waypoint,
  VehicleInput,
} from '../types';
import { ResultPanel } from './ResultPanel';

interface ControlPanelProps {
  config: InputConfig;
  onConfigChange: (config: InputConfig) => void;
  onPlan: () => void;
  result: PlanResult | null;
  loading: boolean;
  activeClickMode: 'start' | 'target' | null;
  onSetClickMode: (mode: 'start' | 'target' | null) => void;
}

export function ControlPanel({
  config,
  onConfigChange,
  onPlan,
  result,
  loading,
  activeClickMode,
  onSetClickMode,
}: ControlPanelProps) {
  const mission = config.mission;
  const update = (patch: Partial<InputConfig>) =>
    onConfigChange({ ...config, ...patch });
  const updateMission = (patch: Partial<InputConfig['mission']>) =>
    update({ mission: { ...mission, ...patch } });

  const updateStart = (patch: Partial<Waypoint>) =>
    updateMission({ start: { ...mission.start, ...patch } });
  const updateTarget = (patch: Partial<Waypoint>) =>
    updateMission({ target: { ...mission.target, ...patch } });

  const updateVehicle = (patch: Partial<VehicleInput>) => {
    const v = mission.vehicles[0];
    if (!v) return;
    const vehicles = [...mission.vehicles];
    vehicles[0] = { ...v, ...patch };
    updateMission({ vehicles });
  };
  const updateProfile = (patch: Partial<VehicleInput['profile']>) => {
    const v = mission.vehicles[0];
    if (!v) return;
    updateVehicle({ profile: { ...v.profile, ...patch } });
  };

  const addRadar = useCallback(() => {
    const id = `radar_${Date.now()}`;
    const radar: Radar = {
      id,
      lon: (mission.start.lon + mission.target.lon) / 2,
      lat: (mission.start.lat + mission.target.lat) / 2,
      radar_type: 'tracking',
      radius_km: 50,
      alt_m: 10,
    };
    updateMission({
      red_forces: {
        ...mission.red_forces,
        radars: [...mission.red_forces.radars, radar],
      },
    });
  }, [mission]);

  const updateRadar = (id: string, patch: Partial<Radar>) =>
    updateMission({
      red_forces: {
        ...mission.red_forces,
        radars: mission.red_forces.radars.map((r) =>
          r.id === id ? { ...r, ...patch } : r,
        ),
      },
    });
  const removeRadar = (id: string) =>
    updateMission({
      red_forces: {
        ...mission.red_forces,
        radars: mission.red_forces.radars.filter((r) => r.id !== id),
      },
    });

  const addZone = useCallback(() => {
    const id = `zone_${Date.now()}`;
    const zone: Zone = {
      id,
      zone_type: 'no_fly',
      shape: 'circle',
      geometry: {
        center: [
          (mission.start.lon + mission.target.lon) / 2,
          (mission.start.lat + mission.target.lat) / 2,
        ],
        radius_km: 20,
      },
      alt_min_m: 0,
      alt_max_m: 12000,
      height_semantics: 'msl',
    };
    updateMission({ no_fly_zones: [...mission.no_fly_zones, zone] });
  }, [mission]);

  const updateZone = (id: string, patch: Partial<Zone>) =>
    updateMission({
      no_fly_zones: mission.no_fly_zones.map((z) =>
        z.id === id ? { ...z, ...patch } : z,
      ),
    });
  const removeZone = (id: string) =>
    updateMission({
      no_fly_zones: mission.no_fly_zones.filter((z) => z.id !== id),
    });

  const v = mission.vehicles[0];

  return (
    <div>
      <h2>AircraftRouterPlanner Demo</h2>
      <div className="subtitle">开发期工具 · schema 0.20</div>

      {/* Click mode toggle */}
      <h3>场景操作（点击地图）</h3>
      <div className="mode-buttons">
        <button
          className={activeClickMode === 'start' ? 'active' : ''}
          onClick={() =>
            onSetClickMode(activeClickMode === 'start' ? null : 'start')
          }
        >
          ✈ 起点
        </button>
        <button
          className={activeClickMode === 'target' ? 'active' : ''}
          onClick={() =>
            onSetClickMode(activeClickMode === 'target' ? null : 'target')
          }
        >
          🎯 终点
        </button>
      </div>

      {/* Start */}
      <h3>起点（经纬高）</h3>
      <div className="field-row">
        <div>
          <label>经度</label>
          <input
            type="number"
            step="0.0001"
            value={mission.start.lon}
            onChange={(e) => updateStart({ lon: +e.target.value })}
          />
        </div>
        <div>
          <label>纬度</label>
          <input
            type="number"
            step="0.0001"
            value={mission.start.lat}
            onChange={(e) => updateStart({ lat: +e.target.value })}
          />
        </div>
        <div>
          <label>高度 (m)</label>
          <input
            type="number"
            value={mission.start.alt_m}
            onChange={(e) => updateStart({ alt_m: +e.target.value })}
          />
        </div>
      </div>

      {/* Target */}
      <h3>目标点（经纬高）</h3>
      <div className="field-row">
        <div>
          <label>经度</label>
          <input
            type="number"
            step="0.0001"
            value={mission.target.lon}
            onChange={(e) => updateTarget({ lon: +e.target.value })}
          />
        </div>
        <div>
          <label>纬度</label>
          <input
            type="number"
            step="0.0001"
            value={mission.target.lat}
            onChange={(e) => updateTarget({ lat: +e.target.value })}
          />
        </div>
        <div>
          <label>高度 (m)</label>
          <input
            type="number"
            value={mission.target.alt_m}
            onChange={(e) => updateTarget({ alt_m: +e.target.value })}
          />
        </div>
      </div>

      {/* Vehicle */}
      <h3>飞行器（vehicles[0]）</h3>
      {v ? (
        <>
          <div className="field-row">
            <div>
              <label>机型</label>
              <select
                value={v.profile.aircraft_type}
                onChange={(e) =>
                  updateProfile({
                    aircraft_type: e.target.value as 'FIXED_WING' | 'ROTORCRAFT',
                  })
                }
              >
                <option value="FIXED_WING">固定翼</option>
                <option value="ROTORCRAFT">旋翼机</option>
              </select>
            </div>
            <div>
              <label>巡航速度 (m/s)</label>
              <input
                type="number"
                value={v.profile.cruise_speed_mps ?? 250}
                onChange={(e) =>
                  updateProfile({ cruise_speed_mps: +e.target.value })
                }
              />
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>最小转弯半径 (m)</label>
              <input
                type="number"
                value={v.profile.min_turn_radius_m ?? 442}
                onChange={(e) =>
                  updateProfile({ min_turn_radius_m: +e.target.value })
                }
              />
            </div>
            <div>
              <label>最大爬升角 (°)</label>
              <input
                type="number"
                value={v.profile.max_climb_angle_deg ?? 15}
                onChange={(e) =>
                  updateProfile({ max_climb_angle_deg: +e.target.value })
                }
              />
            </div>
          </div>
          <div>
            <label>必经点（mid_waypoints）</label>
            {(v.mid_waypoints ?? []).map((m, i) => (
              <div key={i} className="field-row" style={{ marginTop: 4 }}>
                <input
                  type="number"
                  step="0.0001"
                  value={m.lon}
                  onChange={(e) => {
                    const ms = [...(v.mid_waypoints ?? [])];
                    ms[i] = { ...ms[i], lon: +e.target.value };
                    updateVehicle({ mid_waypoints: ms });
                  }}
                  placeholder="lon"
                />
                <input
                  type="number"
                  step="0.0001"
                  value={m.lat}
                  onChange={(e) => {
                    const ms = [...(v.mid_waypoints ?? [])];
                    ms[i] = { ...ms[i], lat: +e.target.value };
                    updateVehicle({ mid_waypoints: ms });
                  }}
                  placeholder="lat"
                />
                <button
                  className="btn-small btn-danger"
                  onClick={() =>
                    updateVehicle({
                      mid_waypoints: (v.mid_waypoints ?? []).filter(
                        (_, j) => j !== i,
                      ),
                    })
                  }
                >
                  ✕
                </button>
              </div>
            ))}
            <button
              className="btn-small"
              style={{ marginTop: 4, background: '#333', color: '#e0e0e0' }}
              onClick={() =>
                updateVehicle({
                  mid_waypoints: [
                    ...(v.mid_waypoints ?? []),
                    {
                      lon: (mission.start.lon + mission.target.lon) / 2,
                      lat: (mission.start.lat + mission.target.lat) / 2,
                      alt_m: mission.start.alt_m,
                    },
                  ],
                })
              }
            >
              + 添加必经点
            </button>
          </div>
        </>
      ) : (
        <div>无飞行器定义</div>
      )}

      {/* Terrain */}
      <h3>地形</h3>
      <div className="field-row">
        <div>
          <label>数据源</label>
          <select
            value={mission.terrain.source}
            onChange={(e) =>
              updateMission({
                terrain: {
                  ...mission.terrain,
                  source: e.target.value as 'none' | 'builtin' | 'path',
                },
              })
            }
          >
            <option value="none">无（海拔 0 平面）</option>
            <option value="builtin">内置数据包</option>
            <option value="path">外部文件</option>
          </select>
        </div>
        {mission.terrain.source === 'path' && (
          <div>
            <label>路径</label>
            <input
              type="text"
              value={mission.terrain.path ?? ''}
              onChange={(e) =>
                updateMission({
                  terrain: { ...mission.terrain, path: e.target.value },
                })
              }
            />
          </div>
        )}
      </div>

      {/* Radars */}
      <h3>雷达 ({mission.red_forces.radars.length})</h3>
      {mission.red_forces.radars.map((r) => (
        <div key={r.id} className="obstacle-item">
          <div className="obstacle-header">
            <span>{r.id}</span>
            <button
              className="btn-small btn-danger"
              onClick={() => removeRadar(r.id)}
            >
              ✕
            </button>
          </div>
          <div className="field-row">
            <div>
              <label>经度</label>
              <input
                type="number"
                step="0.0001"
                value={r.lon}
                onChange={(e) => updateRadar(r.id, { lon: +e.target.value })}
              />
            </div>
            <div>
              <label>纬度</label>
              <input
                type="number"
                step="0.0001"
                value={r.lat}
                onChange={(e) => updateRadar(r.id, { lat: +e.target.value })}
              />
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>半径 (km)</label>
              <input
                type="number"
                value={r.radius_km}
                onChange={(e) =>
                  updateRadar(r.id, { radius_km: +e.target.value })
                }
              />
            </div>
            <div>
              <label>类型</label>
              <select
                value={r.radar_type}
                onChange={(e) =>
                  updateRadar(r.id, {
                    radar_type: e.target.value as Radar['radar_type'],
                  })
                }
              >
                <option value="early_warning">预警</option>
                <option value="tracking">跟踪</option>
                <option value="fire_control">火控</option>
              </select>
            </div>
          </div>
        </div>
      ))}
      <button
        className="btn-small"
        onClick={addRadar}
        style={{ width: '100%', background: '#333', color: '#e0e0e0' }}
      >
        + 添加雷达
      </button>

      {/* Zones */}
      <h3>禁飞区 ({mission.no_fly_zones.length})</h3>
      {mission.no_fly_zones.map((z) => (
        <div key={z.id} className="obstacle-item">
          <div className="obstacle-header">
            <span>{z.id}</span>
            <button
              className="btn-small btn-danger"
              onClick={() => removeZone(z.id)}
            >
              ✕
            </button>
          </div>
          {z.shape === 'circle' ? (
            <>
              <div className="field-row">
                <div>
                  <label>经度</label>
                  <input
                    type="number"
                    step="0.0001"
                    value={(z.geometry as { center: [number, number] }).center[0]}
                    onChange={(e) =>
                      updateZone(z.id, {
                        geometry: {
                          ...(z.geometry as { center: [number, number]; radius_km: number }),
                          center: [
                            +e.target.value,
                            (z.geometry as { center: [number, number] }).center[1],
                          ],
                        },
                      })
                    }
                  />
                </div>
                <div>
                  <label>纬度</label>
                  <input
                    type="number"
                    step="0.0001"
                    value={(z.geometry as { center: [number, number] }).center[1]}
                    onChange={(e) =>
                      updateZone(z.id, {
                        geometry: {
                          ...(z.geometry as { center: [number, number]; radius_km: number }),
                          center: [
                            (z.geometry as { center: [number, number] }).center[0],
                            +e.target.value,
                          ],
                        },
                      })
                    }
                  />
                </div>
                <div>
                  <label>半径 (km)</label>
                  <input
                    type="number"
                    value={(z.geometry as { radius_km: number }).radius_km}
                    onChange={(e) =>
                      updateZone(z.id, {
                        geometry: {
                          ...(z.geometry as { center: [number, number]; radius_km: number }),
                          radius_km: +e.target.value,
                        },
                      })
                    }
                  />
                </div>
              </div>
              <div className="field-row">
                <div>
                  <label>最低高 (m)</label>
                  <input
                    type="number"
                    value={z.alt_min_m}
                    onChange={(e) =>
                      updateZone(z.id, { alt_min_m: +e.target.value })
                    }
                  />
                </div>
                <div>
                  <label>最高高 (m)</label>
                  <input
                    type="number"
                    value={z.alt_max_m}
                    onChange={(e) =>
                      updateZone(z.id, { alt_max_m: +e.target.value })
                    }
                  />
                </div>
                <div>
                  <label>类型</label>
                  <select
                    value={z.zone_type}
                    onChange={(e) =>
                      updateZone(z.id, {
                        zone_type: e.target.value as Zone['zone_type'],
                      })
                    }
                  >
                    <option value="no_fly">禁飞</option>
                    <option value="restricted">限飞</option>
                    <option value="obstacle">障碍</option>
                  </select>
                </div>
              </div>
            </>
          ) : (
            <div>多边形区（仅预览，编辑后置）</div>
          )}
        </div>
      ))}
      <button
        className="btn-small"
        onClick={addZone}
        style={{ width: '100%', background: '#333', color: '#e0e0e0' }}
      >
        + 添加禁飞区
      </button>

      {/* Params */}
      <h3>参数覆盖</h3>
      <div className="field-row">
        <div>
          <label>P_cross</label>
          <input
            type="number"
            step="0.01"
            value={mission.parameters.p_cross ?? 0.1}
            onChange={(e) =>
              updateMission({
                parameters: {
                  ...mission.parameters,
                  p_cross: +e.target.value,
                },
              })
            }
          />
        </div>
        <div>
          <label>探测曲线</label>
          <select
            value={mission.parameters.detection_curve ?? 'exponential'}
            onChange={(e) =>
              updateMission({
                parameters: {
                  ...mission.parameters,
                  detection_curve: e.target.value,
                },
              })
            }
          >
            <option value="linear">线性</option>
            <option value="exponential">指数</option>
          </select>
        </div>
      </div>

      {/* Plan button */}
      <button
        onClick={onPlan}
        disabled={loading}
        style={{
          marginTop: 16,
          padding: '14px',
          fontSize: 16,
          width: '100%',
        }}
      >
        {loading ? '⏳ 计算中...' : '开始规划'}
      </button>

      {/* Results */}
      <ResultPanel result={result} />
    </div>
  );
}

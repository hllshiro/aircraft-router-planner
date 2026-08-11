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
  activeClickMode: 'start' | 'target' | 'midpoint' | 'polygon' | null;
  onSetClickMode: (mode: 'start' | 'target' | 'midpoint' | 'polygon' | null) => void;
  editingZoneId: string | null;
  onEditingZoneId: (id: string | null) => void;
  /** 每机拾取目标下标（点击设置起/终点，2026-08-10） */
  pickVehicleIdx: number | null;
  onPickVehicle: (idx: number | null) => void;
}

/** 每机当前目标（自定义 target_ref 或 mission.target） */
function vehicleTarget(v: VehicleInput, missionTarget: Waypoint): Waypoint {
  const r = (v.target_ref ?? '').trim();
  if (r && r !== 'mission.target') {
    const parts = r.split(',').map((s) => s.trim());
    const lon = +parts[0];
    const lat = +parts[1];
    if (Number.isFinite(lon) && Number.isFinite(lat)) {
      const alt =
        parts.length >= 3 && Number.isFinite(+parts[2])
          ? +parts[2]
          : missionTarget.alt_m;
      return { lon, lat, alt_m: alt };
    }
  }
  return { ...missionTarget };
}

export function ControlPanel({
  config,
  onConfigChange,
  onPlan,
  result,
  loading,
  activeClickMode,
  onSetClickMode,
  editingZoneId,
  onEditingZoneId,
  pickVehicleIdx,
  onPickVehicle,
}: ControlPanelProps) {
  const mission = config.mission;
  const update = (patch: Partial<InputConfig>) =>
    onConfigChange({ ...config, ...patch });
  const updateMission = (patch: Partial<InputConfig['mission']>) =>
    update({ mission: { ...mission, ...patch } });

  const updateVehicleAt = (idx: number, patch: Partial<VehicleInput>) => {
    const vehicles = [...mission.vehicles];
    vehicles[idx] = { ...vehicles[idx], ...patch };
    updateMission({ vehicles });
  };
  const updateProfileAt = (idx: number, patch: Partial<VehicleInput['profile']>) => {
    const vehicles = [...mission.vehicles];
    vehicles[idx] = {
      ...vehicles[idx],
      profile: { ...vehicles[idx].profile, ...patch },
    };
    updateMission({ vehicles });
  };

  // 添加飞机：id 递增（v2/v3…），起点默认 = mission.start，机型默认固定翼
  const addVehicle = () => {
    const n = mission.vehicles.length + 1;
    const existing = new Set(mission.vehicles.map((x) => x.id));
    const id = existing.has(`v${n}`) ? `v${n}_${Date.now() % 10000}` : `v${n}`;
    const vehicle: VehicleInput = {
      id,
      profile: {
        aircraft_type: 'FIXED_WING',
        cruise_speed_mps: 250,
        min_turn_radius_m: 442,
        max_climb_angle_deg: 15,
      },
      start_pose: {
        lon: mission.start.lon,
        lat: mission.start.lat,
        alt_m: mission.start.alt_m,
        heading_deg: 45,
      },
      mid_waypoints: [],
    };
    updateMission({ vehicles: [...mission.vehicles, vehicle] });
  };
  // 删除飞机：至少保留 1 架
  const removeVehicleAt = (idx: number) => {
    if (mission.vehicles.length <= 1) return;
    updateMission({
      vehicles: mission.vehicles.filter((_, i) => i !== idx),
    });
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

  // 限飞区（restricted_zones 独立数组，schema 0.20）
  const addRestrictedZone = useCallback(() => {
    const id = `rz_${Date.now()}`;
    const zone: Zone = {
      id,
      zone_type: 'restricted',
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
    updateMission({
      restricted_zones: [...mission.restricted_zones, zone],
    });
  }, [mission]);

  const updateRestrictedZone = (id: string, patch: Partial<Zone>) =>
    updateMission({
      restricted_zones: mission.restricted_zones.map((z) =>
        z.id === id ? { ...z, ...patch } : z,
      ),
    });
  const removeRestrictedZone = (id: string) =>
    updateMission({
      restricted_zones: mission.restricted_zones.filter((z) => z.id !== id),
    });

  return (
    <div>
      <h2>AircraftRouterPlanner Demo</h2>
      <div className="subtitle">开发期工具 · schema 0.20</div>

      {/* Vehicles（多机：schema 0.20 vehicles 数组，每机独立机型/起点位姿/目标/必经点）。
          全局起终点输入已删除（2026-08-10）：每机独立设置起终点后不再需要；
          mission.start/target 保留在内部状态（默认值），仅作契约兜底。 */}
      <h3>飞行器 ({mission.vehicles.length})</h3>
      {mission.vehicles.map((v, idx) => (
        <div key={v.id} className="obstacle-item">
          <div className="obstacle-header">
            <span>ID</span>
            <input
              type="text"
              value={v.id}
              style={{ width: 90 }}
              onChange={(e) => updateVehicleAt(idx, { id: e.target.value })}
            />
            <button
              className="btn-small btn-danger"
              disabled={mission.vehicles.length <= 1}
              title={mission.vehicles.length <= 1 ? '至少保留 1 架飞机' : '删除飞机'}
              onClick={() => removeVehicleAt(idx)}
            >
              ✕
            </button>
          </div>
          <div className="field-row">
            <div>
              <label>机型</label>
              <select
                value={v.profile.aircraft_type}
                onChange={(e) =>
                  updateProfileAt(idx, {
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
                  updateProfileAt(idx, { cruise_speed_mps: +e.target.value })
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
                  updateProfileAt(idx, { min_turn_radius_m: +e.target.value })
                }
              />
            </div>
            <div>
              <label>最大爬升角 (°)</label>
              <input
                type="number"
                value={v.profile.max_climb_angle_deg ?? 15}
                onChange={(e) =>
                  updateProfileAt(idx, { max_climb_angle_deg: +e.target.value })
                }
              />
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>起点经度</label>
              <input
                type="number"
                step="0.0001"
                value={v.start_pose.lon}
                onChange={(e) =>
                  updateVehicleAt(idx, {
                    start_pose: { ...v.start_pose, lon: +e.target.value },
                  })
                }
              />
            </div>
            <div>
              <label>起点纬度</label>
              <input
                type="number"
                step="0.0001"
                value={v.start_pose.lat}
                onChange={(e) =>
                  updateVehicleAt(idx, {
                    start_pose: { ...v.start_pose, lat: +e.target.value },
                  })
                }
              />
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>起点高度 (m)</label>
              <input
                type="number"
                value={v.start_pose.alt_m}
                onChange={(e) =>
                  updateVehicleAt(idx, {
                    start_pose: { ...v.start_pose, alt_m: +e.target.value },
                  })
                }
              />
            </div>
            <div>
              <label>航向 (°)</label>
              <input
                type="number"
                value={v.start_pose.heading_deg ?? 45}
                onChange={(e) =>
                  updateVehicleAt(idx, {
                    start_pose: { ...v.start_pose, heading_deg: +e.target.value },
                  })
                }
              />
            </div>
          </div>
          {/* 每机独立目标（2026-08-10：target_ref 自定义坐标；改输入即写 target_ref 覆盖全局，删恢复按钮 2026-08-11） */}
          <div className="field-row">
            <div>
              <label>目标经度</label>
              <input
                type="number"
                step="0.0001"
                value={vehicleTarget(v, mission.target).lon}
                onChange={(e) =>
                  updateVehicleAt(idx, {
                    target_ref: `${+e.target.value},${vehicleTarget(v, mission.target).lat},${vehicleTarget(v, mission.target).alt_m}`,
                  })
                }
              />
            </div>
            <div>
              <label>目标纬度</label>
              <input
                type="number"
                step="0.0001"
                value={vehicleTarget(v, mission.target).lat}
                onChange={(e) =>
                  updateVehicleAt(idx, {
                    target_ref: `${vehicleTarget(v, mission.target).lon},${+e.target.value},${vehicleTarget(v, mission.target).alt_m}`,
                  })
                }
              />
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>目标高度 (m)</label>
              <input
                type="number"
                value={vehicleTarget(v, mission.target).alt_m}
                onChange={(e) =>
                  updateVehicleAt(idx, {
                    target_ref: `${vehicleTarget(v, mission.target).lon},${vehicleTarget(v, mission.target).lat},${+e.target.value}`,
                  })
                }
              />
            </div>
          </div>
          {/* 每机独立起终点场景拾取（原全局按钮已转移至此） */}
          <div className="mode-buttons">
            <button
              className={
                pickVehicleIdx === idx && activeClickMode === 'start'
                  ? 'active'
                  : ''
              }
              onClick={() => {
                if (pickVehicleIdx === idx && activeClickMode === 'start') {
                  onSetClickMode(null);
                  onPickVehicle(null);
                } else {
                  onSetClickMode('start');
                  onPickVehicle(idx);
                }
              }}
            >
              🗺 点击设置起点
            </button>
            <button
              className={
                pickVehicleIdx === idx && activeClickMode === 'target'
                  ? 'active'
                  : ''
              }
              onClick={() => {
                if (pickVehicleIdx === idx && activeClickMode === 'target') {
                  onSetClickMode(null);
                  onPickVehicle(null);
                } else {
                  onSetClickMode('target');
                  onPickVehicle(idx);
                }
              }}
            >
              🎯 点击设置终点
            </button>
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
                    updateVehicleAt(idx, { mid_waypoints: ms });
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
                    updateVehicleAt(idx, { mid_waypoints: ms });
                  }}
                  placeholder="lat"
                />
                <button
                  className="btn-small btn-danger"
                  onClick={() =>
                    updateVehicleAt(idx, {
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
              className={
                pickVehicleIdx === idx && activeClickMode === 'midpoint'
                  ? 'btn-small active'
                  : 'btn-small'
              }
              style={{
                marginTop: 4,
                background:
                  pickVehicleIdx === idx && activeClickMode === 'midpoint'
                    ? '#3a5f0b'
                    : '#333',
                color: '#e0e0e0',
              }}
              title="点击后在地图上点选添加必经点；场景中的黄色小球可直接拖动调整位置"
              onClick={() => {
                if (pickVehicleIdx === idx && activeClickMode === 'midpoint') {
                  onSetClickMode(null);
                  onPickVehicle(null);
                } else {
                  onSetClickMode('midpoint');
                  onPickVehicle(idx);
                }
              }}
            >
              🖱 点击场景添加必经点
            </button>
          </div>
        </div>
      ))}
      <button
        className="btn-small"
        onClick={addVehicle}
        style={{ width: '100%', background: '#333', color: '#e0e0e0' }}
      >
        + 添加飞机
      </button>

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
      <h3>禁飞区·no_fly ({mission.no_fly_zones.length})</h3>
      {mission.no_fly_zones.map((z) => (
        <div key={z.id} className="obstacle-item">
          <div className="obstacle-header">
            <span>{z.id}</span>
            <select
              value={z.shape}
              onChange={(e) => {
                const shape = e.target.value as 'circle' | 'polygon';
                if (shape === 'circle') {
                  updateZone(z.id, {
                    shape: 'circle',
                    geometry: { center: [z.zone_type === 'no_fly' ? (mission.start.lon + mission.target.lon) / 2 : (z.geometry as { center?: [number, number] }).center?.[0] ?? (mission.start.lon + mission.target.lon) / 2, (z.geometry as { center?: [number, number] }).center?.[1] ?? (mission.start.lat + mission.target.lat) / 2], radius_km: 20 },
                  });
                } else {
                  updateZone(z.id, {
                    shape: 'polygon',
                    geometry: { vertices: (z.geometry as { vertices?: [number, number][] }).vertices ?? [] },
                  });
                }
              }}
            >
              <option value="circle">圆形</option>
              <option value="polygon">多边形</option>
            </select>
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
            </>
          ) : (
            <>
              <div className="polygon-edit">
                <div className="list-title">顶点（{ (z.geometry as { vertices: [number, number][] }).vertices.length }）</div>
                {(z.geometry as { vertices: [number, number][] }).vertices.map((v, i) => (
                  <div key={i} className="field-row" style={{ marginTop: 4 }}>
                    <input
                      type="number"
                      step="0.0001"
                      value={v[0]}
                      onChange={(e) => {
                        const vs = [...(z.geometry as { vertices: [number, number][] }).vertices];
                        vs[i] = [+e.target.value, vs[i][1]];
                        updateZone(z.id, { geometry: { vertices: vs } });
                      }}
                    />
                    <input
                      type="number"
                      step="0.0001"
                      value={v[1]}
                      onChange={(e) => {
                        const vs = [...(z.geometry as { vertices: [number, number][] }).vertices];
                        vs[i] = [vs[i][0], +e.target.value];
                        updateZone(z.id, { geometry: { vertices: vs } });
                      }}
                    />
                    <button
                      className="btn-small btn-danger"
                      onClick={() =>
                        updateZone(z.id, {
                          geometry: {
                            vertices: (z.geometry as { vertices: [number, number][] }).vertices.filter(
                              (_, j) => j !== i,
                            ),
                          },
                        })
                      }
                    >
                      ✕
                    </button>
                  </div>
                ))}
                <button
                  className={`btn-small ${editingZoneId === z.id && activeClickMode === 'polygon' ? 'active' : ''}`}
                  style={{ marginTop: 4 }}
                  onClick={() => {
                    if (editingZoneId === z.id && activeClickMode === 'polygon') {
                      onEditingZoneId(null);
                      onSetClickMode(null);
                    } else {
                      onEditingZoneId(z.id);
                      onSetClickMode('polygon');
                    }
                  }}
                >
                  {editingZoneId === z.id && activeClickMode === 'polygon'
                    ? '✓ 完成拾取'
                    : '🗺 在场景拾取顶点'}
                </button>
                {(z.geometry as { vertices: [number, number][] }).vertices.length < 3 && (
                  <div style={{ color: '#ffaa44', fontSize: 10, marginTop: 2 }}>
                    至少 3 个顶点（场景点击地面添加）
                  </div>
                )}
              </div>
            </>
          )}
          <div className="field-row" style={{ marginTop: 4 }}>
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
        </div>
      ))}
      <button
        className="btn-small"
        onClick={addZone}
        style={{ width: '100%', background: '#333', color: '#e0e0e0' }}
      >
        + 添加禁飞区
      </button>

      {/* Restricted zones */}
      <h3>限飞区 ({mission.restricted_zones.length})</h3>
      {mission.restricted_zones.map((z) => {
        const g = z.geometry as { center?: [number, number]; radius_km?: number; vertices?: [number, number][] };
        return (
          <div key={z.id} className="obstacle-item">
            <div className="obstacle-header">
              <span>{z.id}</span>
              <select
                value={z.shape}
                onChange={(e) => {
                  const shape = e.target.value as 'circle' | 'polygon';
                  if (shape === 'circle') {
                    updateRestrictedZone(z.id, {
                      shape: 'circle',
                      geometry: {
                        center: g.center ?? [
                          (mission.start.lon + mission.target.lon) / 2,
                          (mission.start.lat + mission.target.lat) / 2,
                        ],
                        radius_km: g.radius_km ?? 20,
                      },
                    });
                  } else {
                    updateRestrictedZone(z.id, {
                      shape: 'polygon',
                      geometry: { vertices: g.vertices ?? [] },
                    });
                  }
                }}
              >
                <option value="circle">圆形</option>
                <option value="polygon">多边形</option>
              </select>
              <button
                className="btn-small btn-danger"
                onClick={() => removeRestrictedZone(z.id)}
              >
                ✕
              </button>
            </div>
            {z.shape === 'circle' ? (
              <div className="field-row">
                <div>
                  <label>经度</label>
                  <input
                    type="number"
                    step="0.0001"
                    value={g.center?.[0] ?? 0}
                    onChange={(e) =>
                      updateRestrictedZone(z.id, {
                        geometry: {
                          ...(z.geometry as { center: [number, number]; radius_km: number }),
                          center: [+e.target.value, g.center?.[1] ?? 0],
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
                    value={g.center?.[1] ?? 0}
                    onChange={(e) =>
                      updateRestrictedZone(z.id, {
                        geometry: {
                          ...(z.geometry as { center: [number, number]; radius_km: number }),
                          center: [g.center?.[0] ?? 0, +e.target.value],
                        },
                      })
                    }
                  />
                </div>
                <div>
                  <label>半径 (km)</label>
                  <input
                    type="number"
                    value={g.radius_km ?? 0}
                    onChange={(e) =>
                      updateRestrictedZone(z.id, {
                        geometry: {
                          ...(z.geometry as { center: [number, number]; radius_km: number }),
                          radius_km: +e.target.value,
                        },
                      })
                    }
                  />
                </div>
              </div>
            ) : (
              <div className="polygon-edit">
                <div className="list-title">顶点（{g.vertices?.length ?? 0}）</div>
                {(g.vertices ?? []).map((v, i) => (
                  <div key={i} className="field-row" style={{ marginTop: 4 }}>
                    <input
                      type="number"
                      step="0.0001"
                      value={v[0]}
                      onChange={(e) => {
                        const vs = [...(g.vertices ?? [])];
                        vs[i] = [+e.target.value, vs[i][1]];
                        updateRestrictedZone(z.id, { geometry: { vertices: vs } });
                      }}
                    />
                    <input
                      type="number"
                      step="0.0001"
                      value={v[1]}
                      onChange={(e) => {
                        const vs = [...(g.vertices ?? [])];
                        vs[i] = [vs[i][0], +e.target.value];
                        updateRestrictedZone(z.id, { geometry: { vertices: vs } });
                      }}
                    />
                    <button
                      className="btn-small btn-danger"
                      onClick={() =>
                        updateRestrictedZone(z.id, {
                          geometry: {
                            vertices: (g.vertices ?? []).filter((_, j) => j !== i),
                          },
                        })
                      }
                    >
                      ✕
                    </button>
                  </div>
                ))}
                <div style={{ color: '#88aacc', fontSize: 10, marginTop: 2 }}>
                  顶点用经纬度表格编辑（至少 3 个形成多边形）
                </div>
              </div>
            )}
            <div className="field-row" style={{ marginTop: 4 }}>
              <div>
                <label>最低高 (m)</label>
                <input
                  type="number"
                  value={z.alt_min_m}
                  onChange={(e) =>
                    updateRestrictedZone(z.id, { alt_min_m: +e.target.value })
                  }
                />
              </div>
              <div>
                <label>最高高 (m)</label>
                <input
                  type="number"
                  value={z.alt_max_m}
                  onChange={(e) =>
                    updateRestrictedZone(z.id, { alt_max_m: +e.target.value })
                  }
                />
              </div>
            </div>
          </div>
        );
      })}
      <button
        className="btn-small"
        onClick={addRestrictedZone}
        style={{ width: '100%', background: '#333', color: '#e0e0e0' }}
      >
        + 添加限飞区
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

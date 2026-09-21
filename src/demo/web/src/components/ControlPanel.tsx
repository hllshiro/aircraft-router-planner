import { useCallback, useEffect, useMemo, useRef, useState } from 'react';
import type {
  Input,
  PlanResult,
  Radar,
  Zone,
  AircraftInput,
  WeaponInput,
  WeaponType,
  BaseMapConfig,
  DataFilesResponse,
} from '../types';
import { WEAPON_DEFAULT_RANGE_KM } from '../types';
import { fetchElevation, sanitizePath } from '../api';
import { ResultPanel } from './ResultPanel';

interface ControlPanelProps {
  config: Input;
  onConfigChange: (config: Input) => void;
  onPlan: () => void;
  result: PlanResult | null;
  loading: boolean;
  activeClickMode: 'start' | 'target' | 'midpoint' | 'polygon' | null;
  onSetClickMode: (mode: 'start' | 'target' | 'midpoint' | 'polygon' | null) => void;
  editingZoneId: string | null;
  onEditingZoneId: (id: string | null) => void;
  /** 拾取目标机下标（点击设置起/终点/必经点，2026-08-10） */
  pickAircraftIdx: number | null;
  onPickAircraft: (idx: number | null) => void;
  /** 掩膜文件选择 */
  baseMapConfig: BaseMapConfig;
  onBaseMapConfigChange: (cfg: BaseMapConfig) => void;
  baseMapLoading: boolean;
  baseMapError: string | null;
  /** 数据文件扫描结果（2026-08-20：demo-server 扫描 data/ 供下拉选择） */
  dataFiles?: DataFilesResponse;
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
  pickAircraftIdx,
  onPickAircraft,
  baseMapConfig,
  onBaseMapConfigChange,
  baseMapLoading,
  baseMapError,
  dataFiles,
}: ControlPanelProps) {
  const update = (patch: Partial<Input>) =>
    onConfigChange({ ...config, ...patch });
  // 单机表单（2026-08-19 契约拍平后 demo 保持单机）：所有编辑落到 aircraft[0]
  const aircraft = config.aircraft[0];
  const updateAircraft = (patch: Partial<AircraftInput>) => {
    if (!config.aircraft.length) return;
    update({
      aircraft: config.aircraft.map((a, i) =>
        i === 0 ? { ...a, ...patch } : a,
      ),
    });
  };
  const updateProfile = (patch: Partial<AircraftInput['profile']>) => {
    // aircraft_type 由默认输入保证存在；cast 仅补全 TS 的 Partial 合并类型
    updateAircraft({
      profile: {
        aircraft_type: 'FIXED_WING',
        ...aircraft.profile,
        ...patch,
      } as AircraftInput['profile'],
    });
  };

  // —— 起终点地面海拔（2026-08-14：高度输入框 min = 地面海拔；低于地面自动抬升）——
  // ref 存最新状态/回调，避免 250ms 防抖查询回调用陈旧闭包覆盖用户的并发修改
  const configRef = useRef(config);
  configRef.current = config;
  const onConfigChangeRef = useRef(onConfigChange);
  onConfigChangeRef.current = onConfigChange;
  const [groundAlt, setGroundAlt] = useState<{
    s: number | null;
    t: number | null;
  }>({ s: null, t: null });

  const updateBaseMap = (patch: Partial<BaseMapConfig>) =>
    onBaseMapConfigChange({ ...baseMapConfig, ...patch });

  // 起终点经纬度签名：仅经纬度变化才重新查询海拔（高度变化不触发）
  const startTargetKey = useMemo(() => {
    if (!config.aircraft.length) return 'none';
    const a = config.aircraft[0];
    return (
      `${a.id}|${a.start.lon.toFixed(5)},${a.start.lat.toFixed(5)}|` +
      `${a.target.lon.toFixed(5)},${a.target.lat.toFixed(5)}`
    );
  }, [config]);

  // 经纬度变化（防抖 250ms）→ 查询该点地面海拔 → 设置 min；
  // 当前高度低于地面 → 自动抬升到地面海拔（主管 2026-08-14）
  useEffect(() => {
    const timers: number[] = [];
    const schedule = (
      lon: number,
      lat: number,
      kind: 's' | 't',
    ) => {
      timers.push(
        window.setTimeout(() => {
          const terrainPath = configRef.current.terrain.arpack ?? null;
          if (terrainPath == null) {
            setGroundAlt((prev) => ({ ...prev, [kind]: null }));
            return;
          }
          fetchElevation(terrainPath, lon, lat).then((e) => {
            setGroundAlt((prev) => ({ ...prev, [kind]: e }));
            if (e == null) return;
            const c = configRef.current;
            const a = c.aircraft[0];
            if (!a) return;
            if (kind === 's') {
              if (a.start.alt_m < e) {
                onConfigChangeRef.current({
                  ...c,
                  aircraft: c.aircraft.map((x, i) =>
                    i === 0 ? { ...x, start: { ...x.start, alt_m: e } } : x,
                  ),
                });
              }
            } else if (a.target.alt_m < e) {
              onConfigChangeRef.current({
                ...c,
                aircraft: c.aircraft.map((x, i) =>
                  i === 0 ? { ...x, target: { ...x.target, alt_m: e } } : x,
                ),
              });
            }
          });
        }, 250),
      );
    };
    const a = configRef.current.aircraft[0];
    if (a) {
      schedule(a.start.lon, a.start.lat, 's');
      schedule(a.target.lon, a.target.lat, 't');
    }
    return () => timers.forEach((t) => window.clearTimeout(t));
  }, [startTargetKey]);

  // 武器（2026-08-19 移入飞行器：aircraft[0].weapon；weapon 出现即启用、weapon_type 必填；
  // 类型缺省 = 不启用 → 直接删除 weapon 字段）
  const weapon = aircraft.weapon;
  const setWeapon = (patch: Partial<WeaponInput>) => {
    // 所有调用点都保证已有 weapon_type（类型下拉 / 射程输入均先判 weapon_type）
    const next = { ...weapon, ...patch } as WeaponInput;
    updateAircraft({ weapon: next });
  };
  const clearWeapon = () => updateAircraft({ weapon: undefined });

  const addRadar = useCallback(() => {
    const id = `radar_${Date.now()}`;
    const radar: Radar = {
      id,
      lon: (aircraft.start.lon + aircraft.target.lon) / 2,
      lat: (aircraft.start.lat + aircraft.target.lat) / 2,
      radius_km: 50,
      alt_m: 10,
    };
    update({
      red_forces: {
        ...config.red_forces,
        radars: [...config.red_forces.radars, radar],
      },
    });
  }, [config]);

  const updateRadar = (id: string, patch: Partial<Radar>) =>
    update({
      red_forces: {
        ...config.red_forces,
        radars: config.red_forces.radars.map((r) =>
          r.id === id ? { ...r, ...patch } : r,
        ),
      },
    });
  const removeRadar = (id: string) =>
    update({
      red_forces: {
        ...config.red_forces,
        radars: config.red_forces.radars.filter((r) => r.id !== id),
      },
    });

  const addZone = useCallback(() => {
    const id = `zone_${Date.now()}`;
    const zone: Zone = {
      id,
      shape: 'circle',
      geometry: {
        center: [
          (aircraft.start.lon + aircraft.target.lon) / 2,
          (aircraft.start.lat + aircraft.target.lat) / 2,
        ],
        radius_km: 20,
      },
      // 禁飞区无高度范围（全高度禁入，2026-08-12）
    };
    update({ zones: [...config.zones, zone] });
  }, [config]);

  const updateZone = (id: string, patch: Partial<Zone>) =>
    update({
      zones: config.zones.map((z) =>
        z.id === id ? { ...z, ...patch } : z,
      ),
    });
  const removeZone = (id: string) =>
    update({
      zones: config.zones.filter((z) => z.id !== id),
    });

  // 限飞区
  const addRestrictedZone = useCallback(() => {
    const id = `rz_${Date.now()}`;
    const zone: Zone = {
      id,
      shape: 'circle',
      geometry: {
        center: [
          (aircraft.start.lon + aircraft.target.lon) / 2,
          (aircraft.start.lat + aircraft.target.lat) / 2,
        ],
        radius_km: 20,
      },
      // 限飞区需要高度区间（[alt_min, alt_max] 高度带禁入）
      alt_min_m: 0,
      alt_max_m: 12000,
    };
    update({ zones: [...config.zones, zone] });
  }, [config]);

  const updateRestrictedZone = (id: string, patch: Partial<Zone>) =>
    update({
      zones: config.zones.map((z) =>
        z.id === id ? { ...z, ...patch } : z,
      ),
    });
  const removeRestrictedZone = (id: string) =>
    update({
      zones: config.zones.filter((z) => z.id !== id),
    });

  return (
    <>
      <div className="panel-scroll">
      <h2>AircraftRouterPlanner Demo</h2>
      <div className="subtitle">开发期工具 · schema 0.21</div>

      {/* 飞行器（单机表单：demo 保持单机，编辑 aircraft[0]；逐机显式 start/target/weapon） */}
      <h3>飞行器 ({config.aircraft.length})</h3>
      {config.aircraft.length > 0 && (
        <div className="obstacle-item">
          <div className="obstacle-header">
            <span>ID</span>
            <input
              type="text"
              value={aircraft.id}
              style={{ width: 90 }}
              onChange={(e) => updateAircraft({ id: e.target.value })}
            />
          </div>
          <div className="field-row">
            <div>
              <label>机型</label>
              <select
                value={aircraft.profile?.aircraft_type}
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
              <label>最大速度 (m/s)</label>
              <input
                type="number"
                value={aircraft.profile?.maximum_speed_mps ?? 600}
                onChange={(e) =>
                  updateProfile({ maximum_speed_mps: +e.target.value })
                }
              />
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>最大转弯角速率 (°/s)</label>
              <input
                type="number"
                value={aircraft.profile?.maximum_turn_rate_dps ?? 20}
                onChange={(e) =>
                  updateProfile({ maximum_turn_rate_dps: +e.target.value })
                }
              />
            </div>
            <div>
              <label>最大爬升率 (m/s)</label>
              <input
                type="number"
                value={aircraft.profile?.maximum_climb_rate_mps ?? 250}
                onChange={(e) =>
                  updateProfile({ maximum_climb_rate_mps: +e.target.value })
                }
              />
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>
                <input
                  type="checkbox"
                  checked={aircraft.profile?.cruise_alt_m === 0}
                  onChange={(e) =>
                    updateProfile({
                      cruise_alt_m: e.target.checked ? 0 : undefined,
                    })
                  }
                />
                {' '}贴地飞行
              </label>
              <span className="field-hint" title="关闭三段式巡航高度控制，路径沿地形自然飞行，适用于贴地躲避雷达探测">ⓘ</span>
            </div>
          </div>
          <div className="field-row">
            <div>
              <label>起点经度</label>
              <input
                type="number"
                step="0.0001"
                value={aircraft.start.lon}
                onChange={(e) =>
                  updateAircraft({
                    start: { ...aircraft.start, lon: +e.target.value },
                  })
                }
              />
            </div>
            <div>
              <label>起点纬度</label>
              <input
                type="number"
                step="0.0001"
                value={aircraft.start.lat}
                onChange={(e) =>
                  updateAircraft({
                    start: { ...aircraft.start, lat: +e.target.value },
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
                value={aircraft.start.alt_m}
                min={groundAlt.s ?? undefined}
                title={
                  groundAlt.s != null
                    ? `最小高度 = 该点地面海拔 ${Math.round(groundAlt.s)}m`
                    : '最小高度 = 地面海拔（查询中…）'
                }
                onBlur={() => {
                  const minAlt = groundAlt.s;
                  if (minAlt != null && aircraft.start.alt_m < minAlt) {
                    updateAircraft({
                      start: { ...aircraft.start, alt_m: minAlt },
                    });
                  }
                }}
                onChange={(e) =>
                  updateAircraft({
                    start: { ...aircraft.start, alt_m: +e.target.value },
                  })
                }
              />
            </div>
          </div>
          {/* 目标（2026-08-19：逐机 target 为必填 Waypoint，直接编辑） */}
          <div className="field-row">
            <div>
              <label>目标经度</label>
              <input
                type="number"
                step="0.0001"
                value={aircraft.target.lon}
                onChange={(e) =>
                  updateAircraft({
                    target: { ...aircraft.target, lon: +e.target.value },
                  })
                }
              />
            </div>
            <div>
              <label>目标纬度</label>
              <input
                type="number"
                step="0.0001"
                value={aircraft.target.lat}
                onChange={(e) =>
                  updateAircraft({
                    target: { ...aircraft.target, lat: +e.target.value },
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
                value={aircraft.target.alt_m}
                min={groundAlt.t ?? undefined}
                title={
                  groundAlt.t != null
                    ? `最小高度 = 该点地面海拔 ${Math.round(groundAlt.t)}m`
                    : '最小高度 = 地面海拔（查询中…）'
                }
                onBlur={() => {
                  const minAlt = groundAlt.t;
                  if (minAlt != null && aircraft.target.alt_m < minAlt) {
                    updateAircraft({
                      target: { ...aircraft.target, alt_m: minAlt },
                    });
                  }
                }}
                onChange={(e) =>
                  updateAircraft({
                    target: { ...aircraft.target, alt_m: +e.target.value },
                  })
                }
              />
            </div>
          </div>
          {/* 武器（2026-08-19 移入飞行器：类型 + 射程；类型缺省 = 不启用 → 删除 weapon 字段） */}
          <div className="field-row" style={{ marginTop: 4 }}>
            <div>
              <label>武器类型</label>
              <select
                value={weapon?.weapon_type ?? ''}
                onChange={(e) => {
                  const wt = e.target.value as WeaponType | '';
                  if (!wt) {
                    clearWeapon();
                  } else {
                    // 类型切换 → 射程回落类型默认（range_km 清空，占位显示默认值）
                    setWeapon({ weapon_type: wt, range_km: undefined });
                  }
                }}
              >
                <option value="">不启用</option>
                <option value="aam">空空导弹 (AAM)</option>
                <option value="agm">空地导弹 (AGM)</option>
                <option value="bomb">航空炸弹</option>
              </select>
            </div>
            <div>
              <label>射程 Rmin (km)</label>
              <input
                type="number"
                min={0}
                disabled={!weapon?.weapon_type}
                placeholder={
                  weapon?.weapon_type
                    ? String(WEAPON_DEFAULT_RANGE_KM[weapon.weapon_type][0])
                    : ''
                }
                value={weapon?.range_km?.[0] ?? ''}
                onChange={(e) => {
                  if (!weapon?.weapon_type) return;
                  const lo = +e.target.value;
                  const hi =
                    weapon.range_km?.[1] ??
                    WEAPON_DEFAULT_RANGE_KM[weapon.weapon_type][1];
                  setWeapon({
                    range_km: [
                      Number.isFinite(lo)
                        ? lo
                        : WEAPON_DEFAULT_RANGE_KM[weapon.weapon_type][0],
                      hi,
                    ],
                  });
                }}
              />
            </div>
            <div>
              <label>射程 Rmax (km)</label>
              <input
                type="number"
                min={0}
                disabled={!weapon?.weapon_type}
                placeholder={
                  weapon?.weapon_type
                    ? String(WEAPON_DEFAULT_RANGE_KM[weapon.weapon_type][1])
                    : ''
                }
                value={weapon?.range_km?.[1] ?? ''}
                onChange={(e) => {
                  if (!weapon?.weapon_type) return;
                  const hi = +e.target.value;
                  const lo =
                    weapon.range_km?.[0] ??
                    WEAPON_DEFAULT_RANGE_KM[weapon.weapon_type][0];
                  setWeapon({
                    range_km: [
                      lo,
                      Number.isFinite(hi)
                        ? hi
                        : WEAPON_DEFAULT_RANGE_KM[weapon.weapon_type][1],
                    ],
                  });
                }}
              />
            </div>
          </div>
          {/* 单机起终点/必经点场景拾取（idx 恒为 0） */}
          <div className="mode-buttons">
            <button
              className={
                pickAircraftIdx === 0 && activeClickMode === 'start'
                  ? 'active'
                  : ''
              }
              onClick={() => {
                if (pickAircraftIdx === 0 && activeClickMode === 'start') {
                  onSetClickMode(null);
                  onPickAircraft(null);
                } else {
                  onSetClickMode('start');
                  onPickAircraft(0);
                }
              }}
            >
              🗺 点击设置起点
            </button>
            <button
              className={
                pickAircraftIdx === 0 && activeClickMode === 'target'
                  ? 'active'
                  : ''
              }
              onClick={() => {
                if (pickAircraftIdx === 0 && activeClickMode === 'target') {
                  onSetClickMode(null);
                  onPickAircraft(null);
                } else {
                  onSetClickMode('target');
                  onPickAircraft(0);
                }
              }}
            >
              🎯 点击设置终点
            </button>
          </div>
          <div>
            <label>必经点（mid_waypoints）</label>
            {(aircraft.mid_waypoints ?? []).map((m, i) => (
              <div key={i} className="field-row" style={{ marginTop: 4 }}>
                <input
                  type="number"
                  step="0.0001"
                  value={m.lon}
                  onChange={(e) => {
                    const ms = [...(aircraft.mid_waypoints ?? [])];
                    ms[i] = { ...ms[i], lon: +e.target.value };
                    updateAircraft({ mid_waypoints: ms });
                  }}
                  placeholder="lon"
                />
                <input
                  type="number"
                  step="0.0001"
                  value={m.lat}
                  onChange={(e) => {
                    const ms = [...(aircraft.mid_waypoints ?? [])];
                    ms[i] = { ...ms[i], lat: +e.target.value };
                    updateAircraft({ mid_waypoints: ms });
                  }}
                  placeholder="lat"
                />
                <input
                  type="number"
                  step="1"
                  value={m.alt_m ?? aircraft.start.alt_m}
                  onChange={(e) => {
                    const ms = [...(aircraft.mid_waypoints ?? [])];
                    ms[i] = { ...ms[i], alt_m: +e.target.value };
                    updateAircraft({ mid_waypoints: ms });
                  }}
                  placeholder="alt(m)"
                  title="必经点高度（MSL 米；2026-08-13 P8 M2 起生效——多锚点分段插值）"
                />
                <button
                  className="btn-small btn-danger"
                  onClick={() =>
                    updateAircraft({
                      mid_waypoints: (aircraft.mid_waypoints ?? []).filter(
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
                pickAircraftIdx === 0 && activeClickMode === 'midpoint'
                  ? 'btn-small active'
                  : 'btn-small'
              }
              style={{
                marginTop: 4,
                background:
                  pickAircraftIdx === 0 && activeClickMode === 'midpoint'
                    ? '#3a5f0b'
                    : '#333',
                color: '#e0e0e0',
              }}
              title="点击后在地图上点选添加必经点；场景中的黄色小球可直接拖动调整位置"
              onClick={() => {
                if (pickAircraftIdx === 0 && activeClickMode === 'midpoint') {
                  onSetClickMode(null);
                  onPickAircraft(null);
                } else {
                  onSetClickMode('midpoint');
                  onPickAircraft(0);
                }
              }}
            >
              🖱 点击场景添加必经点
            </button>
          </div>
        </div>
      )}

      {/* 高程数据 —— 场景 3D 显示的地形文件 */}
      <h3>高程</h3>
      <div className="field-row">
        <div>
          <label>高程数据</label>
          <select
            value={config.terrain.arpack ?? ''}
            onChange={(e) =>
              update({
                terrain: {
                  ...config.terrain,
                  arpack: e.target.value || undefined,
                },
              })
            }
          >
            <option value="">无（海拔 0 平面）</option>
            {(dataFiles?.index?.arpacks ?? []).map((f) => (
              <option key={f.id} value={f.id}>
                {f.desc}
              </option>
            ))}
          </select>
        </div>
      </div>

      {/* 掩膜文件 */}
      <h3>掩膜</h3>
      <div className="field-row">
        <div className="wide">
          <label>掩膜文件</label>
          <select
            value={baseMapConfig.path ?? ''}
            onChange={(e) => updateBaseMap({ path: sanitizePath(e.target.value) || undefined })}
          >
            <option value="">无</option>
            {(dataFiles?.index?.masks ?? []).map((f) => (
              <option key={f.id} value={f.id}>
                {f.desc}
              </option>
            ))}
          </select>
        </div>
      </div>
      {(baseMapLoading || baseMapError) && (
        <div className="field-row basemap-panel-status-row">
          {baseMapLoading && (
            <span className="basemap-panel-status loading">⏳ 掩膜加载中…</span>
          )}
          {baseMapError && (
            <span className="basemap-panel-status error">⚠ {baseMapError}</span>
          )}
        </div>
      )}

      {/* Radars */}
      <h3>雷达 ({config.red_forces.radars.length})</h3>
      {config.red_forces.radars.map((r) => (
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
      <h3>禁飞区 ({config.zones.filter(z => z.alt_min_m == null && z.alt_max_m == null).length})</h3>
      {config.zones.filter(z => z.alt_min_m == null && z.alt_max_m == null).map((z) => (
        <div key={z.id} className="obstacle-item">
          <div className="obstacle-header">
            <span>{z.id}</span>
            <select
              value={z.shape}
              onChange={(e) => {
                const shape = e.target.value as 'circle' | 'polygon';
                if (shape === 'circle') {
                  const g = z.geometry as { center?: [number, number] };
                  updateZone(z.id, {
                    shape: 'circle',
                    geometry: {
                      center: [
                        g.center?.[0] ??
                          (aircraft.start.lon + aircraft.target.lon) / 2,
                        g.center?.[1] ??
                          (aircraft.start.lat + aircraft.target.lat) / 2,
                      ],
                      radius_km: 20,
                    },
                  });
                } else {
                  updateZone(z.id, {
                    shape: 'polygon',
                    geometry: {
                      vertices:
                        (z.geometry as { vertices?: [number, number][] })
                          .vertices ?? [],
                    },
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
          {/* 禁飞区无高度范围、无类型选择（全高度禁入，2026-08-12） */}
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
      <h3>限飞区 ({config.zones.filter(z => z.alt_min_m != null || z.alt_max_m != null).length})</h3>
      {config.zones.filter(z => z.alt_min_m != null || z.alt_max_m != null).map((z) => {
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
                          (aircraft.start.lon + aircraft.target.lon) / 2,
                          (aircraft.start.lat + aircraft.target.lat) / 2,
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
                {(g.vertices?.length ?? 0) < 3 && (
                  <div style={{ color: '#ffaa44', fontSize: 10, marginTop: 2 }}>
                    至少 3 个顶点（场景点击地面添加）
                  </div>
                )}
              </div>
            )}
            <div className="field-row" style={{ marginTop: 4 }}>
              <div>
                <label>最低高 (m)</label>
                <input
                  type="number"
                  value={z.alt_min_m ?? 0}
                  onChange={(e) =>
                    updateRestrictedZone(z.id, { alt_min_m: +e.target.value })
                  }
                />
              </div>
              <div>
                <label>最高高 (m)</label>
                <input
                  type="number"
                  value={z.alt_max_m ?? 0}
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
          <label>精度档位</label>
          <select
            value={config.parameters.precision ?? 'balanced'}
            onChange={(e) =>
              update({
                parameters: {
                  ...config.parameters,
                  precision: e.target.value as 'fast' | 'balanced' | 'accurate',
                },
              })
            }
          >
            <option value="fast">快速 (128)</option>
            <option value="balanced">均衡 (256)</option>
            <option value="accurate">精确 (512)</option>
          </select>
        </div>
      </div>

      {/* Results */}
      <ResultPanel result={result} />
      </div>
      <div className="panel-footer">
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
      </div>
    </>
  );
}

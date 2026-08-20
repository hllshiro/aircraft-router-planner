import type { PlanResult } from '../types';

interface ResultPanelProps {
  result: PlanResult | null;
}

export function ResultPanel({ result }: ResultPanelProps) {
  if (!result) return null;

  if (result.status === 'input_invalid') {
    return (
      <div className="result" style={{ color: '#ff6666' }}>
        <strong>❌ 输入无效</strong>
        {result.error && (
          <>
            <div>代码: {result.error.code}</div>
            <div>消息: {result.error.message}</div>
          </>
        )}
      </div>
    );
  }

  if (result.status === 'no_solution') {
    return (
      <div className="result" style={{ color: '#ff6666' }}>
        <strong>❌ 无解</strong>
        {result.error?.message && <div>消息: {result.error.message}</div>}
        <div className="degradation-list">
          {result.stats.degradations.map((d, i) => (
            <div key={i}>⚠ {d}</div>
          ))}
        </div>
      </div>
    );
  }

  const ok = result.status === 'success' || result.status === 'degraded_timeout';
  if (!ok) return null;

  return (
    <div className="result">
      <strong>
        {result.status === 'success' ? '✅ 规划成功' : '⚠️ 降级完成（degraded_timeout）'}
      </strong>
      <div>耗时: {(result.elapsed_ms ?? 0).toFixed(0)} ms</div>
      <div>
        解算: FMM {(result.stats.fmm_ms ?? 0).toFixed(1)} ms / LOS 检查{' '}
        {result.stats.los_checks} 次
      </div>

      {result.aircraft.map((ao) => (
        <div key={ao.id} className="vehicle-result">
          <div className="vehicle-title">
            {ao.id} —{' '}
            {ao.status === 'planned'
              ? '已规划'
              : ao.status === 'degraded'
                ? '降级'
                : '无解'}
          </div>
          <div>路径长度: {(ao.distance_m / 1000).toFixed(2)} km</div>
          <div>路点数量: {ao.path.length}</div>
          {ao.warnings.length > 0 && (
            <div className="degradation-list">
              <div className="list-title">复验警告（warnings）</div>
              {ao.warnings.map((w, i) => (
                <div key={i}>⚠ {w}</div>
              ))}
            </div>
          )}
        </div>
      ))}

      {result.stats.degradations.length > 0 && (
        <div className="degradation-list">
          <div className="list-title">降级/回退记录（degradations）</div>
          {result.stats.degradations.map((d, i) => (
            <div key={i}>⚠ {d}</div>
          ))}
        </div>
      )}
    </div>
  );
}

import type { InputConfig, PlanResult } from './types';

export async function planRoute(config: InputConfig): Promise<PlanResult> {
  const resp = await fetch('/api/plan', {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(config),
  });
  return resp.json();
}

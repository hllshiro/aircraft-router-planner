#!/usr/bin/env bash
# P6-B 性能回归：release 真实地形案例必须在 3s 预算内完成（不得 degraded_timeout）。
# 用法：scripts/perf_regress.sh [input.json] [terrain.arpack]
# 缺省输入 = cli/tests/perf_gate.json（主管 2026-08-12 案例：3 必经点 + 禁飞圆 + 真实地形）
# 跨平台：WSL/git-bash 下用 wslpath 转 Windows 绝对路径（Windows exe 不解析 Linux 路径）。
set -euo pipefail
cd "$(dirname "$0")/.."

INPUT="${1:-cli/tests/perf_gate.json}"
TERRAIN="${2:-data/east_asia_7p5as.arpack}"
BUDGET="${ARP_BUDGET_MS:-3000}"

echo "    input: $INPUT"
echo "    terrain: $TERRAIN"
echo "    budget: ${BUDGET}ms"

TMP=".perf_tmp"
mkdir -p "$TMP"
trap 'rm -rf "$TMP"' EXIT

if command -v wslpath >/dev/null 2>&1; then
  INPUT_ARG="$(wslpath -w "$PWD/$INPUT")"
  TERRAIN_ARG="$(wslpath -w "$PWD/$TERRAIN")"
  OUT_ARG="$(wslpath -w "$PWD/$TMP/out.json")"
else
  INPUT_ARG="$INPUT"
  TERRAIN_ARG="$TERRAIN"
  OUT_ARG="$TMP/out.json"
fi
# stderr 重定向必须用 bash/Linux 视角路径（Windows 路径含 \ 会被 bash 当转义，
# 生成字面反斜杠文件名；2026-08-12 实测教训）
ERR_ARG="$TMP/err.txt"

START=$(date +%s%N)
"./target/release/aircraft-router-planner-cli.exe" -i "$INPUT_ARG" --terrain "$TERRAIN_ARG" -o "$OUT_ARG" 2>"$ERR_ARG" || {
  echo "FAIL: CLI 运行失败（exit $?）" >&2
  cat "$ERR_ARG" >&2
  exit 1
}
END=$(date +%s%N)
ELAPSED_MS=$(( (END - START) / 1000000 ))

STATUS="parse_error"
if command -v python3 >/dev/null 2>&1; then
  STATUS=$(python3 -c "import json;print(json.load(open('$TMP/out.json'))['status'])" 2>/dev/null || echo "parse_error")
elif command -v python >/dev/null 2>&1; then
  STATUS=$(python -c "import json;print(json.load(open('$TMP/out.json'))['status'])" 2>/dev/null || echo "parse_error")
else
  STATUS=$(grep -oE '"status": *"[a-z_]+"' "$TMP/out.json" 2>/dev/null | head -1 | sed 's/.*"\([a-z_]*\)".*/\1/')
fi
if [ "$STATUS" = "degraded_timeout" ]; then
  echo "FAIL: ${ELAPSED_MS}ms 超过 ${BUDGET}ms 预算 → degraded_timeout（性能回归未达标）" >&2
  cat "$ERR_ARG" >&2
  exit 1
fi
if [ "$STATUS" != "success" ]; then
  echo "FAIL: status=${STATUS}（期望 success）" >&2
  cat "$ERR_ARG" >&2
  exit 1
fi

echo "    PASS: success in ${ELAPSED_MS}ms (budget ${BUDGET}ms)"

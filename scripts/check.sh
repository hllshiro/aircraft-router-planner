#!/usr/bin/env bash
# P6-A（docs/11 §8）：Phase 5 全量门禁——构建 + 全套回归 + 静态红线 + 性能预算。
# 用法：scripts/check.sh [--quick]   （--quick 跳过性能回归与交叉编译）
set -euo pipefail
cd "$(dirname "$0")/.."

QUICK=0
[ "${1:-}" = "--quick" ] && QUICK=1

echo "==> [1/5] release 构建（静态编译红线前置）"
cargo build --release -p aircraft-router-planner-cli

echo "==> [2/5] 回归测试（lib / crash / compare / determinism / regress）"
cargo test --lib
cargo test --test crash
cargo test --test compare
cargo test --test determinism
cargo test --test regress

echo "==> [3/5] 静态依赖红线：禁 blas/proj/C 后端（技术方案 3.2.1）"
if cargo tree -e normal | grep -iE "openblas|zlib|curl|proj|gdal|pcre|ssl"; then
  echo "FAIL: C 依赖泄漏" >&2
  exit 1
fi
echo "    PASS: 无 C 依赖"

echo "==> [4/5] 确定性双跑逐字节门禁（tests/determinism.rs 已含；此处显式复跑）"
cargo test --test determinism

if [ "$QUICK" = "1" ]; then
  echo "==> quick 模式：跳过性能回归"
else
  echo "==> [5/5] 性能回归（release 真实地形 ≤3s 预算，不得 degraded_timeout）"
  bash scripts/perf_regress.sh
fi

echo "PASS: 全部门禁通过"

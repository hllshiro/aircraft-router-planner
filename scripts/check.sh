#!/usr/bin/env bash
# P6-A（docs/11 §8）：Phase 5 全量门禁——构建 + 全套回归 + 静态红线 + 性能预算。
# 用法：scripts/check.sh [--quick] [--with-compare]
#   --quick         跳过性能回归与交叉编译
#   --with-compare  额外运行压缩解压性能/正确性测试（field_build_compare，~7min，
#                   真实地形 1024² 冷缓存多轮 zstd 解压对比）——仅在修改/验证
#                   压缩解压相关代码（builtin.rs 解压 / BulkPrefetch / 代价场构建）
#                   时使用；默认全量门禁不含（主管 2026-08-13 决策）。
set -euo pipefail
cd "$(dirname "$0")/.."

QUICK=0
WITH_COMPARE=0
for arg in "$@"; do
  [ "$arg" = "--quick" ] && QUICK=1
  [ "$arg" = "--with-compare" ] && WITH_COMPARE=1
done

echo "==> [1/5] release 构建（静态编译红线前置）"
cargo build --release -p aircraft-router-planner-cli

echo "==> [2/5] 回归测试（lib / crash / determinism / regress）"
cargo test --lib
cargo test --test crash_suite
cargo test --test determinism
cargo test --test regress_phase0
if [ "$WITH_COMPARE" = "1" ]; then
  echo "==> [2b] 压缩解压性能/正确性（field_build_compare，单独门禁）"
  cargo test --test field_build_compare
else
  echo "    （压缩解压性能/正确性 field_build_compare 不在全量门禁：单独运行"
  echo "      cargo test --test field_build_compare 或 scripts/check.sh --with-compare）"
fi

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

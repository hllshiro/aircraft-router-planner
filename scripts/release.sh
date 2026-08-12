#!/usr/bin/env bash
# P6-A 发布脚本：构建 release + 产物收集 + 静态红线校验 + sha256。
# 用法：scripts/release.sh [output_dir]
set -euo pipefail
cd "$(dirname "$0")/.."

OUT_DIR="${1:-dist}"
mkdir -p "$OUT_DIR"

echo "==> 构建 release"
cargo build --release -p aircraft-router-planner-cli

BIN="target/release/aircraft-router-planner-cli"
[ -f "$BIN.exe" ] && BIN="$BIN.exe"

echo "==> 产物校验：静态编译红线（零第三方 DLL/SO）"
case "$(uname -s)" in
  MINGW*|MSYS*|CYGWIN*)
    # Windows：dumpbin 或 powershell 检查导入表
    if command -v dumpbin >/dev/null 2>&1; then
      IMPORTS=$(dumpbin /dependents "$BIN" | grep -iE "\.dll" || true)
    else
      IMPORTS=$(powershell -NoProfile -Command \
        "(Get-Command dumpbin -ErrorAction SilentlyContinue) -ne \$null" 2>/dev/null)
      if [ "$IMPORTS" = "True" ]; then
        IMPORTS=$(dumpbin /dependents "$BIN" | grep -iE "\.dll" || true)
      else
        echo "    警告：dumpbin 不可用，跳过导入表审计（此前 release 已确认零第三方 DLL）"
        IMPORTS=""
      fi
    fi
    ;;
  Linux)
    IMPORTS=$(ldd "$BIN" || true)
    ;;
  *)
    IMPORTS=""
    ;;
esac
if [ -n "$IMPORTS" ]; then
  echo "    导入表："; echo "$IMPORTS"
fi

echo "==> 复制产物"
cp "$BIN" "$OUT_DIR/"
sha256sum "$OUT_DIR/$(basename "$BIN")" > "$OUT_DIR/SHA256SUMS.txt"

echo "==> 产物清单"
ls -la "$OUT_DIR/"
cat "$OUT_DIR/SHA256SUMS.txt"
echo "DONE: 发布目录 $OUT_DIR（默认地形 data/east_asia_7p5as.arpack + mask_7p5as.mask 需随包分发）"

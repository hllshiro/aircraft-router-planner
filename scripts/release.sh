#!/usr/bin/env bash
# P6-A 发布脚本：构建 release + 产物收集 + 静态红线校验 + sha256。
# 用法：scripts/release.sh [选项] [output_dir]
#
# 选项：
#   --no-demo    跳过 demo-server 构建（仅 CLI 正式发布）
#   --no-web     跳过 web 前端构建（假设 web-dist 已就绪）
#
# 默认输出目录：install/
set -euo pipefail
cd "$(dirname "$0")/.."

SKIP_DEMO=0
SKIP_WEB=0
OUT_DIR="install"

for arg in "$@"; do
  case "$arg" in
    --no-demo) SKIP_DEMO=1 ;;
    --no-web)  SKIP_WEB=1 ;;
    -*)        echo "未知选项: $arg" >&2; exit 1 ;;
    *)         OUT_DIR="$arg" ;;
  esac
done

mkdir -p "$OUT_DIR"

# ── 辅助函数 ──────────────────────────────────────────────────────

audit_binary() {
  local bin="$1"
  echo "==> 静态编译红线校验：$(basename "$bin")"
  case "$(uname -s)" in
    MINGW*|MSYS*|CYGWIN*)
      if command -v dumpbin >/dev/null 2>&1; then
        local imports
        imports=$(dumpbin /dependents "$bin" | grep -iE "\.dll" || true)
        if [ -n "$imports" ]; then
          echo "    导入表："; echo "$imports"
        fi
      else
        echo "    警告：dumpbin 不可用，跳过导入表审计"
      fi
      ;;
    Linux)
      local imports
      imports=$(ldd "$bin" || true)
      if [ -n "$imports" ]; then
        echo "    导入表："; echo "$imports"
      fi
      ;;
  esac
}

# ── 1. 构建 CLI ──────────────────────────────────────────────────

echo "==> [1/4] 构建 CLI (release)"
cargo build --release -p aircraft-router-planner-cli

CLI_BIN="target/release/aircraft-router-planner-cli"
[ -f "$CLI_BIN.exe" ] && CLI_BIN="$CLI_BIN.exe"

audit_binary "$CLI_BIN"

echo "==> 复制 CLI"
cp "$CLI_BIN" "$OUT_DIR/"

# ── 2. 构建 demo-server ──────────────────────────────────────────

if [ "$SKIP_DEMO" -eq 0 ]; then
  echo "==> [2/4] 构建 demo-server (release)"
  cargo build --release -p demo-server

  SERVER_BIN="target/release/demo-server"
  [ -f "$SERVER_BIN.exe" ] && SERVER_BIN="$SERVER_BIN.exe"

  audit_binary "$SERVER_BIN"

  echo "==> 复制 demo-server"
  cp "$SERVER_BIN" "$OUT_DIR/"
else
  echo "==> [2/4] 跳过 demo-server (--no-demo)"
fi

# ── 3. 构建 web 前端 ─────────────────────────────────────────────

if [ "$SKIP_WEB" -eq 0 ]; then
  echo "==> [3/4] 构建 web 前端"
  if ! command -v pnpm >/dev/null 2>&1; then
    echo "    警告：npm 不可用，跳过 web 构建（使用 --no-web 可消除此警告）"
  elif [ ! -d "demo/web" ]; then
    echo "    警告：demo/web 不存在，跳过 web 构建"
  else
    (cd demo/web && pnpm i && pnpm run build)
    rm -rf "$OUT_DIR/web-dist"
    cp -r demo/web/dist "$OUT_DIR/web-dist"
  fi
else
  echo "==> [3/4] 跳过 web 前端 (--no-web)"
fi

# ── 4. 复制数据 + 文档 ──────────────────────────────────────────

echo "==> [4/4] 复制数据与文档"
mkdir -p "$OUT_DIR/data"

for f in data/east_asia_7p5as.arpack data/mask_7p5as.mask; do
  [ -f "$f" ] && cp "$f" "$OUT_DIR/data/"
done

[ -f "install/HOW_TO_USE.md" ] && cp "install/HOW_TO_USE.md" "$OUT_DIR/"

# ── 5. 生成 SHA256SUMS ──────────────────────────────────────────

echo "==> 生成 SHA256SUMS"
(cd "$OUT_DIR" && find . -type f ! -name "SHA256SUMS" | sort | xargs sha256sum) > "$OUT_DIR/SHA256SUMS"

# ── 6. 产物清单 ─────────────────────────────────────────────────

echo ""
echo "==> 产物清单"
(cd "$OUT_DIR" && find . -type f ! -name "SHA256SUMS" | sort) | while read -r f; do
  size=$(du -h "$OUT_DIR/$f" | cut -f1)
  printf "  %8s  %s\n" "$size" "$f"
done

echo ""
echo "==> SHA256SUMS"
cat "$OUT_DIR/SHA256SUMS"

echo ""
echo "DONE: 发布目录 $OUT_DIR"

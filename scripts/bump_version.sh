#!/usr/bin/env bash
# 版本号维护：单一事实来源 = Cargo.toml [workspace.package] version。
# 用法：scripts/bump_version.sh <new_version>   例如 scripts/bump_version.sh 0.2.0
# 说明：只改 Cargo.toml 的 workspace version（cli 通过 version.workspace 继承）；
#       phase0 / demo/server 为内部 crate，版本独立、不跟随发布版本。
set -euo pipefail
cd "$(dirname "$0")/.."

NEW="${1:-}"
if [ -z "$NEW" ]; then
  echo "用法: scripts/bump_version.sh <new_version>" >&2
  exit 1
fi

# 语义化版本 X.Y.Z
if ! echo "$NEW" | grep -qE '^[0-9]+\.[0-9]+\.[0-9]+$'; then
  echo "错误: 版本号需为 X.Y.Z（例如 0.2.0）" >&2
  exit 1
fi

OLD="$(sed -n 's/^version = "\([^"]*\)".*/\1/p' Cargo.toml | head -n1)"
if [ -z "$OLD" ]; then
  echo "错误: 未在 Cargo.toml 找到 [workspace.package] version" >&2
  exit 1
fi

if [ "$OLD" = "$NEW" ]; then
  echo "版本未变化（$OLD），无需升级" >&2
  exit 1
fi

echo "==> 版本升级 $OLD -> $NEW"

# 仅替换 [workspace.package] 下第一处（行首 version = "..."）
sed -i "0,/^version = \"$OLD\"/s//version = \"$NEW\"/" Cargo.toml

echo "==> Cargo.toml 已更新。请手动完成："
echo "    1. 更新 CHANGELOG.md：把 [Unreleased] 内容归入 [$NEW]（附日期），清空 [Unreleased]"
echo "    2. git add -A && git commit -m \"chore: release v$NEW\""
echo "    3. git tag v$NEW && git push && git push --tags   # tag 触发 release workflow 交叉编译"
echo "       （release workflow 会校验 tag 与 Cargo.toml version 一致）"

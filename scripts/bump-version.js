#!/usr/bin/env node
// 版本号维护：单一事实来源 = Cargo.toml [workspace.package] version。
// 用法：node scripts/bump-version.js <new_version>   例如 node scripts/bump-version.js 0.2.0
// 说明：只改 Cargo.toml 的 workspace version（cli 通过 version.workspace 继承）；
//       src/demo/server 为内部 crate，版本独立、不跟随发布版本。

import { readFileSync, writeFileSync } from "node:fs";
import { join } from "node:path";

const ROOT = join(import.meta.dirname, "..");
const CARGO_TOML = join(ROOT, "Cargo.toml");

const NEW = process.argv[2];
if (!NEW) {
  console.error("用法: node scripts/bump-version.js <new_version>");
  process.exit(1);
}

if (!/^\d+\.\d+\.\d+$/.test(NEW)) {
  console.error("错误: 版本号需为 X.Y.Z（例如 0.2.0）");
  process.exit(1);
}

const content = readFileSync(CARGO_TOML, "utf-8");

const match = content.match(/^version\s*=\s*"([^"]+)"/m);
if (!match) {
  console.error("错误: 未在 Cargo.toml 找到 [workspace.package] version");
  process.exit(1);
}

const OLD = match[1];

if (OLD === NEW) {
  console.error(`版本未变化（${OLD}），无需升级`);
  process.exit(1);
}

console.log(`==> 版本升级 ${OLD} -> ${NEW}`);

const updated = content.replace(
  new RegExp(`^(version\\s*=\\s*")${OLD.replace(/\./g, "\\.")}(")`),
  `$1${NEW}$2`
);

writeFileSync(CARGO_TOML, updated);

console.log("==> Cargo.toml 已更新。请手动完成：");
console.log(`    1. 更新 CHANGELOG.md：把 [Unreleased] 内容归入 [${NEW}]（附日期），清空 [Unreleased]`);
console.log(`    2. git add -A && git commit -m "chore: release v${NEW}"`);
console.log(`    3. git tag v${NEW} && git push && git push --tags   # tag 触发 release workflow 交叉编译`);
console.log("       （release workflow 会校验 tag 与 Cargo.toml version 一致）");
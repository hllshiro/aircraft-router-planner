#!/usr/bin/env node
// 门禁检查脚本：构建 + 测试 + 静态红线。

import { execSync } from "node:child_process";
import { join } from "node:path";

const ROOT = join(import.meta.dirname, "..");
process.chdir(ROOT);

// ── 辅助函数 ──────────────────────────────────────────────────────

function run(cmd) {
  console.log(`\n$ ${cmd}`);
  execSync(cmd, { stdio: "inherit" });
}

function step(num, total, msg) {
  console.log(`\n==> [${num}/${total}] ${msg}`);
}

// ── 主流程 ────────────────────────────────────────────────────────

const steps = 5;
let current = 0;

// 1. 构建
current++;
step(current, steps, "构建 (release)");
run("cargo build --release -p arpcli");

// 2. 依赖红线
current++;
step(current, steps, "依赖红线检查");
run("cargo tree -e normal -p arpcli");

// 3. 单元测试
current++;
step(current, steps, "单元测试 (lib)");
run("cargo test --lib");

// 4. 崩溃套件
current++;
step(current, steps, "崩溃套件 (crash_suite)");
run("cargo test --test crash_suite");

// 5. 确定性测试
current++;
step(current, steps, "确定性测试 (determinism)");
run("cargo test --test determinism");

console.log("\n✓ 所有检查通过");
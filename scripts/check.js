#!/usr/bin/env node
// 门禁检查脚本：构建 + 测试 + 静态红线 + 性能预算。
// 用法：node scripts/check.js [--quick] [--with-compare]
//
// 选项：
//   --quick         跳过回归套件
//   --with-compare  额外跑 field_build_compare（~7min，只在改动 builtin.rs 时用）

import { execSync } from "node:child_process";
import { join } from "node:path";

const ROOT = join(import.meta.dirname, "..");
process.chdir(ROOT);

// ── 参数解析 ──────────────────────────────────────────────────────

let quick = false;
let withCompare = false;

for (const arg of process.argv.slice(2)) {
  if (arg === "--quick") quick = true;
  else if (arg === "--with-compare") withCompare = true;
  else if (arg === "--help" || arg === "-h") {
    console.log(`用法: node scripts/check.js [选项]

选项：
  --quick         跳过回归套件
  --with-compare  额外跑 field_build_compare（~7min）
  --help, -h      显示帮助`);
    process.exit(0);
  } else {
    console.error(`未知选项: ${arg}`);
    process.exit(1);
  }
}

// ── 辅助函数 ──────────────────────────────────────────────────────

function run(cmd) {
  console.log(`\n$ ${cmd}`);
  execSync(cmd, { stdio: "inherit" });
}

function step(num, total, msg) {
  console.log(`\n==> [${num}/${total}] ${msg}`);
}

// ── 主流程 ────────────────────────────────────────────────────────

const steps = quick ? 5 : 6;
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

// 6. 回归套件（可跳过）
if (!quick) {
  current++;
  step(current, steps, "回归套件 (regress_phase0)");
  run("cargo test --test regress_phase0");
}

// 7. field_build_compare（可选）
if (withCompare) {
  current++;
  step(current, steps, "field_build_compare (~7min)");
  run("cargo test --test field_build_compare");
}

console.log("\n✓ 所有检查通过");
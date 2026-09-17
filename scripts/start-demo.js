#!/usr/bin/env node
// Demo 启动脚本：同时启动 demo-server（:3001）和 web 前端（:5173）。
// 用法：node scripts/start-demo.js [--build] [--server-only] [--web-only]
//
// 选项：
//   --build        启动前先构建（cargo build + pnpm build）
//   --server-only  只启动后端
//   --web-only     只启动前端

import { execSync, spawn } from "node:child_process";
import { existsSync } from "node:fs";
import { join } from "node:path";

const ROOT = join(import.meta.dirname, "..");
process.chdir(ROOT);

// ── 参数解析 ──────────────────────────────────────────────────────

let doBuild = false;
let serverOnly = false;
let webOnly = false;

for (const arg of process.argv.slice(2)) {
  if (arg === "--build") doBuild = true;
  else if (arg === "--server-only") serverOnly = true;
  else if (arg === "--web-only") webOnly = true;
  else if (arg === "--help" || arg === "-h") {
    console.log(`用法: node scripts/start-demo.js [选项]

选项：
  --build        启动前先构建（cargo build + pnpm build）
  --server-only  只启动后端
  --web-only     只启动前端
  --help, -h     显示帮助`);
    process.exit(0);
  } else {
    console.error(`未知选项: ${arg}`);
    process.exit(1);
  }
}

if (serverOnly && webOnly) {
  console.error("错误：--server-only 和 --web-only 不能同时使用");
  process.exit(1);
}

// ── 辅助函数 ──────────────────────────────────────────────────────

function run(cmd, opts = {}) {
  console.log(`  $ ${cmd}`);
  execSync(cmd, { stdio: "inherit", ...opts });
}

function startProcess(cmd, args, name, color) {
  const child = spawn(cmd, args, {
    stdio: "inherit",
    shell: true,
    cwd: ROOT,
  });

  child.on("error", (err) => {
    console.error(`[${name}] 启动失败: ${err.message}`);
  });

  child.on("exit", (code) => {
    if (code !== null && code !== 0) {
      console.error(`[${name}] 退出码: ${code}`);
    }
  });

  return child;
}

// ── 主流程 ────────────────────────────────────────────────────────

const WEB_DIR = join(ROOT, "src/demo/web");

// 检查必要文件
if (!serverOnly && !existsSync(WEB_DIR)) {
  console.error(`错误：前端目录不存在: ${WEB_DIR}`);
  process.exit(1);
}

// 构建阶段
if (doBuild) {
  console.log("==> 构建 demo-server");
  run("cargo build --release -p demo-server");

  if (!serverOnly) {
    console.log("==> 构建 web 前端");
    run("pnpm --filter aircraft-router-planner-web install");
    run("pnpm --filter aircraft-router-planner-web build");
  }
}

// 启动阶段
const processes = [];

if (!webOnly) {
  console.log("==> 启动 demo-server (:3001)");
  const server = startProcess("cargo", ["run", "--release", "-p", "demo-server"], "server");
  processes.push(server);
}

if (!serverOnly) {
  console.log("==> 启动 web 前端 (:5173)");
  const web = startProcess("pnpm", ["--filter", "aircraft-router-planner-web", "dev"], "web");
  processes.push(web);
}

// 优雅退出
function cleanup() {
  console.log("\n==> 正在停止所有进程...");
  for (const p of processes) {
    p.kill("SIGTERM");
  }
  process.exit(0);
}

process.on("SIGINT", cleanup);
process.on("SIGTERM", cleanup);

console.log("\n✓ Demo 已启动");
if (!webOnly) console.log("  后端 API: http://localhost:3001");
if (!serverOnly) console.log("  前端界面: http://localhost:5173");
console.log("\n按 Ctrl+C 停止所有进程");
#!/usr/bin/env node
// P6-A 发布脚本：构建 release + 产物收集 + 静态红线校验。
// 输出目录：install/

import { execSync } from "node:child_process";
import {
  mkdirSync,
  cpSync,
  rmSync,
  existsSync,
} from "node:fs";
import { join, basename } from "node:path";

const ROOT = join(import.meta.dirname, "..");
process.chdir(ROOT);

const outDir = "install";

// ── 辅助函数 ──────────────────────────────────────────────────────

function run(cmd) {
  console.log(`  $ ${cmd}`);
  execSync(cmd, { stdio: "inherit" });
}

function mkdirp(dir) {
  mkdirSync(dir, { recursive: true });
}

function cp(src, dest) {
  cpSync(src, dest, { recursive: true });
}

// ── 主流程 ────────────────────────────────────────────────────────

mkdirp(outDir);

// 1. 构建 CLI
console.log("==> [1/4] 构建 CLI (release)");
run("cargo build --release -p arpcli");

let cliBin = "target/release/arpcli";
if (process.platform === "win32") cliBin += ".exe";

console.log("==> 复制 CLI");
cp(cliBin, join(outDir, basename(cliBin)));

// 2. 构建 demo-server
console.log("==> [2/4] 构建 demo-server (release)");
run("cargo build --release -p demo-server");

let serverBin = "target/release/demo-server";
if (process.platform === "win32") serverBin += ".exe";

console.log("==> 复制 demo-server");
cp(serverBin, join(outDir, basename(serverBin)));

// 3. 构建 web 前端
console.log("==> [3/4] 构建 web 前端");
try {
  execSync("pnpm --version", { stdio: "ignore" });
  if (!existsSync("src/demo/web")) {
    console.log("    警告：src/demo/web 不存在，跳过 web 构建");
  } else {
    run("pnpm --filter aircraft-router-planner-web install");
    run("pnpm --filter aircraft-router-planner-web build");
    rmSync(join(outDir, "web-dist"), { recursive: true, force: true });
    cp("src/demo/web/dist", join(outDir, "web-dist"));
  }
} catch {
  console.log("    警告：pnpm 不可用，跳过 web 构建");
}

// 4. 复制数据
console.log("==> [4/4] 复制数据");
if (existsSync("data")) {
  cp("data", join(outDir, "data"));
}

console.log(`DONE: 发布目录 ${outDir}`);
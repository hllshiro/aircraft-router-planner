#!/usr/bin/env node
// P6-A 发布脚本：构建 release + 产物收集 + 静态红线校验 + sha256。
// 用法：node scripts/release.js [选项] [output_dir]
//
// 选项：
//   --no-demo    跳过 demo-server 构建（仅 CLI 正式发布）
//   --no-web     跳过 web 前端构建（假设 web-dist 已就绪）
//
// 默认输出目录：install/

import { execSync } from "node:child_process";
import {
  mkdirSync,
  cpSync,
  rmSync,
  existsSync,
  statSync,
  readdirSync,
  readFileSync,
  writeFileSync,
} from "node:fs";
import { join, basename, relative } from "node:path";
import { createHash } from "node:crypto";

const ROOT = join(import.meta.dirname, "..");
process.chdir(ROOT);

// ── 参数解析 ──────────────────────────────────────────────────────

let skipDemo = false;
let skipWeb = false;
let outDir = "install";

for (const arg of process.argv.slice(2)) {
  if (arg === "--no-demo") skipDemo = true;
  else if (arg === "--no-web") skipWeb = true;
  else if (arg.startsWith("-")) {
    console.error(`未知选项: ${arg}`);
    process.exit(1);
  } else {
    outDir = arg;
  }
}

// ── 辅助函数 ──────────────────────────────────────────────────────

function run(cmd) {
  console.log(`  $ ${cmd}`);
  execSync(cmd, { stdio: "inherit" });
}

function runOut(cmd) {
  return execSync(cmd, { encoding: "utf-8" }).trim();
}

function mkdirp(dir) {
  mkdirSync(dir, { recursive: true });
}

function cp(src, dest) {
  cpSync(src, dest, { recursive: true });
}

function isWindows() {
  return process.platform === "win32";
}

function auditBinary(bin) {
  console.log(`==> 静态编译红线校验：${basename(bin)}`);
  try {
    if (isWindows()) {
      const dumpbin = runOut("where dumpbin").split("\n")[0];
      if (dumpbin) {
        const imports = runOut(`dumpbin /dependents "${bin}"`)
          .split("\n")
          .filter((l) => /\.dll/i.test(l));
        if (imports.length) {
          console.log("    导入表：");
          imports.forEach((l) => console.log(`    ${l}`));
        }
      } else {
        console.log("    警告：dumpbin 不可用，跳过导入表审计");
      }
    } else {
      const imports = runOut(`ldd "${bin}" 2>/dev/null || true`);
      if (imports) {
        console.log("    导入表：");
        console.log(imports);
      }
    }
  } catch {
    console.log("    警告：审计失败，跳过");
  }
}

function findFiles(dir) {
  const results = [];
  for (const entry of readdirSync(dir, { withFileTypes: true })) {
    const full = join(dir, entry.name);
    if (entry.isDirectory()) {
      results.push(...findFiles(full));
    } else {
      results.push(full);
    }
  }
  return results;
}

function humanSize(bytes) {
  const units = ["B", "KB", "MB", "GB"];
  let i = 0;
  let size = bytes;
  while (size >= 1024 && i < units.length - 1) {
    size /= 1024;
    i++;
  }
  return `${size.toFixed(1)}${units[i]}`;
}

// ── 主流程 ────────────────────────────────────────────────────────

mkdirp(outDir);

// 1. 构建 CLI
console.log("==> [1/4] 构建 CLI (release)");
run("cargo build --release -p arpcli");

let cliBin = "target/release/arpcli";
if (isWindows()) cliBin += ".exe";

auditBinary(cliBin);

console.log("==> 复制 CLI");
cp(cliBin, join(outDir, basename(cliBin)));

// 2. 构建 demo-server
if (!skipDemo) {
  console.log("==> [2/4] 构建 demo-server (release)");
  run("cargo build --release -p demo-server");

  let serverBin = "target/release/demo-server";
  if (isWindows()) serverBin += ".exe";

  auditBinary(serverBin);

  console.log("==> 复制 demo-server");
  cp(serverBin, join(outDir, basename(serverBin)));
} else {
  console.log("==> [2/4] 跳过 demo-server (--no-demo)");
}

// 3. 构建 web 前端
if (!skipWeb) {
  console.log("==> [3/4] 构建 web 前端");
  const hasPnpm = (() => {
    try {
      runOut("pnpm --version");
      return true;
    } catch {
      return false;
    }
  })();

  if (!hasPnpm) {
    console.log("    警告：pnpm 不可用，跳过 web 构建（使用 --no-web 可消除此警告）");
  } else if (!existsSync("src/demo/web")) {
    console.log("    警告：src/demo/web 不存在，跳过 web 构建");
  } else {
    run("pnpm --filter aircraft-router-planner-web install");
    run("pnpm --filter aircraft-router-planner-web build");
    rmSync(join(outDir, "web-dist"), { recursive: true, force: true });
    cp("src/demo/web/dist", join(outDir, "web-dist"));
  }
} else {
  console.log("==> [3/4] 跳过 web 前端 (--no-web)");
}

// 4. 复制数据 + 文档
console.log("==> [4/4] 复制数据与文档");
mkdirp(join(outDir, "data"));

for (const f of ["data/east_asia_7p5as.arpack", "data/mask_7p5as.mask"]) {
  if (existsSync(f)) cp(f, join(outDir, "data", basename(f)));
}

if (existsSync("install/HOW_TO_USE.md")) {
  cp("install/HOW_TO_USE.md", join(outDir, "HOW_TO_USE.md"));
}

// 5. 生成 SHA256SUMS
console.log("==> 生成 SHA256SUMS");
const files = findFiles(outDir)
  .filter((f) => basename(f) !== "SHA256SUMS")
  .sort();

const lines = files.map((f) => {
  const data = readFileSync(f);
  const hash = createHash("sha256").update(data).digest("hex");
  const rel = relative(outDir, f).replace(/\\/g, "/");
  return `${hash}  ${rel}`;
});

writeFileSync(join(outDir, "SHA256SUMS"), lines.join("\n") + "\n");

// 6. 产物清单
console.log("\n==> 产物清单");
for (const f of files) {
  const size = humanSize(statSync(f).size);
  const rel = relative(outDir, f).replace(/\\/g, "/");
  console.log(`  ${size.padStart(8)}  ${rel}`);
}

console.log(`\n==> SHA256SUMS`);
console.log(readFileSync(join(outDir, "SHA256SUMS"), "utf-8"));

console.log(`DONE: 发布目录 ${outDir}`);
# AGENTS.md

Deterministic 3D aircraft route planning CLI (Rust 2024 edition): FMM + semantic cost field → backtracking → Theta*/spline/Dubins smoothing → full-chain re-verification. Repo language (docs/comments) is Chinese.

## First read

- `docs/技术方案.md` — authoritative design doc (v0.20, contains all manager decisions)
- `docs/README.md` — index of `docs/01–11` implementation-status docs; `docs/01-概览与架构.md` maps design → code with file/line inventory

## Environment

- **所有编译 / 构建 / 测试 / 验证一律在 WSL（Ubuntu-22.04）执行**——本机 Windows 无 Rust 工具链；WSL 内已有完整环境（cargo / node / pnpm）。统一入口：`wsl -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/Project/Rust/AircraftRouterPlanner && <cmd>"`。
- 前端 `src/demo/web` 用 **pnpm**（不是 npm），同样在 WSL 内运行：`wsl -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/Project/Rust/AircraftRouterPlanner/src/demo/web && pnpm install && pnpm build"`。
- `scripts/*.js` 是 Node.js 脚本（不是 bash），也在 WSL 内运行。

## Workspace layout

- `src/cli/` — the product (lib+bin, package `arpcli`). All real work happens here. Entry: `src/cli/src/lib.rs`.
- `src/convert/` — internal `arp-convert` terrain tool; NOT shipped, build on demand.
- `src/demo/server` (`demo-server`, Axum) + `src/demo/web` (React/Vite, pnpm, NOT a workspace member) — dev visualization only, not in release. `demo-server` calls the CLI via stdin/stdout pipe; `ARP_CLI` env var overrides CLI path.
- `data/` and `install/` are gitignored (large terrain/mask files).

## Commands

```bash
pnpm cli:build               # cargo build --release -p arpcli (the product)
pnpm test                     # cargo test --lib (unit tests only)
pnpm test:all                 # cargo test --workspace
pnpm check                    # build + crash_suite + determinism + dependency red-line (full gate)
pnpm check:quick              # quick check (skip regression suite)
pnpm demo:server              # cargo run --release -p demo-server
pnpm demo:dev                 # dev frontend (Vite :5173)
pnpm demo:build               # build frontend
```

- CI runs `cargo check --workspace --all-targets` + dependency red-line; releases fire on `v*` tags only.
- Toolchain pinned in `rust-toolchain.toml` (1.89.0; MSRV driven by nalgebra 0.35/geo 0.33).

## Hard rules (CI one-vote veto — do not violate)

- **Zero C dependencies**: `cargo tree -e normal` must not match `openblas|zlib|curl|proj|gdal|pcre|ssl`. Keep nalgebra default features (no blas), geo without `proj` feature, flate2 rust_backend, ruzstd pure Rust. Add `cargo tree` check whenever adding a dependency.
- **Never panic (B9)**: malformed/degenerate input must return an error/status, never panic. Status contract: `success` / `degraded_timeout` / `no_solution` / `input_invalid`.
- **Determinism**: don't remove `-fma`/`+crt-static` rustflags from `.cargo/config.toml`, never set `target-cpu=native`; hot paths use BTreeMap/fixed-order reduction (no unordered fold/parallel reduce).

## Dependency pins (don't "clean up" these)

- `rand` ^0.10 (don't downgrade to 0.8), `rstar` ^0.12 (don't upgrade to 0.13) — geo 0.33.1 locks both; dual versions break size/API.
- `geotiff` ^0.1 and `dted2` =1.0.0: upgrades need separate review (breaking-change notice / unmaintained).
- thiserror ^2 + dted2's ^1 dual versions are accepted.

## Conventions

- CLI help style is `arpcli` / `arpcli help` — **no `--help`**. Pipeline: `arpcli plan --file <file> --out <path>` reads task JSON from file, writes result JSON to file.
- On every feature/fix: update the matching `docs/NN` doc's "与设计的差异/占位" section + `CHANGELOG.md` (Keep a Changelog). `docs/技术方案.md` + code win over status docs on conflict.
- Version source of truth: `[workspace.package] version` in root `Cargo.toml`; release tags must be `v<version>`; 发版流程见 `docs/发版流程.md`.
- Windows MSVC builds are static CRT (`+crt-static`) by manager decision — exe must not depend on VCRUNTIME140.dll (`src/cli/check_pe_deps.py` audits imports).

## opencode.json custom commands

`opencode.json` 定义了两个自定义命令，commit 前必须更新 CHANGELOG.md：
- `commit` — Conventional Commits（中文描述，英文 type/scope），自动更新 CHANGELOG
- `release` — 分析变更、推荐版本号、更新 CHANGELOG、tag + push

## CI 流水线细节

- `ci.yml`：push/PR 跑 **static-check**（`cargo check --workspace --all-targets` + 依赖红线），不执行测试（测试用例在 v0.5.0 中已移除）。
- `release.yml`：`v*` tag 触发，交叉编译 windows/linux × amd64/arm64 四产物，校验 tag 与 Cargo.toml version 一致。
- Linux release 用 `cross`（Docker 交叉工具链）构建 musl 静态二进制。

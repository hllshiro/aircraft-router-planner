# AGENTS.md

Deterministic 3D aircraft route planning CLI (Rust 2024 edition): FMM + semantic cost field → backtracking → Theta*/spline/Dubins smoothing → full-chain re-verification. Repo language (docs/comments) is Chinese.

## First read

- `docs/README.md` — index of `docs/01–11` implementation-status docs; `docs/01-概览与架构.md` maps design → code with file/line inventory

## Environment

- **All compilation, building, testing, and verification must be executed in WSL (Ubuntu-22.04)** — Windows has no Rust toolchain; WSL has the complete environment (cargo / node / pnpm). Unified entry: `wsl -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/Project/Rust/AircraftRouterPlanner && <cmd>"`.
- Frontend `src/demo/web` must use **pnpm** (not npm), also run in WSL: `wsl -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/Project/Rust/AircraftRouterPlanner/src/demo/web && pnpm install && pnpm build"`.
- `scripts/*.js` are Node.js scripts (not bash), must also be run in WSL.

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

- CI must run `cargo check --workspace --all-targets` + dependency red-line; releases must fire on `v*` tags only.
- Toolchain must be pinned in `rust-toolchain.toml` (1.89.0; MSRV driven by nalgebra 0.35/geo 0.33).

## Hard rules (CI one-vote veto — do not violate)

- **Zero C dependencies**: `cargo tree -e normal` must not match `openblas|zlib|curl|proj|gdal|pcre|ssl`. Keep nalgebra default features (no blas), geo without `proj` feature, flate2 rust_backend, ruzstd pure Rust. Add `cargo tree` check whenever adding a dependency.
- **Never panic (B9)**: malformed/degenerate input must return an error/status, never panic. Status contract: `success` / `degraded_timeout` / `no_solution` / `input_invalid`.
- **Determinism**: do not remove `-fma`/`+crt-static` rustflags from `.cargo/config.toml`, never set `target-cpu=native`; hot paths must use BTreeMap/fixed-order reduction (no unordered fold/parallel reduce).

## Dependency pins (do not "clean up" these)

- `rand` ^0.10 (do not downgrade to 0.8), `rstar` ^0.12 (do not upgrade to 0.13) — geo 0.33.1 locks both; dual versions break size/API.
- `geotiff` ^0.1 and `dted2` =1.0.0: upgrades must have separate review (breaking-change notice / unmaintained).
- thiserror ^2 + dted2's ^1 dual versions are accepted.

## Conventions

- CLI help style must be `arpcli` / `arpcli help` — **no `--help`**. Pipeline: `arpcli plan --file <file> --out <path>` reads task JSON from file, writes result JSON to file.
- On every feature/fix: must update the matching `docs/NN` doc's "与设计的差异/占位" section + `CHANGELOG.md` (Keep a Changelog). `docs/技术方案.md` + code win over status docs on conflict.
- Version source of truth: `[workspace.package] version` in root `Cargo.toml`; release tags must be `v<version>`; see `docs/发版流程.md` for release process.
- Windows MSVC builds must be static CRT (`+crt-static`) — exe must not depend on VCRUNTIME140.dll (`src/cli/check_pe_deps.py` audits imports).

## opencode.json custom commands

`opencode.json` defines two custom commands; must update `CHANGELOG.md` before commit:
- `commit` — Conventional Commits (Chinese description, English type/scope), auto-update CHANGELOG.
- `release` — analyze changes, recommend version, update CHANGELOG, tag + push.

## Branch workflow

```
feature branch (feat/*) ──PR──▶ dev ──PR──▶ master ──tag──▶ release
       │                        │
       └── CI gate ─────────────┘── CI gate
```

- All changes must be committed to a new branch and merged into `dev` via pull request; CI checks must pass before merging.
- `dev`: main development branch, feature branches merge here via PR.
- `master`: stable branch, merges from `dev` for release, tags trigger release pipeline.
- Feature branches: `feat/*`, `fix/*`, `refactor/*`, etc.

## CI pipeline

- `ci.yml`: must run **static-check** (`cargo check --workspace --all-targets` + dependency red-line) on push to dev/master and on PRs; no tests (removed in v0.5.0).
- `release.yml`: must trigger on `v*` tag, cross-compile windows/linux × amd64/arm64, verify tag matches Cargo.toml version.
- Linux release must use `cross` (Docker cross toolchain) for musl static binaries.
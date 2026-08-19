# AGENTS.md

Deterministic 3D aircraft route planning CLI (Rust 2024 edition): FMM + semantic cost field → backtracking → Theta*/spline/Dubins smoothing → full-chain re-verification. Repo language (docs/comments) is Chinese.

## First read

- `docs/技术方案.md` — authoritative design doc (v0.20, contains all manager decisions)
- `docs/README.md` — index of `docs/01–11` implementation-status docs; `docs/01-概览与架构.md` maps design → code with file/line inventory

## Environment

- This dev machine has **no Rust toolchain on Windows** — run all cargo/build/test/verification commands inside WSL (Ubuntu-22.04), e.g. `wsl -d Ubuntu-22.04 -- bash -lc "cd /mnt/d/Project/Rust/AircraftRouterPlanner && cargo test --lib"`. `scripts/*.sh` are bash scripts and also run there.

## Workspace layout

- `cli/` — the product (lib+bin, package `aircraft-router-planner-cli`). All real work happens here.
- `convert/` — internal `arp-convert` terrain tool; NOT shipped, build on demand.
- `phase0/` — historical performance prototype/benches; keep compilable, don't develop features there.
- `demo/server` (`demo-server`, Axum) + `demo/web` (React/Vite, npm, NOT a workspace member) — dev visualization only, not in release. `demo-server` calls the CLI via stdin/stdout pipe; `ARP_CLI` env var overrides CLI path.
- `data/` and `install/` are gitignored (large terrain/mask files); tests degrade gracefully when data is absent (synthetic flat terrain).

## Commands

```bash
cargo build --release -p aircraft-router-planner-cli   # the product
cargo test --lib                                       # unit tests (workspace root)
cargo test --test crash_suite                          # B9 never-crash suite (veto gate)
cargo test --test determinism                          # bit-identical two-run gate
cargo test --test regress_phase0                       # historical-bug regression suite
scripts/check.sh [--quick] [--with-compare]            # full gate: build+test+red-line+perf (bash / Git Bash)
cargo test --test field_build_compare                  # ~7 min; ONLY when touching ARPK1 decompress/BulkPrefetch/costfield
cargo bench -p phase0 && cargo bench -p aircraft-router-planner-cli --bench b_load_decompress
scripts/perf_regress.sh                                # ≤3s/100km budget; ARP_BUDGET_MS (0 = unlimited; tests use 0)
```

- `cargo test --lib` / `--test` run from workspace root target `cli` tests; phase0 has its own.
- Full tests do NOT run on push/PR — CI runs `cargo check --workspace --all-targets` + dependency red-line; test matrix is `workflow_dispatch` only; releases fire on `v*` tags only.
- Toolchain pinned in `rust-toolchain.toml` (1.89.0; MSRV driven by nalgebra 0.35/geo 0.33).

## Hard rules (CI one-vote veto — do not violate)

- **Zero C dependencies**: `cargo tree -e normal` must not match `openblas|zlib|curl|proj|gdal|pcre|ssl`. Keep nalgebra default features (no blas), geo without `proj` feature, flate2 rust_backend, ruzstd pure Rust. Add `cargo tree` check whenever adding a dependency.
- **Never panic (B9)**: malformed/degenerate input must return an error/status, never panic — crash_suite covers this. Status contract: `success` / `degraded_timeout` / `no_solution` / `input_invalid`.
- **Determinism**: don't remove `-fma`/`+crt-static` rustflags from `.cargo/config.toml`, never set `target-cpu=native`; hot paths use BTreeMap/fixed-order reduction (no unordered fold/parallel reduce).

## Dependency pins (don't "clean up" these)

- `rand` ^0.10 (don't downgrade to 0.8), `rstar` ^0.12 (don't upgrade to 0.13) — geo 0.33.1 locks both; dual versions break size/API.
- `geotiff` ^0.1 and `dted2` =1.0.0: upgrades need separate review (breaking-change notice / unmaintained).
- thiserror ^2 + dted2's ^1 dual versions are accepted.

## Conventions

- CLI help style is `arp-cli` / `arp-cli help <command>` — **no `--help`**. Pipeline: `arp-cli plan` reads task JSON from stdin (or `--input`), writes result JSON to stdout.
- `arp-cli schema [input|output|all]` generates JSON Schemas via schemars — code (`cli/src/config.rs` types) is the schema source of truth; keep them in sync when changing the contract.
- New regression case: drop a JSON into `cli/tests/regression/cases/` — auto-discovered. Output paths must never cross any zone; the suite asserts this.
- On every feature/fix: update the matching `docs/NN` doc's "与设计的差异/占位" section + `CHANGELOG.md` (Keep a Changelog). `docs/技术方案.md` + code win over status docs on conflict.
- Version source of truth: `[workspace.package] version` in root `Cargo.toml`; release tags must be `v<version>`; bump via `scripts/bump_version.sh`.
- Windows MSVC builds are static CRT (`+crt-static`) by manager decision — exe must not depend on VCRUNTIME140.dll (`cli/check_pe_deps.py` audits imports).

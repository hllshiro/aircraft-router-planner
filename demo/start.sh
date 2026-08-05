#!/usr/bin/env bash
set -e

ROOT="$(cd "$(dirname "$0")" && pwd)"

echo "==> Building CLI..."
cargo build --release -p aircraft-router-planner-cli

echo "==> Building server..."
cargo build --release -p demo-server

echo "==> Starting server on :3001..."
cargo run --release -p demo-server &
SERVER_PID=$!

# Give server a moment to bind
sleep 1

cleanup() {
    echo ""
    echo "==> Shutting down..."
    kill "$SERVER_PID" 2>/dev/null
    wait "$SERVER_PID" 2>/dev/null
    echo "Done."
}
trap cleanup EXIT INT TERM

echo "==> Installing frontend dependencies..."
cd "$ROOT/web"
pnpm install

echo "==> Starting frontend dev server on :5173..."
pnpm dev

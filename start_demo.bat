@echo off
setlocal enabledelayedexpansion
title AircraftRouterPlanner Demo Launcher
cd /d "%~dp0"

echo ============================================================
echo   AircraftRouterPlanner Demo - one-click launcher (Windows)
echo   backend  : http://localhost:3001  (demo-server)
echo   frontend : http://localhost:5173  (vite dev)
echo   usage    : start_demo.bat            quick start
echo              start_demo.bat rebuild    force rebuild binaries
echo ============================================================
echo.

REM ---- 0. check toolchain ----
where cargo >nul 2>nul
if errorlevel 1 goto fail_no_cargo
where npm >nul 2>nul
if errorlevel 1 goto fail_no_npm

set REBUILD=0
if /i "%~1"=="rebuild" set REBUILD=1

REM ---- 1. build CLI (release) if missing or forced ----
if exist "target\release\aircraft-router-planner-cli.exe" if "!REBUILD!"=="0" goto cli_skip
echo [1/4] building CLI (release) ...
cargo build --release -p aircraft-router-planner-cli
if errorlevel 1 goto fail_cli
:cli_skip

REM ---- 1. build demo-server (release) if missing or forced ----
if exist "target\release\demo-server.exe" if "!REBUILD!"=="0" goto server_skip
echo [1/4] building demo-server (release) ...
cargo build --release -p demo-server
if errorlevel 1 goto fail_server
:server_skip

REM ---- 2. free port 3001 from a lingering old server ----
netstat -ano | findstr ":3001 " >nul
if errorlevel 1 goto port_free
echo [2/4] port 3001 busy - killing old demo-server ...
taskkill /IM demo-server.exe /F >nul 2>nul
ping -n 2 127.0.0.1 >nul
:port_free

REM ---- 3. start backend (own window) ----
echo [2/4] starting demo-server on :3001 ...
start "arp-demo-server" "target\release\demo-server.exe"

set /a tries=0
:wait_server
ping -n 2 127.0.0.1 >nul
netstat -ano | findstr ":3001 " >nul
if not errorlevel 1 goto server_ok
set /a tries+=1
if !tries! lss 20 goto wait_server
echo [ERROR] server did not start in time - see window 'arp-demo-server'
pause
exit /b 1

:server_ok
echo [2/4] server OK - listening on http://localhost:3001

REM ---- 4. frontend dependencies ----
if exist "demo\web\node_modules" goto deps_skip
echo [3/4] installing frontend dependencies (first run, may take minutes) ...
pushd demo\web
call npm install
if errorlevel 1 goto fail_npm
popd
:deps_skip

REM ---- 5. start frontend (own window) + open browser ----
echo [4/4] starting vite dev server on :5173 ...
pushd demo\web
start "arp-demo-web" cmd /c "npm run dev"
popd

set /a tries=0
:wait_web
ping -n 2 127.0.0.1 >nul
netstat -ano | findstr ":5173 " >nul
if not errorlevel 1 goto web_ok
set /a tries+=1
if !tries! lss 40 goto wait_web
echo [WARN] frontend not detected yet - open http://localhost:5173 manually

:web_ok
echo.
echo ============================================================
echo   Demo is running!
echo     frontend : http://localhost:5173
echo     backend  : http://localhost:3001
echo ============================================================
echo   To stop: close the windows 'arp-demo-server' and
echo   'arp-demo-web', or run:  taskkill /IM demo-server.exe /F
echo.
start "" "http://localhost:5173"
echo   Press any key to close this launcher window...
pause >nul
exit /b 0

REM ---- error handlers ----
:fail_no_cargo
echo [ERROR] cargo not found. Install Rust: https://rustup.rs
pause
exit /b 1
:fail_no_npm
echo [ERROR] npm not found. Install Node.js: https://nodejs.org
pause
exit /b 1
:fail_cli
echo [ERROR] CLI build failed
pause
exit /b 1
:fail_server
echo [ERROR] demo-server build failed
pause
exit /b 1
:fail_npm
echo [ERROR] npm install failed
popd
pause
exit /b 1

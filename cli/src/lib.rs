//! AircraftRouterPlanner 核心库（Phase 1 核心骨架）。
//!
//! 模块划分见技术方案 4.1：config（JSON 契约）/ error / coord（坐标系统）/
//! terrain（地形数据源）/ spatial（rstar 索引）/ geometry / costfield（代价场+FMM）。
//! Phase 2 追加 primitives / solver / router；Phase 3 追加 smooth。

pub mod config;
pub mod coord;
pub mod costfield;
pub mod dubins;
pub mod error;
pub mod path;
pub mod smooth;
pub mod solver;
pub mod spatial;
pub mod terrain;
pub mod threat;

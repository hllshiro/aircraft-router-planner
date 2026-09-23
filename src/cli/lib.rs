//! AircraftRouterPlanner 核心库。
//!
//! 模块划分：config（JSON 契约）/ error / coord（坐标系统）/
//! terrain（地形数据源）/ spatial（rstar 索引）/ geometry / costfield（代价场+FMM）。

pub mod config;
pub mod coord;
pub mod costfield;
pub mod dubins;
pub mod error;
pub mod help;
pub mod patch;
pub mod path;
pub mod smooth;
pub mod solver;
pub mod spatial;
pub mod terrain;
pub mod threat;

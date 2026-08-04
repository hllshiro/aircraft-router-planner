//! AircraftRouterPlanner Phase 0 性能原型。
//!
//! S2（B1）：FMM 粗层传播——见 `fmm` 模块。

pub mod dubins;
pub mod fmm;
pub mod terrain;

pub fn add(left: u64, right: u64) -> u64 {
    left + right
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn it_works() {
        let result = add(2, 2);
        assert_eq!(result, 4);
    }
}

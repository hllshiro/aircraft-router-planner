//! 开发调试输出总开关，由 `--verbose` 隐藏开关设置。

use std::sync::atomic::{AtomicBool, Ordering};

static VERBOSE: AtomicBool = AtomicBool::new(false);

pub fn set(v: bool) {
    VERBOSE.store(v, Ordering::Relaxed);
}

pub fn on() -> bool {
    VERBOSE.load(Ordering::Relaxed)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn set_and_on() {
        set(true);
        assert!(on());
        set(false);
        assert!(!on());
    }
}
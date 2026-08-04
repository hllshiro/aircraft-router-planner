//! 合成地形、真实 DEM 加载与射线-地形求交（Phase 0 S5 / B4 基准）。
//!
//! - `synthetic_terrain`：平滑地形（正弦叠加）+ 高频噪声（确定性整数哈希，
//!   模拟 SRTM 高频信号），网格分辨率可配（默认 152.87m 语义）；
//! - `Terrain::from_raw`：加载预处理后的真实 DEM（Float32 raw，0 空洞 → NaN）；
//!   元数据文本：`rows cols cell_mx cell_my`（行/列单位米，支持矩形 cell）；
//! - `height_at`：双线性插值采样；空洞（NaN）或出界 → `None`；
//! - `ray_blocked`：沿射线等距采样 `n` 点比较高度 → LOS 遮挡判断（空洞不遮挡）。

/// 规则地形网格（行优先，行 = x 方向，列 = y 方向，单位米）。
/// 矩形 cell：`cell_mx` = 行向（x）格宽，`cell_my` = 列向（y）格宽。
/// 高度值 `NaN` 表示空洞/无效（如 Beijing_DEM 的 0=NoData 标记）。
pub struct Terrain {
    pub rows: usize,
    pub cols: usize,
    pub cell_mx: f64,
    pub cell_my: f64,
    pub h: Vec<f32>,
}

/// 确定性整数哈希 → [0,1)，用于高频噪声（避免引入 RNG 状态）。
fn hash2(x: i64, y: i64) -> f64 {
    let mut v = (x.wrapping_mul(0x9E3779B97F4A7C15u64 as i64))
        ^ y.wrapping_mul(0xC2B2AE3D27D4EB4Fu64 as i64);
    v = v.wrapping_mul(0x2545F4914F6CDD1Du64 as i64);
    v = v ^ (v >> 33);
    let u = v as u64;
    (u >> 11) as f64 / (1u64 << 53) as f64
}

/// 合成地形：平滑正弦叠加 + 高频噪声（幅度 ~50m，波长 ~1 格）。
pub fn synthetic_terrain(rows: usize, cols: usize, cell_m: f64, seed: i64) -> Terrain {
    let mut h = vec![0f32; rows * cols];
    for r in 0..rows {
        for c in 0..cols {
            let x = r as f64 * cell_m;
            let y = c as f64 * cell_m;
            let smooth = 1800.0
                + 1200.0 * (x / 60_000.0 * std::f64::consts::TAU).sin()
                + 900.0 * (y / 45_000.0 * std::f64::consts::TAU).cos()
                + 450.0 * ((x + y) / 25_000.0 * std::f64::consts::TAU).sin();
            let noise = (hash2(seed + r as i64, c as i64) - 0.5) * 100.0;
            let val = (smooth + noise).clamp(0.0, 4300.0);
            h[r * cols + c] = val as f32;
        }
    }
    Terrain {
        rows,
        cols,
        cell_mx: cell_m,
        cell_my: cell_m,
        h,
    }
}

impl Terrain {
    /// 从预处理 raw 加载真实 DEM。
    /// `raw_path`：Float32 行优先二进制（0 空洞应为 NaN）；`meta_path`：文本
    /// `rows cols cell_mx cell_my`。返回 `(Terrain, 加载耗时秒, 内存 MiB)`。
    pub fn from_raw(raw_path: &str, meta_path: &str) -> std::io::Result<(Terrain, f64, f64)> {
        let t0 = std::time::Instant::now();
        let meta = std::fs::read_to_string(meta_path)?;
        let mut it = meta.split_whitespace();
        let rows: usize = it.next().unwrap().parse().unwrap();
        let cols: usize = it.next().unwrap().parse().unwrap();
        let cell_mx: f64 = it.next().unwrap().parse().unwrap();
        let cell_my: f64 = it.next().unwrap().parse().unwrap();
        let raw = std::fs::read(raw_path)?;
        let mem_mib = raw.len() as f64 / (1024.0 * 1024.0);
        let mut h: Vec<f32> = Vec::with_capacity(rows * cols);
        h.extend(raw.chunks_exact(4).map(|b| f32::from_le_bytes([b[0], b[1], b[2], b[3]])));
        debug_assert_eq!(h.len(), rows * cols, "raw 大小与 meta 不匹配");
        let t_load = t0.elapsed().as_secs_f64();
        Ok((
            Terrain {
                rows,
                cols,
                cell_mx,
                cell_my,
                h,
            },
            t_load,
            mem_mib,
        ))
    }

    /// 双线性插值采样高度（米）。坐标 x,y 为物理坐标（米），原点在 (0,0)。
    /// 超出网格边界、或插值邻域含空洞（NaN）返回 `None`。
    pub fn height_at(&self, x_m: f64, y_m: f64) -> Option<f64> {
        let fr = x_m / self.cell_mx;
        let fc = y_m / self.cell_my;
        let r0 = fr.floor() as isize;
        let c0 = fc.floor() as isize;
        if r0 < 0 || c0 < 0 || r0 + 1 >= self.rows as isize || c0 + 1 >= self.cols as isize {
            return None;
        }
        let (r0, c0) = (r0 as usize, c0 as usize);
        let w_r = fr - r0 as f64; // 行向插值权重
        let w_c = fc - c0 as f64; // 列向插值权重
        let h00 = self.h[r0 * self.cols + c0] as f64;
        let h01 = self.h[r0 * self.cols + c0 + 1] as f64;
        let h10 = self.h[(r0 + 1) * self.cols + c0] as f64;
        let h11 = self.h[(r0 + 1) * self.cols + c0 + 1] as f64;
        // 空洞语义：任一邻域像素为 NaN → 该点无效（不遮挡）
        if h00.is_nan() || h01.is_nan() || h10.is_nan() || h11.is_nan() {
            return None;
        }
        let top = h00 + (h01 - h00) * w_c;
        let bot = h10 + (h11 - h10) * w_c;
        Some(top + (bot - top) * w_r)
    }

    /// 射线遮挡判断：起点 `(ox,oy,oz)` 沿方向 `(dx,dy,dz)` 长度 `len`（米），
    /// 等距采样 `n` 个点；任一采样点高度 ≤ 地形高度则视为被遮挡。
    pub fn ray_blocked(
        &self,
        ox: f64,
        oy: f64,
        oz: f64,
        dx: f64,
        dy: f64,
        dz: f64,
        len: f64,
        n: usize,
    ) -> bool {
        for i in 1..=n {
            let t = len * (i as f64 / n as f64);
            let x = ox + dx * t;
            let y = oy + dy * t;
            let z = oz + dz * t;
            match self.height_at(x, y) {
                Some(ht) if z <= ht => return true,
                None => return false, // 出界视为不遮挡
                _ => {}
            }
        }
        false
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn height_at_interp_and_bounds() {
        let t = synthetic_terrain(64, 64, 1000.0, 7);
        // 中心可采样
        assert!(t.height_at(32_000.0, 32_000.0).is_some());
        // 边界外返回 None
        assert!(t.height_at(-1.0, 1000.0).is_none());
        assert!(t.height_at(100_000.0, 1000.0).is_none());
    }

    #[test]
    fn ray_high_clear_low_blocked() {
        let t = synthetic_terrain(64, 64, 1000.0, 7);
        // 高飞（10km）横穿场景：不遮挡
        assert!(!t.ray_blocked(1000.0, 1000.0, 10_000.0, 1.0, 1.0, 0.0, 60_000.0, 1000));
        // 低飞（海平面以下起点向上爬，终点在山内）→ 遮挡
        // 地形平滑在中心可达 ~3000m+，从 z=100 穿山必遮挡
        assert!(t.ray_blocked(1000.0, 1000.0, 100.0, 1.0, 1.0, 0.0, 60_000.0, 1000));
    }

    #[test]
    fn nan_hole_returns_none() {
        let mut t = synthetic_terrain(8, 8, 1000.0, 1);
        t.h[3 * 8 + 3] = f32::NAN; // 制造空洞
        assert!(t.height_at(3_500.0, 3_500.0).is_none()); // 邻域含 NaN → None
        assert!(t.height_at(1_500.0, 1_500.0).is_some()); // 正常区域可采样
    }
}

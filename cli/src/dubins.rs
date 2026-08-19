//! Dubins 路径基元（Phase 0 S6 / B5 基准）。
//!
//! 实现 CSC 四类型（LSL / RSR / LSR / RSL）最短路径解析求解 + CCC（RLR/LRL）
//! 三圆弧兜底（2026-08-13 P9 T2）：
//! - 坐标系 y 向上、航向角逆时针；L 圆心 = p + R·(−sinθ, cosθ)，R 圆心 = p + R·(sinθ, −cosθ)；
//! - 同向圆（L-L / R-R）用外切线（切线方向 = 圆心连线方向），要求 d ≥ 2R；
//! - 异向圆（L-R / R-L）用内切线（切线方向 = v − 2R·n₁），要求 d ≥ 2R；
//! - 每种类型两个候选切线侧，取总长最小；
//! - 弧角按转向类型计算（L 逆时针 / R 顺时针）。
//!
//! CCC（d < 2R 退化情况，原判无解）：t1 → t2(反向) → t3(=t1) 三圆弧，
//! 相邻圆反向 → 外切 → 圆心距 2R；c2 是圆 (c1, 2R) ∩ (c3, 2R) 的交点，
//! 存在条件 d = |c1c3| ≤ 4R；两交点各算总长取最小（构造保证切点航向连续）。
//! 同点不同向（c1 ≈ c3，d→0）：单圆弧最小转角兜底（L/R 取小者）。

/// 归一化角到 [0, 2π)。
fn norm_angle(a: f64) -> f64 {
    let tau = std::f64::consts::TAU;
    let m = a % tau;
    if m < 0.0 { m + tau } else { m }
}

#[derive(Clone, Copy, PartialEq, Eq, Debug)]
pub enum Turn {
    L,
    R,
}

/// 圆心（真实尺度，乘 R）。L 圆心在航向左侧 R，R 圆心在航向右侧 R。
fn center(p: (f64, f64), th: f64, t: Turn, r: f64) -> (f64, f64) {
    match t {
        Turn::L => (p.0 - r * th.sin(), p.1 + r * th.cos()),
        Turn::R => (p.0 + r * th.sin(), p.1 - r * th.cos()),
    }
}

/// 弧角：从航向 `from` 沿转向 `t` 转到 `to`。
fn arc_angle(from: f64, to: f64, t: Turn) -> f64 {
    match t {
        Turn::L => norm_angle(to - from),
        Turn::R => norm_angle(from - to),
    }
}

/// 单条 CSC 路径（最优候选）。
#[derive(Clone, Debug)]
pub struct DubinsPath {
    pub t1: Turn,
    pub t2: Turn,
    pub a1: f64, // 首弧角（rad）
    pub straight: f64,
    pub a2: f64, // 末弧角（rad；CCC 时 = 中弧角）
    pub c1: (f64, f64),
    pub c2: (f64, f64),
    pub n1: (f64, f64),
    pub r: f64,
    /// CCC（RLR/LRL）三圆弧扩展：None = CSC 路径。
    pub ccc: Option<Ccc>,
}

/// CCC 三圆弧段（2026-08-13 P9 T2）。首弧 t1/a1 绕 c1，中弧 t2/a2 绕 c2（self 字段），
/// 末弧 t3/a3 绕 c3。phi 均为圆上角度（atan2 口径）。
#[derive(Clone, Debug)]
pub struct Ccc {
    pub t3: Turn,
    pub a3: f64,
    pub c3: (f64, f64),
    /// 起点 p0 在 c1 上的角度（首弧起点）。
    pub phi0: f64,
    /// 切点 A 在 c1 上的角度（首弧终点）。
    pub phi_ab: f64,
    /// 切点 A 在 c2 上的角度（中弧起点）。
    pub phi_ba: f64,
    /// 切点 B 在 c2 上的角度（中弧终点）。
    pub phi_bc: f64,
    /// 切点 B 在 c3 上的角度（末弧起点）。
    pub phi_cb: f64,
    /// 终点 p1 在 c3 上的角度（末弧终点）。
    pub phi1: f64,
}

impl DubinsPath {
    pub fn len(&self) -> f64 {
        let a3 = self.ccc.as_ref().map_or(0.0, |c| c.a3);
        self.r * (self.a1 + self.a2 + a3) + self.straight
    }

    /// 采样路径点（含两端）。转向 L 为逆时针（角度增大）。
    /// CSC：首弧 + 直线 + 末弧；CCC：三段圆弧（首弧 + 中弧 + 末弧）。
    pub fn sample(&self, n: usize) -> Vec<(f64, f64)> {
        if let Some(ccc) = &self.ccc {
            return self.sample_ccc(ccc, n);
        }
        let mut pts = Vec::with_capacity(n);
        let r = self.r;
        let segs = n.max(2);
        // 首弧：从 p0（c1 上角度 phi0）沿 t1 转 a1 到切点（角度 atan2(n1)）
        // 起点角度 = 切点角度 ∓ a1（L：phi_tan − a1；R：phi_tan + a1）。
        let phi_tan = self.n1.1.atan2(self.n1.0);
        let phi_start = match self.t1 {
            Turn::L => phi_tan - self.a1,
            Turn::R => phi_tan + self.a1,
        };
        let step1 = self.a1 / segs as f64;
        for i in 0..=segs {
            let ang = match self.t1 {
                Turn::L => phi_start + step1 * i as f64,
                Turn::R => phi_start - step1 * i as f64,
            };
            pts.push((self.c1.0 + r * ang.cos(), self.c1.1 + r * ang.sin()));
        }
        // 直线段：末点 B = c2 + r·n2（n2 = ±n1）
        let n2 = match self.t1 == self.t2 {
            true => self.n1,
            false => (-self.n1.0, -self.n1.1),
        };
        let b = (self.c2.0 + r * n2.0, self.c2.1 + r * n2.1);
        let a = *pts.last().unwrap();
        let (ux, uy) = (b.0 - a.0, b.1 - a.1);
        for i in 1..segs {
            let t = i as f64 / segs as f64;
            pts.push((a.0 + ux * t, a.1 + uy * t));
        }
        // 末弧：从 B（c2 上角度 atan2(n2)）沿 t2 转 a2 到终点
        let phi1 = n2.1.atan2(n2.0);
        let step2 = self.a2 / segs as f64;
        for i in 1..=segs {
            let ang = match self.t2 {
                Turn::L => phi1 + step2 * i as f64,
                Turn::R => phi1 - step2 * i as f64,
            };
            pts.push((self.c2.0 + r * ang.cos(), self.c2.1 + r * ang.sin()));
        }
        pts
    }

    /// CCC 三段圆弧采样：段数按弧长比例分配（每弧至少 1 段）。
    fn sample_ccc(&self, ccc: &Ccc, n: usize) -> Vec<(f64, f64)> {
        let n = n.max(4);
        let total = self.a1 + self.a2 + ccc.a3;
        let (n1, n2, n3) = if total > 0.0 {
            let f1 = self.a1 / total;
            let f2 = self.a2 / total;
            let n1 = ((n as f64 * f1).round() as usize).max(1);
            let n2 = ((n as f64 * f2).round() as usize).max(1);
            let n3 = ((n as f64 * (ccc.a3 / total)).round() as usize).max(1);
            (n1, n2, n3)
        } else {
            (1, 1, 1)
        };
        let mut pts = Vec::with_capacity(n1 + n2 + n3 + 1);
        let r = self.r;
        // 首弧：c1 上 phi0 → phi_ab（沿 t1）
        for k in 0..=n1 {
            let t = k as f64 / n1 as f64;
            let ang = match self.t1 {
                Turn::L => ccc.phi0 + self.a1 * t,
                Turn::R => ccc.phi0 - self.a1 * t,
            };
            pts.push((self.c1.0 + r * ang.cos(), self.c1.1 + r * ang.sin()));
        }
        // 中弧：c2 上 phi_ba → phi_bc（沿 t2；首点与首弧终点重合，从 k=1 起）
        for k in 1..=n2 {
            let t = k as f64 / n2 as f64;
            let ang = match self.t2 {
                Turn::L => ccc.phi_ba + self.a2 * t,
                Turn::R => ccc.phi_ba - self.a2 * t,
            };
            pts.push((self.c2.0 + r * ang.cos(), self.c2.1 + r * ang.sin()));
        }
        // 末弧：c3 上 phi_cb → phi1（沿 t3；首点与中弧终点重合，从 k=1 起）
        for k in 1..=n3 {
            let t = k as f64 / n3 as f64;
            let ang = match ccc.t3 {
                Turn::L => ccc.phi_cb + ccc.a3 * t,
                Turn::R => ccc.phi_cb - ccc.a3 * t,
            };
            pts.push((ccc.c3.0 + r * ang.cos(), ccc.c3.1 + r * ang.sin()));
        }
        pts
    }
}

/// Dubins 最短路径（CSC 四类型 + CCC 三圆弧兜底）。
/// CSC 无解（d < 2R）时尝试 CCC（RLR/LRL，d ≤ 4R）；同点不同向走单圆弧最小转角。
/// 真无解（d > 4R 且 CSC 无解 / 非有限 / r ≤ 0）返回 None。
pub fn dubins_path(
    p0: (f64, f64),
    th0: f64,
    p1: (f64, f64),
    th1: f64,
    r: f64,
) -> Option<DubinsPath> {
    if r <= 0.0 {
        return None;
    }
    // 防御：非有限输入（NaN/Inf）直接判无解，不 panic
    if !p0.0.is_finite()
        || !p0.1.is_finite()
        || !th0.is_finite()
        || !p1.0.is_finite()
        || !p1.1.is_finite()
        || !th1.is_finite()
        || !r.is_finite()
    {
        return None;
    }
    // 同点同向：零长度
    if p0 == p1 && norm_angle(th0) == norm_angle(th1) {
        let c = center(p0, th0, Turn::L, r);
        return Some(DubinsPath {
            t1: Turn::L,
            t2: Turn::L,
            a1: 0.0,
            straight: 0.0,
            a2: 0.0,
            c1: c,
            c2: c,
            n1: (p0.0 - c.0, p0.1 - c.1),
            r,
            ccc: None,
        });
    }
    // 同点不同向：单圆弧最小转角（c1 ≈ c3 的 CCC 退化；CSC 在 d=0 全被跳过）
    if (p0.0 - p1.0).abs() < 1e-12 && (p0.1 - p1.1).abs() < 1e-12 {
        let delta = norm_angle(th1 - th0);
        let (t, a1) = if delta <= std::f64::consts::PI {
            (Turn::L, delta)
        } else {
            (Turn::R, std::f64::consts::TAU - delta)
        };
        let c = center(p0, th0, t, r);
        let phi0 = (p0.1 - c.1).atan2(p0.0 - c.0);
        return Some(DubinsPath {
            t1: t,
            t2: t,
            a1,
            straight: 0.0,
            a2: 0.0,
            c1: c,
            c2: c,
            n1: (phi0.cos(), phi0.sin()),
            r,
            ccc: None,
        });
    }

    let mut best: Option<DubinsPath> = None;

    for t1 in [Turn::L, Turn::R] {
        for t2 in [Turn::L, Turn::R] {
            let c1 = center(p0, th0, t1, r);
            let c2 = center(p1, th1, t2, r);
            let vx = c2.0 - c1.0;
            let vy = c2.1 - c1.1;
            let d = (vx * vx + vy * vy).sqrt();
            if d < 2.0 * r - 1e-9 {
                continue; // 需 CCC，由下方兜底
            }
            if d < 1e-9 {
                continue;
            }
            let vhat = (vx / d, vy / d);
            let candidates: Vec<(f64, f64)> = if t1 == t2 {
                // 外切线：法线 = vhat 旋转 ±90°
                vec![(-vhat.1, vhat.0), (vhat.1, -vhat.0)]
            } else {
                // 内切线：v·n1 = 2R → n1 与 vhat 夹角 acos(2R/d)
                let ang = (2.0 * r / d).acos();
                let (cs, sn) = (ang.cos(), ang.sin());
                vec![
                    (vhat.0 * cs - vhat.1 * sn, vhat.0 * sn + vhat.1 * cs),
                    (vhat.0 * cs + vhat.1 * sn, -vhat.0 * sn + vhat.1 * cs),
                ]
            };
            for n1 in candidates {
                let n2 = match t1 == t2 {
                    true => n1,
                    false => (-n1.0, -n1.1),
                };
                let a = (c1.0 + r * n1.0, c1.1 + r * n1.1);
                let b = (c2.0 + r * n2.0, c2.1 + r * n2.1);
                let ux = b.0 - a.0;
                let uy = b.1 - a.1;
                let straight = (ux * ux + uy * uy).sqrt();
                if straight < 1e-9 {
                    continue;
                }
                let tangent = uy.atan2(ux);
                // 航向连续性：切点 A/B 处航向必须 == 切线方向
                // L 圆上航向 = 圆上角度 + 90°；R 圆上航向 = 圆上角度 − 90°
                let phi_a = n1.1.atan2(n1.0);
                let phi_b = n2.1.atan2(n2.0);
                let head_a = match t1 {
                    Turn::L => phi_a + std::f64::consts::FRAC_PI_2,
                    Turn::R => phi_a - std::f64::consts::FRAC_PI_2,
                };
                let head_b = match t2 {
                    Turn::L => phi_b + std::f64::consts::FRAC_PI_2,
                    Turn::R => phi_b - std::f64::consts::FRAC_PI_2,
                };
                let d_a = norm_angle(head_a - tangent).min(norm_angle(tangent - head_a));
                let d_b = norm_angle(head_b - tangent).min(norm_angle(tangent - head_b));
                if d_a > 1e-6 || d_b > 1e-6 {
                    continue; // 不连续候选（航向翻转 180°），非法
                }
                let a1 = arc_angle(th0, tangent, t1);
                let a2 = arc_angle(tangent, th1, t2);
                let path = DubinsPath {
                    t1,
                    t2,
                    a1,
                    straight,
                    a2,
                    c1,
                    c2,
                    n1,
                    r,
                    ccc: None,
                };
                let len = path.len();
                if best.as_ref().map_or(true, |b: &DubinsPath| len < b.len()) {
                    best = Some(path);
                }
            }
        }
    }
    if best.is_some() {
        return best;
    }

    // CSC 全失败 → CCC（RLR/LRL）兜底：t1 → t2(反向) → t3(=t1) 三圆弧。
    // 相邻圆反向转向 → 外切，圆心距 2R；c2 ∈ 圆(c1,2R) ∩ 圆(c3,2R)，
    // 存在条件 d = |c1c3| ≤ 4R。两交点各算总长取最小。
    for t1 in [Turn::L, Turn::R] {
        let t3 = t1;
        let t2 = match t1 {
            Turn::L => Turn::R,
            Turn::R => Turn::L,
        };
        let c1 = center(p0, th0, t1, r);
        let c3 = center(p1, th1, t3, r);
        let vx = c3.0 - c1.0;
        let vy = c3.1 - c1.1;
        let d = (vx * vx + vy * vy).sqrt();
        if d < 1e-9 || d > 4.0 * r - 1e-9 {
            continue; // 无交点（d>4R）或退化（d≈0 已被单圆弧兜底）
        }
        let th = vy.atan2(vx);
        let gamma = (d / (4.0 * r)).clamp(-1.0, 1.0).acos();
        for sgn in [1.0f64, -1.0] {
            let c2 = (
                c1.0 + 2.0 * r * (th + sgn * gamma).cos(),
                c1.1 + 2.0 * r * (th + sgn * gamma).sin(),
            );
            // 防御：c2 必须同时落在圆(c3, 2R) 上（浮点误差容差）
            let d23 = ((c2.0 - c3.0).powi(2) + (c2.1 - c3.1).powi(2)).sqrt();
            if (d23 - 2.0 * r).abs() > 1e-6 * (2.0 * r).max(1.0) {
                continue;
            }
            let phi_ab = (c2.1 - c1.1).atan2(c2.0 - c1.0); // 切点 A 在 c1 上
            let phi_ba = (c1.1 - c2.1).atan2(c1.0 - c2.0); // 切点 A 在 c2 上（= A + π）
            let phi_bc = (c3.1 - c2.1).atan2(c3.0 - c2.0); // 切点 B 在 c2 上
            let phi_cb = (c2.1 - c3.1).atan2(c2.0 - c3.0); // 切点 B 在 c3 上（= B + π）
            let phi0 = (p0.1 - c1.1).atan2(p0.0 - c1.0); // 起点在 c1 上
            let phi1 = (p1.1 - c3.1).atan2(p1.0 - c3.0); // 终点在 c3 上
            let head1 = match t1 {
                Turn::L => phi_ab + std::f64::consts::FRAC_PI_2,
                Turn::R => phi_ab - std::f64::consts::FRAC_PI_2,
            };
            let head_ba = match t2 {
                Turn::L => phi_ba + std::f64::consts::FRAC_PI_2,
                Turn::R => phi_ba - std::f64::consts::FRAC_PI_2,
            };
            let head_bc = match t2 {
                Turn::L => phi_bc + std::f64::consts::FRAC_PI_2,
                Turn::R => phi_bc - std::f64::consts::FRAC_PI_2,
            };
            let head_cb = match t3 {
                Turn::L => phi_cb + std::f64::consts::FRAC_PI_2,
                Turn::R => phi_cb - std::f64::consts::FRAC_PI_2,
            };
            let a1 = arc_angle(th0, head1, t1);
            let a2 = arc_angle(head_ba, head_bc, t2);
            let a3 = arc_angle(head_cb, th1, t3);
            let path = DubinsPath {
                t1,
                t2,
                a1,
                straight: 0.0,
                a2,
                c1,
                c2,
                n1: (phi0.cos(), phi0.sin()),
                r,
                ccc: Some(Ccc {
                    t3,
                    a3,
                    c3,
                    phi0,
                    phi_ab,
                    phi_ba,
                    phi_bc,
                    phi_cb,
                    phi1,
                }),
            };
            let len = path.len();
            if best.as_ref().map_or(true, |b: &DubinsPath| len < b.len()) {
                best = Some(path);
            }
        }
    }
    best
}

/// Dubins 最短路径长度。无解返回 None。
pub fn dubins_shortest_len(
    p0: (f64, f64),
    th0: f64,
    p1: (f64, f64),
    th1: f64,
    r: f64,
) -> Option<f64> {
    dubins_path(p0, th0, p1, th1, r).map(|p| p.len())
}

/// 走廊内基元拟合成功率：随机起止（位置+航向），Dubins 求解成功即计一次成功。
/// 返回 (成功数, 总样本数, 平均单段长度)。
pub fn dubins_success_rate(
    samples: &[((f64, f64), f64, (f64, f64), f64)],
    r: f64,
) -> (usize, usize, f64) {
    let mut ok = 0usize;
    let mut total_len = 0f64;
    for &((x0, y0), th0, (x1, y1), th1) in samples {
        if let Some(len) = dubins_shortest_len((x0, y0), th0, (x1, y1), th1, r) {
            ok += 1;
            total_len += len;
        }
    }
    let avg = if ok > 0 { total_len / ok as f64 } else { 0.0 };
    (ok, samples.len(), avg)
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn straight_lsl_is_straight() {
        let len = dubins_shortest_len((0.0, 0.0), 0.0, (100.0, 0.0), 0.0, 1.0).unwrap();
        assert!((len - 100.0).abs() < 1e-6, "got {len}");
    }

    #[test]
    fn vertical_lsr_path_valid() {
        // 起点 (0,0) 朝 +x，终点 (0,10) 朝 +x，R=1：采样路径验证端点/航向
        let path = dubins_path((0.0, 0.0), 0.0, (0.0, 10.0), 0.0, 1.0).expect("has solution");
        let pts = path.sample(200);
        let (sx, sy) = pts[0];
        let (ex, ey) = *pts.last().unwrap();
        assert!(
            (sx - 0.0).abs() < 1e-6 && (sy - 0.0).abs() < 1e-6,
            "start {sx},{sy}"
        );
        assert!(
            (ex - 0.0).abs() < 1e-6 && (ey - 10.0).abs() < 1e-6,
            "end {ex},{ey}"
        );
        // 末航向：末两点方向 ≈ 0（+x）
        let n = pts.len();
        let (x1, y1) = pts[n - 2];
        let (x2, y2) = pts[n - 1];
        let heading = (y2 - y1).atan2(x2 - x1);
        // 采样分辨率误差 ≈ 半段弧角（200 段末弧 ≈ 0.009 rad）；0.02 足够宽松
        assert!(heading.abs() < 0.02, "final heading {heading}");
        // 长度合理性：> 直线 10，< 半圆 5π（R=1 的松散上界）
        let len = path.len();
        assert!(len > 10.0 && len < 15.0, "got {len}");
    }

    #[test]
    fn same_point_zero() {
        let len = dubins_shortest_len((5.0, 5.0), 1.0, (5.0, 5.0), 1.0, 1.0).unwrap();
        assert!(len.abs() < 1e-9, "got {len}");
    }

    #[test]
    fn too_close_ccc_solves() {
        // 距离 < 2R 同向：CSC 无解，CCC（RLR/LRL）兜底有解（2026-08-13 P9 T2）
        let len = dubins_shortest_len((0.0, 0.0), 0.0, (1.0, 0.0), 0.0, 1.0);
        let len = len.expect("CCC should solve d<2R");
        assert!(len > 0.0 && len.is_finite(), "got {len}");
    }

    #[test]
    fn ccc_close_path_valid() {
        // 近距 0.1 < 2R=2 且转向 90°：4 个 CSC 类型圆心距全 < 2R 无解 → CCC 三圆弧；
        // 采样路径端点/末航向验证
        let path = dubins_path(
            (0.0, 0.0),
            0.0,
            (0.1, 0.0),
            std::f64::consts::FRAC_PI_2,
            1.0,
        )
        .expect("CCC solution");
        assert!(path.ccc.is_some(), "should be CCC path");
        let pts = path.sample(600);
        let (sx, sy) = pts[0];
        let (ex, ey) = *pts.last().unwrap();
        assert!(
            (sx - 0.0).abs() < 1e-6 && (sy - 0.0).abs() < 1e-6,
            "start {sx},{sy}"
        );
        assert!(
            (ex - 0.1).abs() < 1e-3 && (ey - 0.0).abs() < 1e-3,
            "end {ex},{ey}"
        );
        // 末航向 ≈ 90°（+y）：末两点方向
        let n = pts.len();
        let (x1, y1) = pts[n - 2];
        let (x2, y2) = pts[n - 1];
        let heading = (y2 - y1).atan2(x2 - x1);
        assert!(
            (heading - std::f64::consts::FRAC_PI_2).abs() < 0.05,
            "final heading {heading}"
        );
        // 路径长度：三圆弧 > 单圆周长下限（R=1 至少转 > π 弧度）
        assert!(path.len() > std::f64::consts::PI, "got {}", path.len());
    }

    #[test]
    fn ccc_close_vertical_heading() {
        // 近距掉头（0.1，0→π/2）：4 CSC 全败 → CCC 有解且末点正确（含中弧 > π 的大转角）
        let path = dubins_path(
            (0.0, 0.0),
            0.0,
            (0.1, 0.0),
            std::f64::consts::FRAC_PI_2,
            1.0,
        )
        .expect("CCC solution");
        assert!(path.ccc.is_some());
        assert!(
            path.a2 > std::f64::consts::PI,
            "mid arc should exceed π, got {}",
            path.a2
        );
        let pts = path.sample(600);
        let (ex, ey) = *pts.last().unwrap();
        assert!(
            (ex - 0.1).abs() < 1e-3 && (ey - 0.0).abs() < 1e-3,
            "end {ex},{ey}"
        );
    }

    #[test]
    fn same_point_different_heading_single_arc() {
        // 同点不同向：(0,0) 0 → (0,0) π/2：单圆弧最小转角 π/2·R
        let len = dubins_shortest_len(
            (0.0, 0.0),
            0.0,
            (0.0, 0.0),
            std::f64::consts::FRAC_PI_2,
            1.0,
        )
        .expect("single arc");
        assert!(
            (len - std::f64::consts::FRAC_PI_2).abs() < 1e-6,
            "got {len}"
        );
        // 反向：(0,0) 0 → (0,0) π：最小转角 π
        let len = dubins_shortest_len((0.0, 0.0), 0.0, (0.0, 0.0), std::f64::consts::PI, 1.0)
            .expect("half arc");
        assert!((len - std::f64::consts::PI).abs() < 1e-6, "got {len}");
        // 大转角：(0,0) 0 → (0,0) 3π/2：反向往回转 π/2
        let len = dubins_shortest_len((0.0, 0.0), 0.0, (0.0, 0.0), 1.5 * std::f64::consts::PI, 1.0)
            .expect("reverse arc");
        assert!(
            (len - std::f64::consts::FRAC_PI_2).abs() < 1e-6,
            "got {len}"
        );
    }

    #[test]
    fn ccc_boundary_no_panic() {
        // 边界防御：d 略超 4R / r=0 / 非有限 → 不 panic（None 或 CSC 有解）
        let _ = dubins_shortest_len((0.0, 0.0), 0.0, (10.0, 0.0), 0.0, 3.0);
        let _ = dubins_shortest_len((0.0, 0.0), 0.0, (1.0, 0.0), 0.0, 0.0);
        let _ = dubins_shortest_len((f64::NAN, 0.0), 0.0, (1.0, 0.0), 0.0, 1.0);
        let _ = dubins_shortest_len((0.0, 0.0), 0.0, (1.0, 0.0), 0.0, f64::INFINITY);
    }
}

#!/usr/bin/env python3
"""补充验证：指定数据 任意经纬度区域采样 + 有效覆盖率网格 + zstd 压缩率估算。
用法: python wdem_supplement.py <tif_path> [--no-compress]
布局兼容：tile（含边缘非满）与 strip（rowsperstrip>=1）。
"""
import sys

import numpy as np
import tifffile

TILE = 256


def build_reader(path):
    tf = tifffile.TiffFile(path)
    page = tf.pages[0]
    shape = page.shape
    rows, cols = shape[0], shape[1]
    offsets = page.dataoffsets
    counts = page.databytecounts
    gt_tag = page.tags.get(33550)
    tie_tag = page.tags.get(33922)
    if gt_tag is None or tie_tag is None:
        raise RuntimeError("no GeoTIFF tags")
    scale = gt_tag.value
    tie = tie_tag.value
    gt = (tie[3], scale[0], 0.0, tie[4], 0.0, -scale[1])
    fh = open(path, "rb")  # 保持打开（闭包读取用）

    def read_seg(idx):
        fh.seek(offsets[idx])
        raw = fh.read(counts[idx])
        return np.frombuffer(raw, dtype=np.float32)

    def read_region(r0, c0, h, w):
        if page.is_tiled:
            tpr = (cols + TILE - 1) // TILE
            block = np.zeros((h, w), dtype=np.float32)
            rt0, rt1 = r0 // TILE, (r0 + h - 1) // TILE
            ct0, ct1 = c0 // TILE, (c0 + w - 1) // TILE
            for rt in range(rt0, rt1 + 1):
                for ct in range(ct0, ct1 + 1):
                    idx = rt * tpr + ct
                    th = min(TILE, rows - rt * TILE)
                    tw = min(TILE, cols - ct * TILE)
                    t = read_seg(idx).reshape(th, tw)
                    tr, tc = rt * TILE, ct * TILE
                    dr0 = max(r0, tr) - r0
                    dc0 = max(c0, tc) - c0
                    dr1 = min(r0 + h, tr + th) - r0
                    dc1 = min(c0 + w, tc + tw) - c0
                    block[dr0:dr1, dc0:dc1] = t[
                        dr0 + r0 - tr : dr1 + r0 - tr, dc0 + c0 - tc : dc1 + c0 - tc
                    ]
            return block
        else:
            rps = page.rowsperstrip
            block = np.zeros((h, w), dtype=np.float32)
            for rr in range(r0, r0 + h):
                idx = rr // rps
                line = read_seg(idx).reshape(rps, cols)
                block[rr - r0, :] = line[rr % rps, c0 : c0 + w]
            return block

    return gt, rows, cols, read_region


def lonlat_to_rc(gt, lon, lat):
    c = int((lon - gt[0]) / gt[1])
    r = int((gt[3] - lat) / abs(gt[5]))
    return r, c


def main() -> None:
    path = sys.argv[1]
    do_compress = "--no-compress" not in sys.argv
    gt, rows, cols, read_region = build_reader(path)
    px_deg = abs(gt[1])
    px_arcsec = px_deg * 3600.0
    px_m_eq = px_deg * 111_320.0
    print(f"[header] shape=({rows},{cols}) px={px_deg:.9f}deg = {px_arcsec:.3f} arcsec ≈ {px_m_eq:.2f}m(eq)")
    print(
        f"[extent] lon {gt[0]:.4f}..{gt[0]+cols*px_deg:.4f} "
        f"lat {gt[3]-rows*px_deg:.4f}..{gt[3]:.4f}"
    )

    with open(path, "rb") as _fh_unused:
        pass  # 文件句柄由 build_reader 持有；此块仅为保持原结构
        # 北京（116.4E, 39.9N）
        r_bj, c_bj = lonlat_to_rc(gt, 116.4, 39.9)
        bj = read_region(r_bj - 128, c_bj - 128, 256, 256)
        fin = np.isfinite(bj)
        print(f"\n[beijing] r={r_bj} c={c_bj}")
        print(
            f"  finite={fin.sum()}/{bj.size} ({fin.sum()/bj.size*100:.2f}%) "
            f"min={bj[fin].min():.1f} max={bj[fin].max():.1f} mean={bj[fin].mean():.1f}"
            f" nan={int((~fin).sum())}"
        )

        # 有效覆盖网格：12x12 网格 x 128x128 区域 → 估算整体覆盖率 + 有效边界
        n_grid = 12
        sample = 128
        grid_finite = np.zeros((n_grid, n_grid), dtype=float)
        for gi in range(n_grid):
            r0 = min(max(gi * (rows - sample) // (n_grid - 1), 0), rows - sample)
            for gj in range(n_grid):
                c0 = min(max(gj * (cols - sample) // (n_grid - 1), 0), cols - sample)
                blk = read_region(r0, c0, sample, sample)
                grid_finite[gi, gj] = np.isfinite(blk).mean()
        cover = grid_finite.mean() * 100.0
        print(f"\n[coverage] {n_grid}x{n_grid} 网格有限像素占比 ≈ {cover:.2f}%")
        # 有效行列边界（有限占比 > 50% 的行/列范围）
        row_cov = grid_finite.mean(axis=1)
        col_cov = grid_finite.mean(axis=0)
        rows_ok = np.where(row_cov > 0.5)[0]
        cols_ok = np.where(col_cov > 0.5)[0]
        if len(rows_ok) and len(cols_ok):
            r_min = rows_ok[0] * (rows - sample) // (n_grid - 1)
            r_max = (rows_ok[-1] + 1) * (rows - sample) // (n_grid - 1)
            c_min = cols_ok[0] * (cols - sample) // (n_grid - 1)
            c_max = (cols_ok[-1] + 1) * (cols - sample) // (n_grid - 1)
            print(
                f"  有效覆盖(>50%网格): 行 {r_min}..{r_max} 列 {c_min}..{c_max}"
                f" → lon {gt[0]+c_min*px_deg:.2f}..{gt[0]+c_max*px_deg:.2f}"
                f" lat {gt[3]-r_max*px_deg:.2f}..{gt[3]-r_min*px_deg:.2f}"
            )

        if do_compress:
            import pyzstd

            # 压缩率样本：北京/中心区/东北/西北（取有值区域）
            samples = {}
            for name, (lon, lat) in {
                "beijing": (116.4, 39.9),
                "sichuan": (104.3, 30.0),
                "xinjiang": (85.0, 42.0),
                "northeast": (126.0, 45.0),
                "tibet": (90.0, 30.0),
            }.items():
                r0, c0 = lonlat_to_rc(gt, lon, lat)
                r0 = min(max(r0 - 128, 0), rows - 256)
                c0 = min(max(c0 - 128, 0), cols - 256)
                blk = read_region(r0, c0, 256, 256)
                if np.isfinite(blk).mean() < 0.05:
                    continue
                samples[name] = blk
            print("\n[compression] zstd-19 小样本 (256x256 float32 = 262144B):")
            ratios = []
            for name, arr in samples.items():
                comp = pyzstd.compress(arr.tobytes(), 19)
                ratio = len(arr.tobytes()) / len(comp)
                ratios.append(ratio)
                print(f"  {name:>12}: {len(arr.tobytes())} -> {len(comp)}B ({ratio:.2f}x)")
            if ratios:
                mean_ratio = float(np.mean(ratios))
                raw_bytes = rows * cols * 4
                print(
                    f"\n[estimate] 样本均压缩率 {mean_ratio:.2f}x → "
                    f"全图 {raw_bytes/1e9:.2f}GB 压缩后约 {raw_bytes/mean_ratio/1e9:.2f} GB"
                )


if __name__ == "__main__":
    sys.exit(main())

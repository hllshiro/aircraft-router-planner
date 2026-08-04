#!/usr/bin/env python3
"""World_DEM_L11 真实分辨率/像素类型/空洞率/值域分析（正式数据验证流程）。

读取 tif 头部（尺寸/类型/压缩/GeoTransform）+ 分区域采样统计。
不整读 8.59GB 文件，仅按需读子块。
"""
import json
import os
import sys

import numpy as np
import tifffile

PATH = sys.argv[1] if len(sys.argv) > 1 else r"D:\workspace\code_engineer\3rd_party\World_DEM_L11\World_DEM_L11.tif"
OUT_JSON = os.path.join(
    r"D:\workspace\code_engineer\coding_projects\AircraftRouterPlanner\phase0",
    os.path.splitext(os.path.basename(PATH))[0] + "_analyze_out.json",
)


def main() -> None:
    tf = tifffile.TiffFile(PATH)
    page = tf.pages[0]
    shape = page.shape
    dtype = page.dtype
    print(f"[header] shape={shape} dtype={dtype}")
    print(f"[header] compression={page.compression} planar={page.planarconfig}")

    # GeoTransform（TIFF GeoTIFF 标签）
    px_scale = page.tags.get(33550)  # ModelPixelScaleTag
    tiepoint = page.tags.get(33922)  # ModelTiepointTag
    gt = None
    if px_scale is not None and tiepoint is not None:
        scale = px_scale.value
        tie = tiepoint.value
        # tie: [i, j, k, x, y, z]
        gt = (tie[3], scale[0], 0.0, tie[4], 0.0, -scale[1])
        print(f"[header] ModelPixelScale={scale} (deg/px)")
        print(f"[header] ModelTiepoint={tie}")
        print(f"[header] GeoTransform={gt}")
        px_deg = scale[0]
        px_arcsec = px_deg * 3600.0
        px_m_equator = px_deg * 111_320.0
        print(
            f"[resolution] {px_deg:.9f} deg/px = {px_arcsec:.3f} arcsec ≈ {px_m_equator:.2f} m (equator)"
        )
    else:
        print("[header] no GeoTIFF tags (px_scale/tiepoint missing)")

    # NoData tag
    nd = page.tags.get(42113)  # GDAL_NODATA
    print(f"[header] GDAL_NODATA={nd.value if nd is not None else None}")

    # 分区域采样统计（8 个区域，512x512 每块；按 tile 手动读取，不整读 8.59GB）
    # tile 布局：256x256 float32 = 262144B；序号 = row_tile * tiles_per_row + col_tile
    import struct

    tile_size = 256
    with open(PATH, "rb") as fh:
        offsets = page.dataoffsets
        counts = page.databytecounts

        rows, cols = shape[0], shape[1]

        def read_segment_region(r0, c0, h, w):
            """按数据段读取任意矩形区域（兼容 tile 与 strip 布局）。"""
            if page.is_tiled:
                tpr = (cols + tile_size - 1) // tile_size

                def read_seg(idx):
                    rt = idx // tpr
                    ct = idx % tpr
                    th = min(tile_size, rows - rt * tile_size)
                    tw = min(tile_size, cols - ct * tile_size)
                    fh.seek(offsets[idx])
                    raw = fh.read(counts[idx])
                    return np.frombuffer(raw, dtype=np.float32).reshape(th, tw)

                rt0, rt1 = r0 // tile_size, (r0 + h - 1) // tile_size
                ct0, ct1 = c0 // tile_size, (c0 + w - 1) // tile_size
                block = np.zeros((h, w), dtype=np.float32)
                for rt in range(rt0, rt1 + 1):
                    for ct in range(ct0, ct1 + 1):
                        t = read_seg(rt * tpr + ct)
                        tr, tc = rt * tile_size, ct * tile_size
                        dr0 = max(r0, tr) - r0
                        dc0 = max(c0, tc) - c0
                        dr1 = min(r0 + h, tr + t.shape[0]) - r0
                        dc1 = min(c0 + w, tc + t.shape[1]) - c0
                        block[dr0:dr1, dc0:dc1] = t[
                            dr0 + r0 - tr : dr1 + r0 - tr,
                            dc0 + c0 - tc : dc1 + c0 - tc,
                        ]
                return block
            else:
                # strip 布局：段 idx = 行号（rowsperstrip=1）
                rps = page.rowsperstrip
                block = np.zeros((h, w), dtype=np.float32)
                for rr in range(r0, r0 + h):
                    idx = rr // rps
                    fh.seek(offsets[idx])
                    raw = fh.read(counts[idx])
                    line = np.frombuffer(raw, dtype=np.float32).reshape(-1, cols)
                    in_row = rr % rps
                    block[rr - r0, :] = line[in_row, c0 : c0 + w]
                return block

        regions = [
            (0, 0),
            (0, cols // 2),
            (0, cols - 512),
            (rows // 2, 0),
            (rows // 2, cols // 2),
            (rows - 512, 0),
            (rows - 512, cols // 2),
            (rows - 512, cols - 512),
        ]
        print("\n[regions] 8 x 512x512 samples:")
        all_stats = []
        for (r0, c0) in regions:
            r0 = min(max(r0, 0), rows - 512)
            c0 = min(max(c0, 0), cols - 512)
            block = read_segment_region(r0, c0, 512, 512)
            block = block.astype(np.float64)
            finite = np.isfinite(block)
            n_finite = int(finite.sum())
            if n_finite == 0:
                stats = {
                    "region": [r0, c0],
                    "n": 512 * 512,
                    "n_finite": 0,
                    "zero_pct": float((block == 0).sum() / block.size * 100.0),
                    "min": None,
                    "max": None,
                    "mean": None,
                }
            else:
                z = block[finite]
                zero_pct = float((z == 0).sum() / z.size * 100.0)
                stats = {
                    "region": [r0, c0],
                    "n": 512 * 512,
                    "n_finite": n_finite,
                    "zero_pct": zero_pct,
                    "min": float(z.min()),
                    "max": float(z.max()),
                    "mean": float(z.mean()),
                }
            all_stats.append(stats)
            print(
                f"  r{r0:>6} c{c0:>6}: finite={n_finite:>7}/{512*512} "
                f"zero={stats['zero_pct']:.2f}% min={stats['min']} "
                f"max={stats['max']} mean={stats['mean']}"
            )

    total = sum(s["n"] for s in all_stats)
    nfinite = sum(s["n_finite"] for s in all_stats)
    zeros = sum(s["zero_pct"] * s["n"] / 100.0 for s in all_stats)
    print(
        f"\n[summary] finite={nfinite}/{total} ({nfinite/total*100:.2f}%) "
        f"zero(==0)={zeros/total*100:.2f}% (含 NoData 或真实海平面)"
    )

    out = {
        "path": PATH,
        "shape": list(shape),
        "dtype": str(dtype),
        "compression": str(page.compression),
        "geotransform": gt,
        "px_deg": px_deg if gt else None,
        "px_arcsec": px_arcsec if gt else None,
        "px_m_equator": px_m_equator if gt else None,
        "nodata_tag": nd.value if nd is not None else None,
        "samples": all_stats,
    }
    with open(OUT_JSON, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    print(f"\n[out] {OUT_JSON} written")


if __name__ == "__main__":
    sys.exit(main())

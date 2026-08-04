"""Beijing_DEM 测试数据分析：真实 dtype / 高度范围 / NaN 与特殊值 / 直方图。

注意 GDAL 报 NoData=nan 而 band 类型为 Int16 —— 这里实证实际存储值与语义。
"""
import json
import os
import numpy as np
import tifffile

SRC = r'D:\workspace\code_engineer\3rd_party\Beijing_DEM\Beijing_DEM.tif'
OUT = os.path.join(os.path.dirname(__file__), '..', '..', 'phase0_out', 'beijing_dem_analysis.json')

os.makedirs(os.path.dirname(OUT), exist_ok=True)

tif = tifffile.TiffFile(SRC)
page = tif.pages[0]
print('tifffile shape:', page.shape, 'dtype:', page.dtype)

arr = page.asarray()
print('array OK, shape', arr.shape, 'dtype', arr.dtype)
print('sample:', arr.flat[::max(1, arr.size // 20)].tolist()[:20])

# 特殊值分布（Int16 可能的空洞标记：-32768 / 0 / 极小负值）
n = arr.size
vals, counts = np.unique(arr, return_counts=True)
print('unique values count:', vals.size)
for v in vals:
    if v == -32768 or v == 0 or (v < -100) or (v > 8500):
        idx = np.where(vals == v)[0][0]
        print(f'  special val {v}: {counts[idx]} px ({100.0 * counts[idx] / n:.4f}%)')

mask_nan = np.isnan(arr.astype(np.float64))
print('isnan on float-cast count:', int(mask_nan.sum()))

good = arr[~mask_nan].astype(np.float64)
print('height min/max:', good.min(), good.max())
print('mean:', good.mean())
print('p05/p50/p95:', np.percentile(good, [5, 50, 95]))

# 直方图（-200..3500 主带 + 越界计数）
HIST_LO, HIST_HI, NB = -200.0, 3500.0, 74
h, edges = np.histogram(good, bins=NB, range=(HIST_LO, HIST_HI))
below = int((good < HIST_LO).sum())
above = int((good > HIST_HI).sum())
print('hist bins [-200..3500]:', h.tolist())
print('below/above:', below, above)

report = {
    'shape': list(page.shape),
    'dtype': str(page.dtype),
    'pixel_deg': 0.000343322753906,
    'resolution_arcsec': 0.000343322753906 * 3600,
    'nan_count': int(mask_nan.sum()),
    'nan_ratio': float(mask_nan.sum() / n),
    'height_min': float(good.min()),
    'height_max': float(good.max()),
    'mean': float(good.mean()),
    'p05': float(np.percentile(good, 5)),
    'p50': float(np.percentile(good, 50)),
    'p95': float(np.percentile(good, 95)),
    'special_vals': {str(int(v)): int(counts[i]) for i, v in enumerate(vals)
                     if v == -32768 or v == 0 or v < -100 or v > 8500},
    'histogram': h.tolist(),
    'below_hist': below,
    'above_hist': above,
}
with open(OUT, 'w', encoding='utf-8') as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
print('report saved ->', OUT)

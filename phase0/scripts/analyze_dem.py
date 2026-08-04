"""World_DEM_L10 数据分析：memmap 分块统计高度范围 / NaN 比例 / 直方图。"""
import json
import os
import numpy as np
import tifffile

SRC = r'D:\workspace\code_engineer\3rd_party\World_DEM_L10\World_DEM_L10.tif'
OUT = os.path.join(os.path.dirname(__file__), '..', '..', 'phase0_out', 'dem_analysis.json')

os.makedirs(os.path.dirname(OUT), exist_ok=True)

tif = tifffile.TiffFile(SRC)
page = tif.pages[0]
print('shape:', page.shape, 'dtype:', page.dtype)

# 全量读入（2GB）后分块统计
arr = page.asarray()
print('array OK, shape', arr.shape, 'dtype', arr.dtype)

n = arr.size
vmin = np.float32('inf')
vmax = np.float32('-inf')
nan_cnt = 0
total = 0.0
hist = np.zeros(64, dtype=np.int64)
BINS = np.linspace(-500.0, 9000.0, 65)
CHUNK = 8192 * 1024
flat = arr.reshape(-1)
for start in range(0, n, CHUNK):
    a = flat[start:start + CHUNK]
    mask = np.isnan(a)
    nan_cnt += int(mask.sum())
    good = a[~mask]
    if good.size == 0:
        continue
    total += float(good.sum())
    vmin = min(vmin, float(good.min()))
    vmax = max(vmax, float(good.max()))
    h, _ = np.histogram(good, bins=BINS)
    hist += h

print('NaN count:', nan_cnt, f'({100.0 * nan_cnt / n:.2f}%)')
print('height min/max:', vmin, vmax)
print('mean:', total / (n - nan_cnt))
print('histogram bins [-500..9000]:')
print(hist)

report = {
    'shape': list(page.shape),
    'dtype': str(page.dtype),
    'pixel_deg': 0.010986328125,
    'resolution_arcsec': 0.010986328125 * 3600,
    'nan_count': int(nan_cnt),
    'nan_ratio': float(nan_cnt / n),
    'height_min': float(vmin),
    'height_max': float(vmax),
    'mean': float(total / (n - nan_cnt)),
    'histogram': hist.tolist(),
}
with open(OUT, 'w', encoding='utf-8') as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
print('report saved ->', OUT)

"""压缩率探针：代表性瓦片 × zstd 级别 → 压缩率/速度对比。
格式原型（方案 4.2.5）：Float32 → Int16(米) → 行内差分 → zstd。"""
import json
import os
import time
import numpy as np
import pyzstd
import tifffile

SRC = r'D:\workspace\code_engineer\3rd_party\World_DEM_L10\World_DEM_L10.tif'
OUT = os.path.join(os.path.dirname(__file__), '..', '..', 'phase0_out', 'compress_probe.json')
os.makedirs(os.path.dirname(OUT), exist_ok=True)

PIX = 0.010986328125  # deg/px


def row_col(lat, lon):
    return int(round((90.0 - lat) / PIX)), int(round((lon + 180.0) / PIX))


TILES = {
    'himalaya': (28.0, 87.0),   # 喜马拉雅高频山区
    'pacific': (0.0, -150.0),    # 太平洋深海平原（低频）
    'sahara': (22.0, 5.0),       # 撒哈拉沙漠（中频）
    'mariana': (11.3, 142.2),    # 马里亚纳海沟（极值区）
}

LEVELS = [3, 9, 19]
TILE = 1024

print('reading full array...')
tif = tifffile.TiffFile(SRC)
arr = tif.pages[0].asarray().astype(np.float32)
h, w = arr.shape
print('full array', arr.shape, '-> int16 conversion...')
i16 = np.round(arr).astype(np.int16)  # Int16 米，零损失覆盖 -10685..7529
del arr

report = {'pixel_deg': PIX, 'int16_range': [int(i16.min()), int(i16.max())]}
for name, (lat, lon) in TILES.items():
    r, c = row_col(lat, lon)
    r = min(max(r - TILE // 2, 0), h - TILE)
    c = min(max(c - TILE // 2, 0), w - TILE)
    tile = i16[r:r + TILE, c:c + TILE]
    raw_bytes = tile.nbytes
    print(f'--- {name} (row {r}, col {c}) raw {raw_bytes} bytes ---')

    # 行内差分（Int16 差值范围 -65535..65535，块内安全）
    d = tile.astype(np.int32)
    d[:, 1:] -= d[:, :-1]
    diff_bytes = d.astype(np.int16).tobytes()

    # 无差分基线（zstd 直接压 int16）
    for lvl in LEVELS:
        t0 = time.perf_counter()
        comp = pyzstd.compress(tile.tobytes(), lvl)
        dt = time.perf_counter() - t0
        print(f'  raw-int16  zstd-{lvl}: {len(comp)} bytes  ratio {raw_bytes/len(comp):.1f}x  {dt*1000:.0f}ms')
    print(f'  diff-int16 raw: {len(diff_bytes)} bytes')
    for lvl in LEVELS:
        t0 = time.perf_counter()
        comp = pyzstd.compress(diff_bytes, lvl)
        dt = time.perf_counter() - t0
        ratio = raw_bytes / len(comp)
        print(f'  diff-int16 zstd-{lvl}: {len(comp)} bytes  ratio {ratio:.1f}x  {dt*1000:.0f}ms')
        report.setdefault(name, {})[str(lvl)] = {'compressed_bytes': len(comp), 'ratio': round(ratio, 2)}

with open(OUT, 'w', encoding='utf-8') as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
print('report ->', OUT)

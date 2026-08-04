"""Beijing_DEM 全量压缩验证：Int16 → 256x256 块内行差分 → zstd 逐块。
输出 phase0_out/beijing_terrain.zstd + 报告（含空洞 0 值口径）。"""
import json
import os
import random
import struct
import time
import numpy as np
import pyzstd
import tifffile

SRC = r'D:\workspace\code_engineer\3rd_party\Beijing_DEM\Beijing_DEM.tif'
OUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'phase0_out')
os.makedirs(OUT_DIR, exist_ok=True)
OUT = os.path.join(OUT_DIR, 'beijing_terrain.zstd')
REPORT = os.path.join(OUT_DIR, 'beijing_compress_full.json')

PIX = 0.000343322753906
BLOCK = 256
ZSTD_LEVEL = 9

print('reading full array...')
t0 = time.perf_counter()
tif = tifffile.TiffFile(SRC)
raw = tif.pages[0].asarray()
i16 = raw.astype(np.int16)
del raw
h, w = i16.shape
zero_ratio = float((i16 == 0).mean())
print(f'int16 ready {h}x{w} in {time.perf_counter()-t0:.1f}s  zero_ratio={zero_ratio:.3f}')

bh, bw = h // BLOCK, w // BLOCK
magic = b'ARPTERR1'
header = struct.pack('<8sHIIfIIB', magic, 1, h, w, PIX, bh, bw, ZSTD_LEVEL)
body_start = len(header) + bh * bw * 16

compressed_blocks = []
for br in range(bh):
    rows = slice(br * BLOCK, (br + 1) * BLOCK)
    for bc in range(bw):
        cols = slice(bc * BLOCK, (bc + 1) * BLOCK)
        tile = i16[rows, cols]
        d = tile.astype(np.int32)
        d[:, 1:] = d[:, 1:] - d[:, :-1]
        comp = pyzstd.compress(d.astype(np.int16).tobytes(), ZSTD_LEVEL)
        compressed_blocks.append(comp)

offsets = []
pos = body_start
for comp in compressed_blocks:
    offsets.append((pos, len(comp)))
    pos += len(comp)

with open(OUT, 'wb') as f:
    f.write(header)
    for (o, s) in offsets:
        f.write(struct.pack('<QQ', o, s))
    for comp in compressed_blocks:
        f.write(comp)

total = pos
eff_px = h * w * (1.0 - zero_ratio)  # 有效（非空洞）像素
ratio_all = (i16.nbytes) / total
ratio_eff = (eff_px * 2) / total
print(f'compressed: {total} bytes ({total/1048576:.1f} MiB)')
print(f'ratio vs int16(all px): {ratio_all:.2f}x ; vs int16(valid px only): {ratio_eff:.2f}x')
print(f'time: {time.perf_counter()-t0:.1f}s')

random.seed(42)
ok = True
t_d0 = time.perf_counter()
for _ in range(5):
    br = random.randrange(bh)
    bc = random.randrange(bw)
    o, s = offsets[br * bw + bc]
    with open(OUT, 'rb') as f:
        f.seek(o)
        comp = f.read(s)
    d = np.frombuffer(pyzstd.decompress(comp), dtype=np.int16).reshape(BLOCK, BLOCK).astype(np.int32)
    restored = np.cumsum(d, axis=1).astype(np.int16)
    orig = i16[br * BLOCK:(br + 1) * BLOCK, bc * BLOCK:(bc + 1) * BLOCK]
    if not np.array_equal(restored, orig):
        ok = False
        print(f'MISMATCH at block {br},{bc}')
t_decomp = time.perf_counter() - t_d0
print(f'decompress verify: {"PASS" if ok else "FAIL"} (5 blocks, {t_decomp*1000:.0f}ms)')

report = {
    'shape': [h, w], 'pixel_deg': PIX, 'resolution_arcsec': PIX * 3600,
    'zstd_level': ZSTD_LEVEL, 'zero_ratio': zero_ratio,
    'compressed_bytes': int(total), 'compressed_mib': round(total / 1048576, 2),
    'ratio_vs_int16_all': round(ratio_all, 2),
    'ratio_vs_int16_valid': round(ratio_eff, 2),
    'verify': 'PASS' if ok else 'FAIL',
}
with open(REPORT, 'w', encoding='utf-8') as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
print('report ->', REPORT)

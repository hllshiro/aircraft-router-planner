"""全量压缩验证：Float32 → Int16 → 256x256 块内行差分 → zstd 逐块。
输出 phase0_out/terrain.zstd（原型格式：magic + 头 + 块偏移索引 + 压缩块），
并随机抽块解压回验（与原值对比）。"""
import json
import os
import struct
import time
import numpy as np
import pyzstd
import tifffile

SRC = r'D:\workspace\code_engineer\3rd_party\World_DEM_L10\World_DEM_L10.tif'
OUT_DIR = os.path.join(os.path.dirname(__file__), '..', '..', 'phase0_out')
os.makedirs(OUT_DIR, exist_ok=True)
OUT = os.path.join(OUT_DIR, 'terrain.zstd')
REPORT = os.path.join(OUT_DIR, 'compress_full.json')

PIX = 0.010986328125
BLOCK = 256
ZSTD_LEVEL = 9

print('reading full array...')
t0 = time.perf_counter()
tif = tifffile.TiffFile(SRC)
arr = tif.pages[0].asarray().astype(np.float32)
i16 = np.round(arr).astype(np.int16)
del arr
h, w = i16.shape
print(f'int16 ready {h}x{w} in {time.perf_counter()-t0:.1f}s')

bh, bw = h // BLOCK, w // BLOCK
# 头部：magic(8) + version(u16) + rows(u32) + cols(u32) + pixel_deg(f64) + bh(u32) + bw(u32) + zstd_level(u8)
magic = b'ARPTERR1'
header = struct.pack('<8sHIIfIIB', magic, 1, h, w, PIX, bh, bw, ZSTD_LEVEL)
index = np.zeros((bh, bw, 2), dtype=np.int64)  # (offset, size) 压缩块位置
body_start = len(header) + bh * bw * 16

compressed_blocks = []
for br in range(bh):
    rows = slice(br * BLOCK, (br + 1) * BLOCK)
    for bc in range(bw):
        cols = slice(bc * BLOCK, (bc + 1) * BLOCK)
        tile = i16[rows, cols]
        d = tile.astype(np.int32)
        # 行内差分（显式 RHS 求值——重叠视图禁止 in-place 运算）
        d[:, 1:] = d[:, 1:] - d[:, :-1]
        comp = pyzstd.compress(d.astype(np.int16).tobytes(), ZSTD_LEVEL)
        compressed_blocks.append(comp)

# 写文件
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
ratio = (i16.nbytes) / total
print(f'compressed: {total} bytes ({total/1024/1024:.1f} MiB)  ratio {ratio:.2f}x (int16 base) / {h*w*4/total:.2f}x (float32 base)')
print(f'time: {time.perf_counter()-t0:.1f}s')

# ---- 解压回验：随机抽 5 块 ----
import random
random.seed(42)
ok = True
t_decomp_start = time.perf_counter()
for _ in range(5):
    br = random.randrange(bh)
    bc = random.randrange(bw)
    o, s = offsets[br * bw + bc]
    with open(OUT, 'rb') as f:
        f.seek(o)
        comp = f.read(s)
    dbytes = pyzstd.decompress(comp)
    d = np.frombuffer(dbytes, dtype=np.int16).reshape(BLOCK, BLOCK).astype(np.int32)
    # 行内差分还原 = 前缀累加（int32 避免中间溢出）
    restored = np.cumsum(d, axis=1).astype(np.int16)
    orig = i16[br * BLOCK:(br + 1) * BLOCK, bc * BLOCK:(bc + 1) * BLOCK]
    if not np.array_equal(restored, orig):
        ok = False
        print(f'MISMATCH at block {br},{bc}')
t_decomp = time.perf_counter() - t_decomp_start
print(f'decompress verify: {"PASS" if ok else "FAIL"} (5 blocks, {t_decomp*1000:.0f}ms)')

report = {
    'shape': [h, w], 'pixel_deg': PIX, 'zstd_level': ZSTD_LEVEL,
    'compressed_bytes': int(total), 'compressed_mib': round(total / 1048576, 2),
    'ratio_vs_int16': round(ratio, 2), 'ratio_vs_float32': round(h * w * 4 / total, 2),
    'verify': 'PASS' if ok else 'FAIL',
    'target_mib': 800, 'target_ok': total <= 800 * 1048576,
}
with open(REPORT, 'w', encoding='utf-8') as f:
    json.dump(report, f, ensure_ascii=False, indent=2)
print('report ->', REPORT)

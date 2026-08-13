import os
import numpy as np
import tifffile

path = os.path.join('data', 'overview_test_multi.tif')
if os.path.exists(path):
    os.remove(path)

# 主图 512x512（tile 16 → 1024 chunks > 64 → lazy LRU 路径）；cell 0.001°
# north-up（tiepoint (0,0)=左上角 116.0, 38.7；tifffile 默认 sy 正 → 行 0 在南，
# 需 row_flip 判定 → 用 resolve_sy 逻辑：都合理 + tj=0 → north-up 翻转）
N = 512
base = (np.arange(N * N, dtype=np.uint16).reshape(N, N) % 5000)
tie = np.array([0, 0, 0, 116.0, 38.7, 0], dtype=np.float64)
scale = np.array([0.001, 0.001, 0], dtype=np.float64)
tifffile.imwrite(path, base, tile=(16, 16), photometric='minisblack',
                 metadata=None, extratags=[
                     (33922, 12, 3, tie.tobytes(), True),
                     (33550, 12, 3, scale.tobytes(), True),
                 ])
# 第二 IFD：128x128 overview（cell 0.004°，4x4 块均值）
M = N // 4
ov = base.reshape(M, 4, M, 4).mean(axis=(1, 3)).astype(np.uint16)
tie2 = np.array([0, 0, 0, 116.0, 38.7, 0], dtype=np.float64)
scale2 = np.array([0.004, 0.004, 0], dtype=np.float64)
tifffile.imwrite(path, ov, append=True, tile=(16, 16), photometric='minisblack',
                 metadata=None, extratags=[
                     (33922, 12, 3, tie2.tobytes(), True),
                     (33550, 12, 3, scale2.tobytes(), True),
                 ])
print('written', path, os.path.getsize(path))

with tifffile.TiffFile(path) as tf:
    print('pages:', len(tf.pages))
    for i, p in enumerate(tf.pages):
        print(i, 'shape', p.shape)
        print('  tie  ', p.tags.get(33922).value if 33922 in p.tags else None)
        print('  scale', p.tags.get(33550).value if 33550 in p.tags else None)

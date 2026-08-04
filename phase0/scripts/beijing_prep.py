"""Beijing_DEM 预处理：tif → f32 raw（0 空洞 → NaN）+ meta 文本。

输出 phase0/data/：
  beijing_dem_f32.raw   全量 6091x4712 Float32（0→NaN）
  beijing_dem_f32.2x.raw   2x2 块平均 3046x2356
  beijing_dem_f32.4x.raw   4x4 块平均 1523x1178
  beijing_dem.meta      文本：rows cols cell_mx cell_my（cell 单位米）

坐标约定：原点左上角；行 = 纬度向（南，cell_mx），列 = 经度向（东，cell_my）。
Beijing 40N: 0.0003433228° 纬度 = 38.09m（行向），经度 = 29.28m（列向）。
"""
import os
import numpy as np
import tifffile

SRC = r'D:\workspace\code_engineer\3rd_party\Beijing_DEM\Beijing_DEM.tif'
OUT_DIR = os.path.join(os.path.dirname(__file__), '..', 'data')
os.makedirs(OUT_DIR, exist_ok=True)

CELL_MX = 38.09  # 行向（纬度向）米
CELL_MY = 29.28  # 列向（经度向）米
NAN_RATIO_MIN = 0.25  # 降采样块内有效占比阈值


def to_float_with_nan(raw: np.ndarray) -> np.ndarray:
    f = raw.astype(np.float32)
    f[raw == 0] = np.nan
    return f


def downsample_block(f: np.ndarray, block: int) -> np.ndarray:
    """块平均；块内有效占比 < NAN_RATIO_MIN → NaN。"""
    h, w = f.shape
    oh, ow = h // block, w // block
    out = np.full((oh, ow), np.nan, dtype=np.float32)
    for r in range(oh):
        for c in range(ow):
            blk = f[r * block:(r + 1) * block, c * block:(c + 1) * block]
            good = blk[~np.isnan(blk)]
            if good.size >= block * block * NAN_RATIO_MIN:
                out[r, c] = good.mean()
    return out


def main():
    raw = tifffile.imread(SRC)
    print('src', raw.shape, raw.dtype, 'zero_ratio', (raw == 0).mean())
    f = to_float_with_nan(raw)
    valid = np.isfinite(f)
    print('valid ratio', valid.mean(), 'height range',
          f[valid].min(), f[valid].max())

    f.tofile(os.path.join(OUT_DIR, 'beijing_dem_f32.raw'))
    write_meta(OUT_DIR, 'beijing_dem.meta', f.shape[0], f.shape[1], CELL_MX, CELL_MY)
    for block, name in [(2, '2x'), (4, '4x')]:
        d = downsample_block(f, block)
        d.tofile(os.path.join(OUT_DIR, f'beijing_dem_f32.{name}.raw'))
        write_meta(OUT_DIR, f'beijing_dem.{name}.meta',
                   d.shape[0], d.shape[1], CELL_MX * block, CELL_MY * block)
        dv = np.isfinite(d)
        print(f'{name} ->', d.shape, 'valid ratio', dv.mean(),
              'range', d[dv].min() if dv.any() else '-', d[dv].max() if dv.any() else '-')

    print('meta written; files:')
    for fn in os.listdir(OUT_DIR):
        print(' ', fn, os.path.getsize(os.path.join(OUT_DIR, fn)))


def write_meta(out_dir, name, rows, cols, cell_mx, cell_my):
    with open(os.path.join(out_dir, name), 'w') as fp:
        fp.write(f'{rows} {cols} {cell_mx:.4f} {cell_my:.4f}\n')


if __name__ == '__main__':
    main()

import numpy as np
import tifffile

rows, cols = 300, 200
h = np.zeros((rows, cols), dtype=np.float32)
for r in range(rows):
    for c in range(cols):
        h[r, c] = 100.0 + r * 1.5 - c * 0.5
h[120:140, 80:100] = -25.0
h[290:300, 190:200] = np.nan
extratags = [
    (33550, 12, 3, (0.001, 0.001, 0.0), True),   # ModelPixelScaleTag DOUBLE
    (33922, 12, 6, (0, 0, 0, 116.0, 39.0, 0.0), True),  # ModelTiepointTag DOUBLE
]
tifffile.imwrite(
    r"D:\workspace\code_engineer\coding_projects\AircraftRouterPlanner\phase0\data\_test_small.tif",
    h,
    photometric="minisblack",
    extratags=extratags,
)
print("written 300x200 test tif (extratags geo)")

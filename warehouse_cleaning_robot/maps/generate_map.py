#!/usr/bin/env python3
"""
Generate a Nav2-compatible occupancy map (.pgm + .yaml) from the known
warehouse geometry. Walls/racks = black (occupied), free space = white.

Map matches the Gazebo world exactly so AMCL can localize against it.
"""

import numpy as np

# --- Map config ---
resolution = 0.05          # metres per pixel (Nav2 standard)
world_w = 15.0             # warehouse width (m)
world_h = 20.0             # warehouse height (m)
W = int(world_w / resolution)   # pixels wide
H = int(world_h / resolution)   # pixels tall

# Occupancy grid: 254 = free (white), 0 = occupied (black)
grid = np.full((H, W), 254, dtype=np.uint8)


def fill_rect(xc, yc, lx, ly):
    """Mark a rectangle (world metres, centre xc,yc, size lx,ly) as occupied."""
    x0 = int((xc - lx / 2) / resolution)
    x1 = int((xc + lx / 2) / resolution)
    y0 = int((yc - ly / 2) / resolution)
    y1 = int((yc + ly / 2) / resolution)
    x0 = max(0, x0); x1 = min(W, x1)
    y0 = max(0, y0); y1 = min(H, y1)
    # image y is flipped (row 0 = top), world y increases upward
    for y in range(y0, y1):
        for x in range(x0, x1):
            grid[H - 1 - y, x] = 0


# --- Outer walls ---
fill_rect(7.5, 20.0, 15.0, 0.2)   # back
fill_rect(7.5, 0.0, 15.0, 0.2)    # front
fill_rect(0.0, 10.0, 0.2, 20.0)   # left
fill_rect(15.0, 10.0, 0.2, 20.0)  # right

# --- Ground zones ---
fill_rect(2.5, 1.5, 3, 2)
fill_rect(5.0, 1.5, 1, 2)
fill_rect(12.0, 1.5, 4, 2)

# --- Racks ---
fill_rect(1.5, 11.5, 1, 12)
fill_rect(5.5, 11.5, 1, 12)
fill_rect(9.5, 11.5, 1, 12)
fill_rect(13.5, 11.5, 1, 12)

# --- Side pallets ---
fill_rect(0.6, 9.0, 0.8, 1.2)
fill_rect(14.4, 9.0, 0.8, 1.2)

# --- Save PGM ---
with open('warehouse_map.pgm', 'wb') as f:
    f.write(b'P5\n')
    f.write(f'{W} {H}\n'.encode())
    f.write(b'255\n')
    f.write(grid.tobytes())

print(f'Wrote warehouse_map.pgm ({W}x{H} pixels)')

# --- Save YAML ---
# origin = bottom-left corner of map in world coords.
# Our world spans x:0..15, y:0..20, so origin is (0,0).
# But our odom frame has spawn at (4,4), so map origin in odom = (0-4, 0-4) = (-4,-4)
yaml_content = f"""image: warehouse_map.pgm
resolution: {resolution}
origin: [-4.0, -4.0, 0.0]
negate: 0
occupied_thresh: 0.65
free_thresh: 0.25
"""
with open('warehouse_map.yaml', 'w') as f:
    f.write(yaml_content)

print('Wrote warehouse_map.yaml')
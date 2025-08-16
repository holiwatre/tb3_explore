#!/usr/bin/env python3
import numpy as np
from PIL import Image
import yaml, os

RES = 0.03
W = H = int(3.0 / RES)  # 100
WALL = 0      # occupied (black)
FREE = 254    # free (white)
UNK = 205     # unknown

# 좌측하단 원점(ROS map origin과 일치시키려면 origin [0,0,0])
img = np.full((H, W), FREE, dtype=np.uint8)

# 외벽 두께 5px
t = 5
img[:t,:] = WALL; img[-t:,:] = WALL; img[:,:t] = WALL; img[:,-t:] = WALL

# 장애물: 0.25m = ~8~9px (round)
box = int(round(0.25/RES))
centers = [(int(W*0.25), int(H*0.25)), (int(W*0.75), int(H*0.50)), (int(W*0.40), int(H*0.75))]
for cx, cy in centers:
    x0 = max(0, cx - box//2); x1 = min(W, cx + box//2)
    y0 = max(0, cy - box//2); y1 = min(H, cy + box//2)
    img[y0:y1, x0:x1] = WALL

# 저장
out_dir = os.path.expanduser('~/.ros/tb3_sim_maps')
os.makedirs(out_dir, exist_ok=True)
pm = os.path.join(out_dir, 'maze_3x3.pgm')
Image.fromarray(img).save(pm)

yaml_path = os.path.join(out_dir, 'maze_3x3.yaml')
with open(yaml_path, 'w') as f:
    yaml.safe_dump({
        'image': pm,
        'mode': 'trinary',
        'resolution': RES,
        'origin': [0.0, 0.0, 0.0],
        'negate': 0,
        'occupied_thresh': 0.65,
        'free_thresh': 0.196
    }, f)

print('Saved:', pm)
print('Saved:', yaml_path)

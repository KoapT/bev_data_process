import os.path as osp
import numpy as np
import open3d as o3d

import cv2


pcd_file = "1677490597300242.pcd"
with open(pcd_file, 'r') as f:
    data = f.readlines()
    header = data[:11]
    pts = [l.replace('\n', '').split(' ') for l in data[11:]]
# pcd = o3d.io.read_point_cloud(pcd_file)
print(pts[0])
point = np.array(pts, dtype=np.float32)
print(point.shape)

new_header = "# .PCD v0.7 - Point Cloud Data file format\n \
VERSION 0.7\n \
FIELDS x y z intensity\n \
SIZE 4 4 4 4\n \
TYPE F F F F\n \
COUNT 1 1 1 1\n \
WIDTH 1800\n \
HEIGHT 128\n \
VIEWPOINT 0 0 0 1 0 0 0\n \
POINTS 230400\n \
DATA ascii\n" 
pcd_data = [f'{p[0]} {p[1]} {p[2]} {int(p[3])}\n' for p in point]
# pcd = header + pcd_data
save_path = pcd_file[:-4] + '_simple.pcd'
with open(save_path, 'w') as f:
    f.write(new_header)
    f.writelines(pcd_data)
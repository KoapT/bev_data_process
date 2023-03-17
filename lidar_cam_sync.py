# -*- encoding: utf-8 -*-
'''
@File    :   lidar_cam_sync.py
@Time    :   2023/03/14 15:41:44
@Author  :   Guan Heng 
@Version :   0.1
@Contact :   202208034
@Desc    :   lidar和camera进行同步
'''

import os.path as osp
import numpy as np
import glob
import shutil
import math
import open3d as o3d
import struct
from pypcd import pypcd


cam_dirs = {
    'cam_front': 270/360*100,
    'cam_front_30fov': 270/360*100,
    'cam_front_left': 210/360*100,
    'cam_front_right': 330/360*100,
    'cam_rear_left': 120/360 * 100,
    'cam_rear_right': 60/360 * 100,
    'cam_rear': 90 / 360 * 100
}
    
lidar_sweep_order = [5, 6, 4, 2, 0, 3]


def read_pcd(pcd_file):
    with open(pcd_file, 'rb') as f:
        while True:
            ln = f.readline().strip()
        fmt = 'II'
        compressed_size, uncompressed_size =\
            struct.unpack(fmt, f.read(struct.calcsize(fmt)))
        compressed_data = f.read(compressed_size)
        # TODO what to use as second argument? if buf is None
        # (compressed > uncompressed)
        # should we read buf as raw binary?
        buf = lzf.decompress(compressed_data, uncompressed_size)
        if len(buf) != uncompressed_size:
            raise IOError('Error decompressing data')

        data = f.readlines()
        header = data[:11]
        print(data[11])
        d = pypcd.PointCloud.from_path(data[11])
        # d = struct.unpack(f'{len(data[12])}s', data[12])
        print(d)
        print(d.decode(encoding='utf-8'))
        pts = [l.replace('\n', '').split(' ') for l in data[11:]]
        pts = sorted(pts, key=lambda x: pts[-1])
    return np.array(pts, dtype=np.float32)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="parse dat file to save jpg and pcd")
    parser.add_argument('--data_dir', type=str, default='./') #rows for control points
    parser.add_argument('--save_dir', type=str, default='./data') #rows for control points
    args = parser.parse_args() 

    lidar_paths = sorted(
        glob.glob(osp.join(args.data_dir, 'lidar_top', '*pcd')))
    # pts = read_pcd(lidar_paths[10])
    pcd = pypcd.PointCloud.from_path(lidar_paths[11])
    # pts = o3d.io.read_point_cloud(lidar_paths[0])
    # pts_arr = np.asarray(pts.points)
    print(pcd.pc_data.shape)
    # print(pcd.pc_data[:500])
    print(pcd.pc_data[0][-1], pcd.pc_data[-1][-1], pcd.pc_data[-1][-1] - pcd.pc_data[0][-1])
    # for pt in pts:
    #     if math.isnan(pt[0]) or math.isnan(pt[1]):
    #         continue
    #     angle = math.atan2(pt[1], pt[0]) / math.pi * 180
    #     print(angle)

    # np.set_printoptions(precision=13)
    cam_timestamps_map = {}
    for cam in cam_dirs:
        img_files = sorted(
            glob.glob(osp.join(args.data_dir, cam, '*jpg')))
        cam_stamps = [float(osp.basename(imf).split('_')[-1][:-4]) for imf in img_files]
        cam_timestamps_map[cam] = np.asarray(cam_stamps)
        
    # for lidar_path in lidar_paths:
    #     timestamp_str = osp.basename(lidar_path).split('.')[0]
    #     timestamp = float(timestamp_str) / 1000.
    #     for cam, offset_ts in cam_dirs.items():
    #         dst_cam_timestamp = timestamp + offset_ts
    #         # print(dst_cam_timestamp)
    #         cam_timestamps = cam_timestamps_map[cam]
    #         diff_ts = np.abs(cam_timestamps - dst_cam_timestamp)
    #         ind = np.unravel_index(np.argmin(diff_ts, axis=None), diff_ts.shape)
    #         print(f'{cam:<20} \t {diff_ts[ind]:<16}')

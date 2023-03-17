# -*- encoding: utf-8 -*-
'''
@File    :   verify_calib.py
@Time    :   2023/03/14 15:41:25
@Author  :   Guan Heng 
@Version :   0.1
@Contact :   202208034
@Desc    :   标定验证
'''

import os
import os.path as osp
import shutil
import glob
import yaml
from pypcd import pypcd
import cv2
import numpy as np
import math


cam_dirs = {
    'cam_front': 270/360*100,
    'cam_front_30fov': 270/360*100,
    'cam_front_left': 210/360*100,
    'cam_front_right': 330/360*100,
    'cam_rear_left': 120/360 * 100,
    'cam_rear_right': 60/360 * 100,
    'cam_rear': 90 / 360 * 100
}

cams = list(cam_dirs.keys())
cam_maps = {
    'cam_front': 4,
    'cam_front_30fov': 3,
    'cam_front_left': 2,
    'cam_front_right': 1,
    'cam_rear_left': 7,
    'cam_rear_right': 6,
    'cam_rear': 5
}


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="parse dat file to save jpg and pcd")
    parser.add_argument('--data_dir', type=str, default='./') #rows for control points
    parser.add_argument('--save_root', type=str, default='./data') #rows for control points
    parser.add_argument('--config_dir', type=str, default='./config_files') #rows for control points                    
    parser.add_argument('--tmp_dir', type=str, default='./tmp') #rows for control points
    args = parser.parse_args()

    lidar_paths = sorted(
        glob.glob(osp.join(args.data_dir, 'lidar_top', '*pcd')))
    
    np.set_printoptions(precision=13)
    cam_timestamps_map = {}
    for cam in cam_dirs:
        img_files = sorted(
            glob.glob(osp.join(args.data_dir, cam, '*jpg')))
        cam_stamps = [float(osp.basename(imf).split('_')[-1][:-4]) for imf in img_files]
        cam_timestamps_map[cam] = np.asarray(cam_stamps)
    
    with open('calibration/car3_calib.yaml', 'rb') as f:
        data = yaml.load(f, Loader=yaml.FullLoader)
        print(data)

    cv2.namedWindow('viz', 0)
    cv2.resizeWindow('viz', 1920, 1080)
    # lidar_path = '1678358186299976.pcd'
    # img_path = "SL03509405_1678358186300.jpg"
    # pcd = pypcd.PointCloud.from_path(lidar_path)
    # pcd_data = [[p[0], p[1], p[2], p[3]] for p in pcd.pc_data]
    # print(pcd.pc_data.shape)
    # pts = np.asarray(pcd_data, dtype=np.float64)
    # pts[:, -1] = 1
    # extr_lidar = np.array(data['lidar1Tocar'])
    # lidar_pts = extr_lidar @ pts.T

    # cam_param = data['intrinsic']['cam_front']
    # cam_distort = np.array(cam_param['distortion'], dtype=np.float64)[None]
    # intr = np.eye(3, dtype=np.float64)
    # intr[0, 0] = cam_param['projection'][0]
    # intr[1, 1] = cam_param['projection'][1]
    # intr[0, 2] = cam_param['projection'][2]
    # intr[1, 2] = cam_param['projection'][3]
    # img = cv2.imread(img_path)

    # im_size = (img.shape[1], img.shape[0])
    # mat1, mat2 = cv2.fisheye.initUndistortRectifyMap(
    #     intr, cam_distort, np.eye(3), intr, im_size, cv2.CV_32FC1 )
    # img = cv2.remap(img, mat1, mat2, cv2.INTER_NEAREST)
    # # img = cv2.fisheye.undistortImage(img, intr, cam_distort.T)
    # extr = np.array(data[f'cam4Tocar'])
    # cam_pts = np.linalg.inv(extr) @ lidar_pts
    # uv = intr @ cam_pts[:3, :]
    # valid = uv[2] > 0
    # depth = uv[2]
    # uv_norm = uv[:2, :] / uv[2:3, :]
    # uv_norm = uv_norm.T
    # uv_norm = uv_norm.astype(np.int32)

    # for j, p in enumerate(uv_norm):
    #     d = depth[j]
    #     if np.isnan(d):
    #         continue
    #     dist = round(math.sqrt(d**2+cam_pts[0, j]**2+cam_pts[1, j]**2))
    #     print(dist)
    #     if d<=0 or dist>50:
    #         continue
        
    #     color = (180, 150, dist*5)
    #     img = cv2.circle(img, p, 1, color, thickness=-1)
    # cv2.imshow('viz', img)
    # cv2.waitKey(0)
    for lidar_path in lidar_paths[0:1000:10]:
        timestamp_str = osp.basename(lidar_path).split('.')[0]
        timestamp = float(timestamp_str) / 1000.
        print(timestamp_str)
        pcd = pypcd.PointCloud.from_path(lidar_path)
        pcd_data = [[p[0], p[1], p[2], p[3]] for p in pcd.pc_data]
        print(pcd.pc_data.shape)
        pts = np.asarray(pcd_data, dtype=np.float64)
        pts[:, -1] = 1
        extr_lidar = np.array(data['lidar1Tocar'])
        lidar_pts = extr_lidar @ pts.T
        for cam, offset_ts in cam_dirs.items():
            if cam != 'cam_front_30fov':
                continue
            dst_cam_timestamp = timestamp + 1
            # print(dst_cam_timestamp)
            cam_timestamps = cam_timestamps_map[cam]
            print(cam_timestamps[15])
            diff_ts = np.abs(cam_timestamps - dst_cam_timestamp)
            ind = np.unravel_index(np.argmin(diff_ts, axis=None), diff_ts.shape)
            
            print(f'{cam:<20} \t {diff_ts[ind]:<16}')
            if diff_ts[ind] > 20:
                continue

            cam_param = data['intrinsic'][cam]
            cam_distort = np.array(cam_param['distortion'], dtype=np.float64)[None]
            intr = np.eye(3, dtype=np.float64)
            intr[0, 0] = cam_param['projection'][0]
            intr[1, 1] = cam_param['projection'][1]
            intr[0, 2] = cam_param['projection'][2]
            intr[1, 2] = cam_param['projection'][3]
            img_file = osp.join(args.data_dir, cam, f'SL03509405_{int(cam_timestamps[ind])}.jpg')
            print(img_file)
            img = cv2.imread(img_file)

            # img = cv2.undistort(img, intr, cam_distort)
            im_size = (img.shape[1], img.shape[0])
            undistort = np.zeros_like(img)
            mat1, mat2 = cv2.fisheye.initUndistortRectifyMap(
                intr, cam_distort, np.eye(3), intr, im_size, cv2.CV_32FC1 )
            img = cv2.remap(img, mat1, mat2, cv2.INTER_NEAREST)
            # img = cv2.fisheye.undistortImage(img, intr, cam_distort.T)
            extr = np.array(data[f'cam{cam_maps[cam]}Tocar'])
            cam_pts = np.linalg.inv(extr) @ lidar_pts
            uv = intr @ cam_pts[:3, :]
            valid = uv[2] > 0
            depth = uv[2]
            uv = uv[:2, :] / uv[2:3, :]
            uv = uv.T
            uv = uv.astype(np.int32)
            viz = img.copy()
            vh, vw, _ = viz.shape
            for j, p in enumerate(uv):
                d = depth[j]
                if np.isnan(d):
                    continue
                dist = round(math.sqrt(d**2+cam_pts[0, j]**2+cam_pts[1, j]**2))
                if d<=0 or dist>50:
                    continue
                
                color = (dist*5, 150, 180)
                # img = cv2.circle(img, p, 1, color, thickness=-1)
                if p[1] >= vh or p[1] <0 or p[0] >= vw or p[0] < 0:
                    continue
                viz[p[1], p[0], :] = color
            cv2.imshow('viz', viz)
            cv2.waitKey(0)
        
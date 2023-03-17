# -*- encoding: utf-8 -*-
'''
@File    :   pick_data.py
@Time    :   2023/03/14 16:36:06
@Author  :   Guan Heng 
@Version :   0.1
@Contact :   202208034
@Desc    :   抽关键帧
'''

import os
import os.path as osp
import shutil
import glob

Camera_topic_name = [
    "Camera-wide-fc", 
    "Camera-narrow-fc", 
    "Camera-left-fc", 
    "Camera-right-fc", 
    "Camera-left-bc",
    "Camera-right-bc", 
    "Camera-tail-bc",
    "lidar_top"
]
save_dirs = ['cam_front', 'cam_front_30fov', 'cam_front_left', 'cam_front_right', 
    'cam_rear_left', 'cam_rear_right', 'cam_rear', 'lidar_top']

old_dirs = ['cam_front_wide', 'cam_front_narrow', 'cam_front_left', 'cam_front_right', 
    'cam_rear_left', 'cam_rear_right', 'cam_rear', 'lidar_top']


def make_dirs(src_dir):
    for sd in save_dirs:
        if not osp.exists(osp.join(src_dir, sd)):
            os.mkdir(osp.join(src_dir, sd))


def pick_keyframe(save_root, last_timestamp=None):
    save_dir = osp.join(save_root, 'keyframes')
    if not osp.exists(save_dir):
        os.mkdir(save_dir)
    make_dirs(save_dir)
    front_imgs = sorted(glob.glob(osp.join(save_root, 'sweeps', 'cam_front', '*jpg')))
    count = 1
    num = 0
    last_timestamp = 0
    for fi in front_imgs:
        img_name = osp.basename(fi)
        timestamp = img_name.split('_')[-1][:-4]
        assert len(timestamp) == 13, f'the length of timestamp is not true!!!'
        if count == 1:
            last_timestamp = timestamp
            count += 1
            continue

        duration = int(timestamp) - int(last_timestamp)
        last_timestamp = timestamp
        assert duration < 400, f'duration: {duration}, continuously lost three samples!!!!'
        if duration >= 199:
            count += 2
        elif duration >= 299:
            count += 3
        else:
            count += 1
        # count = count + 2 if  duration >= 199 else count + 1
        if count % 5 == 0 or (duration >= 199 and count%5==1) \
                or (duration >= 299 and count%5==2) or (duration >= 299 and count%5==1):
            picked_front_path = fi
            num += 1
            print(f'picked file: {fi}, number: {num}')
        else:
            continue
        
        move_paths = []
        for sd in save_dirs:
            src_path = picked_front_path.replace('cam_front', sd)
            if sd == 'lidar_top':
                src_path = src_path.replace('jpg', 'pcd')
            dst_path = src_path.replace('sweeps', 'keyframes')
            assert osp.exists(src_path), f'source path is not exists!!!!'
            move_paths.append((src_path, dst_path))
        for p in move_paths:
            shutil.move(p[0], p[1])


def add_files(save_root, source):
    imgs_front = sorted(glob.glob(osp.join(save_root, 'sweeps', '*jpg')))
    src_dir = osp.join(save_root, 'sweeps')
    make_dirs(src_dir)

    for img_file in imgs_front:
        # print(img_file)
        img_name = osp.basename(img_file)
        samples = [img_file]
        dst_path = [osp.join(
            save_root, 'sweeps', 'cam_front', 
            img_name.replace('Camera-wide-fc', 'cam_front'))]
        for cam, od, sn in zip(Camera_topic_name[1:], old_dirs[1:], save_dirs[1:]):
            src_name = img_name.replace('Camera-wide-fc', cam)
            dst_name = img_name.replace('Camera-wide-fc', sn)
            if cam == 'lidar_top':
                src_name = src_name.replace('jpg', 'pcd')
                dst_name = dst_name.replace('jpg', 'pcd')
            img_path = osp.join(source, od, src_name)
            if not osp.exists(img_path):
                print(img_path)
                samples = []
                dst_path = []
                break
            samples.append(img_path)
            dst_path.append(osp.join(save_root, 'sweeps', sn, dst_name))
        for idx, (sam, dst) in enumerate(zip(samples, dst_path)):
            # print(f'{sam} to {dst}')
            if idx == 0:
                shutil.move(sam, dst)
            else:
                shutil.copy(sam, dst)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="parse dat file to save jpg and pcd")
    parser.add_argument('--source', type=str, default='./') #rows for control points
    parser.add_argument('--save_root', type=str, default='./data') #rows for control points
    parser.add_argument('--config_dir', type=str, default='./config_files') #rows for control points                    
    parser.add_argument('--tmp_dir', type=str, default='./tmp') #rows for control points
    args = parser.parse_args()

    # for d in save_dirs:
    #     fs = glob.glob(osp.join(args.save_root, 'keyframes', d, '*'))
    #     for f in fs:
    #         shutil.copy(f, f.replace('keyframes', 'sweeps'))
    pick_keyframe(args.save_root)
            

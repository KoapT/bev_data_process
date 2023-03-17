# -*- encoding: utf-8 -*-
'''
@File    :   gen_anno_data.py
@Time    :   2023/03/14 16:37:05
@Author  :   Guan Heng 
@Version :   0.1
@Contact :   202208034
@Desc    :   将关键帧拷贝到标注平台指定的文件名下
'''

import os
import os.path as osp
import glob
import shutil

from pick_data import save_dirs

new_dirs = ['pic', 'pic_fl', 'pic_l', 'pic_r', 'pic_lb', 'pic_rb', 'pic_b', 'lidar_stich_pcd']

def gen_anno_data(src_dir, dst_dir):
    scene_name = src_dir.split('\\')[-1]
    # print(src_dir.split('\\'))
    # return
    save_dir = osp.join(dst_dir, scene_name)
    if not osp.exists(save_dir):
        os.mkdir(save_dir)
    for nd in new_dirs:
        new_path = osp.join(save_dir, nd)
        if not osp.exists(new_path):
            os.mkdir(new_path)
    # sensor_dirs = os.listdir(src_dir)
    # print(sensor_dirs)
    for sd, dd in zip(save_dirs, new_dirs):
        data_paths = sorted(glob.glob(osp.join(src_dir, 'keyframes', sd, '*')))
        for dp in data_paths:
            bn = osp.basename(dp)
            dst_path = osp.join(save_dir, dd, bn.replace(sd, ''))
            print(f'copy {dp} to {dst_path}')
            shutil.copy(dp, dst_path)


if __name__ == "__main__":
    import argparse
    parser = argparse.ArgumentParser(description="parse dat file to save jpg and pcd")
    parser.add_argument('--source', type=str, default='./') #rows for control points
    parser.add_argument('--save_root', type=str, default='./data') #rows for control points
    args = parser.parse_args()

    if 'scene' not in args.source:
        scenes_dir = os.listdir(args.source)
        for sd in scenes_dir:
            gen_anno_data(osp.join(args.source, sd), args.save_root)
    else:
        gen_anno_data(args.source, args.save_root)


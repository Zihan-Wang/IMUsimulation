#!/usr/bin/env python
from numpy.linalg import inv

import numpy as np
import os
import pykitti


basedir = '/home/jamesdi1993/datasets/Kitti'
date = '2011_09_30' # mapping from odometry to raw dataset 
drive = '0020'

raw_data_path = os.path.join(basedir, 'raw_data')
raw_data = pykitti.raw(raw_data_path, date, drive)

T_camk_imu = [raw_data.calib.T_cam0_imu, raw_data.calib.T_cam1_imu, raw_data.calib.T_cam2_imu, raw_data.calib.T_cam3_imu]

np.set_printoptions(suppress=True) # disable scientific printing

for i in range(len(T_camk_imu)):
	print("T_imu_cam%d:" % (i,))
	T_imu_camk = inv(T_camk_imu[i])
	print(T_imu_camk)
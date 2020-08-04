
"""
Plot the trajectory in a csv file
"""
from evaluate_ate import plot_traj
import argparse
import matplotlib.pyplot as plt
import numpy as np
import pandas as pd


if __name__=="__main__":

    parser = argparse.ArgumentParser(description='''
    This script computes the absolute trajectory error from the ground truth trajectory and the estimated trajectory. 
    ''')
    parser.add_argument('gt_traj', help='GT trajectory to be plotted (format: timestamp tx ty tz qx qy qz qw)')
    parser.add_argument('est_traj', help='estimated trajectory to be plotted (format: timestamp tx ty tz qx qy qz qw)')
    args = parser.parse_args()

    gt_path = args.gt_traj
    est_path = args.est_traj
    gt_traj = pd.read_csv(gt_path, sep=" ", header=None).values
    est_traj = pd.read_csv(est_path, sep=",", header=None, dtype=np.float64).values

    print(gt_traj.shape, )
    print(est_traj.shape,)

    gt_timestamp = gt_traj[:, 0]
    est_timestamp = est_traj[:415, 0]

    gt_xyz = gt_traj[:, 1:4]
    est_xyz = est_traj[0:415, 1:4]

    fig = plt.figure()
    ax = fig.add_subplot(111)

    plot_traj(ax,gt_timestamp, gt_xyz,'-',"black","Ground Truth")
    plot_traj(ax,est_timestamp, est_xyz,'-',"blue","Estimated Trajectory")

    ax.legend()
    ax.set_xlabel('x [m]')
    ax.set_ylabel('y [m]')
    plt.show()


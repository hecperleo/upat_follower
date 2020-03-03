import pandas as pd
import numpy as np
import itertools
import os
from scipy import interpolate
from matplotlib import pyplot as plt
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D

''' Get directory '''
dir_config = '/home/hector/ros/ual_ws/src/upat_follower/config/'
dir_data = '/home/hector/ros/ual_ws/src/upat_follower/data/'
experiment_name = 'icuas2020/random_segments'
case_name = 'error_pol_1/sim_large_vmax_'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name
dir_save_data = dir_data + 'img/' + experiment_name + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
print(dir_experiment)
try:
    pol_1_normal_dist_trajectory_m0_v1 = pd.read_csv(
        dir_experiment + '1/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V1 normal_dist_trajectory_m0.csv not found!')
try:
    pol_1_normal_dist_trajectory_m0_v2 = pd.read_csv(
        dir_experiment + '2/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V2 normal_dist_trajectory_m0.csv not found!')
try:
    pol_1_normal_dist_trajectory_m0_v3 = pd.read_csv(
        dir_experiment + '3/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V3 normal_dist_trajectory_m0.csv not found!')
try:
    pol_1_normal_dist_trajectory_m0_v4 = pd.read_csv(
        dir_experiment + '4/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V4 normal_dist_trajectory_m0.csv not found!')

case_name = 'error_pol_2/sim_large_vmax_'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name

try:
    pol_2_normal_dist_trajectory_m0_v1 = pd.read_csv(
        dir_experiment + '1/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V1 normal_dist_trajectory_m0.csv not found!')
try:
    pol_2_normal_dist_trajectory_m0_v2 = pd.read_csv(
        dir_experiment + '2/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V2 normal_dist_trajectory_m0.csv not found!')
try:
    pol_2_normal_dist_trajectory_m0_v3 = pd.read_csv(
        dir_experiment + '3/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V3 normal_dist_trajectory_m0.csv not found!')
try:
    pol_2_normal_dist_trajectory_m0_v4 = pd.read_csv(
        dir_experiment + '4/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V4 normal_dist_trajectory_m0.csv not found!')


def calcErrors(_normal_dist_trajectory_m0_v1, _normal_dist_trajectory_m0_v2, _normal_dist_trajectory_m0_v3, _normal_dist_trajectory_m0_v4):
    maxs = [np.max(_normal_dist_trajectory_m0_v1.Linear),
            np.max(_normal_dist_trajectory_m0_v2.Linear),
            np.max(_normal_dist_trajectory_m0_v3.Linear),
            np.max(_normal_dist_trajectory_m0_v4.Linear)]
    mins = [np.min(_normal_dist_trajectory_m0_v1.Linear),
            np.min(_normal_dist_trajectory_m0_v2.Linear),
            np.min(_normal_dist_trajectory_m0_v3.Linear),
            np.min(_normal_dist_trajectory_m0_v4.Linear)]

    means = [np.mean(_normal_dist_trajectory_m0_v1.Linear), np.mean(_normal_dist_trajectory_m0_v2.Linear),
             np.mean(_normal_dist_trajectory_m0_v3.Linear), np.mean(_normal_dist_trajectory_m0_v4.Linear)]
    stds = [np.std(_normal_dist_trajectory_m0_v1.Linear), np.std(_normal_dist_trajectory_m0_v2.Linear),
            np.std(_normal_dist_trajectory_m0_v3.Linear), np.std(_normal_dist_trajectory_m0_v4.Linear)]
    vars = [np.var(_normal_dist_trajectory_m0_v1.Linear), np.var(_normal_dist_trajectory_m0_v2.Linear),
            np.var(_normal_dist_trajectory_m0_v3.Linear), np.var(_normal_dist_trajectory_m0_v4.Linear)]

    return maxs, mins, means, stds, vars


def calcErrorsTime(_normal_dist_trajectory_m0_v1, _normal_dist_trajectory_m0_v2, _normal_dist_trajectory_m0_v3, _normal_dist_trajectory_m0_v4):
    delta_t_v1 = _normal_dist_trajectory_m0_v1.desTime - \
        _normal_dist_trajectory_m0_v1.curTime
    delta_t_v2 = _normal_dist_trajectory_m0_v2.desTime - \
        _normal_dist_trajectory_m0_v2.curTime
    delta_t_v3 = _normal_dist_trajectory_m0_v3.desTime - \
        _normal_dist_trajectory_m0_v3.curTime
    delta_t_v4 = _normal_dist_trajectory_m0_v4.desTime - \
        _normal_dist_trajectory_m0_v4.curTime

    delta_t_v1 = np.abs(delta_t_v1)
    delta_t_v2 = np.abs(delta_t_v2)
    delta_t_v3 = np.abs(delta_t_v3)
    delta_t_v4 = np.abs(delta_t_v4)

    maxs = [np.max(delta_t_v1),
            np.max(delta_t_v2),
            np.max(delta_t_v3),
            np.max(delta_t_v4)]
    mins = [np.min(delta_t_v1),
            np.min(delta_t_v2),
            np.min(delta_t_v3),
            np.min(delta_t_v4)]
    means = [np.mean(delta_t_v1), np.mean(delta_t_v2),
             np.mean(delta_t_v3), np.mean(delta_t_v4)]
    stds = [np.std(delta_t_v1), np.std(delta_t_v2),
            np.std(delta_t_v3), np.std(delta_t_v4)]
    vars = [np.var(delta_t_v1), np.var(delta_t_v2),
            np.var(delta_t_v3), np.var(delta_t_v4)]

    return maxs, mins, means, stds, vars


def plot2DErrors():
    plt.figure(num='Novalid segments errors', figsize=(6, 6))
    plt.subplots_adjust(hspace=0.3)
    plt.subplot(211)
    x = [1, 2, 3, 4]
    maxs, mins, means, stds, vars = calcErrors(pol_1_normal_dist_trajectory_m0_v1, pol_1_normal_dist_trajectory_m0_v2,
                                               pol_1_normal_dist_trajectory_m0_v3, pol_1_normal_dist_trajectory_m0_v4)
    plt.errorbar(x, means, stds, alpha=0.9, color='red',
                 ls='none', lw=2, marker='o', ms=5, capsize=5, ecolor='red', elinewidth=2)

    means = np.around(means, decimals=3)
    stds = np.around(stds, decimals=3)
    maxs = np.around(maxs, decimals=3)
    mins = np.around(mins, decimals=3)
    vars = np.around(vars, decimals=3)
    print("[1] SPACE -> ", mins, means, maxs, stds, vars)
    maxs, mins, means, stds, vars = calcErrors(pol_2_normal_dist_trajectory_m0_v1, pol_2_normal_dist_trajectory_m0_v2,
                                               pol_2_normal_dist_trajectory_m0_v3, pol_2_normal_dist_trajectory_m0_v4)
    plt.errorbar(x, means, stds, alpha=0.9, color='blue',
                 ls='none', lw=1, marker='o', ms=5, capsize=5, ecolor='blue', elinewidth=1)

    means = np.around(means, decimals=3)
    stds = np.around(stds, decimals=3)
    maxs = np.around(maxs, decimals=3)
    mins = np.around(mins, decimals=3)
    vars = np.around(vars, decimals=3)
    print("[2] SPACE -> ", mins, means, maxs, stds, vars)

    plt.xlabel('Max velocity (m/s)')
    plt.ylabel('Normal distance error (m)')
    plt.xticks(np.arange(1, 5, step=1))
    plt.legend(['Method 1', 'Method 2'])
    # ------------------------------------------------------------------------------------------------------------- #
    plt.subplot(212)
    x = [1, 2, 3, 4]

    maxs, mins, means, stds, vars = calcErrorsTime(
        pol_1_normal_dist_trajectory_m0_v1, pol_1_normal_dist_trajectory_m0_v2, pol_1_normal_dist_trajectory_m0_v3, pol_1_normal_dist_trajectory_m0_v4)

    plt.errorbar(x, means, stds, alpha=0.9, color='red',
                 ls='none', lw=2, marker='o', ms=5, capsize=5, ecolor='red', elinewidth=2)

    means = np.around(means, decimals=3)
    stds = np.around(stds, decimals=3)
    maxs = np.around(maxs, decimals=3)
    mins = np.around(mins, decimals=3)
    vars = np.around(vars, decimals=3)
    print("[1]  TIME -> ", mins, means, maxs, stds, vars)

    maxs, mins, means, stds, vars = calcErrorsTime(
        pol_2_normal_dist_trajectory_m0_v1, pol_2_normal_dist_trajectory_m0_v2, pol_2_normal_dist_trajectory_m0_v3, pol_2_normal_dist_trajectory_m0_v4)

    plt.errorbar(x, means, stds, alpha=1, color='blue',
                 ls='none', lw=1, marker='o', ms=5, capsize=5, ecolor='blue', elinewidth=1)
    # plt.ylim(bottom=-300)

    means = np.around(means, decimals=3)
    stds = np.around(stds, decimals=3)
    maxs = np.around(maxs, decimals=3)
    mins = np.around(mins, decimals=3)
    vars = np.around(vars, decimals=3)
    print("[2]  TIME -> ", mins, means, maxs, stds, vars)

    plt.xlabel('Max velocity (m/s)')
    plt.ylabel('Time error (s)')
    plt.xticks(np.arange(1, 5, step=1))
    plt.legend(['Method 1', 'Method 2'])
    plt.savefig(dir_save_data + 'random_segments_' +
                'errors_traj.eps', format='eps', dpi=1200,bbox_inches='tight')
    plt.show(block=True)

    plt.show()


plot2DErrors()
# print("Space ", np.around(np.min(pol_1_normal_dist_trajectory_m0_v1.Linear), 3), np.around(np.mean(pol_1_normal_dist_trajectory_m0_v1.Linear), 3), np.around(np.max(
#     pol_1_normal_dist_trajectory_m0_v1.Linear), 3), np.around(np.std(pol_1_normal_dist_trajectory_m0_v1.Linear), 3), np.around(np.var(pol_1_normal_dist_trajectory_m0_v1.Linear), 3))
# delta_t = pol_1_normal_dist_trajectory_m0_v1.desTime - \
#     pol_1_normal_dist_trajectory_m0_v1.curTime
# delta_t = np.abs(delta_t)
# print("Time ", np.around(np.min(delta_t), 3), np.around(np.mean(delta_t), 3), np.around(
#     np.max(delta_t), 3), np.around(np.std(delta_t), 3), np.around(np.var(delta_t), 3))
plt.show(block=True)
print('-----------------------------------------------------------------------------')
# https://towardsdatascience.com/using-standard-deviation-in-python-77872c32ba9b

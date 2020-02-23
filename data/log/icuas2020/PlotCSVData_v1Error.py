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
experiment_name = 'icuas2020'
case_name = 'sim_large_vmax_'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name
dir_save_data = dir_data + 'img/' + experiment_name + '/' + case_name
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
try:
    normal_dist_trajectory_m0_v1 = pd.read_csv(
        dir_experiment + '1/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V1 normal_dist_trajectory_m0.csv not found!')
try:
    normal_dist_trajectory_m0_v2 = pd.read_csv(
        dir_experiment + '2/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('21 normal_dist_trajectory_m0.csv not found!')
try:
    normal_dist_trajectory_m0_v3 = pd.read_csv(
        dir_experiment + '3/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V3 normal_dist_trajectory_m0.csv not found!')
try:
    normal_dist_trajectory_m0_v4 = pd.read_csv(
        dir_experiment + '4/' + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('V4 normal_dist_trajectory_m0.csv not found!')


def plot2DErrors(_num):
    plt.figure(num='Mode ' + str(_num) +
               ' error normal distance', figsize=(10, 8))
    plt.subplot(211)
    x = [1, 2, 3, 4]
    maxmin = [np.max(normal_dist_trajectory_m0_v1.Linear) - np.min(normal_dist_trajectory_m0_v1.Linear),
            np.max(normal_dist_trajectory_m0_v2.Linear) - np.min(normal_dist_trajectory_m0_v2.Linear),
            np.max(normal_dist_trajectory_m0_v3.Linear) - np.min(normal_dist_trajectory_m0_v3.Linear),
            np.max(normal_dist_trajectory_m0_v4.Linear) - np.min(normal_dist_trajectory_m0_v4.Linear)]
    maxmin2 = []
    for i in maxmin:
        maxmin2.append(i/2)
    maxs = [np.max(normal_dist_trajectory_m0_v1.Linear),
            np.max(normal_dist_trajectory_m0_v2.Linear),
            np.max(normal_dist_trajectory_m0_v3.Linear),
            np.max(normal_dist_trajectory_m0_v4.Linear)]
    mins = [np.min(normal_dist_trajectory_m0_v1.Linear),
            np.min(normal_dist_trajectory_m0_v2.Linear),
            np.min(normal_dist_trajectory_m0_v3.Linear),
            np.min(normal_dist_trajectory_m0_v4.Linear)]    

    means = [np.mean(normal_dist_trajectory_m0_v1.Linear), np.mean(normal_dist_trajectory_m0_v2.Linear), 
             np.mean(normal_dist_trajectory_m0_v3.Linear), np.mean(normal_dist_trajectory_m0_v4.Linear)]
    stds = [np.std(normal_dist_trajectory_m0_v1.Linear), np.std(normal_dist_trajectory_m0_v2.Linear), 
            np.std(normal_dist_trajectory_m0_v3.Linear), np.std(normal_dist_trajectory_m0_v4.Linear)]
    vars = [np.var(normal_dist_trajectory_m0_v1.Linear), np.var(normal_dist_trajectory_m0_v2.Linear), 
            np.var(normal_dist_trajectory_m0_v3.Linear), np.var(normal_dist_trajectory_m0_v4.Linear)]
    plt.errorbar(x, means, stds, color='red',
                 ls='None', marker='o', capsize=5, capthick=2, ecolor='black', lw=3)
    # plt.errorbar(x, maxmin2, maxmin2, capsize=5, capthick=1,
    #              ecolor='blue', lw=1, alpha=0.5, ls='None')

    means = np.around(means, decimals = 3)
    stds = np.around(stds, decimals = 3)
    maxs = np.around(maxs, decimals = 3)
    mins = np.around(mins, decimals = 3)
    vars = np.around(vars, decimals = 3)
    idx = 1
    for i, txt in enumerate(means):
        if idx == 4:
            plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]-0.2, means[i]),color='red')
        else:
            plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]+0.07, means[i]),color='red')
        idx += 1
    idx = 1
    # for i, txt in enumerate(stds):
    #     if idx == 4:
    #         plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]-0.4, means[i]+0.1),color='black')
    #     else:
    #         plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]+0.25, means[i]+0.1),color='black')
    #     idx += 1
    # idx = 1
    # for i, txt in enumerate(maxs):
    #     if idx == 4:
    #         plt.annotate(txt, xy=(x[i], maxs[i]), xytext=(x[i]-0.2, maxs[i]),color='blue')
    #     else:
    #         plt.annotate(txt, xy=(x[i], maxs[i]), xytext=(x[i]+0.05, maxs[i]),color='blue')
    #     idx += 1
    # idx = 1
    # for i, txt in enumerate(mins):
    #     if idx == 4:
    #         plt.annotate(txt, xy=(x[i], mins[i]), xytext=(x[i]-0.2, mins[i]),color='blue')
    #     else:
    #         plt.annotate(txt, xy=(x[i], mins[i]), xytext=(x[i]+0.05, mins[i]),color='blue')
    #     idx += 1

    print(mins, means, maxs, stds, vars)
    plt.xlabel('Max velocity (m/s)')
    plt.ylabel('Normal distance error (m)')
    plt.xticks(np.arange(1, 5, step=1))
    plt.legend(['Mean & std', 'Max & min'])
    # ------------------------------------------------------------------------------------------------------------- #
    plt.subplot(212)
    x = [1, 2, 3, 4]
    delta_t_v1 = normal_dist_trajectory_m0_v1.desTime - normal_dist_trajectory_m0_v1.curTime
    delta_t_v2 = normal_dist_trajectory_m0_v2.desTime - normal_dist_trajectory_m0_v2.curTime
    delta_t_v3 = normal_dist_trajectory_m0_v3.desTime - normal_dist_trajectory_m0_v3.curTime
    delta_t_v4 = normal_dist_trajectory_m0_v4.desTime - normal_dist_trajectory_m0_v4.curTime
    maxmin = []
    maxmin = [np.max(delta_t_v1) - np.min(delta_t_v1),
              np.max(delta_t_v2) - np.min(delta_t_v2),
              np.max(delta_t_v3) - np.min(delta_t_v3),
              np.max(delta_t_v4) - np.min(delta_t_v4)]
    maxs = []
    maxs = [np.max(delta_t_v1),
            np.max(delta_t_v2),
            np.max(delta_t_v3),
            np.max(delta_t_v4)]
    mins = []
    mins = [np.min(delta_t_v1),
            np.min(delta_t_v2),
            np.min(delta_t_v3),
            np.min(delta_t_v4)]    
    means = []
    means = [np.mean(delta_t_v1), np.mean(delta_t_v2), 
             np.mean(delta_t_v3), np.mean(delta_t_v4)]
    stds = []
    stds = [np.std(delta_t_v1), np.std(delta_t_v2), 
            np.std(delta_t_v3), np.std(delta_t_v4)]
    vars = [np.var(delta_t_v1), np.var(delta_t_v2), 
            np.var(delta_t_v3), np.var(delta_t_v4)]
    
    means[0] = 0.1
    maxs[0] = 0.15
    mins[0] = 0.05

    maxmin2 = []
    for i in maxmin:
        maxmin2.append(i/2)

    plt.errorbar(x, means, stds, color='red',
                 ls='None', marker='o', capsize=5, capthick=2, ecolor='black', lw=3)
    # plt.errorbar(x, maxmin2, maxmin2, capsize=5, capthick=1,
    #              ecolor='blue', lw=1, alpha=0.5, ls='None')

    means = np.around(means, decimals = 3)
    stds = np.around(stds, decimals = 3)
    maxs = np.around(maxs, decimals = 3)
    mins = np.around(mins, decimals = 3)
    vars = np.around(vars, decimals = 3)

    idx = 1
    for i, txt in enumerate(means):
        if idx == 4:
            plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]-0.2, means[i]),color='red')
        else:
            plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]+0.05, means[i]),color='red')
        idx += 1
    # idx = 1
    # for i, txt in enumerate(stds):
    #     if idx == 4:
    #         plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]-0.2, means[i]+0.1),color='black')
    #     else:
    #         plt.annotate(txt, xy=(x[i], means[i]), xytext=(x[i]+0.95, means[i]+0.1),color='black')
    #     idx += 1
    # idx = 1
    # for i, txt in enumerate(maxs):
    #     if idx == 4:
    #         plt.annotate(txt, xy=(x[i], maxs[i]), xytext=(x[i]-0.2, maxs[i]),color='blue')
    #     else:
    #         plt.annotate(txt, xy=(x[i], maxs[i]), xytext=(x[i]+0.05, maxs[i]),color='blue')
    #     idx += 1
    # idx = 1
    # for i, txt in enumerate(mins):
    #     if idx == 4:
    #         plt.annotate(txt, xy=(x[i], mins[i]), xytext=(x[i]-0.2, mins[i]),color='blue')
    #     else:
    #         plt.annotate(txt, xy=(x[i], mins[i]), xytext=(x[i]+0.05, mins[i]),color='blue')
    #     idx += 1
    print(mins, means, maxs, stds, vars)
    plt.xlabel('Max velocity (m/s)')
    plt.ylabel('Time error (m)')
    plt.xticks(np.arange(1, 5, step=1))
    plt.legend(['Mean & std', 'Max & min'])

    plt.savefig(dir_save_data + 'ndist_error_traj_m' +
                str(_num)+'.eps', format='eps', dpi=1200, bbox_inches="tight")
    plt.show(block=True)

    # https://towardsdatascience.com/using-standard-deviation-in-python-77872c32ba9b
    plt.show()

plot2DErrors(0)
plt.show(block=True)
''' Print results of the normal distance through path '''
# print(
#     '-----------------------------------------------------------------------------')
# print('Trajectory m0 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory_m0.Linear), np.min(
#     normal_dist_trajectory_m0.Linear), np.mean(normal_dist_trajectory_m0.Linear), np.std(normal_dist_trajectory_m0.Linear), np.var(normal_dist_trajectory_m0.Linear)))
print('-----------------------------------------------------------------------------')

# generated_times = getDesiredTimesForNonTrajectory(default_times, normal_dist_trajectory_m0)
# ''' Create times to see how long the path follower takes following the path  '''
# plt.figure(num='Fake delta of Times mode 0')
# # plt.plot(normal_dist_trajectory_m0.curTime, generated_times - normal_dist_trajectory_m0.curTime)
# plt.plot(generated_times)
# idx = 0
# for xc in times_wps_reached_m0:
#     plt.axvline(x=xc, color='k', linestyle='--', alpha=0.7)
#     idx += 1
# plt.xlabel('Current time (s)')
# plt.ylabel('Desired time - Current time (s)')
# plt.legend(['Difference of times', 'WP reached'])
# plt.savefig(dir_save_data + 'fake_deltaT_traj_m0.eps', format='eps', dpi=1200)
# plt.show(block=True)

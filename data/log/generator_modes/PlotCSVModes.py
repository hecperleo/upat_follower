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
experiment_name = 'generator_modes'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/' + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
try:
    default_init_path = pd.read_csv(
        dir_experiment + 'init_waypoints.csv', names=['x', 'y', 'z', 'Times'])
except FileNotFoundError:
    print('init.csv not found!')
try:
    generated_path_m0 = pd.read_csv(
        dir_experiment + 'generated_path_m0.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('generated_path_m0.csv not found!')
try:
    generated_path_m1 = pd.read_csv(
        dir_experiment + 'generated_path_m1.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('generated_path_m1.csv not found!')
try:
    generated_path_m2 = pd.read_csv(
        dir_experiment + 'generated_path_m2.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('generated_path_m2.csv not found!')

def plot3DFigure(_compare_path, _num):
    figN = plt.figure(num='Mode ' + str(_num) +
                      ' 3D behaviour', figsize=(6, 4))
    axN = Axes3D(figN)
    axN.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
             #  color="0.5"
             )
    # ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'y')
    axN.plot(_compare_path.x, _compare_path.y, _compare_path.z, 'r',
             #  color="0"
             )
    axN.legend(['Waypoints', 'Generated path'])
    # axN.set_zlim(0, 3)
    axN.set_xlabel('X axis')
    axN.set_ylabel('Y axis')
    axN.set_zlabel('Z axis')
    figN.savefig(dir_save_data + 'mode' +
                 str(_num) + '.png', format='png', bbox_inches="tight")
    return figN

if 'default_init_path' in globals():
    if 'generated_path_m0' in globals():
        plot3DFigure(generated_path_m0, 0)
        plt.show(block=False)
    if 'generated_path_m1' in globals():
        plot3DFigure(generated_path_m1, 1)
        plt.show(block=False)
    if 'generated_path_m2' in globals():
        plot3DFigure(generated_path_m2, 2)
        plt.show(block=True)
print('-----------------------------------------------------------------------------')

# plt.figure(num='Delta of Times methods', figsize=(6, 3))
# times_wps_reached_m0 = getTimesWPsReached(
#     default_init_path, normal_dist_trajectory_m0)
# print(times_wps_reached_m0)
# plt.plot(normal_dist_trajectory_m0.curTime,
#          normal_dist_trajectory_m0.desTime - normal_dist_trajectory_m0.curTime, color='r',label='Method 1 $t_d$')
# idx = 0
# for xc in times_wps_reached_m0:
#     plt.axvline(x=xc, color='r', linestyle='--', alpha=0.5)
#     idx += 1

# case_name = 'novalid_segments/error_pol_1/sim_large_vmax_4'
# dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
# dir_save_data = dir_data + 'img/' + experiment_name + '/' + case_name + '/'
# try:
#     normal_dist_trajectory_m0 = pd.read_csv(
#         dir_experiment + 'normal_dist_trajectory_m0.csv', names=['curTime', 'desTime', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
# except FileNotFoundError:
#     print('normal_dist_trajectory_m0.csv not found!')
# try:
#     default_init_path = pd.read_csv(
#         dir_experiment + 'init_waypoints.csv', names=['x', 'y', 'z', 'Times'])
# except FileNotFoundError:
#     print('init.csv not found!')

# times_wps_reached_m0 = getTimesWPsReached(
#     default_init_path, normal_dist_trajectory_m0)
# print(times_wps_reached_m0)
# plt.plot(normal_dist_trajectory_m0.curTime,
#          normal_dist_trajectory_m0.desTime - normal_dist_trajectory_m0.curTime, color='b', label='Method 1 $t_d$')
# idx = 0
# for xc in times_wps_reached_m0:
#     plt.axvline(x=xc, color='b', linestyle='--', alpha=0.5)
#     idx += 1

# plt.xlabel('Current time (s)')
# plt.ylabel('Desired time - Current time (s)')
# plt.ylim(bottom=-6)
# plt.legend(['Method 1 difference of times', 'Method 1 WP reached', 'Method 2 difference of times', 'Method 2 WP reached'])
# ax = plt.gca()
# leg = ax.get_legend()
# leg.legendHandles[0].set_color('red')
# leg.legendHandles[1].set_color('red')
# leg.legendHandles[2].set_color('blue')
# leg.legendHandles[2].set_linestyle('-')
# leg.legendHandles[3].set_color('blue')
# # plt.legend()
# plt.savefig(dir_save_data + 'deltaT_methods.eps',
#             format='eps', dpi=1200, bbox_inches='tight')
# plt.show(block=True)




# generated_times = getDesiredTimesForNonTrajectory(default_times, normal_dist_trajectory_m0)
# ''' Create times to see how long the path follower takes following the path  '''
# plt.figure(num='Fake delta of Times mode 0')
# # plt.plot(normal_dist_trajectory_m0.curTime, generated_times - normal_dist_trajectory_m0.curTime)
# plt.plot(generated_times)
# idx = 0
# for xc in times_wps_reached_m0:
#     plt.axvline(x=xc, color='grey', linestyle='--', alpha=0.7)
#     idx += 1
# plt.xlabel('Current time (s)')
# plt.ylabel('Desired time - Current time (s)')
# plt.legend(['Difference of times', 'WP reached'])
# plt.savefig(dir_save_data + 'fake_deltaT_traj_m0.eps', format='eps', dpi=1200)
# plt.show(block=True)

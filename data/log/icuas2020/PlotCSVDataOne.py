import pandas as pd
import numpy as np
import itertools
import os
from matplotlib import pyplot as plt
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D

''' Get directory '''
dir_config = '/home/hector/ros/ual_ws/src/upat_follower/config/'
dir_default_splines = '/home/hector/ros/ual_ws/src/upat_follower/tests/splines/'
dir_data = '/home/hector/ros/ual_ws/src/upat_follower/data/'
experiment_name = 'icuas2020'
case_name = 'optitrack_sim'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/' + case_name + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
try:
    default_times = pd.read_csv(
        dir_config + 'optitrack_t.csv', names=['Times'])
except FileNotFoundError:
    print('init.csv not found!')
try:
    default_init_path = pd.read_csv(
        dir_config + 'optitrack.csv', names=['x', 'y', 'z'])
    # dir_default_splines + 'init.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('init.csv not found!')
try:
    default_smooth_spline_path = pd.read_csv(
        dir_default_splines + 'smooth_spline.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('smooth_spline.csv not found!')
try:
    default_cubic_spline_path = pd.read_csv(
        dir_default_splines + 'cubic_spline.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('cubic_spline.csv not found!')
try:
    normal_dist_trajectory_m0 = pd.read_csv(
        dir_experiment + 'normal_dist_trajectory_m0.csv', names=['Time', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('normal_dist_trajectory_m0.csv not found!')
try:
    normal_dist_trajectory_m1 = pd.read_csv(
        dir_experiment + 'normal_dist_trajectory_m1.csv', names=['Time', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('normal_dist_trajectory_m1.csv not found!')
try:
    normal_dist_trajectory_m2 = pd.read_csv(
        dir_experiment + 'normal_dist_trajectory_m2.csv', names=['Time', 'Spline', 'Linear', 'PosX', 'PosY', 'PosZ', 'curVelx', 'curVely', 'curVelz', 'desVelx', 'desVely', 'desVelz'])
except FileNotFoundError:
    print('normal_dist_trajectory_m2.csv not found!')
try:
    current_trajectory_m0 = pd.read_csv(
        dir_experiment + 'current_trajectory_m0.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('current_trajectory_m0.csv not found!')
try:
    current_trajectory_m1 = pd.read_csv(
        dir_experiment + 'current_trajectory_m1.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('current_trajectory_m1.csv not found!')
try:
    current_trajectory_m2 = pd.read_csv(
        dir_experiment + 'current_trajectory_m2.csv', names=['x', 'y', 'z'])
except FileNotFoundError:
    print('current_trajectory_m2.csv not found!')
try:
    reach_times_trajectory_m0 = pd.read_csv(
        dir_experiment + 'reach_times_trajectory_m0.csv', names=['Time'])
except FileNotFoundError:
    print('reach_times_trajectory_m0.csv not found!')
try:
    reach_times_trajectory_m1 = pd.read_csv(
        dir_experiment + 'reach_times_trajectory_m2.csv', names=['Time'])
except FileNotFoundError:
    print('reach_times_trajectory_m2.csv not found!')
try:
    reach_times_trajectory_m2 = pd.read_csv(
        dir_experiment + 'reach_times_trajectory_m1.csv', names=['Time'])
except FileNotFoundError:
    print('reach_times_trajectory_m1.csv not found!')


def getTimesWPsReached(_init_path, _normal_dist_trajectory):
    min_dist = 1000000
    times_wps_reached = []
    idx = jdx = 0
    for i in _init_path.values:
        p1 = np.asarray([_init_path.values[idx, 0],
                         _init_path.values[idx, 1], _init_path.values[idx, 2]])
        min_dist = 1000000
        jdx = 0
        for j in _normal_dist_trajectory.values:
            p2 = np.asarray([_normal_dist_trajectory.values[jdx, 3],
                             _normal_dist_trajectory.values[jdx, 4], _normal_dist_trajectory.values[jdx, 5]])
            temp_dist = np.linalg.norm(p2 - p1)
            if temp_dist < min_dist:
                min_dist = temp_dist
                t_wp_reached = _normal_dist_trajectory.values[jdx, 0]
            if jdx < _normal_dist_trajectory.shape[0]-1:
                jdx += 1
        times_wps_reached.append(t_wp_reached)
        idx += 1
    return times_wps_reached


def getModVelocity(_normal_dist_trajectory):
    mod_cur_vel = []
    mod_des_vel = []
    idx = 0
    for i in _normal_dist_trajectory.values:
        mod_cur_vel.append(np.sqrt(_normal_dist_trajectory.values[idx, 6] * _normal_dist_trajectory.values[idx, 6] +
                                   _normal_dist_trajectory.values[idx, 7] * _normal_dist_trajectory.values[idx, 7] +
                                   _normal_dist_trajectory.values[idx, 8] * _normal_dist_trajectory.values[idx, 8]))
        mod_des_vel.append(np.sqrt(_normal_dist_trajectory.values[idx, 9] * _normal_dist_trajectory.values[idx, 9] +
                                   _normal_dist_trajectory.values[idx, 10] * _normal_dist_trajectory.values[idx, 10] +
                                   _normal_dist_trajectory.values[idx, 11] * _normal_dist_trajectory.values[idx, 11]))
        idx += 1
    return mod_cur_vel, mod_des_vel


def plot3DFigure(_compare_path, _current_trajectory, _num):
    figN = plt.figure(num='Mode ' + str(_num) + ' 3D behaviour')
    axN = Axes3D(figN)
    axN.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
             #  color="0.5"
             )
    # ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'y')
    axN.plot(_compare_path.x, _compare_path.y, _compare_path.z, 'r--',
             #  color="0"
             )
    axN.plot(_current_trajectory.x, _current_trajectory.y,
             _current_trajectory.z, 'b', alpha = 0.9
             #  color="0.4"
             )
    axN.legend(['Waypoints', 'Generated path', 'Actual path'])
    axN.set_zlim(0, 10)
    axN.set_xlabel('X axis')
    axN.set_ylabel('Y axis')
    axN.set_zlabel('Z axis')
    figN.savefig(dir_save_data + '3D_traj_m' +
                 str(_num) + '.eps', format='eps', dpi=1200)
    return figN


def plot2DFigures(_normal_dist_trajectory, _times_wps_reached, _default_times, _num):
    plt.figure(num='Mode ' + str(_num) + ' normal distance')
    plt.plot(_normal_dist_trajectory.Time,
             _normal_dist_trajectory.Spline, 'b', label="Normal distance to path")
    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    # plt.ylim(top=2)
    # plt.axes().xaxis.set_major_locator(ticker.MultipleLocator(5))
    idx = 0
    for xc in _default_times:
        plt.axvline(x=xc, color='r', linestyle='--', alpha=0.7, 
                    label='WP ' + str(idx+1) + ' desired time ' +
                    str(_default_times[idx, 0])
                    )
        idx += 1
    idx = 0
    for xc in _times_wps_reached:
        plt.axvline(x=xc, color='k', linestyle='--', alpha=0.7,
                    label='WP ' + str(idx+1) + ' reached at ' +
                    str(_times_wps_reached[idx])
                    )
        idx += 1
    # str_times_wps_reached = "WPs reached at " + str(_times_wps_reached[0]) + "s, " + str(_times_wps_reached[1]) + "s, " + str(_times_wps_reached[2]) + "s, " + str(_times_wps_reached[3]) + "s"
    # str_default_times = "WPs desired time " + str(_default_times[0, 0]) + "s, " + str(_default_times[1, 0]) + "s, " + str(_default_times[2, 0]) + "s, " + str(_default_times[3, 0]) + "s"
    # plt.legend(['Normal distance to path', str_default_times,
    #             str_times_wps_reached], fontsize='small')
    # plt.gca().get_legend().legendHandles[0].set_color('blue')
    # plt.gca().get_legend().legendHandles[1].set_color('green')
    # plt.gca().get_legend().legendHandles[2].set_color('red')
    plt.legend(fontsize='small')
    plt.savefig(dir_save_data + 'ndist_traj_m' +
                str(_num)+'.eps', format='eps', dpi=1200)
    plt.show(block=False)
    ''' Plot Velocities trajectory m0 '''
    mod_cur_vel = []
    mod_des_vel = []
    mod_cur_vel, mod_des_vel = getModVelocity(_normal_dist_trajectory)
    plt.figure(num='Velocities trajectory mode '+str(_num))
    idx = 0
    plt.plot(_normal_dist_trajectory.Time, mod_cur_vel, label="Current |v|")
    plt.plot(_normal_dist_trajectory.Time, mod_des_vel, label="Desired |v|", color='r', alpha=0.7)
    for xc in _times_wps_reached:
        plt.axvline(x=xc, color='k', linestyle='--', alpha=0.7, label='WP ' + str(idx+1) + ' reached: ' +
                    str(_times_wps_reached[idx]))
        idx += 1
    plt.xlabel('Time (s)')
    plt.ylabel('Velocity (m/s)')
    plt.legend(['Current |v|', 'Desired |v|', 'WP reached'],fontsize='medium')
    plt.savefig(dir_save_data + 'vel_traj_m' +
                str(_num)+'.eps', format='eps', dpi=1200)
    plt.show(block=False)


if 'default_init_path' in globals():
    if 'normal_dist_trajectory_m0' in globals():
        times_wps_reached_m0 = getTimesWPsReached(
            default_init_path, normal_dist_trajectory_m0)
        plot3DFigure(default_init_path, current_trajectory_m0, 0)
        plot2DFigures(normal_dist_trajectory_m0,
                      times_wps_reached_m0, default_times.values, 0)
        plt.show(block=True)
        ''' Print results of the normal distance through path '''
        print(
            '-----------------------------------------------------------------------------')
        print('Trajectory m0 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory_m0.Linear), np.min(
            normal_dist_trajectory_m0.Linear), np.mean(normal_dist_trajectory_m0.Linear), np.std(normal_dist_trajectory_m0.Linear), np.var(normal_dist_trajectory_m0.Linear)))
    if 'normal_dist_trajectory_m1' in globals():
        times_wps_reached_m1 = getTimesWPsReached(
            default_init_path, normal_dist_trajectory_m1)
        plot3DFigure(default_smooth_spline_path, current_trajectory_m1, 1)
        plot2DFigures(normal_dist_trajectory_m1, times_wps_reached_m1, 1)
        plt.show(block=True)
        print(
            '-----------------------------------------------------------------------------')
        print('Trajectory m1 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory_m1.Spline), np.min(
            normal_dist_trajectory_m1.Spline), np.mean(normal_dist_trajectory_m1.Spline), np.std(normal_dist_trajectory_m1.Spline), np.var(normal_dist_trajectory_m1.Spline)))
    if 'normal_dist_trajectory_m2' in globals():
        times_wps_reached_m2 = getTimesWPsReached(
            default_init_path, normal_dist_trajectory_m2)
        plot3DFigure(default_cubic_spline_path, current_trajectory_m2, 2)
        plot2DFigures(normal_dist_trajectory_m2, times_wps_reached_m2, 2)
        plt.show(block=True)
        print(
            '-----------------------------------------------------------------------------')
        print('Trajectory m2 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory_m2.Spline), np.min(
            normal_dist_trajectory_m2.Spline), np.mean(normal_dist_trajectory_m2.Spline), np.std(normal_dist_trajectory_m2.Spline), np.var(normal_dist_trajectory_m2.Spline)))
print('-----------------------------------------------------------------------------')

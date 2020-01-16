import pandas as pd
import numpy as np
import os
from matplotlib import pyplot as plt
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D

''' Get directory '''
dir_default_splines = '/home/hector/ros/ual_ws/src/upat_follower/tests/splines/'
dir_data = '/home/hector/ros/ual_ws/src/upat_follower/data/'
experiment_name = 'icuas2020'
case_name = 'la_1'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/' + case_name + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
default_init_path = pd.read_csv(
    dir_default_splines + 'init.csv', names=['x', 'y', 'z'])
default_smooth_spline_path = pd.read_csv(
    dir_default_splines + 'smooth_spline.csv', names=['x', 'y', 'z'])
default_cubic_spline_path = pd.read_csv(
    dir_default_splines + 'cubic_spline.csv', names=['x', 'y', 'z'])
normal_dist_trajectory_m0 = pd.read_csv(
    dir_experiment + 'normal_dist_trajectory_m0.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_trajectory_m1 = pd.read_csv(
    dir_experiment + 'normal_dist_trajectory_m1.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_trajectory_m2 = pd.read_csv(
    dir_experiment + 'normal_dist_trajectory_m2.csv', names=['Time', 'Spline', 'Linear'])
current_trajectory_m0 = pd.read_csv(
    dir_experiment + 'current_trajectory_m0.csv', names=['x', 'y', 'z'])
current_trajectory_m1 = pd.read_csv(
    dir_experiment + 'current_trajectory_m1.csv', names=['x', 'y', 'z'])
current_trajectory_m2 = pd.read_csv(
    dir_experiment + 'current_trajectory_m2.csv', names=['x', 'y', 'z'])
reach_times_trajectory_m0 = pd.read_csv(
    dir_experiment + 'reach_times_trajectory_m0.csv', names=['Time'])
reach_times_trajectory_m1 = pd.read_csv(
    dir_experiment + 'reach_times_trajectory_m2.csv', names=['Time'])
reach_times_trajectory_m2 = pd.read_csv(
    dir_experiment + 'reach_times_trajectory_m1.csv', names=['Time'])

''' Plot linear interpolation '''
fig1 = plt.figure(num="Linear interpolation 3D behavior")
ax1 = Axes3D(fig1)
ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
         #  color="0.5"
         )
#ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'y')
ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'r--',
         #  color="0"
         )
ax1.plot(current_trajectory_m0.x, current_trajectory_m0.y,
         current_trajectory_m0.z, 'b',
         #  color="0.4"
         )
ax1.legend(['Waypoints', 'Generated path', 'Actual path'])
ax1.set_zlim(0, 10)
ax1.set_xlabel('X axis')
ax1.set_ylabel('Y axis')
ax1.set_zlabel('Z axis')
fig1.savefig(dir_save_data + 'trajectory_m0.eps', format='eps', dpi=1200)
plt.figure(num="Linear interpolation normal distance")
plt.plot(normal_dist_trajectory_m0.Time, normal_dist_trajectory_m0.Linear, 'b', label="Normal distance to path")
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.axes().xaxis.set_major_locator(ticker.MultipleLocator(5))
idx = 0
for xc in reach_times_trajectory_m0.values:
    plt.axvline(x=xc, color='r', linestyle='--', label='WP '+ str(idx+1) +' reached: ' +
                str(reach_times_trajectory_m0.values[idx]))
    idx += 1
plt.legend()
plt.savefig(dir_save_data + 'ndist_traj_m0.eps', format='eps', dpi=1200)
plt.show(block=False)
''' Plot Smooth spline '''
fig2 = plt.figure(num="Smooth spline 3D behavior")
ax2 = Axes3D(fig2)
ax2.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
         #  color="0.5"
         )
#ax2.plot(default_smooth_spline_path.x, default_smooth_spline_path.y, default_smooth_spline_path.z, 'y')
ax2.plot(default_smooth_spline_path.x, default_smooth_spline_path.y, default_smooth_spline_path.z, 'r--',
         #  color="0"
         )
ax2.plot(current_trajectory_m1.x, current_trajectory_m1.y, current_trajectory_m1.z, 'b',
         #  color="0.4"
         )
ax2.legend(['Waypoints', 'Generated path', 'Actual path'])
ax2.set_zlim(0, 10)
ax2.set_xlabel('X axis')
ax2.set_ylabel('Y axis')
ax2.set_zlabel('Z axis')
fig2.savefig(dir_save_data + 'trajectory_m1.eps', format='eps', dpi=1200)
plt.figure(num="Smooth spline normal distance")
plt.plot(normal_dist_trajectory_m1.Time, normal_dist_trajectory_m1.Spline, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.axes().xaxis.set_major_locator(ticker.MultipleLocator(5))
idx = 0
for xc in reach_times_trajectory_m1.values:
    plt.axvline(x=xc, color='r', linestyle='--', label='WP '+ str(idx+1) +' reached: ' +
                str(reach_times_trajectory_m1.values[idx]))
    idx += 1
plt.legend()
plt.savefig(dir_save_data + 'ndist_traj_m1.eps', format='eps', dpi=1200)
plt.show(block=False)
''' Plot cubic spline '''
fig3 = plt.figure(num="Cubic spline 3D behavior")
ax3 = Axes3D(fig3)
ax3.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
         #  color="0.5"
         )
#ax3.plot(default_cubic_spline_path.x, default_cubic_spline_path.y, default_cubic_spline_path.z, 'y')
ax3.plot(default_cubic_spline_path.x, default_cubic_spline_path.y, default_cubic_spline_path.z, 'r--',
         #  color="0"
         )
ax3.plot(current_trajectory_m2.x, current_trajectory_m2.y, current_trajectory_m2.z, 'b',
         #  color="0.4"
         )
ax3.legend(['Waypoints', 'Generated path', 'Actual path'])
ax3.set_zlim(0, 10)
ax3.set_xlabel('X axis')
ax3.set_ylabel('Y axis')
ax3.set_zlabel('Z axis')
fig3.savefig(dir_save_data + 'trajectory_m2.eps', format='eps', dpi=1200)
plt.figure(num="Cubic spline normal distance")
plt.plot(normal_dist_trajectory_m2.Time, normal_dist_trajectory_m2.Spline, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.axes().xaxis.set_major_locator(ticker.MultipleLocator(5))
idx = 0
for xc in reach_times_trajectory_m2.values:
    plt.axvline(x=xc, color='r', linestyle='--', label='WP '+ str(idx+1) +' reached: ' +
                str(reach_times_trajectory_m2.values[idx]))
    idx += 1
plt.legend()
plt.savefig(dir_save_data + 'ndist_traj_m2.eps', format='eps', dpi=1200)
plt.show()

''' Print results of the normal distance through path '''
print('-----------------------------------------------------------------------------')
print('Trajectory m0 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory_m0.Linear), np.min(
    normal_dist_trajectory_m0.Linear), np.mean(normal_dist_trajectory_m0.Linear), np.std(normal_dist_trajectory_m0.Linear), np.var(normal_dist_trajectory_m0.Linear)))
print('-----------------------------------------------------------------------------')
print('Trajectory m1 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory_m1.Spline), np.min(
    normal_dist_trajectory_m1.Spline), np.mean(normal_dist_trajectory_m1.Spline), np.std(normal_dist_trajectory_m1.Spline), np.var(normal_dist_trajectory_m1.Spline)))
print('-----------------------------------------------------------------------------')
print('Trajectory m2 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory_m2.Spline), np.min(
    normal_dist_trajectory_m2.Spline), np.mean(normal_dist_trajectory_m2.Spline), np.std(normal_dist_trajectory_m2.Spline), np.var(normal_dist_trajectory_m2.Spline)))
print('-----------------------------------------------------------------------------')

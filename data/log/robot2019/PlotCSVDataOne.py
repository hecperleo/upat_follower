import pandas as pd
import numpy as np
import os
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

''' Get directory '''
dir_default_splines = '/home/hector/ros/tfm_ws/src/upat_follower/tests/splines/' 
dir_data = '/home/hector/ros/tfm_ws/src/upat_follower/data/'
experiment_name = 'robot2019'
case_name = '2020-01-29_13-01-08'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/' + case_name + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
default_init_path = pd.read_csv(dir_default_splines + 'init.csv', names=['x', 'y', 'z'])
default_cubic_spline_loyal_path = pd.read_csv(
    dir_default_splines + 'cubic_spline_loyal.csv', names=['x', 'y', 'z'])
default_cubic_spline_path = pd.read_csv(
    dir_default_splines + 'cubic_spline.csv', names=['x', 'y', 'z'])
normal_dist_linear_interp = pd.read_csv(
    dir_experiment + 'normal_dist_linear_interp.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_spline = pd.read_csv(
    dir_experiment + 'normal_dist_cubic_spline.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_loyal = pd.read_csv(
    dir_experiment + 'normal_dist_cubic_loyal_spline.csv', names=['Time', 'Spline', 'Linear'])
current_path_linear_interp = pd.read_csv(
    dir_experiment + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
current_path_cubic_spline = pd.read_csv(
    dir_experiment + 'current_path_cubic_spline.csv', names=['x', 'y', 'z'])
current_path_cubic_loyal = pd.read_csv(
    dir_experiment + 'current_path_cubic_loyal_spline.csv', names=['x', 'y', 'z'])

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
ax1.plot(current_path_linear_interp.x, current_path_linear_interp.y,
         current_path_linear_interp.z, 'b',
         #  color="0.4"
         )
ax1.legend(['Waypoints', 'Generated path', 'Actual path'])
ax1.set_xlabel('X axis')
ax1.set_ylabel('Y axis')
ax1.set_zlabel('Z axis')
fig1.savefig(dir_save_data + 'pathmode0.eps', format='eps', dpi=1200)
plt.figure(num="Linear interpolation normal distance")
plt.plot(normal_dist_linear_interp.Time, normal_dist_linear_interp.Linear, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.legend(["Follower"])
plt.savefig(dir_save_data + 'ndistmode0.eps', format='eps', dpi=1200)
plt.show(block=False)
''' Plot cubic loyal spline '''
fig2 = plt.figure(num="Cubic loyal spline 3D behavior")
ax2 = Axes3D(fig2)
ax2.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
         #  color="0.5"
         )
#ax2.plot(default_cubic_spline_loyal_path.x, default_cubic_spline_loyal_path.y, default_cubic_spline_loyal_path.z, 'y')
ax2.plot(default_cubic_spline_loyal_path.x, default_cubic_spline_loyal_path.y, default_cubic_spline_loyal_path.z, 'r--',
         #  color="0"
         )
ax2.plot(current_path_cubic_loyal.x, current_path_cubic_loyal.y, current_path_cubic_loyal.z, 'b',
         #  color="0.4"
         )
ax2.legend(['Waypoints', 'Generated path', 'Actual path'])
ax2.set_xlabel('X axis')
ax2.set_ylabel('Y axis')
ax2.set_zlabel('Z axis')
fig2.savefig(dir_save_data + 'pathmode1.eps', format='eps', dpi=1200)
plt.figure(num="Cubic loyal spline normal distance")
plt.plot(normal_dist_cubic_loyal.Time, normal_dist_cubic_loyal.Spline, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.legend(["Follower"])
plt.savefig(dir_save_data + 'ndistmode1.eps', format='eps', dpi=1200)
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
ax3.plot(current_path_cubic_spline.x, current_path_cubic_spline.y, current_path_cubic_spline.z, 'b',
         #  color="0.4"
         )
ax3.legend(['Waypoints', 'Generated path', 'Actual path'])
ax3.set_xlabel('X axis')
ax3.set_ylabel('Y axis')
ax3.set_zlabel('Z axis')
fig3.savefig(dir_save_data + 'pathmode2.eps', format='eps', dpi=1200)
plt.figure(num="Cubic spline normal distance")
plt.plot(normal_dist_cubic_spline.Time, normal_dist_cubic_spline.Spline, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.legend(["Follower"])
plt.savefig(dir_save_data + 'ndistmode2.eps', format='eps', dpi=1200)
plt.show()

''' Print results of the normal distance through path '''
print('----------------------------------------------------------------------')
print('Interp1 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_linear_interp.Linear), np.min(
    normal_dist_linear_interp.Linear), np.mean(normal_dist_linear_interp.Linear), np.std(normal_dist_linear_interp.Linear), np.var(normal_dist_linear_interp.Linear)))
print('----------------------------------------------------------------------')
print('Cubic L -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_loyal.Spline), np.min(
    normal_dist_cubic_loyal.Spline), np.mean(normal_dist_cubic_loyal.Spline), np.std(normal_dist_cubic_loyal.Spline), np.var(normal_dist_cubic_loyal.Spline)))
print('----------------------------------------------------------------------')
print('Cubic   -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_spline.Spline), np.min(
    normal_dist_cubic_spline.Spline), np.mean(normal_dist_cubic_spline.Spline), np.std(normal_dist_cubic_spline.Spline), np.var(normal_dist_cubic_spline.Spline)))
print('----------------------------------------------------------------------')
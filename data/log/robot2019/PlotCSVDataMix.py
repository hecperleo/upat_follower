import pandas as pd
import numpy as np
import os
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

''' Get directory '''
dir_default_splines = '/home/hector/ros/tfm_ws/src/upat_follower/tests/splines/'
dir_data = '/home/hector/ros/tfm_ws/src/upat_follower/data/'
experiment_name = 'robot2019'
case_name_medium = 'la_0-4_spd_1'
case_name_fast = 'la_0-8_spd_2'
case_name_slow = 'la_0-8_spd_0-5'
dir_experiment_medium = dir_data + 'log/' + \
    experiment_name + '/' + case_name_medium + '/'
dir_experiment_fast = dir_data + 'log/' + \
    experiment_name + '/' + case_name_fast + '/'
dir_experiment_slow = dir_data + 'log/' + \
    experiment_name + '/' + case_name_slow + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/' + 'merge_cases' + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
default_init_path = pd.read_csv(
    dir_default_splines + 'init.csv', names=['x', 'y', 'z'])
default_cubic_spline_loyal_path = pd.read_csv(
    dir_default_splines + 'cubic_spline_loyal.csv', names=['x', 'y', 'z'])
default_cubic_spline_path = pd.read_csv(
    dir_default_splines + 'cubic_spline.csv', names=['x', 'y', 'z'])

normal_dist_linear_interp_medium = pd.read_csv(
    dir_experiment_medium + 'normal_dist_linear_interp.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_spline_medium = pd.read_csv(
    dir_experiment_medium + 'normal_dist_cubic_spline.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_loyal_medium = pd.read_csv(
    dir_experiment_medium + 'normal_dist_cubic_loyal_spline.csv', names=['Time', 'Spline', 'Linear'])
current_path_linear_interp_medium = pd.read_csv(
    dir_experiment_medium + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
current_path_cubic_spline_medium = pd.read_csv(
    dir_experiment_medium + 'current_path_cubic_spline.csv', names=['x', 'y', 'z'])
current_path_cubic_loyal_medium = pd.read_csv(
    dir_experiment_medium + 'current_path_cubic_loyal_spline.csv', names=['x', 'y', 'z'])

normal_dist_linear_interp_fast = pd.read_csv(
    dir_experiment_fast + 'normal_dist_linear_interp.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_spline_fast = pd.read_csv(
    dir_experiment_fast + 'normal_dist_cubic_spline.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_loyal_fast = pd.read_csv(
    dir_experiment_fast + 'normal_dist_cubic_loyal_spline.csv', names=['Time', 'Spline', 'Linear'])
current_path_linear_interp_fast = pd.read_csv(
    dir_experiment_fast + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
current_path_cubic_spline_fast = pd.read_csv(
    dir_experiment_fast + 'current_path_cubic_spline.csv', names=['x', 'y', 'z'])
current_path_cubic_loyal_fast = pd.read_csv(
    dir_experiment_fast + 'current_path_cubic_loyal_spline.csv', names=['x', 'y', 'z'])

normal_dist_linear_interp_slow = pd.read_csv(
    dir_experiment_slow + 'normal_dist_linear_interp.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_spline_slow = pd.read_csv(
    dir_experiment_slow + 'normal_dist_cubic_spline.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_loyal_slow = pd.read_csv(
    dir_experiment_slow + 'normal_dist_cubic_loyal_spline.csv', names=['Time', 'Spline', 'Linear'])
current_path_linear_interp_slow = pd.read_csv(
    dir_experiment_slow + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
current_path_cubic_spline_slow = pd.read_csv(
    dir_experiment_slow + 'current_path_cubic_spline.csv', names=['x', 'y', 'z'])
current_path_cubic_loyal_slow = pd.read_csv(
    dir_experiment_slow + 'current_path_cubic_loyal_spline.csv', names=['x', 'y', 'z'])


''' Plot linear interpolation '''
fig1 = plt.figure(num="Linear interpolation 3D behavior")
ax1 = Axes3D(fig1)
ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
         #  color="0.5"
         )
# ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'y')
# ax1.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'r--',
#          #  color="0"
#          )
ax1.plot(current_path_linear_interp_medium.x, current_path_linear_interp_medium.y,
         current_path_linear_interp_medium.z, 'b',
         #  color="0.4"
         )
ax1.plot(current_path_linear_interp_fast.x, current_path_linear_interp_fast.y,
         current_path_linear_interp_fast.z,
         #  color="0.4"
         )
ax1.plot(current_path_linear_interp_slow.x, current_path_linear_interp_slow.y,
         current_path_linear_interp_slow.z,
         #  color="0.4"
         )
ax1.legend(['Waypoints', 'Generated path',
            'Medium speed', 'High speed', 'Slow speed'])
ax1.set_xlabel('X axis')
ax1.set_ylabel('Y axis')
ax1.set_zlabel('Z axis')
fig1.savefig(dir_save_data + 'pathmode0.eps', format='eps', dpi=1200)
plt.figure(num="Linear interpolation normal distance")
plt.plot(normal_dist_linear_interp_medium.Time,
         normal_dist_linear_interp_medium.Linear, 'b')
plt.plot(normal_dist_linear_interp_fast.Time,
         normal_dist_linear_interp_fast.Linear)
plt.plot(normal_dist_linear_interp_slow.Time,
         normal_dist_linear_interp_slow.Linear)
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.legend(["Medium speed", "High speed", "Slow speed"])
plt.savefig(dir_save_data + 'ndistmode0.eps', format='eps', dpi=1200)
plt.show(block=False)
''' Plot cubic loyal spline '''
fig2 = plt.figure(num="Cubic loyal spline 3D behavior")
ax2 = Axes3D(fig2)
ax2.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
         #  color="0.5"
         )
# ax2.plot(default_cubic_spline_loyal_path.x, default_cubic_spline_loyal_path.y, default_cubic_spline_loyal_path.z, 'y')
# ax2.plot(default_cubic_spline_loyal_path.x, default_cubic_spline_loyal_path.y, default_cubic_spline_loyal_path.z, 'r--',
#          #  color="0"
#          )
ax2.plot(current_path_cubic_loyal_medium.x, current_path_cubic_loyal_medium.y, current_path_cubic_loyal_medium.z, 'b',
         #  color="0.4"
         )
ax2.plot(current_path_cubic_loyal_fast.x, current_path_cubic_loyal_fast.y, current_path_cubic_loyal_fast.z,
         #  color="0.4"
         )
ax2.plot(current_path_cubic_loyal_slow.x, current_path_cubic_loyal_slow.y, current_path_cubic_loyal_slow.z,
         #  color="0.4"
         )
ax2.legend(['Waypoints', 'Generated path',
            'Medium speed', 'High speed', 'Slow speed'])
ax2.set_xlabel('X axis')
ax2.set_ylabel('Y axis')
ax2.set_zlabel('Z axis')
fig2.savefig(dir_save_data + 'pathmode1.eps', format='eps', dpi=1200)
plt.figure(num="Cubic loyal spline normal distance")
plt.plot(normal_dist_cubic_loyal_medium.Time,
         normal_dist_cubic_loyal_medium.Spline, 'b')
plt.plot(normal_dist_cubic_loyal_fast.Time,
         normal_dist_cubic_loyal_fast.Spline)
plt.plot(normal_dist_cubic_loyal_slow.Time,
         normal_dist_cubic_loyal_slow.Spline)
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.legend(["Medium speed", "High speed", "Slow speed"])
plt.savefig(dir_save_data + 'ndistmode1.eps', format='eps', dpi=1200)
plt.show(block=False)
''' Plot cubic spline '''
fig3 = plt.figure(num="Cubic spline 3D behavior")
ax3 = Axes3D(fig3)
ax3.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko',
         #  color="0.5"
         )
# ax3.plot(default_cubic_spline_path.x, default_cubic_spline_path.y, default_cubic_spline_path.z, 'y')
# ax3.plot(default_cubic_spline_path.x, default_cubic_spline_path.y, default_cubic_spline_path.z, 'r--',
#          #  color="0"
#          )
ax3.plot(current_path_cubic_spline_medium.x, current_path_cubic_spline_medium.y, current_path_cubic_spline_medium.z, 'b',
         #  color="0.4"
         )
ax3.plot(current_path_cubic_spline_fast.x, current_path_cubic_spline_fast.y, current_path_cubic_spline_fast.z,
         #  color="0.4"
         )
ax3.plot(current_path_cubic_spline_slow.x, current_path_cubic_spline_slow.y, current_path_cubic_spline_slow.z,
         #  color="0.4"
         )
ax3.legend(['Waypoints', 'Generated path',
            'Medium speed', 'High speed', 'Slow speed'])
ax3.set_xlabel('X axis')
ax3.set_ylabel('Y axis')
ax3.set_zlabel('Z axis')
fig3.savefig(dir_save_data + 'pathmode2.eps', format='eps', dpi=1200)
plt.figure(num="Cubic spline normal distance")
plt.plot(normal_dist_cubic_spline_medium.Time,
         normal_dist_cubic_spline_medium.Spline, 'b')
plt.plot(normal_dist_cubic_spline_fast.Time,
         normal_dist_cubic_spline_fast.Spline)
plt.plot(normal_dist_cubic_spline_slow.Time,
         normal_dist_cubic_spline_slow.Spline)
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.legend(["Medium speed", "High speed", "Slow speed"])
plt.savefig(dir_save_data + 'ndistmode2.eps', format='eps', dpi=1200)
plt.show()

''' Print results of the normal distance through path '''
print('----------------------------- Medium ---------------------------------')
print('Interp1 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_linear_interp_medium.Linear), np.min(
    normal_dist_linear_interp_medium.Linear), np.mean(normal_dist_linear_interp_medium.Linear), np.std(normal_dist_linear_interp_medium.Linear), np.var(normal_dist_linear_interp_medium.Linear)))
print('----------------------------------------------------------------------')
print('Cubic L -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_loyal_medium.Spline), np.min(
    normal_dist_cubic_loyal_medium.Spline), np.mean(normal_dist_cubic_loyal_medium.Spline), np.std(normal_dist_cubic_loyal_medium.Spline), np.var(normal_dist_cubic_loyal_medium.Spline)))
print('----------------------------------------------------------------------')
print('Cubic   -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_spline_medium.Spline), np.min(
    normal_dist_cubic_spline_medium.Spline), np.mean(normal_dist_cubic_spline_medium.Spline), np.std(normal_dist_cubic_spline_medium.Spline), np.var(normal_dist_cubic_spline_medium.Spline)))
print('----------------------------------------------------------------------')
print('-----------------------------  Fast  ---------------------------------')
print('Interp1 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_linear_interp_fast.Linear), np.min(
    normal_dist_linear_interp_fast.Linear), np.mean(normal_dist_linear_interp_fast.Linear), np.std(normal_dist_linear_interp_fast.Linear), np.var(normal_dist_linear_interp_fast.Linear)))
print('----------------------------------------------------------------------')
print('Cubic L -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_loyal_fast.Spline), np.min(
    normal_dist_cubic_loyal_fast.Spline), np.mean(normal_dist_cubic_loyal_fast.Spline), np.std(normal_dist_cubic_loyal_fast.Spline), np.var(normal_dist_cubic_loyal_fast.Spline)))
print('----------------------------------------------------------------------')
print('Cubic   -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_spline_fast.Spline), np.min(
    normal_dist_cubic_spline_fast.Spline), np.mean(normal_dist_cubic_spline_fast.Spline), np.std(normal_dist_cubic_spline_fast.Spline), np.var(normal_dist_cubic_spline_fast.Spline)))
print('----------------------------------------------------------------------')
print('-----------------------------  Slow  ---------------------------------')
print('Interp1 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_linear_interp_slow.Linear), np.min(
    normal_dist_linear_interp_slow.Linear), np.mean(normal_dist_linear_interp_slow.Linear), np.std(normal_dist_linear_interp_slow.Linear), np.var(normal_dist_linear_interp_slow.Linear)))
print('----------------------------------------------------------------------')
print('Cubic L -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_loyal_slow.Spline), np.min(
    normal_dist_cubic_loyal_slow.Spline), np.mean(normal_dist_cubic_loyal_slow.Spline), np.std(normal_dist_cubic_loyal_slow.Spline), np.var(normal_dist_cubic_loyal_slow.Spline)))
print('----------------------------------------------------------------------')
print('Cubic   -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_cubic_spline_slow.Spline), np.min(
    normal_dist_cubic_spline_slow.Spline), np.mean(normal_dist_cubic_spline_slow.Spline), np.std(normal_dist_cubic_spline_slow.Spline), np.var(normal_dist_cubic_spline_slow.Spline)))
print('----------------------------------------------------------------------')


''' Plot all 3 generator modes '''
fig4 = plt.figure(num="Generator modes")
ax4 = Axes3D(fig4)
ax4.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko')
ax4.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'b')
ax4.plot(default_cubic_spline_path.x,
         default_cubic_spline_path.y, default_cubic_spline_path.z)
ax4.plot(default_cubic_spline_loyal_path.x,
         default_cubic_spline_loyal_path.y, default_cubic_spline_loyal_path.z)
ax4.legend(['Waypoints', 'Linear interpolation',
            'Cubic interpolation', 'Cubic loyal interpolation'])
ax4.set_xlabel('X axis')
ax4.set_ylabel('Y axis')
ax4.set_zlabel('Z axis')
fig4.savefig(dir_save_data + 'genmodes.eps', format='eps', dpi=1200)
plt.show()

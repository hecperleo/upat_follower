import pandas as pd
import numpy as np
import os
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

''' Get directory '''
dir_default_splines = '/home/hector/ros/tfm_ws/src/upat_follower/tests/splines/'
dir_data = '/home/hector/ros/tfm_ws/src/upat_follower/data/'
experiment_name = 'robot2019'
case_name = 'la_0-8_spd_2'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/' + case_name + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
default_init_path = pd.read_csv(
    dir_default_splines + 'init.csv', names=['x', 'y', 'z'])
# default_trajectory = pd.read_csv(
#     dir_default_splines + 'trajectory.csv', names=['x', 'y', 'z'])
normal_dist_linear_interp = pd.read_csv(
    dir_experiment + 'normal_dist_linear_interp.csv', names=['Time', 'Spline', 'Linear'])
# normal_dist_trajectory = pd.read_csv(
#     dir_experiment + 'normal_dist_trajectory.csv', names=['Time', 'Spline', 'Linear'])
current_path_linear_interp = pd.read_csv(
    dir_experiment + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
# current_trajectory = pd.read_csv(
#     dir_experiment + 'current_trajectory.csv', names=['x', 'y', 'z'])

''' PX4 vs UPAT: Diagonal and horizontal Line'''

''' Get directory '''
case_name = 'compare_line'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
''' Get csv files '''
default_horizontal = pd.read_csv(
    dir_experiment + 'horizontal.csv', names=['x', 'y', 'z'])
default_diagonal = pd.read_csv(
    dir_experiment + 'diagonal.csv', names=['x', 'y', 'z'])
ual_current_horizontal = pd.read_csv(
    dir_experiment + 'ual_current_horizontal.csv', names=['x', 'y', 'z'])
upat_current_horizontal = pd.read_csv(
    dir_experiment + 'upat_current_horizontal.csv', names=['x', 'y', 'z'])
ual_current_diagonal = pd.read_csv(
    dir_experiment + 'ual_current_diagonal.csv', names=['x', 'y', 'z'])
upat_current_diagonal = pd.read_csv(
    dir_experiment + 'upat_current_diagonal.csv', names=['x', 'y', 'z'])
ual_normal_horizontal = pd.read_csv(
    dir_experiment + 'ual_normal_horizontal.csv', names=['Time', 'Linear'])
upat_normal_horizontal = pd.read_csv(
    dir_experiment + 'upat_normal_horizontal.csv', names=['Time', 'Spline', 'Linear'])
ual_normal_diagonal = pd.read_csv(
    dir_experiment + 'ual_normal_diagonal.csv', names=['Time', 'Linear'])
upat_normal_diagonal = pd.read_csv(
    dir_experiment + 'upat_normal_diagonal.csv', names=['Time', 'Spline', 'Linear'])
''' Plot horizontal line '''
fig4 = plt.figure()
ax4 = Axes3D(fig4)
# ax4.set_proj_type('ortho')
ax4.set_zlim3d(10, 14)
ax4.plot(default_horizontal.x, default_horizontal.y,
         default_horizontal.z, 'ko')
ax4.plot(upat_current_horizontal.x,
         upat_current_horizontal.y, upat_current_horizontal.z, 'b')
ax4.plot(ual_current_horizontal.x,
         ual_current_horizontal.y, ual_current_horizontal.z, 'r-.')
ax4.legend(['Waypoints', 'Follower', 'PX4'])
ax4.set_xlabel('X axis')
ax4.set_ylabel('Y axis')
ax4.set_zlabel('Z axis')
ax4.view_init(30, 30)
fig4.savefig(dir_save_data + 'horizontal.eps', format='eps', dpi=1200)
plt.figure()
plt.plot(ual_normal_horizontal.Time,
         ual_normal_horizontal.Linear, 'r', linestyle='-.')
plt.plot(upat_normal_horizontal.Time, upat_normal_horizontal.Linear, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.legend(["PX4", "Follower"])
plt.ylim(top=1)
plt.savefig(dir_save_data + 'ndisthorizontal.eps', format='eps', dpi=1200)
plt.show(block=False)
''' Plot diagonal line '''
fig6 = plt.figure()
ax6 = Axes3D(fig6)
ax6.plot(default_diagonal.x, default_diagonal.y, default_diagonal.z, 'ko')
ax6.plot(upat_current_diagonal.x,
         upat_current_diagonal.y, upat_current_diagonal.z, 'b')
ax6.plot(ual_current_diagonal.x, ual_current_diagonal.y,
         ual_current_diagonal.z, 'r-.')
ax6.legend(['Waypoints', 'Follower', 'PX4'])
ax6.set_xlabel('X axis')
ax6.set_ylabel('Y axis')
ax6.set_zlabel('Z axis')
fig6.savefig(dir_save_data + 'diagonal.eps', format='eps', dpi=1200)
plt.figure()
plt.plot(ual_normal_diagonal.Time,
         ual_normal_diagonal.Linear, 'r', linestyle='-.')
plt.plot(upat_normal_diagonal.Time, upat_normal_diagonal.Linear, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.legend(["PX4", "Follower"])
plt.savefig(dir_save_data + 'ndistdiagonal.eps', format='eps', dpi=1200)
plt.show()

print('----------------------------------------------------------------------')
print('Hor PX4 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(ual_normal_horizontal.Linear), np.min(
    ual_normal_horizontal.Linear), np.mean(ual_normal_horizontal.Linear), np.std(ual_normal_horizontal.Linear), np.var(ual_normal_horizontal.Linear)))
print('Hor UPAT-> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(upat_normal_horizontal.Linear), np.min(
    upat_normal_horizontal.Linear), np.mean(upat_normal_horizontal.Linear), np.std(upat_normal_horizontal.Linear), np.var(upat_normal_horizontal.Linear)))
print('----------------------------------------------------------------------')
print('Dia PX4 -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(ual_normal_diagonal.Linear), np.min(
    ual_normal_diagonal.Linear), np.mean(ual_normal_diagonal.Linear), np.std(ual_normal_diagonal.Linear), np.var(ual_normal_diagonal.Linear)))
print('Dia UPAT-> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(upat_normal_diagonal.Linear), np.min(
    upat_normal_diagonal.Linear), np.mean(upat_normal_diagonal.Linear), np.std(upat_normal_diagonal.Linear), np.var(upat_normal_diagonal.Linear)))
print('----------------------------------------------------------------------')

''' PX4 vs UPAT: Waypoint list '''
case_name = 'ual_reach_wp'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
''' Get csv files '''
default_init_path = pd.read_csv(
    dir_default_splines + 'init.csv', names=['x', 'y', 'z'])
# current_path_linear_interp = pd.read_csv(
#     dir_experiment + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
px4_normal_dist_linear_interp = pd.read_csv(
    dir_experiment + 'normal_dist_linear_interp.csv', names=['Time', 'Linear'])
px4_current_path_linear_interp = pd.read_csv(
    dir_experiment + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
''' Plot PX4 vs Follower on linear interpolation '''
fig7 = plt.figure()
ax7 = Axes3D(fig7)
ax7.plot(default_init_path.x, default_init_path.y, default_init_path.z, 'ko')
ax7.plot(current_path_linear_interp.x,
         current_path_linear_interp.y, current_path_linear_interp.z, 'b')
ax7.plot(px4_current_path_linear_interp.x,
         px4_current_path_linear_interp.y, px4_current_path_linear_interp.z, 'r-.')
ax7.legend(['Waypoints', 'Follower', 'PX4'])
ax7.set_xlabel('X axis')
ax7.set_ylabel('Y axis')
ax7.set_zlabel('Z axis')
fig7.savefig(dir_save_data + 'path.eps', format='eps', dpi=1200)
plt.figure()
plt.plot(px4_normal_dist_linear_interp.Time,
         px4_normal_dist_linear_interp.Linear, 'r', linestyle='-.')
plt.plot(normal_dist_linear_interp.Time,
         normal_dist_linear_interp.Linear, 'b')
plt.legend(["PX4", "Follower"])
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.ylim(top=1)
plt.savefig(dir_save_data + 'ndistpx4.eps', format='eps', dpi=1200)
plt.show()
print('PX4-UPAT-> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(px4_normal_dist_linear_interp.Linear), np.min(
    px4_normal_dist_linear_interp.Linear), np.mean(px4_normal_dist_linear_interp.Linear), np.std(px4_normal_dist_linear_interp.Linear), np.var(normal_dist_linear_interp.Linear)))
print('----------------------------------------------------------------------')


# # ''' Plot trajectory '''
# # fig = plt.figure(num="Trajectory 3D behavior")
# # ax = Axes3D(fig)
# # ax.plot(default_trajectory.x, default_trajectory.y, default_trajectory.z)
# # ax.plot(current_trajectory.x, current_trajectory.y, current_trajectory.z)
# # plt.show()
# # plt.figure(num="Trajectory normal distance")
# # plt.plot(normal_dist_trajectory.Time, normal_dist_trajectory.Spline)
# # plt.plot(normal_dist_trajectory.Time, normal_dist_trajectory.Linear)
# # plt.xlabel('Time (s)')
# # plt.ylabel('Distance (m)')
# # plt.show()
# # print('Traject -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory.Linear), np.min(
# #     normal_dist_trajectory.Linear), np.mean(normal_dist_trajectory.Linear), np.std(normal_dist_trajectory.Linear), np.var(normal_dist_trajectory.Linear)))
# # print('----------------------------------------------------------------------')

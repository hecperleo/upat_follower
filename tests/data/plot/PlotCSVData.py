import pandas as pd
import numpy as np
from matplotlib import pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

''' Get directory '''
dir_data = '/home/hector/ros/tfm_ws/src/upat_follower/tests/data/'
dir_test = dir_data + 'plot/la_0-4_spd_1/'
dir_save_data = '/home/hector/ros/tfm_ws/src/upat_follower/tests/data/plot/overleaf/'
''' Get csv files '''
default_init_path = pd.read_csv(dir_data + 'init.csv', names=['x', 'y', 'z'])
default_cubic_spline_loyal_path = pd.read_csv(
    dir_data + 'cubic_spline_loyal.csv', names=['x', 'y', 'z'])
default_cubic_spline_path = pd.read_csv(
    dir_data + 'cubic_spline.csv', names=['x', 'y', 'z'])
default_trajectory = pd.read_csv(
    dir_data + 'trajectory.csv', names=['x', 'y', 'z'])
normal_dist_linear_interp = pd.read_csv(
    dir_test + 'normal_dist_linear_interp.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_spline = pd.read_csv(
    dir_test + 'normal_dist_cubic_spline.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_cubic_loyal = pd.read_csv(
    dir_test + 'normal_dist_cubic_loyal_spline.csv', names=['Time', 'Spline', 'Linear'])
normal_dist_trajectory = pd.read_csv(
    dir_test + 'normal_dist_trajectory.csv', names=['Time', 'Spline', 'Linear'])
current_path_linear_interp = pd.read_csv(
    dir_test + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
current_path_cubic_spline = pd.read_csv(
    dir_test + 'current_path_cubic_spline.csv', names=['x', 'y', 'z'])
current_path_cubic_loyal = pd.read_csv(
    dir_test + 'current_path_cubic_loyal_spline.csv', names=['x', 'y', 'z'])
current_trajectory = pd.read_csv(
    dir_test + 'current_trajectory.csv', names=['x', 'y', 'z'])

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

''' PX4 vs UPAT: Diagonal and horizontal Line'''

''' Get directory '''
dir_test = dir_data + 'plot/compare_line/'
''' Get csv files '''
default_horizontal = pd.read_csv(
    dir_test + 'horizontal.csv', names=['x', 'y', 'z'])
default_diagonal = pd.read_csv(
    dir_test + 'diagonal.csv', names=['x', 'y', 'z'])
ual_current_horizontal = pd.read_csv(
    dir_test + 'ual_current_horizontal.csv', names=['x', 'y', 'z'])
upat_current_horizontal = pd.read_csv(
    dir_test + 'upat_current_horizontal.csv', names=['x', 'y', 'z'])
ual_current_diagonal = pd.read_csv(
    dir_test + 'ual_current_diagonal.csv', names=['x', 'y', 'z'])
upat_current_diagonal = pd.read_csv(
    dir_test + 'upat_current_diagonal.csv', names=['x', 'y', 'z'])
ual_normal_horizontal = pd.read_csv(
    dir_test + 'ual_normal_horizontal.csv', names=['Time', 'Linear'])
upat_normal_horizontal = pd.read_csv(
    dir_test + 'upat_normal_horizontal.csv', names=['Time', 'Spline', 'Linear'])
ual_normal_diagonal = pd.read_csv(
    dir_test + 'ual_normal_diagonal.csv', names=['Time', 'Linear'])
upat_normal_diagonal = pd.read_csv(
    dir_test + 'upat_normal_diagonal.csv', names=['Time', 'Spline', 'Linear'])
''' Plot horizontal line '''
fig4 = plt.figure()
ax4 = Axes3D(fig4)
# ax4.set_proj_type('ortho')
ax4.set_zlim3d(10, 14)
ax4.plot(default_horizontal.x, default_horizontal.y, default_horizontal.z, 'ko')
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
plt.plot(ual_normal_horizontal.Time, ual_normal_horizontal.Linear, 'r', linestyle='-.')
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
plt.plot(ual_normal_diagonal.Time, ual_normal_diagonal.Linear, 'r', linestyle='-.')
plt.plot(upat_normal_diagonal.Time, upat_normal_diagonal.Linear, 'b')
plt.xlabel('Time (s)')
plt.ylabel('Distance (m)')
plt.legend(["PX4", "Follower"])
plt.savefig(dir_save_data + 'ndistdiagonal.eps', format='eps', dpi=1200)
plt.show()

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

dir_test = dir_data + 'plot/ual_reach_wp/'
''' Get csv files '''
default_init_path = pd.read_csv(dir_data + 'init.csv', names=['x', 'y', 'z'])
# current_path_linear_interp = pd.read_csv(
#     dir_test + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
px4_normal_dist_linear_interp = pd.read_csv(
    dir_test + 'normal_dist_linear_interp.csv', names=['Time', 'Linear'])
px4_current_path_linear_interp = pd.read_csv(
    dir_test + 'current_path_linear_interp.csv', names=['x', 'y', 'z'])
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




# ''' Plot trajectory '''
# fig = plt.figure(num="Trajectory 3D behavior")
# ax = Axes3D(fig)
# ax.plot(default_trajectory.x, default_trajectory.y, default_trajectory.z)
# ax.plot(current_trajectory.x, current_trajectory.y, current_trajectory.z)
# plt.show()
# plt.figure(num="Trajectory normal distance")
# plt.plot(normal_dist_trajectory.Time, normal_dist_trajectory.Spline)
# plt.plot(normal_dist_trajectory.Time, normal_dist_trajectory.Linear)
# plt.xlabel('Time (s)')
# plt.ylabel('Distance (m)')
# plt.show()
# print('Traject -> max: {:.3f}, min: {:.3f}, mean: {:.3f}, std: {:.3f}, var: {:.3f}'.format(np.max(normal_dist_trajectory.Linear), np.min(
#     normal_dist_trajectory.Linear), np.mean(normal_dist_trajectory.Linear), np.std(normal_dist_trajectory.Linear), np.var(normal_dist_trajectory.Linear)))
# print('----------------------------------------------------------------------')
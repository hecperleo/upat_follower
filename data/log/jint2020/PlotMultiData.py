import pandas as pd
import numpy as np
import itertools
import os
from scipy import interpolate
from matplotlib import pyplot as plt
import matplotlib.ticker as ticker
from mpl_toolkits.mplot3d import Axes3D
import copy

''' Get directory '''
dir_config = '/home/hector/ros/ual_ws/src/upat_follower/config/'
dir_data = '/home/hector/ros/ual_ws/src/upat_follower/data/'
experiment_name = 'jint2020'
case_name = '3UAV-4D-pyviz'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/' + case_name + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
print(dir_experiment)
try:
    multi_uav = pd.read_csv(
        dir_experiment + 'multi_uav_log.csv', names=['curTime', 
                                                     'dist_01', 'dist_02', 'dist_12',
                                                     'ual_0_x', 'ual_0_y', 'ual_0_z',
                                                     'ual_1_x', 'ual_1_y', 'ual_1_z',
                                                     'ual_2_x', 'ual_2_y', 'ual_2_z'])
except FileNotFoundError:
    print('multi_uav_log.csv not found!')



def plot2D():
    plt.figure(num='Distance between UAVs')
    plt.axhline(y=9, color='k', linestyle='--')
    plt.plot(multi_uav.curTime, multi_uav.dist_01)
    plt.plot(multi_uav.curTime, multi_uav.dist_02)
    plt.plot(multi_uav.curTime, multi_uav.dist_12)

    plt.xlabel('Time (s)')
    plt.ylabel('Distance (m)')
    plt.legend(['Limit = 10m', 'Distance 01', 'Distance 02', 'Distance 12'])
    plt.savefig(dir_save_data + case_name + '.eps', format='eps', dpi=1200, bbox_inches='tight')
    plt.show()

def plot3D():
    figN = plt.figure(num='3D Visualization')
    axN = Axes3D(figN)
    uav_0_plan_x = copy.deepcopy(multi_uav.ual_0_x)
    uav_0_plan_y = copy.deepcopy(multi_uav.ual_0_y)
    uav_0_plan_z = copy.deepcopy(multi_uav.ual_0_z)
    idx = 0;
    for i in uav_0_plan_z:
        if uav_0_plan_z[idx] != 5:
            uav_0_plan_z[idx] = 5
        idx +=1
    axN.plot(uav_0_plan_x, uav_0_plan_y, uav_0_plan_z, 'k--')
    axN.plot(multi_uav.ual_0_x, multi_uav.ual_0_y, multi_uav.ual_0_z, 'r')
    axN.plot(multi_uav.ual_1_x, multi_uav.ual_1_y, multi_uav.ual_1_z, 'b')
    axN.plot(multi_uav.ual_2_x, multi_uav.ual_2_y, multi_uav.ual_2_z, 'g')

    axN.legend(['UAV 0 flight plan', 'UAV 0 trajectory', 'UAV 1 trajectory', 'UAV 2 trajectory'])
    # axN.set_xlim(-50, 100)
    # axN.set_ylim(-50, 100)
    axN.set_zlim(0, 15)
    # axN.view_init(elev= 20, azim=-40) # for 2 UAV
    axN.view_init(elev= 25, azim=-70) # for 3 UAV
    axN.set_xlabel('X axis')
    axN.set_ylabel('Y axis')
    axN.set_zlabel('Z axis')
    figN.savefig(dir_save_data + '3DVisualization.eps', format='eps', dpi=1200, bbox_inches="tight")
    return figN

plot2D()
plot3D()
plt.show()
print('-----------------------------------------------------------------------------')
# https://towardsdatascience.com/using-standard-deviation-in-python-77872c32ba9b

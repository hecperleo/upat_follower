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
experiment_name = 'jint2020'
case_name = '3UAV-3D-2Conflict'
dir_experiment = dir_data + 'log/' + experiment_name + '/' + case_name + '/'
dir_save_data = dir_data + 'img/' + experiment_name + '/'
''' Create folder to save data '''
if not os.path.exists(dir_save_data):
    os.makedirs(dir_save_data)
''' Get csv files '''
print(dir_experiment)
try:
    multi_uav = pd.read_csv(
        dir_experiment + 'multi_uav_log.csv', names=['curTime', 'dist_01', 'dist_02', 'dist_12'])
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
    # plt.xticks(np.arange(1, 5, step=1))
    plt.legend(['Limit', 'Distance 01', 'Distance 02', 'Distance 12'])

    plt.show()


plot2D()
plt.show()
print('-----------------------------------------------------------------------------')
# https://towardsdatascience.com/using-standard-deviation-in-python-77872c32ba9b

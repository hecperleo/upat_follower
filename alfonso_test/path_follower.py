#!/usr/bin/env python
import rospy
from uav_abstraction_layer.srv import TakeOff, Land
from uav_abstraction_layer.msg import State
from geometry_msgs.msg import Twist, TwistStamped, PoseStamped 
from nav_msgs.msg import Path
import time
#rospy.wait_for_service(go_to_waypoint_url)
import scipy.io as sio
import matplotlib.pyplot as plt
import numpy as np
from numpy import linalg as LA
import rospkg
import math

#params

current_pose_x = 0.0
current_pose_y = 0.0
state = State()
def cal_pose_on_path(x,y):
    min_distance = 10000000
    for i in range(len(x)):
        aux = LA.norm([current_pose_x-x[i], current_pose_y-y[i]])
        if aux<min_distance:
            min_distance = aux
            pose_on_path = i
    return pose_on_path

def cal_pose_look_ahead(x,y,look_ahead,pose_on_path):
    for i in range(pose_on_path, len(x)):
        aux = LA.norm([x[i]-x[pose_on_path], y[i]-y[pose_on_path]])
        if aux>look_ahead:
            return i
    return i

def calculate_vel(look_a_x,look_a_y,vel_x,vel_y):
    distance = LA.norm([look_a_x-current_pose_x, look_a_y-current_pose_y])
    vect_uni = [look_a_x-current_pose_x, look_a_y-current_pose_y]/distance
    vel_module = LA.norm([vel_x, vel_y])
    return vect_uni*vel_module

def uav_state_callback(data):
    global state
    state = data

# subscribing to the own pose
def uav_pose_callback(data):
    global current_pose_x
    global current_pose_y
    global current_pose_z
    current_pose_x = data.pose.position.x
    current_pose_y = data.pose.position.y
    current_pose_z = data.pose.position.z

rospy.init_node('FORCES_path_follower')#take_off_url = '/drone_1/drone_1/ual/take_off'
trajectory_name = rospy.get_param('~trajectory_name')
steps = rospy.get_param('~time_horizon')


rospack = rospkg.RosPack()
folder_pack = rospack.get_path('uav_path_manager')
route = folder_pack + "/alfonso_test/"+ trajectory_name
file = sio.loadmat(route) #TO DO: set the URL of the package
matrix = file['TEMP']
#plt.plot(matrix[3,:],matrix[4,:])

vel_x = matrix[6,:]
vel_y = matrix[7,:]
vel_z = matrix[8,:]

pos_x = matrix[3,:]
pos_y = matrix[4,:]
pos_z = matrix[5,:]


#plt.show()


velocity_pub= rospy.Publisher('/drone_1/ual/set_velocity', TwistStamped, queue_size=1)
pose_pub = rospy.Publisher('/drone_1/ual/set_pose',PoseStamped,queue_size=1)
path_pub = rospy.Publisher('/path',Path, queue_size=10)
rospy.Subscriber('/drone_1/ual/pose', PoseStamped, uav_pose_callback, queue_size=1)
rospy.Subscriber('/drone_1/ual/state',State, uav_state_callback,queue_size=1)
try:
   take_off_service = "/drone_1/ual/take_off"
   take_off_client = rospy.ServiceProxy(take_off_service, TakeOff)
   take_off_client.call(3.0,False)
   print("taking off")
except rospy.ServiceException, e:
   print "Service call failed: %s"%e

while state.state != 4:
    time.sleep(0.5)


path_to_publish = Path()
path_to_publish.header.frame_id ='map'
for k in range(len(vel_x)):
    pose_of_the_path = PoseStamped()
    pose_of_the_path.pose.position.x = pos_x[k]
    pose_of_the_path.pose.position.y = pos_y[k]
    pose_of_the_path.pose.position.z = pos_z[k]
    path_to_publish.poses.append(pose_of_the_path)

path_pub.publish(path_to_publish)
#plot path

print("going to the start pose")

pos = PoseStamped()
pos.header.frame_id = 'map'
pos.pose.position.x = pos_x[0]
pos.pose.position.y = pos_y[0]
pos.pose.position.z = pos_z[0]
pos.pose.orientation.w = 1
pose_pub.publish(pos)

distance=LA.norm([current_pose_x-pos_x[0], current_pose_y-pos_y[0],current_pose_z-pos_z[0]])
while distance>1.5:
    distance=LA.norm([current_pose_x-pos_x[0], current_pose_y-pos_y[0],current_pose_z-pos_z[0]])
    time.sleep(0.1)

print("starting_trajectory)")
pose_on_path = cal_pose_on_path(pos_x, pos_y)
init_time = time.time()

vel = TwistStamped()
while pose_on_path<steps-1:
    pose_on_path = cal_pose_on_path(pos_x, pos_y)
    if pose_on_path == 0:
        pose_on_path = 3
    look_ahead = 1
    target_pose = cal_pose_look_ahead(pos_x,pos_y,look_ahead,pose_on_path)
    vel_to_command = calculate_vel(pos_x[target_pose],pos_y[target_pose], vel_x[pose_on_path],vel_y[pose_on_path])

    vel.twist.linear.x = vel_to_command[0]
    vel.twist.linear.y = vel_to_command[1]
    vel.twist.linear.z = 0

    velocity_pub.publish(vel)
    time.sleep(0.1)

print(time.time()-init_time)


# pos.pose.position.x = pos_x[0]
# pos.pose.position.y = pos_y[0]
# pos.pose.position.z = pos_z[0]
# pos.pose.orientation.w = 1
# pose_pub.publish(pos)
# time.sleep(20)
# init_time = time.time()
# print("starting flyover")
# time.sleep(1)
# for i in range(len(vel_x)):
#     vel.twist.linear.x = vel_x[i]
#     vel.twist.linear.y = vel_y[i]
#     vel.twist.linear.z = vel_z[i]

#     pos.pose.position.x = pos_x[i]
#     pos.pose.position.y = pos_y[i]
#     pos.pose.position.z = pos_z[i]
#     pos.header.frame_id = "map"
#     velocity_pub.publish(vel)
#     #pose_pub.publish(pos)
#     time.sleep(0.1)

# print(time.time()-init_time)
# time.sleep(40)
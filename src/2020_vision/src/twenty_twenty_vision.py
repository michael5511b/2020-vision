#!/usr/bin/env python
import ctypes
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time

# Definitions
cloud_sub_buf_size = 52428800
# Number of groups we want to divide the point clouds into along the z axis
num_z_groups = int(rospy.get_param('twenty_twenty_vision/num_z_groups')) 

# Globals
class pc_container:
    def __init__(self):
        self.points_array_list = [[], [], []]
        self.cam1_flag = False
        self.cam2_flag = False
        self.cam3_flag = False
        self.cloud_sub_1 = rospy.Subscriber("/first_r200/camera/depth_registered/points", PointCloud2,
                                       lambda m: pc2_callback1(m, pc_obj), queue_size=1, buff_size=cloud_sub_buf_size)
        self.cloud_sub_2 = rospy.Subscriber("/second_r200/camera/depth_registered/points", PointCloud2,
                                       lambda m: pc2_callback2(m, pc_obj), queue_size=1, buff_size=cloud_sub_buf_size)
        self.cloud_sub_3 = rospy.Subscriber("/third_r200/camera/depth_registered/points", PointCloud2,
                                       lambda m: pc2_callback3(m, pc_obj), queue_size=1, buff_size=cloud_sub_buf_size)



def pc2_callback1(ros_point_cloud, pc_obj):
    # Initialize variables.
    points_list = []

    sorted_points_list = []
    for i in range(num_z_groups):
        sorted_points_list.append([[0, 0, 0]])

    start_flag = False
    z_min = 0
    z_max = 0
    
    # Read from raw data.
    for data in pc2.read_points(ros_point_cloud, skip_nans=True):
        # Record min and max coordinates of z
        if start_flag == False:
            z_max = data[2]
            z_min = data[2]
            start_flag = True
        else:
            if data[2] > z_max:
                z_max = data[2]
            if data[2] < z_min:
                z_min = data[2]

        # Append to points_list from the raw data
        # each line in points_list = [x, y, z]
        points_list.append([data[0], data[1], data[2]])

    # The length margin of each z axis group 
    group_len = float((z_max - z_min) / num_z_groups)
    
    # Sort each point into the list of different groups
    for line in points_list:
        # Get the group index of this point
        group_ind = int((line[2] - z_min) / group_len)

        # If the current point is the point with z_max 
        if group_ind == num_z_groups:
            group_ind -= 1

        # Append.
        sorted_points_list[group_ind].append(line)

    # Remove place-holder entry [0, 0, 0]
    for i in range(num_z_groups):
        sorted_points_list[i].pop(0)

    # Update global list (of this camera)
    pc_obj.points_array_list[0] = sorted_points_list
    # print(len(pc_obj.points_array_list[0][0]))
    if len(pc_obj.points_array_list[0][0]) > 0:
        pc_obj.cam1_flag = True
        pc_obj.cloud_sub_1.unregister()



def pc2_callback2(ros_point_cloud, pc_obj):
    # Initialize variables.
    pc_obj.cam2_flag = True
    points_list = []

    sorted_points_list = []
    for i in range(num_z_groups):
        sorted_points_list.append([[0, 0, 0]])

    start_flag = False
    z_min = 0
    z_max = 0

    # Read from raw data.
    for data in pc2.read_points(ros_point_cloud, skip_nans=True):
        # Record min and max coordinates of z
        if start_flag == False:
            z_max = data[2]
            z_min = data[2]
            start_flag = True
        else:
            if data[2] > z_max:
                z_max = data[2]
            if data[2] < z_min:
                z_min = data[2]

        # Append to points_list from the raw data
        # each line in points_list = [x, y, z]
        points_list.append([data[0], data[1], data[2]])

    # The length margin of each z axis group
    group_len = float((z_max - z_min) / num_z_groups)

    # Sort each point into the list of different groups
    for line in points_list:
        # Get the group index of this point
        group_ind = int((line[2] - z_min) / group_len)

        # If the current point is the point with z_max
        if group_ind == num_z_groups:
            group_ind -= 1

        # Append.
        sorted_points_list[group_ind].append(line)

    # Remove place-holder entry [0, 0, 0]
    for i in range(num_z_groups):
        sorted_points_list[i].pop(0)

    # Update global list (of this camera)
    pc_obj.points_array_list[1] = sorted_points_list
    # print(len(points_array_list[cam_num][0]))
    if len(pc_obj.points_array_list[1][0]) > 0:
        pc_obj.cam2_flag = True
        pc_obj.cloud_sub_2.unregister()

def pc2_callback3(ros_point_cloud, pc_obj):
    # Initialize variables.
    pc_obj.cam3_flag = True
    points_list = []

    sorted_points_list = []
    for i in range(num_z_groups):
        sorted_points_list.append([[0, 0, 0]])

    start_flag = False
    z_min = 0
    z_max = 0

    # Read from raw data.
    for data in pc2.read_points(ros_point_cloud, skip_nans=True):
        # Record min and max coordinates of z
        if start_flag == False:
            z_max = data[2]
            z_min = data[2]
            start_flag = True
        else:
            if data[2] > z_max:
                z_max = data[2]
            if data[2] < z_min:
                z_min = data[2]

        # Append to points_list from the raw data
        # each line in points_list = [x, y, z]
        points_list.append([data[0], data[1], data[2]])

    # The length margin of each z axis group
    group_len = float((z_max - z_min) / num_z_groups)

    # Sort each point into the list of different groups
    for line in points_list:
        # Get the group index of this point
        group_ind = int((line[2] - z_min) / group_len)

        # If the current point is the point with z_max
        if group_ind == num_z_groups:
            group_ind -= 1

        # Append.
        sorted_points_list[group_ind].append(line)

    # Remove place-holder entry [0, 0, 0]
    for i in range(num_z_groups):
        sorted_points_list[i].pop(0)

    # Update global list (of this camera)
    pc_obj.points_array_list[2] = sorted_points_list
    # print(len(points_array_list[cam_num][0]))
    if len(pc_obj.points_array_list[2][0]) > 0:
        pc_obj.cam3_flag = True
        pc_obj.cloud_sub_3.unregister()

if __name__ == '__main__':
    pc_obj = pc_container()
    rospy.init_node('pc2_sub', anonymous=True)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        if pc_obj.cam1_flag == True and pc_obj.cam2_flag == True and pc_obj.cam3_flag == True:
            print("1", len(pc_obj.points_array_list[0]))
            print("2", len(pc_obj.points_array_list[1]))
            print("3", len(pc_obj.points_array_list[2]))
            pc_obj.cam1_flag, pc_obj.cam2_flag, pc_obj.cam3_flag = False, False, False
            break
        rate.sleep()  # Delays while loop to match the specified rate

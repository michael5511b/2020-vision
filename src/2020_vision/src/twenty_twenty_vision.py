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
points_array_1 = []
points_array_2 = []
points_array_3 = []
points_array_list = [points_array_1, points_array_2, points_array_3]


def pc2_callback(ros_point_cloud, cam_num):
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
    points_array_list[cam_num] = sorted_points_list
    print(len(points_array_list[cam_num][0]))


if __name__ == '__main__':
    rospy.init_node('pc2_sub', anonymous=True)
    cloud_sub_1 = rospy.Subscriber("/first_r200/camera/depth_registered/points", PointCloud2, pc2_callback, 0, queue_size=1, buff_size=cloud_sub_buf_size)
    cloud_sub_2 = rospy.Subscriber("/second_r200/camera/depth_registered/points", PointCloud2, pc2_callback, 1, queue_size=1, buff_size=cloud_sub_buf_size)
    cloud_sub_3 = rospy.Subscriber("/third_r200/camera/depth_registered/points", PointCloud2, pc2_callback, 2, queue_size=1, buff_size=cloud_sub_buf_size)
    rospy.spin()
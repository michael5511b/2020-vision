#! /usr/bin/env python
import ctypes
import numpy as np
import rospy
from sensor_msgs.msg import PointCloud2, PointField
import sensor_msgs.point_cloud2 as pc2
import struct
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
import time
import open3d as o3d

# Definitions
cloud_sub_buf_size = 52428800
# Number of groups we want to divide the point clouds into along the z axis
num_z_groups = int(rospy.get_param('twenty_twenty_vision/num_z_groups')) 


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

# Globals
rpy_1, rpy_2, rpy_3 = [0, 0.3927, 0.7854], [0, 0.3927, -0.7854], [0, 0.3927, 3.927]
translation_1, translation_2, translation_3 = [0.4, 0.4, 2.3], [0.4, 9.6, 2.3], [9.6, 9.6, 2.3]

def points_to_list(ros_point_cloud, rpy, translation):
    # Transform setup.
    rot_x = [[1, 0, 0],
            [0, np.cos(rpy[0]), -np.sin(rpy[0])],
            [0, np.sin(rpy[0]), np.cos(rpy[0])]]

    rot_y = [[np.cos(rpy[1]), 0, np.sin(rpy[1])],
            [0, 1, 0],
            [-np.sin(rpy[1]), 0, np.cos(rpy[1])]]

    rot_z = [[np.cos(rpy[2]), -np.sin(rpy[1]), 0],
            [np.sin(rpy[2]), np.cos(rpy[1]), 0],
            [0, 0, 1]]

    # transform = np.linalg.inv(np.dot(rot_x, np.dot(rot_y, rot_z)))
    transform = np.dot(rot_x, np.dot(rot_y, rot_z))
    translation = np.transpose([translation])
    # translation = np.transpose([[0, 0, 0]])
    transform = np.append(transform, translation, axis=1)
    transform = np.append(transform, [[0, 0, 0, 1]], axis=0)
    # transform = np.linalg.inv(transform)

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
        # Transform of the points from camera frame to world frame (x, y, z)
        raw_world_coord = np.transpose([[data[0], data[2], -data[1], 1]])
        # raw_world_coord = np.transpose([[data[0], data[1], data[2], 1]])
        # transform = np.eye(4)
        world_coord = np.dot(transform, raw_world_coord)
        x, y, z = world_coord[0][0], world_coord[1][0], world_coord[2][0] 

        # Record min and max world_coordinates of z
        if start_flag == False:
            z_max = z
            z_min = z
            start_flag = True
        else:
            if z > z_max:
                z_max = z
            if z < z_min:
                z_min = z

        # Append to points_list from the raw data
        # each line in points_list = [x, y, z]
        points_list.append([x, y, z])
        

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

    return sorted_points_list

def pc2_callback1(ros_point_cloud, pc_obj):
    sorted_points_list = points_to_list(ros_point_cloud, rpy_1, translation_1)
    
    # Update global list (of this camera)
    pc_obj.points_array_list[0] = sorted_points_list
    # print(len(pc_obj.points_array_list[0][0]))
    if len(pc_obj.points_array_list[0][0]) > 0:
        pc_obj.cam1_flag = True
        pc_obj.cloud_sub_1.unregister()


def pc2_callback2(ros_point_cloud, pc_obj):
    sorted_points_list = points_to_list(ros_point_cloud, rpy_2, translation_2)

    # Update global list (of this camera)
    pc_obj.points_array_list[1] = sorted_points_list
    # print(len(points_array_list[cam_num][0]))
    if len(pc_obj.points_array_list[1][0]) > 0:
        pc_obj.cam2_flag = True
        pc_obj.cloud_sub_2.unregister()

def pc2_callback3(ros_point_cloud, pc_obj):
    sorted_points_list = points_to_list(ros_point_cloud, rpy_3, translation_3)

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
            for i in range(5):
                print("Slice Number: " + str(i), len(pc_obj.points_array_list[0][i]))
            # print("2", len(pc_obj.points_array_list[1]))
            # print("3", len(pc_obj.points_array_list[2]))
            pc_obj.cam1_flag, pc_obj.cam2_flag, pc_obj.cam3_flag = False, False, False
            break
        rate.sleep()  # Delays while loop to match the specified rate

    voxel_size = 0.03  # Voxel size for downsampling
    print("AFTER ROS WHILE LOOP")
    pcd1 = o3d.geometry.PointCloud()  # Create point cloud object
    pcd1.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[0][0])  # Input point cloud points into object
    o3d.visualization.draw_geometries([pcd1])  # Visualize the points
    # pcd1_numpy = np.asarray(pcd1.points)  # Convert points to numpy array

    # downpcd1 = pcd1.voxel_down_sample(voxel_size=0.03)
    # o3d.visualization.draw_geometries([downpcd1])

    # downpcd1.estimate_normals(search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.1, max_nn=30))
    # o3d.visualization.draw_geometries([downpcd1])
    #
    #
    # cluster = downpcd1.cluster_dbscan(0.05, 50)
    # print(cluster)
    #
    pcd2 = o3d.geometry.PointCloud()
    pcd2.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[0][1])
    o3d.visualization.draw_geometries([pcd2])
    
    pcd3 = o3d.geometry.PointCloud()
    pcd3.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[0][2])
    o3d.visualization.draw_geometries([pcd3])
    
    pcd4 = o3d.geometry.PointCloud()
    pcd4.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[0][3])
    o3d.visualization.draw_geometries([pcd4])

    
    
    pcd5 = o3d.geometry.PointCloud()
    pcd5.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[0][4])
    o3d.visualization.draw_geometries([pcd5])

    o3d.visualization.draw_geometries([pcd1, pcd2, pcd3, pcd4, pcd5])





    # # Not working
    # with open("cam1.txt", "w+") as f:
    #     for pt in pc_obj.points_array_list[0][0]:
    #         print(pt)
    #         f.write(str(pt))
    #     f.close()

    # with h5py.File("random.hdf5", "w") as f:
    #     dset = f.create_dataset("cam1", data=pc_obj.points_array_list[0][0])


    # with h5py.File('random.hdf5', 'r') as f:
    #    data = f['cam1']
    #    print(min(data))
    #    print(max(data))
    #    print(data[:15])
    #
    # with h5py.File("random.hdf5", "w") as f:
    #     dset = f.create_dataset("cam1", data=a)
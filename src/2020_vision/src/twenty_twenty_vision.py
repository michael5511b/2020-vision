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
import matplotlib.pyplot as plt
import scipy.spatial

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
        # self.cloud_sub_1 = rospy.Subscriber("/first_r200/camera/depth_registered/points", PointCloud2,
        #                                lambda m: pc2_callback1(m, pc_obj), queue_size=1, buff_size=cloud_sub_buf_size)
        self.cloud_sub_2 = rospy.Subscriber("/second_r200/camera/depth_registered/points", PointCloud2,
                                       lambda m: pc2_callback2(m, pc_obj), queue_size=1, buff_size=cloud_sub_buf_size)
        # self.cloud_sub_3 = rospy.Subscriber("/third_r200/camera/depth_registered/points", PointCloud2,
        #                                lambda m: pc2_callback3(m, pc_obj), queue_size=1, buff_size=cloud_sub_buf_size)

# Globals
rpy_1, rpy_2, rpy_3 = [0, 0.3927, 0.7854], [0, 0.3927, -0.7854], [0, 0.3927, 3.927]
translation_1, translation_2, translation_3 = [0.4, 0.4, 2.3], [0.4, 9.6, 2.3], [9.6, 9.6, 2.3]

def points_to_list(ros_point_cloud, rpy, translation, cam_num):
    # # Transform stuff that didn't work.
    # # Transform setup.
    # rot_x = [[1, 0, 0, 0],
    #         [0, np.cos(rpy[0]), -np.sin(rpy[0]), 0],
    #         [0, np.sin(rpy[0]), np.cos(rpy[0]), 0],
    #         [0, 0, 0, 1]]

    # rot_y = [[np.cos(rpy[1]), 0, np.sin(rpy[1]), 0],
    #         [0, 1, 0, 0],
    #         [-np.sin(rpy[1]), 0, np.cos(rpy[1]), 0],
    #         [0, 0, 0, 1]]

    # rot_z = [[np.cos(rpy[2]), -np.sin(rpy[2]), 0, 0],
    #         [np.sin(rpy[2]), np.cos(rpy[2]), 0, 0],
    #         [0, 0, 1, 0],
    #         [0, 0, 0, 1]]

    # trans = [[1, 0, 0, translation[0]],
    #          [0, 1, 0, translation[1]],
    #          [0, 0, 1, translation[2]],
    #          [0, 0, 0, 1]]

    # # trans = np.linalg.inv(trans)
    # rot = np.dot(rot_x, np.dot(rot_z, rot_y))
    # transform = np.dot(trans, rot)
    
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
        # Lame hack to fix the wierd raw pointcloud coordinates
        x, y, z = data[2], data[0], -data[1]

        # No walls.
        if x + y > 8 * np.sqrt(2) or x - y > 8 * np.sqrt(2):
            continue

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

    # remove the group with the floor
    sorted_points_list.pop(0)

    return sorted_points_list

def pc2_callback1(ros_point_cloud, pc_obj):
    sorted_points_list = points_to_list(ros_point_cloud, rpy_1, translation_1, 1)
    
    # Update global list (of this camera)
    pc_obj.points_array_list[0] = sorted_points_list
    # print(len(pc_obj.points_array_list[0][0]))
    if len(pc_obj.points_array_list[0][0]) > 0:
        pc_obj.cam1_flag = True
        pc_obj.cloud_sub_1.unregister()


def pc2_callback2(ros_point_cloud, pc_obj):
    sorted_points_list = points_to_list(ros_point_cloud, rpy_2, translation_2, 2)

    # Update global list (of this camera)
    pc_obj.points_array_list[1] = sorted_points_list
    # print(len(points_array_list[cam_num][0]))
    if len(pc_obj.points_array_list[1][0]) > 0:
        pc_obj.cam2_flag = True
        pc_obj.cloud_sub_2.unregister()

def pc2_callback3(ros_point_cloud, pc_obj):
    sorted_points_list = points_to_list(ros_point_cloud, rpy_3, translation_3, 3)

    # Update global list (of this camera)
    pc_obj.points_array_list[2] = sorted_points_list
    # print(len(points_array_list[cam_num][0]))
    if len(pc_obj.points_array_list[2][0]) > 0:
        pc_obj.cam3_flag = True
        pc_obj.cloud_sub_3.unregister()


def cluster_centers(pts, labels):
    pts = np.asarray(pts)  # Input as open3d.points; convert to np array; matrix size (Npts x 3)
    max_label = labels.max()  # Max label + 1 is number of clusters; labels obtained from dbscan function
    cluster_pts_list = []
    cluster_centers_list = []
    for i in range(max_label+1):
        cluster_pts = pts[np.where(labels == i)[0], :]  # Take pts of label i
        cluster_pts_list.append(cluster_pts)
        xmean = cluster_pts[:, 0].mean()
        ymean = cluster_pts[:, 1].mean()
        zmean = cluster_pts[:, 2].mean()
        cluster_centers_list.append([xmean, ymean, zmean])

    return cluster_centers_list, cluster_pts_list

def calculate_distances(cluster_centers, thresh):
    # print("cluster_centers", cluster_centers)
    A = np.array(cluster_centers)[:, 0:2]  # Select only the x,y coordinates
    distances = scipy.spatial.distance.cdist(A, A)  # jth col is distances to pt at ith row in 2nd argument
    print("Cluster center distances:")
    print(distances)
    below_thresh = np.argwhere(distances < thresh)  # Gives list of index pairs of points within thresh of each other
    sm = np.argsort(below_thresh, axis=1)  # For each row, determine order
    socially_undistant = np.take_along_axis(below_thresh, sm, axis=1)  # Sort values in each row to determine duplicates
    socially_undistant = np.unique(socially_undistant, axis=0)  # Remove duplicate rows
    socially_undistant = socially_undistant[socially_undistant[:, 0] != socially_undistant[:, 1]]  # Remove self-compared points
    return socially_undistant



if __name__ == '__main__':
    pc_obj = pc_container()
    rospy.init_node('pc2_sub', anonymous=True)

    rate = rospy.Rate(30)
    while not rospy.is_shutdown():
        # if pc_obj.cam1_flag == True and pc_obj.cam2_flag == True and pc_obj.cam3_flag == True:
        if pc_obj.cam2_flag == True:
            for i in range(len(pc_obj.points_array_list[0])):
                print("Slice Number: " + str(i), len(pc_obj.points_array_list[0][i]))
            # pc_obj.cam1_flag, pc_obj.cam2_flag, pc_obj.cam3_flag = False, False, False
            pc_obj.cam2_flag = False
            break
        rate.sleep()  # Delays while loop to match the specified rate

    voxel_size = 0.03  # Voxel size for downsampling
    
    # Visualization
    pcd1 = o3d.geometry.PointCloud()  # Create point cloud object
    pcd1.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[1][0])  # Input point cloud points into object
    downpcd1 = pcd1.voxel_down_sample(voxel_size=0.03)  # Downsample

    pcd2 = o3d.geometry.PointCloud()  # Create point cloud object
    pcd2.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[1][1])  # Input point cloud points into object
    downpcd2 = pcd2.voxel_down_sample(voxel_size=0.03)  # Downsample

    pcd3 = o3d.geometry.PointCloud()  # Create point cloud object
    pcd3.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[1][2])  # Input point cloud points into object
    downpcd3 = pcd3.voxel_down_sample(voxel_size=0.03)  # Downsample

    pcd4 = o3d.geometry.PointCloud()  # Create point cloud object
    pcd4.points = o3d.utility.Vector3dVector(pc_obj.points_array_list[1][3])  # Input point cloud points into object
    downpcd4 = pcd4.voxel_down_sample(voxel_size=0.03)  # Downsample

    # o3d.visualization.draw_geometries([downpcd1])
    # o3d.visualization.draw_geometries([downpcd2])
    # o3d.visualization.draw_geometries([downpcd3])
    # o3d.visualization.draw_geometries([downpcd4])
    # o3d.visualization.draw_geometries([downpcd1, downpcd2, downpcd3, downpcd4])


    with o3d.utility.VerbosityContextManager(o3d.utility.VerbosityLevel.Debug) as cm:
        labels = np.array(downpcd1.cluster_dbscan(eps=0.4, min_points=20, print_progress=True))  
    # eps = max Eucl dist between 2 pts that should belong to same cluster
    # min_points = minimum number of pts requires to create a cluster



    max_label = labels.max()
    print()
    print(f"point cloud has {max_label + 1} clusters")
    colors = plt.get_cmap("tab20")(labels / (max_label if max_label > 0 else 1))
    colors[labels < 0] = 0
    downpcd1.colors = o3d.utility.Vector3dVector(colors[:, :3])
    # o3d.visualization.draw_geometries([downpcd1], zoom=0.455, front=[-0.4999, -0.1659, -0.8499], lookat=[2.1813, 2.0619, 2.0999], up=[0.1204, -0.9852, 0.1215])

    cluster_centers_list, cluster_pts_list = cluster_centers(downpcd1.points, labels)
    # print("cluster list", cluster_pts_list)
    print("Cluster means:")
    print("Cluster 1: ", cluster_centers_list[0])
    print("Cluster 2: ", cluster_centers_list[1])
    print("Cluster 3: ", cluster_centers_list[2])
    print("Cluster 4: ", cluster_centers_list[3])
    print()

    cluster1 = o3d.geometry.PointCloud()
    cluster1.points = o3d.utility.Vector3dVector(np.append(cluster_pts_list[0], np.array([cluster_centers_list[0]]), axis=0))

    socially_undistant = calculate_distances(cluster_centers_list, thresh=1.8)
    print()
    print("Not socially distant clusters: ", socially_undistant)

    # Visualization of result:

    # Walls
    corner_1 = [0, 0]
    corner_2 = [5 * np.sqrt(2), 5 * np.sqrt(2)]
    corner_3 = [10 * np.sqrt(2), 0]
    corner_4 = [5 * np.sqrt(2), -5 * np.sqrt(2)]
    x_values = [corner_1[0], corner_2[0]]
    y_values = [corner_1[1], corner_2[1]]
    plt.plot(x_values, y_values, 'b')
    x_values = [corner_2[0], corner_3[0]]
    y_values = [corner_2[1], corner_3[1]]
    plt.plot(x_values, y_values, 'b')
    x_values = [corner_3[0], corner_4[0]]
    y_values = [corner_3[1], corner_4[1]]
    plt.plot(x_values, y_values, 'b')
    x_values = [corner_1[0], corner_4[0]]
    y_values = [corner_1[1], corner_4[1]]
    plt.plot(x_values, y_values, 'b')
    
        
    # Plot cluster centers
    cluster_x = []
    cluster_y = []
    for i in range(len(cluster_centers_list)):
        cluster_x.append(cluster_centers_list[i][0])
        cluster_y.append(cluster_centers_list[i][1])
    plt.scatter(cluster_x,cluster_y,s=100,c='red')

    # Annotate cluster centers
    plt.annotate("0", # this is the text
                 (cluster_x[0],cluster_y[0]), # this is the point to label
                 textcoords="offset points", # how to position the text
                 xytext=(0,10), # distance from text to points (x,y)
                 ha='center') # horizontal alignment can be left, right or center
    plt.annotate("1", # this is the text
                 (cluster_x[1],cluster_y[1]), # this is the point to label
                 textcoords="offset points", # how to position the text
                 xytext=(0,10), # distance from text to points (x,y)
                 ha='center') # horizontal alignment can be left, right or center
    plt.annotate("2", # this is the text
                 (cluster_x[2],cluster_y[2]), # this is the point to label
                 textcoords="offset points", # how to position the text
                 xytext=(0,10), # distance from text to points (x,y)
                 ha='center') # horizontal alignment can be left, right or center
    plt.annotate("3", # this is the text
                 (cluster_x[3],cluster_y[3]), # this is the point to label
                 textcoords="offset points", # how to position the text
                 xytext=(0,10), # distance from text to points (x,y)
                 ha='center') # horizontal alignment can be left, right or center
    
    # Mark x on the not social distant people
    for pair in socially_undistant:
        plt.scatter(cluster_x[pair[0]], cluster_y[pair[0]], s=100, c='black', marker='x')
        plt.scatter(cluster_x[pair[1]], cluster_y[pair[1]], s=100, c='black', marker='x')
        

    plt.axis([-5, 10, -5, 10])
    plt.axis('square')
    plt.show()
    

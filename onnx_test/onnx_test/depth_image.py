import open3d as o3d
import numpy as np
import time
import random
import math
import matplotlib.pyplot as plt

from sklearn.cluster import KMeans

import cv2

# Load point cloud from PCD file
# color_raw = o3d.io.read_image("/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/color.png")
depth_raw = o3d.io.read_image("/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/depth16.png")
# rgbd_image = o3d.geometry.RGBDImage.create_from_color_and_depth(
#     color_raw, depth_raw)

# plt.subplot(1, 2, 1)
# plt.title('Redwood grayscale image')
# plt.imshow(rgbd_image.color)
# plt.subplot(1, 2, 2)
# plt.title('Redwood depth image')
# plt.imshow(rgbd_image.depth)
# plt.show()

fx = 2240.591797
fy = 2239.406738
cx = 1899.003296
cy = 1080.935913

camera_matrix = np.array([[fx,  0, cx],
                          [ 0, fy, cy],
                          [ 0,  0,  1]], dtype=np.float32)

dist_coeffs = np.array([0.079749, -0.110377, -0.000354, 0.000473, 0.046372], dtype=np.float32)

map1, map2 = cv2.initUndistortRectifyMap(camera_matrix, dist_coeffs, None, camera_matrix, (3840, 2160), cv2.CV_16SC2)

depth_image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/depth16.png', cv2.IMREAD_UNCHANGED)
undistorted_depth_image = cv2.remap(depth_image, map1, map2, interpolation=cv2.INTER_NEAREST)
depth_o3d = o3d.geometry.Image(undistorted_depth_image)

# test1 = np.asarray(depth_raw)
# test2 = np.asarray(o3d.geometry.Image(depth_image))

# plt.subplot(1, 2, 1)
# plt.title('Distorted')
# plt.imshow(depth_image)
# plt.subplot(1, 2, 2)
# plt.title('Undistorted')
# plt.imshow(undistorted_depth_image)
# plt.show()

intrinsics =o3d.camera.PinholeCameraIntrinsic(3840, 2160, fx, fy, cx, cy)
# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
#     rgbd_image,
#     intrinsics)
pcd = o3d.geometry.PointCloud.create_from_depth_image(
    o3d.geometry.Image(undistorted_depth_image),
    intrinsics)

downpcd = pcd.voxel_down_sample(voxel_size=0.01)
o3d.visualization.draw_geometries([downpcd])

intrinsics =o3d.camera.PinholeCameraIntrinsic(3840, 2160, fx, fy, cx, cy)
# pcd = o3d.geometry.PointCloud.create_from_rgbd_image(
#     rgbd_image,
#     intrinsics)
pcd = o3d.geometry.PointCloud.create_from_depth_image(
    o3d.geometry.Image(depth_image),
    intrinsics)

downpcd = pcd.voxel_down_sample(voxel_size=0.01)
o3d.visualization.draw_geometries([downpcd])


# o3d.io.write_point_cloud("/home/ofa/ros2_ws/src/ros-weed-control/onnx_test/pc_o3d.pcd", pcd, write_ascii=True)

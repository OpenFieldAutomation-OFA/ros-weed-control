import open3d as o3d
import numpy as np
import time
import random
import math
import matplotlib.pyplot as plt

from sklearn.cluster import KMeans

import cv2

pcd = o3d.io.read_point_cloud("/home/ofa/ros2_ws/src/ros-weed-control/onnx_test/pcl.pcd")
print(pcd)

fx = 2240.591797
fy = 2239.406738
cx = 1899.003296
cy = 1080.935913

camera_matrix = np.array([[fx,  0, cx],
                          [ 0, fy, cy],
                          [ 0,  0,  1]], dtype=np.float32)

dist_coeffs = np.array([0.079749, -0.110377, -0.000354, 0.000473, 0.046372], dtype=np.float32)

point_3d = np.array([[-207.38992, 75.742683, 706]], dtype=np.float32)
rvec = np.zeros((3, 1), dtype=np.float32)  # Assuming no rotation
tvec = np.zeros((3, 1), dtype=np.float32)  # Assuming the camera is at the origin

points_2d, _ = cv2.projectPoints(point_3d, rvec, tvec, camera_matrix, dist_coeffs)

print("Projected 2D point:", points_2d)
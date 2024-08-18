import open3d as o3d
import numpy as np
import time
import random
import math

from sklearn.cluster import KMeans, DBSCAN
import cv2
import matplotlib.pyplot as plt

draw_image = True

# Projection parameters
fx = 2240.591797
fy = 2239.406738
cx = 1899.003296
cy = 1080.935913
camera_matrix = np.array([[fx,  0, cx],
                          [ 0, fy, cy],
                          [ 0,  0,  1]], dtype=np.float32)
dist_coeffs = np.array([0.079749, -0.110377, -0.000354, 0.000473, 0.046372], dtype=np.float32)
rvec = np.zeros((3, 1), dtype=np.float32)
tvec = np.zeros((3, 1), dtype=np.float32)

# Load point cloud from PCD file
pcd = o3d.io.read_point_cloud("/home/ofa/ros2_ws/src/ros-weed-control/onnx_test/pcl.pcd")
print(pcd)

pcd = pcd.voxel_down_sample(2.0)
print(pcd)

# start = time.time()
# points = np.asarray(pcd.points)
# db = DBSCAN(eps=10.0, min_samples=100).fit(points)
# labels = db.labels_
# end = time.time()
# print(f"Cluster time sklearn: {end-start}")

points = np.asarray(pcd.points)
points[:, 2] /= 2.0
pcd.points = o3d.utility.Vector3dVector(points)

start = time.time()
labels = np.array(pcd.cluster_dbscan(eps=10.0, min_points=20))
end = time.time()
print(f"Cluster time open3d: {end-start}")

points = np.asarray(pcd.points)
points[:, 2] *= 2.0
pcd.points = o3d.utility.Vector3dVector(points)


colors = np.zeros((len(labels), 3))

# Print the number of clusters
max_label = labels.max()
print(f"Point cloud has {max_label + 1} clusters")

bounding_boxes = []
positions = []

start = time.time()
points_2d = cv2.projectPoints(np.asarray(pcd.points), rvec, tvec, camera_matrix, dist_coeffs)[0]
end = time.time()
print(f"Reprojection time: {end-start}")

if draw_image:
    cluster_image = np.ones((2160, 3840, 3), dtype=np.uint8) * 255
    color_image = cv2.imread("/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/color.png")

start = time.time()
for i in range(max_label + 1):
    cluster_points = pcd.select_by_index(np.where(labels == i)[0])
    # print(len(cluster_points.points))
    k = math.ceil(len(cluster_points.points) / 5000)
    points = np.asarray(cluster_points.points)
    # points[:, 2] /= 2.0

    # Compute bounding box
    points_2d = cv2.projectPoints(points, rvec, tvec, camera_matrix, dist_coeffs)[0]
    points_2d = np.round(points_2d).astype(int).reshape(-1, 2)
    min_col = np.min(points_2d[:, 0])
    max_col = np.max(points_2d[:, 0])
    min_row = np.min(points_2d[:, 1])
    max_row = np.max(points_2d[:, 1])
    bounding_boxes.append([min_col, max_col, min_row, max_row])

    cluster_color = np.random.rand(3)
    colors[labels == i] = cluster_color
    if draw_image:
        cluster_color *= 255
        for point in points_2d:
            cluster_image[point[1], point[0]] = cluster_color
        cv2.rectangle(cluster_image, (min_col, min_row), (max_col, max_row), cluster_color, 10)
        cv2.rectangle(color_image, (min_col, min_row), (max_col, max_row), (255, 0, 0), 10)

    if k == 1:
        centroid = points.mean(axis=0)
        positions.append(centroid)
        if draw_image:
            centroid_2d = cv2.projectPoints(centroid, rvec, tvec, camera_matrix, dist_coeffs)[0]
            centroid_2d = np.round(centroid_2d).astype(int).reshape(-1, 2)
            cv2.circle(cluster_image, (centroid_2d[0, 0], centroid_2d[0, 1]), radius=10, color=cluster_color, thickness=-1)
            cv2.circle(color_image, (centroid_2d[0, 0], centroid_2d[0, 1]), radius=10, color=(0, 0, 255), thickness=-1)
    else:
        # Perform K-Means clustering
        kmeans = KMeans(n_clusters=k, n_init=1).fit(points)
        
        # Get centroids of the subclouds
        centroids = kmeans.cluster_centers_

        positions.append(centroids)
        if draw_image:
            centroids_2d = cv2.projectPoints(centroids, rvec, tvec, camera_matrix, dist_coeffs)[0]
            centroids_2d = np.round(centroids_2d).astype(int).reshape(-1, 2)
            for centroid in centroids_2d:
                cv2.circle(cluster_image, (centroid[0], centroid[1]), radius=10, color=cluster_color, thickness=-1)
                cv2.circle(color_image, (centroid[0], centroid[1]), radius=10, color=(0, 0, 255), thickness=-1)


end = time.time()
print(f"KMeans Time: {end-start}")

# print("Bounding Boxes:")
# print(bounding_boxes)
# print("Positions:")
# print(positions)

if draw_image:
    plt.imshow(cv2.cvtColor(cluster_image, cv2.COLOR_BGR2RGB))
    plt.show()
    plt.imshow(cv2.cvtColor(color_image, cv2.COLOR_BGR2RGB))
    plt.show()

# Generate a random color for each cluster
colors = np.zeros((len(labels), 3))  # Initialize all points with black color

for i in range(max_label + 1):
    # Assign a random color to each cluster
    cluster_color = [random.uniform(0, 1) for _ in range(3)]
    colors[labels == i] = cluster_color

# Assign black color to noise points
colors[labels == -1] = [0, 0, 0]

# Apply colors to the point cloud
pcd.colors = o3d.utility.Vector3dVector(colors)

# Visualize the clustered point cloud
o3d.visualization.draw_geometries([pcd])
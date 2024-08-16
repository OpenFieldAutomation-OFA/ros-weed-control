import open3d as o3d
import numpy as np
import time
import random
import math

from sklearn.cluster import KMeans

# Load point cloud from PCD file
pcd = o3d.io.read_point_cloud("/home/ofa/ros2_ws/src/ros-weed-control/onnx_test/pcl.pcd")
print(pcd)

start = time.time()
labels = np.array(pcd.cluster_dbscan(eps=10.0, min_points=100))
end = time.time()
print(f"Cluster time: {end-start}")

# Print the number of clusters
max_label = labels.max()
print(f"Point cloud has {max_label + 1} clusters")

start = time.time()
for i in range(max_label + 1):
    # Extract points that belong to cluster `i`
    cluster_points = pcd.select_by_index(np.where(labels == i)[0])
    print(len(cluster_points.points))
    k = math.ceil(len(cluster_points.points) / 5000)
    points = np.asarray(pcd.points)
    if k == 1:
        centroid = points.mean(axis=0)
        print(f"Only one centroid: {centroid}")
    else:      
        # Perform K-Means clustering
        kmeans = KMeans(n_clusters=k, n_init=1).fit(points)
        
        # Get centroids of the subclouds
        centroids = kmeans.cluster_centers_
        print(f'Multiple centroids: {centroids}')
        
        # Assign colors to each subcloud (optional)
        # labels = kmeans.labels_
        # colors = np.zeros((points.shape[0], 3))
        # for i in range(k):
        #     colors[labels == i] = np.random.uniform(0, 1, 3)
        
        # pcd.colors = o3d.utility.Vector3dVector(colors)

        
        # # Save the cluster to a new PCD file
        # o3d.io.write_point_cloud(f"cluster_{i}.pcd", cluster_points)

end = time.time()
print(f"KMeans Time: {end-start}")


# # Generate a random color for each cluster
# colors = np.zeros((len(labels), 3))  # Initialize all points with black color

# for i in range(max_label + 1):
#     # Assign a random color to each cluster
#     cluster_color = [random.uniform(0, 1) for _ in range(3)]
#     colors[labels == i] = cluster_color

# # Assign black color to noise points
# colors[labels == -1] = [0, 0, 0]

# # Apply colors to the point cloud
# pcd.colors = o3d.utility.Vector3dVector(colors)

# # Visualize the clustered point cloud
# o3d.visualization.draw_geometries([pcd])
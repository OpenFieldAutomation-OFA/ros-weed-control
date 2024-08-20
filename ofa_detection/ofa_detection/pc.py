import open3d as o3d
import numpy as np
import time
import random
import math
import os
import csv

from sklearn.cluster import KMeans
import cv2
import matplotlib.pyplot as plt
import onnxruntime as ort

save_runs = True
file_path = '/home/ofa/ros2_ws/src/ros-weed-control/runs/predictions.txt'
do_classification = True
main_crop = 'Zea mays L.'
# main_crop = 'Beta vulgaris L.'

if do_classification:
    # load mappings
    from ament_index_python.packages import get_package_share_directory
    package_share_directory = get_package_share_directory('ofa_detection')
    print(package_share_directory)
    class_mapping_file = os.path.join(package_share_directory, 'class_mapping.txt')
    species_mapping_file = os.path.join(package_share_directory, 'species_id_to_name.txt')
    with open(class_mapping_file) as f:
        class_mapping = {i: int(line.strip()) for i, line in enumerate(f)}
    species_mapping = {}
    with open(species_mapping_file, mode='r', encoding='utf-8') as file:
        reader = csv.reader(file, delimiter=';')
        next(reader)
        for row in reader:
            species_id = int(row[0].strip('"'))
            species_name = row[1].strip('"')
            species_mapping[species_id] = species_name
    

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
pcd = o3d.io.read_point_cloud("/home/ofa/ros2_ws/src/ros-weed-control/ofa_detection/pcl.pcd")
print(pcd)
# pcd.transform([[1, 0, 0, 0], [0, -1, 0, 0], [0, 0, -1, 0], [0, 0, 0, 1]])
# o3d.visualization.draw_geometries([pcd])


pcd = pcd.voxel_down_sample(2.0)
print(pcd)

start = time.time()
labels = np.array(pcd.cluster_dbscan(eps=10.0, min_points=40))
end = time.time()
print(f"Cluster time : {end-start}")

colors = np.zeros((len(labels), 3))

# Print the number of clusters
max_label = labels.max()
print(f"Point cloud has {max_label + 1} clusters")

bounding_boxes = []
positions = []

# load network
color_image = cv2.imread("/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/color.png")
providers = [
    ('TensorrtExecutionProvider', {
        'trt_engine_cache_enable': True,
        'trt_engine_cache_path': '/home/ofa/ros2_ws/src/ros-weed-control/ofa_detection/model/trt_engine',
    }),
    # 'CUDAExecutionProvider',
    'CPUExecutionProvider'
]
session = ort.InferenceSession("/home/ofa/ros2_ws/src/ros-weed-control/ofa_detection/model/finetuned.onnx", 
    providers=providers)
input_name = session.get_inputs()[0].name

if save_runs:
    cluster_drawn = np.ones((2160, 3840, 3), dtype=np.uint8) * 255
    color_drawn = cv2.imread("/home/ofa/ros2_ws/src/ros-weed-control/ofa_weed_detection/images/mock_images/back/color.png")
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.0
    font_thickness = 2
    box_thickness = 5
    if do_classification:
        save_data = []

start = time.time()
for i in range(max_label + 1):
    cluster_points = pcd.select_by_index(np.where(labels == i)[0])
    points = np.asarray(cluster_points.points)
    # points[:, 2] /= 2.0
    points_2d = cv2.projectPoints(points, rvec, tvec, camera_matrix, dist_coeffs)[0]
    points_2d = np.round(points_2d).astype(int).reshape(-1, 2)

    # Compute bounding box
    x1 = np.min(points_2d[:, 0])
    x2 = np.max(points_2d[:, 0])
    y1 = np.min(points_2d[:, 1])
    y2 = np.max(points_2d[:, 1])
    bounding_boxes.append([x1, x2, y1, y2])

    cluster_color = np.random.rand(3)
    colors[labels == i] = cluster_color
    if save_runs:
        cluster_color *= 255
        for point in points_2d:
            cluster_drawn[point[1], point[0]] = cluster_color
        cv2.rectangle(cluster_drawn, (x1, y1), (x2, y2), cluster_color, box_thickness)

    if do_classification:
        plant_image = color_image[y1:y2, x1:x2]
        plant_image = cv2.cvtColor(plant_image, cv2.COLOR_BGR2RGB)
        # plt.imshow(plant_image)
        # plt.show()

        # resize and crop
        target_size = 518
        height, width = plant_image.shape[:2]
        if width < height:
            new_width = target_size
            new_height = int(target_size * height / width)
        else:
            new_height = target_size
            new_width = int(target_size * width / height)
        plant_image = cv2.resize(plant_image, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        start_x = (new_width - target_size) // 2
        start_y = (new_height - target_size) // 2
        plant_image = plant_image[start_y:start_y + target_size, start_x:start_x + target_size]
        # plt.imshow(plant_image)
        # plt.show()

        # normalize
        mean=[123.675, 116.28, 103.53]
        std=[58.395, 57.12, 57.375]
        plant_image = (plant_image - mean) / std

        # convert dimensions
        plant_image = np.transpose(plant_image, (2, 0, 1))
        plant_image = np.expand_dims(plant_image, axis=0)
        plant_image = plant_image.astype(np.float32)

        # pass through network
        outputs = session.run(None, {input_name: plant_image})
        predictions = outputs[0][0]
        max_ind = np.argmax(predictions)
        prob = predictions[max_ind]
        print(f"Prediction: {max_ind}, {prob*100}%")
        species_name = species_mapping[class_mapping[max_ind]]
        text = str(i) + ", " + species_name  # add box id
        if save_runs:
            if species_name == main_crop:
                color = (0, 0, 0)
            else:
                color = (255, 0, 0)
            cv2.rectangle(color_drawn, (x1, y1), (x2, y2), color, box_thickness)
            text_size = cv2.getTextSize(text, font, font_scale, font_thickness)[0]
            if y1 - text_size[1] - 5 > 0:
                # Draw the label above the bounding box
                text_x = x1
                text_y = y1 - 5
                cv2.rectangle(color_drawn, (x1, y1 - text_size[1] - 5), (x1 + text_size[0], y1), color, -1)
            else:
                # Draw the label inside the bounding box
                text_x = x1
                text_y = y1 + text_size[1] + 5
                cv2.rectangle(color_drawn, (x1, y1), (x1 + text_size[0], y1 + text_size[1] + 5), color, -1)
            cv2.putText(color_drawn, text, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness)
            
        if species_name == main_crop:
            # main crop detected, don't compute positions
            continue
    else:
        if save_runs:
            cv2.rectangle(color_drawn, (x1, y1), (x2, y2), (255, 0, 0), box_thickness)

    # compute positions
    k = math.ceil(len(cluster_points.points) / 5000)
    if k == 1:
        centroid = points.mean(axis=0)
        positions.append([centroid[0], centroid[1], centroid[2]])
        if save_runs:
            centroid_2d = cv2.projectPoints(centroid, rvec, tvec, camera_matrix, dist_coeffs)[0]
            centroid_2d = np.round(centroid_2d).astype(int).reshape(-1, 2)
            cv2.circle(cluster_drawn, (centroid_2d[0, 0], centroid_2d[0, 1]), radius=10, color=cluster_color, thickness=-1)
            cv2.circle(color_drawn, (centroid_2d[0, 0], centroid_2d[0, 1]), radius=10, color=(0, 0, 255), thickness=-1)
    else:
        kmeans = KMeans(n_clusters=k, init='k-means++', n_init=1).fit(points)
        centroids = kmeans.cluster_centers_
        for centroid in centroids:
            positions.append([centroid[0], centroid[1], centroid[2]])
        if save_runs:
            centroids_2d = cv2.projectPoints(centroids, rvec, tvec, camera_matrix, dist_coeffs)[0]
            centroids_2d = np.round(centroids_2d).astype(int).reshape(-1, 2)
            for centroid in centroids_2d:
                cv2.circle(cluster_drawn, (centroid[0], centroid[1]), radius=10, color=cluster_color, thickness=-1)
                cv2.circle(color_drawn, (centroid[0], centroid[1]), radius=10, color=(0, 0, 255), thickness=-1)
    
    if do_classification:
        save_data.append(
            {"id": i,
             "species": species_name,
             "probability": prob * 100,
             "num_points": k},
        )

end = time.time()
print(f"Classification & KMeans Time: {end-start}")

# print("Bounding Boxes:")
# print(bounding_boxes)
# print("Positions:")
# print(positions)

if save_runs:
    if do_classification:
        with open(file_path, mode='w', newline='', encoding='utf-8') as file:
            writer = csv.writer(file)
            writer.writerow(["ID", "Predicted Species", "Probability", "Number of Points"])
            for entry in save_data:
                writer.writerow([entry["id"], entry["species"],
                                 entry['probability'], entry["num_points"]])


# Show visualizations (useful for debugging)
if save_runs:
    plt.imshow(cv2.cvtColor(cluster_drawn, cv2.COLOR_BGR2RGB))
    plt.show()
    plt.imshow(cv2.cvtColor(color_drawn, cv2.COLOR_BGR2RGB))
    plt.show()

# colors[labels == -1] = [0, 0, 0]
# pcd.colors = o3d.utility.Vector3dVector(colors)
# o3d.visualization.draw_geometries([pcd])

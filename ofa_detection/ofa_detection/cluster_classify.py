import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer

from geometry_msgs.msg import Vector3

from ofa_interfaces.action import ClusterClassify

from ament_index_python.packages import get_package_share_directory

import time
import cv2
import onnxruntime as ort
import numpy as np
import os
import open3d as o3d
import matplotlib.pyplot as plt
import math
import csv

from sklearn.cluster import KMeans
import matplotlib.pyplot as plt
import onnxruntime as ort

class ClusterClassifyActionServer(Node):

    def __init__(self):
        super().__init__('cluster_classify_action_server', automatically_declare_parameters_from_overrides=True)
        self._action_server = ActionServer(
            self,
            ClusterClassify,
            'cluster_classify',
            execute_callback=self.execute_callback)

        # load network
        self.model = self.get_parameter('model').get_parameter_value().string_value
        model_directory = '/home/ofa/ros2_ws/src/ros-weed-control/ofa_detection/model'
        providers = [
            ('TensorrtExecutionProvider', {
                'trt_engine_cache_enable': True,
                'trt_engine_cache_path': os.path.join(model_directory, 'trt_engine'),
            }),
            # 'CUDAExecutionProvider',
            'CPUExecutionProvider'
        ]
        if self.model == 'base':
            file = 'finetuned.onnx'
        elif self.model == 'small':
            file = 'finetuned_small.onnx'
        self.session = ort.InferenceSession(os.path.join(model_directory, file),
            providers=providers)
        self.get_logger().info('Loaded network.')
        self.input_name = self.session.get_inputs()[0].name

        # image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/ofa_detection/ofa_detection/00dcd0ff0c50e304d43e519f0eafc849.jpg')
        # self.get_logger().info(f"The value of the first pixel is: {image[0, 0]}")
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        # self.get_logger().info(f"The value of the first pixel is: {image[0, 0]}")
        # target_size = 518
        # height, width = image.shape[:2]
        # if width < height:
        #     new_width = target_size
        #     new_height = int(target_size * height / width)
        # else:
        #     new_height = target_size
        #     new_width = int(target_size * width / height)
        # image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        # start_x = (new_width - target_size) // 2
        # start_y = (new_height - target_size) // 2
        # image = image[start_y:start_y + target_size, start_x:start_x + target_size]
        # self.get_logger().info(f"The value of the first pixel is: {image[0, 0]}")
        # mean=[123.675, 116.28, 103.53]
        # std=[58.395, 57.12, 57.375]
        # image = (image - mean) / std
        # print(image[0, 0])
        # image = np.transpose(image, (2, 0, 1))
        # input_tensor = np.expand_dims(image, axis=0)
        # input_tensor = input_tensor.astype(np.float32)
        # input_name = self.session.get_inputs()[0].name
        # input_shape = self.session.get_inputs()[0].shape
        # self.get_logger().info(f"Input name: {input_name}, input shape: {input_shape}")
        # start = time.time()
        # outputs = self.session.run(None, {input_name: input_tensor})
        # end = time.time()
        # self.get_logger().info(f"Time: {end-start}")
        # output_name = self.session.get_outputs()[0].name
        # predictions = outputs[0][0]
        # max_indices = np.argmax(predictions)
        # print(predictions)
        # print(predictions[max_indices])
        # print(max_indices)

        # load mappings
        self.class_mapping = {}
        self.species_mapping = {}
        from ament_index_python.packages import get_package_share_directory
        package_share_directory = get_package_share_directory('ofa_detection')
        class_mapping_file = os.path.join(package_share_directory, 'mappings', 'class_mapping.txt')
        species_mapping_file = os.path.join(package_share_directory, 'mappings', 'species_id_to_name.txt')
        with open(class_mapping_file) as f:
            self.class_mapping = {i: int(line.strip()) for i, line in enumerate(f)}
        with open(species_mapping_file, mode='r', encoding='utf-8') as file:
            reader = csv.reader(file, delimiter=';')
            next(reader)
            for row in reader:
                species_id = int(row[0].strip('"'))
                species_name = row[1].strip('"')
                self.species_mapping[species_id] = species_name

        # Projection parameters
        fx = 2240.591797
        fy = 2239.406738
        cx = 1899.003296
        cy = 1080.935913
        self.camera_matrix = np.array([[fx,  0, cx],
                                [ 0, fy, cy],
                                [ 0,  0,  1]], dtype=np.float32)
        self.dist_coeffs = np.array([0.079749, -0.110377, -0.000354, 0.000473, 0.046372], dtype=np.float32)
        self.rvec = np.zeros((3, 1), dtype=np.float32)
        self.tvec = np.zeros((3, 1), dtype=np.float32)


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # get goal request
        ros_image = goal_handle.request.color_image
        height = ros_image.height
        width = ros_image.width
        color_image = np.array(ros_image.data, dtype=np.uint8).reshape((height, width, -1))
        ros_pc = goal_handle.request.pc
        point_cloud_np = np.frombuffer(ros_pc.data, dtype=np.float32).reshape((ros_pc.width, 3))
        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(point_cloud_np)
        folder = goal_handle.request.folder
        save_runs = goal_handle.request.save_runs

        # get parameters
        do_classification = self.get_parameter('do_classification').get_parameter_value().bool_value
        main_crop = self.get_parameter('main_crop').get_parameter_value().integer_value
        if self.model == 'base':
            increase_output = 0
        elif self.model == 'small':
            increase_output = 0.1

        log_text = ""

        # downsample cloud
        self.get_logger().info(f"Point cloud loaded with {len(pc.points)} points")
        start = time.time()
        pc = pc.voxel_down_sample(2.0)
        end = time.time()
        log_text += f"Downsample time: {round((end-start)*1000)} ms\n"
        self.get_logger().info(f"Point cloud downsampled to {len(pc.points)} points")

        # cluster cloud
        start = time.time()
        labels = np.array(pc.cluster_dbscan(eps=8.0, min_points=30))
        end = time.time()
        log_text += f"DBSCAN time: {round((end-start)*1000)} ms\n"
        max_label = labels.max()
        self.get_logger().info(f"Point cloud has {max_label + 1} clusters")

        colors = np.zeros((len(labels), 3))

        if save_runs:
            cluster_drawn = np.ones((2160, 3840, 3), dtype=np.uint8) * 255
            color_drawn = color_image.copy()
            font = cv2.FONT_HERSHEY_SIMPLEX
            font_scale = 1.0
            font_thickness = 2
            box_thickness = 5
            if do_classification:
                save_data = []
        
        positions = []

        # iterate over clusters
        start = time.time()
        for i in range(max_label + 1):
            cluster_points = pc.select_by_index(np.where(labels == i)[0])
            points = np.asarray(cluster_points.points)
            points_2d = cv2.projectPoints(points, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)[0]
            points_2d = np.round(points_2d).astype(int).reshape(-1, 2)

            # Compute bounding box
            x1 = np.min(points_2d[:, 0])
            x2 = np.max(points_2d[:, 0])
            y1 = np.min(points_2d[:, 1])
            y2 = np.max(points_2d[:, 1])

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

                # normalize
                mean=[123.675, 116.28, 103.53]
                std=[58.395, 57.12, 57.375]
                plant_image = (plant_image - mean) / std

                # convert dimensions
                plant_image = np.transpose(plant_image, (2, 0, 1))
                plant_image = np.expand_dims(plant_image, axis=0)
                plant_image = plant_image.astype(np.float32)

                # pass through network
                outputs = self.session.run(None, {self.input_name: plant_image})
                predictions = outputs[0][0]
                predictions[main_crop] += increase_output
                max_ind = np.argmax(predictions)
                prob = predictions[max_ind]
                self.get_logger().info(f"Prediction: {max_ind}, {prob*100}%")
                species_name = self.species_mapping[self.class_mapping[max_ind]]
                text = str(i) + ", " + species_name  # add box id
                if save_runs:
                    if max_ind == main_crop:
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
                    
                if max_ind == main_crop:
                    # main crop detected, don't compute positions
                    if save_runs:
                        save_data.append(
                            {"id": i,
                            "species": species_name,
                            "probability": prob * 100,
                            "num_points": 0}
                        )
                    continue
            else:
                if save_runs:
                    cv2.rectangle(color_drawn, (x1, y1), (x2, y2), (255, 0, 0), box_thickness)

            # compute positions
            k = math.ceil(len(cluster_points.points) / 5000)
            if k == 1:
                centroid = points.mean(axis=0)
                positions.append(Vector3(x=centroid[0], y=centroid[1], z=centroid[2]))
                if save_runs:
                    centroid_2d = cv2.projectPoints(centroid, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)[0]
                    centroid_2d = np.round(centroid_2d).astype(int).reshape(-1, 2)
                    cv2.circle(cluster_drawn, (centroid_2d[0, 0], centroid_2d[0, 1]), radius=10, color=cluster_color, thickness=-1)
                    cv2.circle(color_drawn, (centroid_2d[0, 0], centroid_2d[0, 1]), radius=10, color=(0, 0, 255), thickness=-1)
            else:
                kmeans = KMeans(n_clusters=k, init='k-means++', n_init=1).fit(points)
                centroids = kmeans.cluster_centers_
                for centroid in centroids:
                    positions.append(Vector3(x=centroid[0], y=centroid[1], z=centroid[2]))
                if save_runs:
                    centroids_2d = cv2.projectPoints(centroids, self.rvec, self.tvec, self.camera_matrix, self.dist_coeffs)[0]
                    centroids_2d = np.round(centroids_2d).astype(int).reshape(-1, 2)
                    for centroid in centroids_2d:
                        cv2.circle(cluster_drawn, (centroid[0], centroid[1]), radius=10, color=cluster_color, thickness=-1)
                        cv2.circle(color_drawn, (centroid[0], centroid[1]), radius=10, color=(0, 0, 255), thickness=-1)
            
            if do_classification and save_runs:
                save_data.append(
                    {"id": i,
                    "species": species_name,
                    "probability": prob * 100,
                    "num_points": k}
                )
        end = time.time()
        log_text += f"Classification & KMeans time: {round((end-start)*1000)} ms\n"

        if save_runs:
            if do_classification:
                file_path = os.path.join(folder, "predictions.txt")
                with open(file_path, mode='w', newline='', encoding='utf-8') as file:
                    writer = csv.writer(file)
                    writer.writerow(["ID", "Predicted Species", "Probability", "Number of Points"])
                    for entry in save_data:
                        writer.writerow([entry["id"], entry["species"],
                                        entry['probability'], entry["num_points"]])
            cv2.imwrite(os.path.join(folder, 'images', 'components3d.png'), cluster_drawn)
            cv2.imwrite(os.path.join(folder, 'images', 'classified.png'), color_drawn)
            with open(os.path.join(folder, 'log_file.txt'), "a") as file:
                file.write(log_text)

        # Show visualizations (useful for debugging)
        # if save_runs:
        #     plt.imshow(cv2.cvtColor(cluster_drawn, cv2.COLOR_BGR2RGB))
        #     plt.show()
        #     plt.imshow(cv2.cvtColor(color_drawn, cv2.COLOR_BGR2RGB))
        #     plt.show()
        # colors[labels == -1] = [0, 0, 0]
        # pcd.colors = o3d.utility.Vector3dVector(colors)
        # o3d.visualization.draw_geometries([pcd])

        
        self.get_logger().info('Finished clustering and classification')

        goal_handle.succeed()
        result = ClusterClassify.Result()
        result.positions = positions
        return result


def main(args=None):
    rclpy.init(args=args)
    cluster_classify_action_server = ClusterClassifyActionServer()
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(cluster_classify_action_server, executor=executor)

if __name__ == '__main__':
    main()

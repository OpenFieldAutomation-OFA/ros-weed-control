import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.action import ActionServer

from ofa_interfaces.action import ClusterClassify

from ament_index_python.packages import get_package_share_directory

import time
import cv2
import onnxruntime as ort
import numpy as np
import os
import open3d as o3d
import matplotlib.pyplot as plt

class ClusterClassifyActionServer(Node):

    def __init__(self):
        super().__init__('cluster_classify_action_server')
        self._action_server = ActionServer(
            self,
            ClusterClassify,
            'cluster_classify',
            execute_callback=self.execute_callback)

        # load network
        # model_directory = '/home/ofa/ros2_ws/src/ros-weed-control/ofa_detection/model'
        # providers = [
        #     ('TensorrtExecutionProvider', {
        #         'trt_engine_cache_enable': True,
        #         'trt_engine_cache_path': os.path.join(model_directory, 'trt_engine'),
        #     }),
        #     # 'CUDAExecutionProvider',
        #     'CPUExecutionProvider'
        # ]
        # self._session = ort.InferenceSession(os.path.join(model_directory, 'finetuned.onnx'),
        #     providers=providers)
        # self.get_logger().info('Loaded network.')


    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        # get goal request
        # pc = goal_handle.request.pc
        ros_image = goal_handle.request.color_image
        height = ros_image.height
        width = ros_image.width
        color_image = np.array(ros_image.data, dtype=np.uint8).reshape((height, width, -1))

        ros_pc = goal_handle.request.pc
        # point_cloud = list(pc2.read_points(ros_pc, field_names=("x", "y", "z")))
        # point_cloud_np = np.array(point_cloud)

        point_cloud_np = np.frombuffer(ros_pc.data, dtype=np.float32).reshape((ros_pc.width, 3))
        self.get_logger().info(f"Read PointCloud2 with {point_cloud_np.shape[0]} points")

        pc = o3d.geometry.PointCloud()
        pc.points = o3d.utility.Vector3dVector(point_cloud_np)
        # o3d.visualization.draw_geometries([pc])

        # input_name = self._session.get_inputs()[0].name

        plt.imshow(color_image)
        plt.show()

        goal_handle.succeed()
        result = ClusterClassify.Result()
        self.get_logger().info('Done')
        return result


def main(args=None):
    rclpy.init(args=args)
    cluster_classify_action_server = ClusterClassifyActionServer()
    # Use a MultiThreadedExecutor to enable processing goals concurrently
    executor = MultiThreadedExecutor()
    rclpy.spin(cluster_classify_action_server, executor=executor)

if __name__ == '__main__':
    main()

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
        time.sleep(10)

        # get goal request
        # pc = goal_handle.request.pc
        # ros_image = goal_handle.request.color_image
        # height = ros_image.height
        # width = ros_image.width
        # color_image = np.array(ros_image.data, dtype=np.uint8).reshape((height, width, -1))

        # input_name = self._session.get_inputs()[0].name

        # cv2.imshow('Color Image', color_image)
        # cv2.waitKey(1)

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

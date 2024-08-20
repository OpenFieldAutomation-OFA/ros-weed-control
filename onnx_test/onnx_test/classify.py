import rclpy
from rclpy.node import Node

from std_msgs.msg import String

import time
import cv2
import onnxruntime as ort
import numpy as np

class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')
        self.publisher_ = self.create_publisher(String, 'topic', 10)
        timer_period = 1  # seconds
        self.i = 0

        image = cv2.imread('/home/ofa/ros2_ws/src/ros-weed-control/onnx_test/onnx_test/00dcd0ff0c50e304d43e519f0eafc849.jpg')
        self.get_logger().info(f"The value of the first pixel is: {image[0, 0]}")
        # image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
        self.get_logger().info(f"The value of the first pixel is: {image[0, 0]}")
        target_size = 518
        height, width = image.shape[:2]
        if width < height:
            new_width = target_size
            new_height = int(target_size * height / width)
        else:
            new_height = target_size
            new_width = int(target_size * width / height)
        image = cv2.resize(image, (new_width, new_height), interpolation=cv2.INTER_CUBIC)
        start_x = (new_width - target_size) // 2
        start_y = (new_height - target_size) // 2
        image = image[start_y:start_y + target_size, start_x:start_x + target_size]
        self.get_logger().info(f"The value of the first pixel is: {image[0, 0]}")
        mean=[123.675, 116.28, 103.53]
        std=[58.395, 57.12, 57.375]
        image = (image - mean) / std
        print(image[0, 0])
        image = np.transpose(image, (2, 0, 1))
        input_tensor = np.expand_dims(image, axis=0)
        input_tensor = input_tensor.astype(np.float32)

        self.get_logger().info(f"Providers: {ort.get_available_providers()}")

        providers = [
            # ('TensorrtExecutionProvider', {
            #     # 'trt_fp16_enable': True,
            #     'trt_engine_cache_enable': True,
            #     'trt_engine_cache_path': '/home/ofa/ros2_ws/src/ros-weed-control/onnx_test/model/trt_engine',
            # }),
            'CUDAExecutionProvider',
            'CPUExecutionProvider'
        ]
        start = time.time()
        session = ort.InferenceSession("/home/ofa/ros2_ws/src/ros-weed-control/onnx_test/model/finetuned.onnx", 
            providers=providers)
        end = time.time()
        self.get_logger().info(f"Time load: {end-start}")
        input_name = session.get_inputs()[0].name
        input_shape = session.get_inputs()[0].shape
        self.get_logger().info(f"Input name: {input_name}, input shape: {input_shape}")
        input_data = np.ones((input_shape)).astype(np.float32)
        start = time.time()
        self.timer = self.create_timer(timer_period, self.timer_callback)
        outputs = session.run(None, {input_name: input_tensor})
        end = time.time()
        self.get_logger().info(f"Time: {end-start}")
        output_name = session.get_outputs()[0].name
        predictions = outputs[0][0]
        max_indices = np.argmax(predictions)
        test = predictions[max_indices]
        print('{0:.16f}'.format(test))

        print(predictions)
        print(predictions[max_indices])
        print(max_indices)



    def timer_callback(self):

        msg = String()
        msg.data = 'Hello World: %d' % self.i
        self.publisher_.publish(msg)
        self.get_logger().info('Publishing: "%s"' % msg.data)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

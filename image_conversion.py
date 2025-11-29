#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_srvs.srv import SetBool
from cv_bridge import CvBridge
import cv2

class ImageConversionNode(Node):
    def __init__(self):
        super().__init__('image_conversion')

        # Parameters
        self.declare_parameter('input_topic', '/image_raw')
        self.declare_parameter('output_topic', '/image_converted')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        # Image converter
        self.bridge = CvBridge()
        self.mode = 2  # 1 = grayscale mono, 2 = color

        # Subscriber
        self.sub = self.create_subscription(Image, input_topic, self.image_callback, 10)

        # Publisher
        self.pub = self.create_publisher(Image, output_topic, 10)

        # Service
        self.srv = self.create_service(SetBool, 'set_mode', self.set_mode_callback)

        self.get_logger().info(f"Subscribed to {input_topic}, publishing to {output_topic}")

    def set_mode_callback(self, request, response):
        self.mode = 1 if request.data else 2
        mode_str = "Grayscale" if self.mode == 1 else "Color"
        self.get_logger().info(f"Mode changed to: {mode_str}")
        response.success = True
        response.message = f"Mode set to {mode_str}"
        return response

    def image_callback(self, msg):
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            if self.mode == 1:
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='mono8')
            else:
                ros_image = self.bridge.cv2_to_imgmsg(cv_image, encoding='bgr8')
            ros_image.header = msg.header
            self.pub.publish(ros_image)
        except Exception as e:
            self.get_logger().error(f"Image conversion failed: {e}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageConversionNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

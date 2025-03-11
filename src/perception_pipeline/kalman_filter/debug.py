import rclpy
from rclpy.node import Node
import cv2
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class ImageDebugger(Node):
    def __init__(self):
        super().__init__('image_debugger')
        self.bridge = CvBridge()
        self.subscription = self.create_subscription(
            Image,
            '/perception_pipeline/output_6d',  # Replace with your actual topic name
            self.image_callback,
            10)
        self.subscription  # Prevent unused variable warning

    def image_callback(self, msg):
        try:
            # Convert ROS2 image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding="32FC7")

            # Debug shape and type
            print(f"Image Shape: {cv_image.shape}, Data Type: {cv_image.dtype}")

            # Check pixel values at (100,100) for debugging
            print(f"Sample Pixel (100,100): {cv_image[100,100]}")

            # Split channels
            channels = cv2.split(cv_image)

            # Display last channel
            cv2.imshow('Last Channel', channels[-1] / np.max(channels[-1]))  # Normalize for visualization
            
        
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f"Error processing image: {str(e)}")

def main(args=None):
    rclpy.init(args=args)
    node = ImageDebugger()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()

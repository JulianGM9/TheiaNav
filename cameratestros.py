#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from picamera2 import Picamera2, Preview

class CameraPublisher(Node):
    def __init__(self):
        super().__init__('camera_publisher_picamera2')
        self.publisher_ = self.create_publisher(Image, 'camera/image_raw', 10)
        self.bridge = CvBridge()

        # Initialize Picamera2
        self.picam2 = Picamera2()
        config = self.picam2.create_preview_configuration(main={"format": "BGR888", "size": (640, 480)})
        self.picam2.configure(config)
        self.picam2.start()

        # Publish at 10 Hz
        self.timer = self.create_timer(0.1, self.timer_callback)

    def timer_callback(self):
        frame = self.picam2.capture_array()
        if frame is not None:
            msg = self.bridge.cv2_to_imgmsg(frame, encoding='rgb8')
            self.publisher_.publish(msg)
            self.get_logger().info('Published camera frame')
        else:
            self.get_logger().warn('Failed to capture frame')

    def destroy_node(self):
        self.picam2.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()

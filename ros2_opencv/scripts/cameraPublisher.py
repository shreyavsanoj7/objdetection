#!/usr/bin/env python3

import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge

class PublisherNodeClass(Node):
    def __init__(self):
        super().__init__('publisher_node')
        
        self.cameraDeviceNumber = 2  # Corrected variable name
        self.camera = cv2.VideoCapture(self.cameraDeviceNumber)
        
        if not self.camera.isOpened():
            self.get_logger().error("Failed to open camera")
            raise RuntimeError("Failed to open camera")

        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.publisher = self.create_publisher(Image, self.topicNameFrames, self.queueSize)
        self.periodCommunication = 0.02
        self.timer = self.create_timer(self.periodCommunication, self.timer_callbackFunction)
        self.i = 0

    def timer_callbackFunction(self):
        success, frame = self.camera.read()
        if not success:
            self.get_logger().warn("Failed to capture image")
            return
        
        # Resize the frame
        frame = cv2.resize(frame, (820, 640), interpolation=cv2.INTER_CUBIC)
        
        # Convert the frame to a ROS Image message with encoding
        ROS2ImageMessage = self.bridgeObject.cv2_to_imgmsg(frame, encoding='bgr8')
        self.publisher.publish(ROS2ImageMessage)
        
        self.get_logger().info(f'Publishing image no {self.i}')
        self.i += 1

    def destroy_node(self):
        self.camera.release()  # Release the camera when node is destroyed
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    publisherObject = PublisherNodeClass()
    try:
        rclpy.spin(publisherObject)
    except KeyboardInterrupt:
        pass
    finally:
        publisherObject.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()


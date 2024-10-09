#!/usr/bin/env python3
import cv2
import rclpy
from sensor_msgs.msg import Image
from rclpy.node import Node
from cv_bridge import CvBridge
import numpy as np

# Initialize the class labels and colors for object detection
CLASSES = ["aeroplane", "background", "bicycle", "bird", "boat",
           "bottle", "bus", "car", "cat", "chair", "cow", "diningtable",
           "dog", "horse", "motorbike", "person", "pottedplant", "sheep",
           "sofa", "train", "tvmonitor"]

COLORS = np.random.uniform(0, 255, size=(len(CLASSES), 3))

# Load the pre-trained model
print("[INFO] loading model...")
net = cv2.dnn.readNetFromCaffe("/home/shreya/again/src/ros2_opencv/scripts/MobileNetSSD_deploy.prototxt.txt", "/home/shreya/again/src/ros2_opencv/scripts/MobileNetSSD_deploy.caffemodel")

class SubscriberNodeClass(Node):
    def __init__(self):
        super().__init__('subscriber_node')
        self.bridgeObject = CvBridge()
        self.topicNameFrames = 'topic_camera_image'
        self.queueSize = 20
        self.subscription = self.create_subscription(
            Image, 
            self.topicNameFrames, 
            self.listener_callbackFunction, 
            self.queueSize
        )
        self.subscription

    def listener_callbackFunction(self, imageMessage):
        self.get_logger().info("Received image message")

        # Convert ROS image message to OpenCV image
        openCVImage = self.bridgeObject.imgmsg_to_cv2(imageMessage)

        # Resize the image for object detection
        frame = cv2.resize(openCVImage, (400, 400))
        (h, w) = frame.shape[:2]

        # Prepare the frame for object detection
        blob = cv2.dnn.blobFromImage(frame, 1/127.5, (300, 300), 127.5, swapRB=True)
        net.setInput(blob)
        predictions = net.forward()

        # Loop over the predictions
        for i in np.arange(0, predictions.shape[2]):
            confidence = predictions[0, 0, i, 2]
            if confidence > 0.2:
                idx = int(predictions[0, 0, i, 1])
                box = predictions[0, 0, i, 3:7] * np.array([w, h, w, h])
                (startX, startY, endX, endY) = box.astype("int")

                label = "{}: {:.2f}%".format(CLASSES[idx], confidence * 100)
                cv2.rectangle(frame, (startX, startY), (endX, endY), COLORS[idx], 2)
                y = startY - 15 if startY - 15 > 15 else startY + 15
                cv2.putText(frame, label, (startX, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5, COLORS[idx], 2)

        # Show the output frame
        cv2.imshow("Frame", frame)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    subscriberNode = SubscriberNodeClass()

    rclpy.spin(subscriberNode)

    subscriberNode.destroy_node()
    rclpy.shutdown()
    cv2.destroyAllWindows()

if __name__ == '__main__':
    main()


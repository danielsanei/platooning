import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String,Float32
from cv_bridge import CvBridge
import requests
import cv2
import numpy as np


API_URL = "http://192.168.10.117:8080/detect"  

class ObjectDetectionNodeAPI(Node):
    def __init__(self):
        super().__init__('object_detection_node_api')


        self.subscription = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.process_image,
            10
        )


        self.result_publisher = self.create_publisher(Float32, '/object_detection/results', 10)


        self.bridge = CvBridge()
        self.error = 0
        self.get_logger().info("Object Detection Node API has started.")

    def process_image(self, msg):
        self.error = Float32()
        try:

            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='rgb8')
            #bgr_image = cv2.cvtColor(input_image, cv2.COLOR_RGB2BGR)


            _, img_encoded = cv2.imencode('.jpg', cv_image)
            files = {'file': ('image.jpg', img_encoded.tobytes(), 'image/jpeg')}


            response = requests.post(API_URL, files=files)
            self.get_logger().info("request sent")
            if response.status_code == 200:
                detections = response.json().get("detections", [])
                height, width, _= cv_image.shape
                frame_center = width/2
                frame_center = int(frame_center)
                x1 = 0
                x2 = 0
                self.get_logger().info(f"c: {frame_center}")
                x_center = 0
                max_confidence = 0
                for box in detections:
                     x1, y1, x2, y2, confidence, class_id = box
                     self.get_logger().info(f"x1{x1} y1{y1} x2{x2} y2{y2} confidence{confidence}")
                     x1 = float(x1)
                     x2 = float(x2)
                 # if confidence > max_confidence:
                  #       max_confidence = confidence
                     x_center = x1 + (x2 - x1)/2
                if not(x_center):
                     x_center = frame_center
                self.error.data =float(frame_center - x_center)
                self.get_logger().info(f'error: {self.error}')
                self.result_publisher.publish(self.error)
                self.get_logger().info(f"Detections: {detections}")
               # self.visualize_detections(cv_image, detections)
            else:
                self.error.data = float(0)
                self.get_logger().error(f"Failed to call API: {response.status_code}, {response.text}")
                self.result_publisher.publish(self.error)
        except Exception as e:
            self.get_logger().error(f"Error processing image: {e}")
            self.get_logger().info(f"{e}")
    def visualize_detections(self, image, detections):
 
        for box in detections:
            x1, y1, x2, y2, confidence, class_id = box
            cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
            cv2.putText(image, f"Conf: {confidence:.2f}", (int(x1), int(y1) - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)


        bgr_image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        cv2.imshow("Detections", bgr_image)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = ObjectDetectionNodeAPI()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down Object Detection Node API...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

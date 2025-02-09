import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import cv2
import torch
import numpy as np
import pyrealsense2 as rs
from ultralytics import YOLO

model = torch.hub.load('ultralytics/yolov5', 'yolov5s', pretrained=True)

pipeline = rs.pipeline()
config = rs.config()
config.enable_stream(rs.stream.color, 640, 480, rs.format.bgr8, 30)
config.enable_stream(rs.stream.depth, 640, 480, rs.format.z16, 30)
pipeline.start(config)

class Publisher(Node):
    def __init__(self):
        super().__init__("Publisher")
        self.publisher = self.create_publisher(String, '/camera_info', 10)
        self.time_period = 0.2
        self.timer = self.create_timer(self.time_period, self.timer_callback)

    def timer_callback(self):
        depth_scale = 0.0010000000474974513

      

        frames = pipeline.wait_for_frames()
        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()

        color_image = np.asanyarray(color_frame.get_data())
        depth_image = np.asanyarray(depth_frame.get_data())


        depth_image = depth_image * depth_scale

        results = model(color_image)

        for result in results.xyxy[0]:
            x1, y1, x2, y2, confidence, class_id = result

            object_depth = np.median(depth_image[int(y1):int(y2), int(x1):int(x2)])
            label = f"{object_depth:.2f}m"

            # Convert coordinates to a string message
            msg = String()
            msg.data = f"label:{model.names[int(class_id)]},{x1},{y1},{x2},{y2},{object_depth:.2f}m"

            self.publisher.publish(msg)

            cv2.rectangle(color_image, (int(x1), int(y1)), (int(x2), int(y2)), (252, 119, 30), 2)

            # Draw the bounding box
            cv2.putText(color_image, model.names[int(class_id)], (int(x1), int(y1)-10), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (252, 119, 30), 2)

            # Show the image
            cv2.imshow("Color Image", color_image)
            cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = Publisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == "__main__":
    main()

#!/usr/bin/env python
from roboflowoak import RoboflowOak
import cv2
import time
import numpy as np
import rclpy
import math
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArrray
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

class target_publisher(Node):
    def __init__(self, node_name="target_publisher"):
        self._node_name = node_name
        self.angle_per_pix = 0.01
        self.center_x = 200
        self.rf = RoboflowOak(model="detect-car-jsvsk", confidence=0.05, overlap=0.5,
            version="1", api_key="7CfAEQ0li4t5GeTTqLXK", rgb=True,
            depth=True, device=None, blocking=True)
        
        super().__init__(self._node_name)
        
        self.publisher_ = self.create_publisher(Float32MultiArrray, '/distance_to_target', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.publish_target)

        self.get_logger().info(self._node_name +" Ready...")


    def publish_target(self):
        msg = Float32MultiArrray()
        data = []
        result, frame, raw_frame, depth = self.rf.detect()
        predictions = result["predictions"]
        if len(predictions) == 0:
            msg.data = [float('inf'), float('inf')]
            self.publisher_.publish(msg)
        ind = 0
        confidence = 0
        for i in range(len(predictions)):
            if predictions[i][confidence] > confidence:
                confidence = predictions[i][confidence]
                ind = i
        x = predictions[ind][x]
        y = predictions[ind][y]
        data.append(depth[x][y])
        #calculate angle here
        data.append((x-self.center_x) * self.angle_per_pix)
        msg.data = data
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = target_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

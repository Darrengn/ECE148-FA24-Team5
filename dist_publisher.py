#!/usr/bin/env python

import rclpy
import math
from rclpy.node import Node
# import the LaserScan module from sensor_msgs interface
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32
# import Quality of Service library, to set the correct profile and reliability to read sensor data.
from rclpy.qos import ReliabilityPolicy, QoSProfile

class dist_publisher(Node):
    def __init__(self, node_name="dist_publisher"):
        self._node_name = node_name
        self.data = []
        self.tar_ang = 0.0
        self.tar_ind = 0
        self.ang_inc = 0.2
        super().__init__(self._node_name)

        #subscriber to get lidar data
        self.subscriber = self.create_subscription(
            LaserScan,
            '/scan',
            self.laserscan_callback,
            QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE))
        
        self.publisher_ = self.create_publisher(Float32, '/distance_to_target', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.publish_dist)

        self.get_logger().info(self._node_name +" Ready...")

    # callback for lidar subscriber
    def laserscan_callback(self, msg):
        # Save the data for the node to use later
        self.data = msg.ranges
        self.ang_inc = msg.angle_increment

    # callback for camera target subscriber
    def target_callback(self,msg):
        self.tar_ang = msg.data
        self.tar_ind = round(self.tar_ang/self.ang_inc)

    def publish_dist(self):
        msg = Float32()
        if len(self.data) == 0:
            msg.data = -1.0
            self.get_logger().info('No data')
        else:
            msg.data = self.data[self.tar_ind]
            self.get_logger().info('Distance to target: "%f"' % msg.data)
        self.publisher_.publish(msg)
        
        
def main(args=None):
    rclpy.init(args=args)
    node = dist_publisher()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

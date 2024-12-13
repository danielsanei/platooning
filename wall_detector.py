import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import os
import signal
import subprocess
import time
from std_msgs.msg import Float32
class WallDetector(Node):
    def __init__(self):
        super().__init__('wall_detector')
        self.wall_distance_publisher = self.create_publisher(Float32,'/wall_detect',10)
        self.subscription = self.create_subscription(
            LaserScan, '/scan', self.scan_callback, 10)
        self.emergency_stop_triggered = False

    def scan_callback(self, msg):
        # Simplified logic for detecting walls in front
        need_stop = Float32()
        need_stop.data = 0.0
        front_ranges = msg.ranges[1:8]  # Adjust indices for front-facing direction
        front_filtered = [r for r in front_ranges if (not r == float('inf') and r <= float(6))]
        if front_filtered and not self.emergency_stop_triggered:
            avg_distance = sum(front_filtered) / len(front_filtered)
            self.get_logger().info(f'Average front distance: {avg_distance}')
            self.get_logger().info(f'{front_ranges}') 
            # Trigger emergency stop if wall is within 1 meters
            if avg_distance <= 1 and avg_distance > 0:
                need_stop.data = 1.0
                self.get_logger().info("Wall detected within 0.5 meters!")
                #self.emergency_stop()
        self.wall_distance_publisher.publish(need_stop)
    def emergency_stop(self):
        # Stop all ROS2 nodes across all terminals by using pkill
        subprocess.run(["pkill", "-f", "ros2"])
        self.emergency_stop_triggered = True  # Prevent multiple triggers

def main():
    rclpy.init()
    node = WallDetector()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()

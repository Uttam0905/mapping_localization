#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
import csv
import time

class NavLogger(Node):
    def __init__(self):
        super().__init__('nav_logger')

        self.x = 0.0
        self.y = 0.0

        # Save file in home (safe location)
        self.file = open('/home/uttam/nav_data.csv', 'w', newline='')
        self.writer = csv.writer(self.file)
        self.writer.writerow(['time','x','y'])

        self.start_time = time.time()

        self.create_subscription(Odometry, '/odom', self.odom_cb, 10)
        self.timer = self.create_timer(0.1, self.log_data)

        self.get_logger().info("Navigation logger started")

    def odom_cb(self, msg):
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

    def log_data(self):
        t = time.time() - self.start_time
        self.writer.writerow([t, self.x, self.y])

def main():
    rclpy.init()
    node = NavLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
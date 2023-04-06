import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8, String
from datetime import datetime, timedelta
import numpy as np
import pickle
import math
import cmath
import time

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        self.yaw = 0
        self.px = 0
        self.py = 0
        self.target_angle = 0
        self.end_yaw = 0
        self.table = 0
        self.laser_range = np.array([])
        self.has_can = False
        self.ir_status = ''

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        # np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def get_distance(self):
        while True:
            front30 = np.append(self.laser_range[-15:-1], self.laser_range[0:14])
            lr2i = np.nanargmin(front30)
            print(front30[lr2i])

def main(args = None):
    rclpy.init(args = args)
    auto_nav = AutoNav()
    auto_nav.get_distance()
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
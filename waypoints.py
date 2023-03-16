# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import OccupancyGrid
import numpy as np
import math
import cmath
import time
import pickle

# constants
rotatechange = 0.1
speedchange = 0.05
occ_bins = [-1, 0, 100, 101]
stop_distance = 0.25
front_angle = 30
front_angles = range(-front_angle,front_angle+1,1)
scanfile = 'lidar.txt'
mapfile = 'map.txt'
waypoints = {1: [], 2: [], 3: [], 4: [], 5: [], 6: []}

class Waypoint(Node):

    def __init__(self):
        super().__init__('waypoint')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        self.odom_subscription  # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        inp = input("Enter input")
        if inp == "w":
            numbers = int(input("Enter table numbers:"))
            print("saving")
            orien =  msg.pose.pose.orientation
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            ox = orien.x
            oy = orien.y
            oz = orien.z
            ow = orien.w
            # self.get_logger().info(orien)
            while numbers != 0:
                num = numbers % 10
                numbers = (numbers // 10)
                data = (px, py, ox, oy, oz, ow)
                waypoints[num].append(data)
            print(waypoints)

        elif inp == "s":
            print("saving...")
            with open('filename.pickle', 'wb') as handle:
                pickle.dump(waypoints, handle, protocol=pickle.HIGHEST_PROTOCOL)

def main(args=None):
    rclpy.init(args=args)
    try:
        waypoint = Waypoint()
        start = input("Press s to start")
        if start == "s":
            rclpy.spin(waypoint)

    except KeyboardInterrupt:
        waypoint.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

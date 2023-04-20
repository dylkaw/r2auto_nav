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
from geometry_msgs.msg import Pose
import tf2_ros
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

# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)

    return roll_x, pitch_y, yaw_z # in radians
    
class Waypoint(Node):
    def __init__(self):
        super().__init__('waypoint')
        self.map2base_sub = self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            1)
        self.map2base_sub # prevent unused variable warning
        # initialize variables
        self.roll = 0
        self.pitch = 0
        self.yaw = 0
        self.orien = 0
        self.x = 0
        self.y = 0

    def map2base_callback(self, msg):
        # self.get_logger().info('In map2basecallback')
        self.x, self.y = msg.position.x, msg.position.y
        self.roll, self.pitch, self.yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)

def main(args=None):
    rclpy.init(args=args)

    try:
        waypoint = Waypoint()
        start = input("Press s to start")
        if start == "s":
            rclpy.spin_once(waypoint)
        while True:
            inp = input("Enter input: 'w' to save waypoint, 's' to save waypoint file and exit")
            if inp == "w":
                numbers = int(input("Enter table numbers:"))
                rclpy.spin_once(waypoint)
                print("saving")
                while numbers != 0:
                    num = numbers % 10
                    numbers = (numbers // 10)
                    data = (waypoint.x, waypoint.y, waypoint.roll, waypoint.pitch, waypoint.yaw)
                    waypoints[num].append(data)
                print(waypoints)

            elif inp == "s":
                print("saving...")
                with open('waypoints.pickle', 'wb') as handle:
                    pickle.dump(waypoints, handle, protocol=pickle.HIGHEST_PROTOCOL)
                waypoint.destroy_node()
                rclpy.shutdown()
                break

    except KeyboardInterrupt:
        waypoint.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

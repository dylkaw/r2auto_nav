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
import tf2_ros
from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
import numpy as np
import math
import cmath
import time
import pickle
import scipy.stats

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

    # def __init__(self):
    #     super().__init__('waypoint')
    #     self.occ_subscription = self.create_subscription(
    #         OccupancyGrid,
    #         'map',
    #         self.occ_callback,
    #         qos_profile_sensor_data)
    #     self.occ_subscription  # prevent unused variable warning
    #     self.tfBuffer = tf2_ros.Buffer()
    #     self.tfListener = tf2_ros.TransformListener(self.tfBuffer, self)

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

    # def occ_callback(self, msg):
    #     # create numpy array
    #     inp = str(input("enter input:"))
    #     if inp == "w":
    #         occdata = np.array(msg.data)
    #         # compute histogram to identify bins with -1, values between 0 and below 50, 
    #         # and values between 50 and 100. The binned_statistic function will also
    #         # return the bin numbers so we can use that easily to create the image 
    #         occ_counts, edges, binnum = scipy.stats.binned_statistic(occdata, np.nan, statistic='count', bins=occ_bins)
    #         # get width and height of map
    #         iwidth = msg.info.width
    #         iheight = msg.info.height
    #         # calculate total number of bins
    #         total_bins = iwidth * iheight
    #         # log the info
    #         # self.get_logger().info('Unmapped: %i Unoccupied: %i Occupied: %i Total: %i' % (occ_counts[0], occ_counts[1], occ_counts[2], total_bins))

    #         # find transform to obtain base_link coordinates in the map frame
    #         # lookup_transform(target_frame, source_frame, time)
    #         try:
    #             trans = self.tfBuffer.lookup_transform('map', 'base_link', rclpy.time.Time())
    #         except (LookupException, ConnectivityException, ExtrapolationException) as e:
    #             self.get_logger().info('No transformation found')
    #             return
                
    #         cur_pos = trans.transform.translation
    #         cur_rot = trans.transform.rotation
    #         self.get_logger().info('Trans: %f, %f' % (cur_pos.x, cur_pos.y))
    #         # convert quaternion to Euler angles
    #         roll, pitch, yaw = euler_from_quaternion(cur_rot.x, cur_rot.y, cur_rot.z, cur_rot.w)
    #         self.get_logger().info('Rot-Yaw: R: %f D: %f' % (yaw, np.degrees(yaw)))
    #         numbers = int(input("Enter table numbers:"))
    #         while numbers != 0:
    #             num = numbers % 10
    #             numbers = (numbers // 10)
    #             data = (cur_pos.x, cur_pos.y, roll, pitch, yaw)
    #             waypoints[num].append(data)
    #         print(waypoints)

    #     elif inp == "s":
    #         print("exporting pickle")
    #         with open('waypoints.pickle', 'wb') as handle:
    #             pickle.dump(waypoints, handle, protocol=pickle.HIGHEST_PROTOCOL)


    # def readKey(self):
    #     try:
    #         while True:
    #             cmd_char = str(input("w to save waypoint, s to export pickle"))
    #             if cmd_char == "w":
    #                 print("saving waypoint")
    #                 rclpy.spin_once(self)
                
    #             elif cmd_char == "s":
    #                 print("exporting pickle")
    #                 with open('waypoints.pickle', 'wb') as handle:
    #                     pickle.dump(waypoints, handle, protocol=pickle.HIGHEST_PROTOCOL)
        
    #     except Exception as e:
    #         print(e)
            
	# 	# Ctrl-c detected
    #     finally:
    #     	# stop moving
    #         print('waypoint mapping completed')



    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        inp = input("Enter input")
        if inp == "w":
            numbers = int(input("Enter table numbers:"))
            print("saving")
            orien =  msg.pose.pose.orientation
            px = msg.pose.pose.position.x
            py = msg.pose.pose.position.y
            roll, pitch, yaw = euler_from_quaternion(orien.x, orien.y, orien.z, orien.w)
            # self.get_logger().info(orien)
            while numbers != 0:
                num = numbers % 10
                numbers = (numbers // 10)
                data = (px, py, roll, pitch, yaw)
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

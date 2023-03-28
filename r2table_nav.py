import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
import numpy as np
import pickle
import math
import cmath
import time


rotatechange = 0.1
speedchange = 0.1
rot_q = 0.0 
theta = 0.0
scanfile = 'lidar.txt'

with open('waypoints.pickle', 'rb') as f:
    waypoints = pickle.load(f)
    print(waypoints)


# code from https://automaticaddison.com/how-to-convert-a-quaternion-into-euler-angles-in-python/
def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    return yaw_z # in radians

class AutoNav(Node):

    def __init__(self):
        super().__init__('auto_nav')
        
        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        # self.get_logger().info('Created subscriber')
        # initialize variables
        self.yaw = 0
        self.px = 0
        self.py = 0
        self.target_angle = 0
        self.end_yaw = 0
        self.table = 0
        self.has_can = False

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning
        
        self.laser_range = np.array([])

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orien =  msg.pose.pose.orientation
        pos = msg.pose.pose.position
        self.yaw = euler_from_quaternion(orien.x, orien.y, orien.z, orien.w)
        self.px, self.py = pos.x, pos.y

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    # function to rotate the TurtleBot
    def rotatebot(self):
        try:
            rclpy.spin_once(self)
            twist = Twist()
            twist.linear.x = 0.0

            # logic to determine turning direction
            if (self.target_angle > 0 and self.yaw > 0) or (self.target_angle < 0 and self.yaw < 0):
                if self.target_angle > self.yaw:
                    twist.angular.z = 0.1
                else:
                    twist.angular.z = -0.1
            elif (self.target_angle < 0 and self.yaw > 0):
                if abs(self.yaw - self.target_angle) > math.pi:
                    twist.angular.z = 0.1
                else:
                    twist.angular.z = -0.1
            elif (self.target_angle > 0 and self.yaw < 0):
                if (self.target_angle - self.yaw) > math.pi:
                    twist.angular.z = 0.1
                else:
                    twist.angular.z = -0.1

            while abs(self.yaw - self.target_angle) > 0.005:
                self.publisher_.publish(twist)
                rclpy.spin_once(self)
                self.get_logger().info(f'Rotating to {math.degrees(self.target_angle)} from {math.degrees(self.yaw)}')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            self.get_logger().info('Rotated to target angle')
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)

    def move_to_point(self):
        self.get_logger().info('In move_to_point')
        twist = Twist()
        # Start moving
        twist.angular.z = 0.0
        twist.linear.x = 0.1
        distance = math.sqrt(math.pow(self.goal_x - self.px, 2) + math.pow(self.goal_y - self.py, 2))
        self.get_logger().info('Initial Distance: %f' % (distance))
        while distance >= 0.025:
            rclpy.spin_once(self)
            self.publisher_.publish(twist)
            distance = math.sqrt(math.pow(self.goal_x - self.px, 2) + math.pow(self.goal_y - self.py, 2))
            self.get_logger().info('Distance: %f' % (distance))

        self.get_logger().info('Reached goal')

    def get_close_to_table(self):
        self.get_logger().info("Moving close to table")
        rclpy.spin_once(self)
        twist = Twist()
        if self.table == 6:
            front180 = self.laser_range[-90:-1] + self.laser_range[0:89]
            tableAngleDeg = np.argmin(front180)
            while front180[tableAngleDeg] > 0.75:
                rclpy.spin_once(self)
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                front180 = self.laser_range[-90:-1] + self.laser_range[0:89]
                tableAngleDeg = np.argmin(front180)
            tableAngleRad = math.radians(tableAngleDeg - 90)
            self.target_angle = tableAngleRad
            self.rotatebot()
            dist_to_table = self.laser_range[tableAngleDeg - 90]
            while dist_to_table > 0.15:
                rclpy.spin_once(self)
                self.get_logger().info(f"Distance to table: {dist_to_table}")
                dist_to_table = self.laser_range[tableAngleDeg - 90]
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

        else:
            front30 = self.laser_range[-15:-1] + self.laser_range[0:14]
            tableAngleDeg = np.argmin(front30)
            tableAngleRad = math.radians(tableAngleDeg - 15)
            self.get_logger().info(f"curr_yaw: {self.yaw}, deg: {tableAngleDeg}, rad: {tableAngleRad}")
            self.target_angle = tableAngleRad
            self.rotatebot()
            dist_to_table = self.laser_range[tableAngleDeg - 15]
            while dist_to_table > 0.2:
                rclpy.spin_once(self)
                self.get_logger().info(f"Distance to table: {dist_to_table}")
                dist_to_table = self.laser_range[tableAngleDeg - 15]
                twist.linear.x = 0.1
                twist.angular.z = 0.0
                self.publisher_.publish(twist)

        twist.linear.x = 0.0
        twist.angular.z = 0.0
        self.publisher_.publish(twist)
        self.get_logger().info("Reached table")



    def return_home(self):
        for waypoint in reversed(waypoints[self.table]):
            self.get_logger().info("Returning home!")
            self.goal_x = waypoint[0]
            self.goal_y = waypoint[1]
            self.end_yaw = waypoint[4]
            rot_angle = math.atan2(self.goal_y - self.py, self.goal_x - self.px)
            self.target_angle = rot_angle
            self.rotatebot()
            self.move_to_point()
        self.target_angle = self.end_yaw
        self.rotatebot()


    def mover(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            table_no = int(input("Enter table number:"))
            self.table = table_no
            for waypoint in waypoints[table_no]:
                self.goal_x = waypoint[0]
                self.goal_y = waypoint[1]
                self.end_yaw = waypoint[4]
                rot_angle = math.atan2(self.goal_y - self.py, self.goal_x - self.px)
                self.target_angle = rot_angle
                self.rotatebot()
                self.move_to_point()
            self.target_angle = self.end_yaw
            self.rotatebot()
            self.get_close_to_table()
            self.return_home()
            print("ending...")
            break


def main(args = None):
        rclpy.init(args = args)
        auto_nav = AutoNav()
        auto_nav.mover()
        auto_nav.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()



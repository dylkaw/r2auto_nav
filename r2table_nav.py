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
    def rotatebot(self, rot_angle):
        try:
            rclpy.spin_once(self)
            twist = Twist()
            twist.linear.x = 0.0
            if (rot_angle - self.yaw < 0):
                twist.angular.z = -0.1
            else:
                twist.angular.z = 0.1
            self.publisher_.publish(twist)
            while abs(self.yaw - rot_angle) > 0.05:
                rclpy.spin_once(self)
                self.get_logger().info(f'Rotating to {math.degrees(rot_angle)} from {math.degrees(self.yaw)}')
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
        finally:
            twist.linear.x = 0.0
            twist.angular.z = 0.0
            self.publisher_.publish(twist)


        # self.get_logger().info('In rotatebot')
        # # create Twist object
        # twist = Twist()
        
        # # get current yaw angle
        # current_yaw = self.yaw
        # # log the info
        # self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # # we are going to use complex numbers to avoid problems when the angles go from
        # # 360 to 0, or from -180 to 180
        # c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # # calculate desired yaw
        # target_yaw = current_yaw + math.radians(rot_angle)
        # # convert to complex notation
        # c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        # self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # # divide the two complex numbers to get the change in direction
        # c_change = c_target_yaw / c_yaw
        # # get the sign of the imaginary component to figure out which way we have to turn
        # c_change_dir = np.sign(c_change.imag)
        # # set linear speed to zero so the TurtleBot rotates on the spot
        # twist.linear.x = 0.0
        # # set the direction to rotate
        # twist.angular.z = c_change_dir * rotatechange
        # # start rotation
        # self.publisher_.publish(twist)

        # # we will use the c_dir_diff variable to see if we can stop rotating
        # c_dir_diff = c_change_dir
        # # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # # becomes -1.0, and vice versa
        # while(c_change_dir * c_dir_diff > 0):
        #     # allow the callback functions to run
        #     rclpy.spin_once(self)
        #     current_yaw = self.yaw
        #     # convert the current yaw to complex form
        #     c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        #     self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
        #     # get difference in angle between current and target
        #     c_change = c_target_yaw / c_yaw
        #     # get the sign to see if we can stop
        #     c_dir_diff = np.sign(c_change.imag)
        #     # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        # self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # # set the rotation speed to 0
        # twist.angular.z = 0.0
        # # stop the rotation
        # self.publisher_.publish(twist)

    def move_to_point(self, goal_x, goal_y):
        self.get_logger().info('In move_to_point')
        twist = Twist()
        # Start moving
        twist.angular.z = 0.0
        twist.linear.x = 0.1
        self.publisher_.publisher(twist)
        distance = math.sqrt(math.pow(goal_x - self.px, 2) + math.pow(goal_y - self.py, 2))
        self.get_logger().info('Initial Distance: %f' % (distance))
        while distance != 0:
            rclpy.spin_once(self)
            distance = math.sqrt(math.pow(goal_x - self.px, 2) + math.pow(goal_y - self.py, 2))
            self.get_logger().info('Distance: %f' % (distance))

        self.get_logger().info('Reached goal')


    def mover(self):
        while rclpy.ok():
            rclpy.spin_once(self)
            table_no = int(input("Enter table number:"))
            for waypoint in waypoints[table_no]:
                goal_x = waypoint[0]
                goal_y = waypoint[1]
                goal_yaw = waypoint[4]
                rot_angle = math.atan2(goal_y - self.py, goal_x - self.px)
                self.rotatebot(rot_angle)
                # self.move_to_point(goal_x, goal_y)
                # self.rotatebot(goal_yaw - self.yaw)
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



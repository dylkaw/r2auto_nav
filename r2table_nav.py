import rclpy
from rclpy.node import Node
import geometry_msgs.msg
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Int8
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
WAYPOINT_THRESHOLD = 0.04
STOPPING_THRESHOLD = 0.5

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
        self.ir_status = 0

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
        
        # create subscription to track orientation
        self.odom_subscription = self.create_subscription(
            Odometry,
            'odom',
            self.odom_callback,
            10)
        self.odom_subscription
        # self.get_logger().info('Created subscriber')
        # initialize variables

        self.scan_subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.scan_callback,
            qos_profile_sensor_data)
        self.scan_subscription  # prevent unused variable warning

        self.can_subscription = self.create_subscription(
            Bool,
            'can_pub',
            self.can_callback,
            10)
        self.can_subscription  # prevent unused variable warning

        self.infra_subscription = self.create_subscription(
            Int8,
            'infra_pub',
            self.infra_callback,
            10)
        self.infra_subscription

    def odom_callback(self, msg):
        # self.get_logger().info('In odom_callback')
        orien =  msg.pose.pose.orientation
        pos = msg.pose.pose.position
        _, _, self.yaw = euler_from_quaternion(orien.x, orien.y, orien.z, orien.w)
        self.px, self.py = pos.x, pos.y

    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # print to file
        np.savetxt(scanfile, self.laser_range)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def can_callback(self, msg):
        # print(msg.data)
        self.has_can = msg.data

    def infra_callback(self, msg):
        self.ir_status = msg.data

    # function to rotate the TurtleBot
    def rotatebot(self, rot_angle):
        # self.get_logger().info('In rotatebot')
        # create Twist object
        twist = Twist()
        
        # get current yaw angle
        current_yaw = self.yaw
        # log the info
        self.get_logger().info('Current: %f' % math.degrees(current_yaw))
        # we are going to use complex numbers to avoid problems when the angles go from
        # 360 to 0, or from -180 to 180
        c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
        # calculate desired yaw
        target_yaw = current_yaw + math.radians(rot_angle)
        # convert to complex notation
        c_target_yaw = complex(math.cos(target_yaw),math.sin(target_yaw))
        self.get_logger().info('Desired: %f' % math.degrees(cmath.phase(c_target_yaw)))
        # divide the two complex numbers to get the change in direction
        c_change = c_target_yaw / c_yaw
        print("c_change: " + str(c_change))
        # get the sign of the imaginary component to figure out which way we have to turn
        c_change_dir = np.sign(c_change.imag)
        print("c_change_dir: " + str(c_change_dir))
        # set linear speed to zero so the TurtleBot rotates on the spot
        twist.linear.x = 0.0
        print("linear.x = 0")
        # set the direction to rotate
        twist.angular.z = c_change_dir * 0.1
        print("c_change_dir: " + str(c_change_dir))
        # start rotation
        self.publisher_.publish(twist)

        print("published twist")
        # we will use the c_dir_diff variable to see if we can stop rotating
        c_dir_diff = c_change_dir
        # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))
        # if the rotation direction was 1.0, then we will want to stop when the c_dir_diff
        # becomes -1.0, and vice versa
        while(c_change_dir * c_dir_diff > 0):
            # allow the callback functions to run
            rclpy.spin_once(self)
            current_yaw = self.yaw
            # convert the current yaw to complex form
            c_yaw = complex(math.cos(current_yaw),math.sin(current_yaw))
            # self.get_logger().info('Current Yaw: %f' % math.degrees(current_yaw))
            # get difference in angle between current and target
            c_change = c_target_yaw / c_yaw
            # get the sign to see if we can stop
            c_dir_diff = np.sign(c_change.imag)
            # self.get_logger().info('c_change_dir: %f c_dir_diff: %f' % (c_change_dir, c_dir_diff))

        self.get_logger().info('End Yaw: %f' % math.degrees(current_yaw))
        # set the rotation speed to 0
        twist.angular.z = 0.0
        # stop the rotation
        self.publisher_.publish(twist)

    def move_to_point(self):
        try:
            self.get_logger().info('In move_to_point')
            twist = Twist()
            # Start moving
            twist.angular.z = 0.0
            twist.linear.x = 0.1
            distance = math.sqrt(math.pow(self.goal_x - self.px, 2) + math.pow(self.goal_y - self.py, 2))
            self.get_logger().info('Initial Distance: %f' % (distance))
            while distance >= WAYPOINT_THRESHOLD:
                rclpy.spin_once(self)
                self.publisher_.publish(twist)
                distance = math.sqrt(math.pow(self.goal_x - self.px, 2) + math.pow(self.goal_y - self.py, 2))
                self.get_logger().info('Distance: %f' % (distance))

            self.get_logger().info('Reached goal')
        except Exception as e:
            self.stopbot()
            self.destroy_node()
            rclpy.shutdown()
        finally:
            self.stopbot()


    def get_close_to_table(self):
        self.get_logger().info("Moving close to table")
        rclpy.spin_once(self)
        twist = Twist()
        try:
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
                tableAngleDeg = 360 - (90 - tableAngleDeg) if tableAngleDeg > 180 else tableAngleDeg - 90
                self.target_angle = tableAngleDeg
                self.rotatebot(self.target_angle - math.degrees(self.yaw))
                dist_to_table = self.laser_range[tableAngleDeg]
                while dist_to_table > STOPPING_THRESHOLD:
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Distance to table: {dist_to_table}")
                    dist_to_table = self.laser_range[tableAngleDeg - 90]
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)

            else:
                front30 = self.laser_range[-15:-1] + self.laser_range[0:14]
                self.get_logger().info(f"min 360: {np.argmin(self.laser_range)}, min 30: {np.argmin(front30)}")
                tableAngleDeg = np.argmin(front30)
                if tableAngleDeg > 15:
                    tableAngleDeg = tableAngleDeg - 15
                else:
                    360 - (15 - tableAngleDeg)
                self.get_logger().info(f"curr_yaw: {self.yaw}, deg: {tableAngleDeg}")
                self.target_angle = tableAngleDeg
                self.rotatebot(self.target_angle - math.degrees(self.yaw))
                dist_to_table = self.laser_range[tableAngleDeg]
                while dist_to_table > STOPPING_THRESHOLD:
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Distance to table: {dist_to_table}")
                    front30 = self.laser_range[-15:-1] + self.laser_range[0:14]
                    dist_to_table = np.min(front30)
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
        except Exception as e:
            self.stopbot()
            self.destroy_node()
            rclpy.shutdown()
        finally:
            self.stopbot()
        self.get_logger().info("Reached table")

    def return_home(self):
        while self.has_can:
            rclpy.spin_once(self)
            self.get_logger().info("Waiting for can to be removed...")
        for waypoint in reversed(waypoints[self.table]):
            self.get_logger().info("Returning home!")
            self.goal_x = waypoint[0]
            self.goal_y = waypoint[1]
            self.end_yaw = waypoint[4]
            rot_angle = math.degrees(math.atan2(self.goal_y - self.py, self.goal_x - self.px))
            self.target_angle = rot_angle
            self.rotatebot(self.target_angle - math.degrees(self.yaw))
            self.move_to_point()
        self.target_angle = self.end_yaw
        self.rotatebot(self.target_angle - math.degrees(self.yaw))

    def stopbot(self):
        self.get_logger().info('Stopping')
        # publish to cmd_vel to move TurtleBot
        twist = Twist()
        twist.linear.x = 0.0
        twist.angular.z = 0.0
        # time.sleep(1)
        self.publisher_.publish(twist)

    def mover(self):
        try:
            while rclpy.ok():
                rclpy.spin_once(self)
                table_no = int(input("Enter table number:"))
                rclpy.spin_once(self)
                if self.has_can:
                    self.table = table_no
                    for waypoint in waypoints[table_no]:
                        self.goal_x = waypoint[0]
                        self.goal_y = waypoint[1]
                        self.end_yaw = waypoint[4]
                        rot_angle = math.degrees(math.atan2(self.goal_y - self.py, self.goal_x - self.px))
                        self.target_angle = rot_angle
                        self.rotatebot(self.target_angle - math.degrees(self.yaw))
                        self.move_to_point()
                    self.target_angle = math.degrees(self.end_yaw)
                    self.rotatebot(self.target_angle - math.degrees(self.yaw))
                    self.get_close_to_table()
                    self.return_home()
                    print("ending...")
                    break
                else:
                    self.get_logger().info("No can!")
        finally:
            self.stopbot()

    def dock(self):
        #TODO wall following thing
        rclpy.spin_once(self)
        twist = Twist()
        while self.ir_status == 0:
            self.get_logger().info("Searching for ir emitter")
            twist.linear.x = 0.1
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            rclpy.spin_once(self)
        self.stopbot()
    
        if self.ir_status == 1:
            self.rotatebot(270)
        elif self.ir_status == 10:
            self.rotatebot(90)

        twist.linear.x = 0.05
        twist.angular.z = 0.0
        self.publisher_.publish(twist)

def main(args = None):
    rclpy.init(args = args)
    auto_nav = AutoNav()
    auto_nav.mover()
    auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



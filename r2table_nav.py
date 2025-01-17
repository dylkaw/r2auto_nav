import rclpy
from rclpy.node import Node
import paho.mqtt.client as mqtt
import numpy as np
import pickle
import math
import time
from geometry_msgs.msg import Twist
from geometry_msgs.msg import Pose
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, String
from datetime import datetime, timedelta

MQTT_ADDRESS = '172.20.10.8' # Change according to MQTT Broker IP
MQTT_TOPIC_TABLE = 'table'
WAYPOINT_THRESHOLD = 0.04
STOPPING_THRESHOLD = 0.35
ANGLE_THRESHOLD = 0.5
LINEAR_SPEED = 0.15

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

    return roll_x, pitch_y, math.degrees(yaw_z) # in radians

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
        self.mqttclient = mqtt.Client()
        self.mqttclient.on_connect = self.on_connect
        self.mqttclient.on_message = self.on_message

        # create publisher for moving TurtleBot
        self.publisher_ = self.create_publisher(Twist,'cmd_vel',10)
        # self.get_logger().info('Created publisher')
 
        # create subscription for map2base
        self.map2base_sub = self.create_subscription(
            Pose,
            'map2base',
            self.map2base_callback,
            1)
        self.map2base_sub # prevent unused variable warning

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
            String,
            'infra_pub',
            self.infra_callback,
            10)
        self.infra_subscription
    
    def on_connect(self, client, userdata, flags, rc):
        print('Connected with result code ' + str(rc))
        client.subscribe(MQTT_TOPIC_TABLE)

    def on_message(self, client, userdata, msg):
        val = str(msg.payload.decode('utf-8'))
        if msg.topic == 'table' and self.table == 0:
            if val in ['1','2','3','4','5','6']:
                self.table = int(val)
                print(f'Received table {self.table}')
                self.mover()
        print(msg.topic + ' ' + val)

    def connect_to_mqtt(self):
        print('Connecting...')
        self.mqttclient.connect(MQTT_ADDRESS, 1883)
        self.mqttclient.loop_forever()

    def map2base_callback(self, msg):
        # self.get_logger().info('In map2basecallback')
        self.px, self.py = msg.position.x, msg.position.y
        _, _, self.yaw = euler_from_quaternion(msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
    
    def scan_callback(self, msg):
        # self.get_logger().info('In scan_callback')
        # create numpy array
        self.laser_range = np.array(msg.ranges)
        # replace 0's with nan
        self.laser_range[self.laser_range==0] = np.nan

    def can_callback(self, msg):
        # print(msg.data)
        self.has_can = msg.data

    def infra_callback(self, msg):
        self.ir_status = msg.data
        print(f'pub: {self.ir_status}')

    def get_turn_direction(self, current_orientation, target_orientation):
        diff = target_orientation - current_orientation
        if diff > 180:
            diff -= 360
        elif diff <= -180:
            diff += 360
        if diff > 0:
            return 1
        elif diff < 0:
            return -1
        else:
            return 0
    
    def rotatebot(self, rot_angle):
        try:
            rclpy.spin_once(self)
            twist = Twist()
            twist.linear.x = 0.0
            turn_dir = self.get_turn_direction(self.yaw, rot_angle)
            twist.angular.z = 0.4 * turn_dir
            
            self.get_logger().info(f'Current angle: {self.yaw}')
            self.get_logger().info(f'Desired angle: {rot_angle}')
            self.get_logger().info(f'Rotate Direction: {twist.angular.z}')
            while abs(self.yaw - rot_angle) > ANGLE_THRESHOLD:
                if (abs(self.yaw - rot_angle) - ANGLE_THRESHOLD) < 20:
                    # Turn slower when nearing angle
                    twist.angular.z = 0.1 * turn_dir
                else:
                    twist.angular.z = 0.4 * turn_dir
                self.publisher_.publish(twist)
                rclpy.spin_once(self)
                self.get_logger().info(f'Rotating to {rot_angle} from {self.yaw}')
            self.stopbot()
            self.get_logger().info('Rotated to target angle')
        finally:
            self.stopbot()

    def move_to_point(self):
        try:
            self.get_logger().info('In move_to_point')
            twist = Twist()
            # Start moving
            twist.angular.z = 0.0
            twist.linear.x = LINEAR_SPEED
            distance = math.sqrt(math.pow(self.goal_x - self.px, 2) + math.pow(self.goal_y - self.py, 2))
            self.get_logger().info('Initial Distance: %f' % (distance))
            prev_distance = distance
            i = 0
            while distance >= WAYPOINT_THRESHOLD:
                i += 1
                rclpy.spin_once(self)
                self.publisher_.publish(twist)
                if i % 30 == 0:
                    prev_distance = distance
                distance = math.sqrt(math.pow(self.goal_x - self.px, 2) + math.pow(self.goal_y - self.py, 2))
                # If delivery robot is off target, recalculate movement direction
                if distance - prev_distance > 0.03:
                    self.get_logger().info('Recalibrating...')
                    rclpy.spin_once(self)
                    self.stopbot()
                    rot_angle = math.degrees(math.atan2(self.goal_y - self.py, self.goal_x - self.px))
                    self.target_angle = rot_angle
                    self.rotatebot(self.target_angle)
                    twist.angular.z = 0.0
                    twist.linear.x = LINEAR_SPEED
                    self.publisher_.publish(twist)

                self.get_logger().info('Distance: %f' % (distance))

            self.get_logger().info('Reached goal')
        except Exception as e:
            self.stopbot()
            self.destroy_node()
            rclpy.shutdown()
        finally:
            self.stopbot()

    def nav_to_table(self):
        self.get_logger().info(f'Moving to table {self.table}!')
        # Reverse out of dispenser
        twist = Twist()
        twist.linear.x = -0.1
        twist.angular.z = 0.0
        end_time = datetime.now() + timedelta(seconds=1)
        while datetime.now() < end_time:
            self.publisher_.publish(twist)
        self.stopbot()

        dock_point = True
        for waypoint in waypoints[self.table]:
            # Skip docking point when moving to table
            if dock_point:
                dock_point = False
                continue
            self.goal_x = waypoint[0]
            self.goal_y = waypoint[1]
            self.end_yaw = waypoint[4]
            rot_angle = math.degrees(math.atan2(self.goal_y - self.py, self.goal_x - self.px))
            self.target_angle = rot_angle
            self.rotatebot(self.target_angle)
            self.move_to_point()

    def get_close_to_table(self):
        self.get_logger().info("Moving close to table")
        rclpy.spin_once(self)
        twist = Twist()
        try:
            if self.table == 6:
                front140 = np.append(self.laser_range[-70:-1], self.laser_range[0:69])
                lr2i = np.nanargmin(front140)
                while front140[lr2i] > 0.5:
                    rclpy.spin_once(self)
                    twist.linear.x = 0.1
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    front140 = np.append(self.laser_range[-70:-1], self.laser_range[0:69])
                    lr2i = np.nanargmin(front140)
                    
                to_angle = lr2i - 70
                if to_angle < -180:
                    to_angle = to_angle + 360
                elif to_angle > 180:
                    to_angle = to_angle - 360

                self.target_angle = to_angle
                self.rotatebot(self.target_angle)
                front30 = np.append(self.laser_range[-15:-1], self.laser_range[0:14])
                lr2i = np.nanargmin(front30)
                dist_to_table = front30[lr2i]
                while dist_to_table > STOPPING_THRESHOLD:
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Distance to table: {front30[lr2i]}")
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    front30 = np.append(self.laser_range[-15:-1], self.laser_range[0:14])
                    lr2i = np.nanargmin(front30)
                    dist_to_table = front30[lr2i]

            else:
                rclpy.spin_once(self)
                front30 = np.append(self.laser_range[-15:-1], self.laser_range[0:14])
                lr2i = np.nanargmin(front30)
                while front30[lr2i] > STOPPING_THRESHOLD:
                    rclpy.spin_once(self)
                    self.get_logger().info(f"Distance to table: {front30[lr2i]}")
                    twist.linear.x = 0.05
                    twist.angular.z = 0.0
                    self.publisher_.publish(twist)
                    front30 = np.append(self.laser_range[-15:-1], self.laser_range[0:14])
                    lr2i = np.nanargmin(front30)

        except Exception as e:
            print(e)
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
            self.rotatebot(self.target_angle)
            self.move_to_point()
        self.target_angle = self.end_yaw
        self.rotatebot(self.target_angle)

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
                while not self.has_can:
                    rclpy.spin_once(self)
                    self.get_logger().info('Waiting for can...')
                self.nav_to_table()
                self.target_angle = math.degrees(self.end_yaw)
                self.rotatebot(self.target_angle)
                self.get_close_to_table()
                self.return_home()
                self.dock()
                print("ending...")
        finally:
            self.stopbot()

    def dock(self):
        try:
            rclpy.spin_once(self)
            twist = Twist()
            while self.ir_status == '':
                self.get_logger().info("Searching for ir emitter")
                twist.linear.x = 0.02
                twist.angular.z = 0.0
                self.publisher_.publish(twist)
                rclpy.spin_once(self)
        
            if self.ir_status == 'L':
                self.get_logger().info("Detected left!")
                end_time = datetime.now() + timedelta(seconds=2.25)
                while datetime.now() < end_time:
                    self.publisher_.publish(twist)
                self.stopbot()
                to_angle = self.yaw + 90
                if to_angle > 180:
                    to_angle = (to_angle % 180) - 180
                self.rotatebot(to_angle)
                self.stopbot()
                self.get_logger().info("Docking!")  
            elif self.ir_status == 'R':
                self.get_logger().info("Detected right!")
                end_time = datetime.now() + timedelta(seconds=2.25)
                while datetime.now() < end_time:
                    self.publisher_.publish(twist)
                self.stopbot()
                to_angle = self.yaw - 90
                if to_angle < -180:
                    to_angle = 180 + (to_angle % 180)
                self.rotatebot(self.yaw - 90)
                self.stopbot()
                self.get_logger().info("Docking!")  
            self.stopbot()

            twist.linear.x = 0.03
            twist.angular.z = 0.0
            self.publisher_.publish(twist)
            time.sleep(5.5)
            self.stopbot()
            self.stopbot()
            self.table = 0
        finally:
            self.stopbot()

def main(args = None):
    rclpy.init(args = args)
    auto_nav = AutoNav()
    # auto_nav.mover()
    auto_nav.connect_to_mqtt()
    # auto_nav.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()



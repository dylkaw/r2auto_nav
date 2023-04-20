import RPi.GPIO as GPIO
import rclpy
from rclpy.node import Node
from hx711 import HX711
from std_msgs.msg import Bool

THRESHOLD = 200
RATIO = 1795.584848484

GPIO.setmode(GPIO.BCM)
# Initialize the HX711 module
hx = HX711(dout_pin=6, pd_sck_pin=5)
# hx.set_reading_format("MSB", "MSB")
# hx.set_reference_unit(1)
# hx.reset()
hx.zero()

class CanPub(Node):

    def __init__(self):
        super().__init__('can_pub')
        self.publisher_ = self.create_publisher(Bool, 'can_pub', 10)
        timer_period = 0.2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Bool()
        weight = hx.get_weight_mean()
        if weight < THRESHOLD:
            msg.data = False
        else:
            msg.data = True
        self.publisher_.publish(msg)
        self.get_logger().info(f'Has can: "{msg.data}"')

def main(args=None):
    # input("place can to calibrate load")
    #reading = hx.get_data_mean(readings=100)
    #ratio = reading/CAN_MASS
    hx.set_scale_ratio(RATIO)

    rclpy.init(args=args)

    canPub = CanPub()

    rclpy.spin(canPub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    canPub.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()


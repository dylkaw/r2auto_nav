import RPi.GPIO as GPIO
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import Int8


# Set up GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(23, GPIO.IN)
GPIO.setup(24, GPIO.IN)
GPIO.setup(25, GPIO.IN)

# Wait for IR signal
print("Waiting for IR signal...")

class InfraPub(Node):

    def __init__(self):
        super().__init__('infra_pub')
        self.publisher_ = self.create_publisher(Int8, 'infra_pub', 10)
        timer_period = 0.5  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        msg = Int8()
        val = 0
        if GPIO.input(23) == GPIO.LOW:
            print("L  IR signal detected")
            val += 1
            time.sleep(0.2) # Debounce time

        if GPIO.input(24) == GPIO.LOW:
            print("R  IR signal detected")
            val += 10
            time.sleep(0.2) # Debounce time

        if GPIO.input(25) == GPIO.LOW:
            print("F  IR signal detected")
            val += 100
            time.sleep(0.2) # Debounce time
        self.publisher_.publish(msg)
        self.get_logger().info(f'IR Readings: "{msg.data}"')

def main(args=None):

    rclpy.init(args=args)

    infraPub = InfraPub()

    rclpy.spin(infraPub)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    infraPub.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
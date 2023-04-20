import RPi.GPIO as GPIO
import rclpy
import time
from rclpy.node import Node
from std_msgs.msg import String


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
        self.publisher_ = self.create_publisher(String, 'infra_pub', 10)
        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.val = ''
        self.count = 0

    def timer_callback(self):
        msg = String()
        if GPIO.input(23) == GPIO.LOW:
            print("L  IR signal detected")
            self.val = 'L'
            self.count = 0
            time.sleep(0.2) # Debounce time
        elif GPIO.input(24) == GPIO.LOW:
            print("R  IR signal detected")
            self.val = 'R'
            self.count = 0 
            time.sleep(0.2) # Debounce time
        elif GPIO.input(25) == GPIO.LOW:
            print("F  IR signal detected")
            self.val = 'F'
            time.sleep(0.2) # Debounce time
        msg.data = self.val
        if self.count == 50:
            self.count = 0
            self.val = ''
        self.count += 1
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

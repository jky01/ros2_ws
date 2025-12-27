import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import smbus2
import time

class SimplePCA9685:
    def __init__(self, bus_num, address=0x40):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.reset()
        self.set_freq(60) # Donkeycar uses 60Hz

    def reset(self):
        self.bus.write_byte_data(self.address, 0x00, 0x00)

    def set_freq(self, freq):
        prescale = int(25000000.0 / 4096.0 / freq - 1.0)
        old_mode = self.bus.read_byte_data(self.address, 0x00)
        new_mode = (old_mode & 0x7F) | 0x10
        self.bus.write_byte_data(self.address, 0x00, new_mode)
        self.bus.write_byte_data(self.address, 0xFE, prescale)
        self.bus.write_byte_data(self.address, 0x00, old_mode)
        time.sleep(0.005)
        self.bus.write_byte_data(self.address, 0x00, old_mode | 0xa0)

    def set_pwm(self, channel, on, off):
        self.bus.write_byte_data(self.address, 0x06 + 4*channel, on & 0xFF)
        self.bus.write_byte_data(self.address, 0x07 + 4*channel, on >> 8)
        self.bus.write_byte_data(self.address, 0x08 + 4*channel, off & 0xFF)
        self.bus.write_byte_data(self.address, 0x09 + 4*channel, off >> 8)

class MotorDriver(Node):
    def __init__(self):
        super().__init__('motor_driver')
        
        try:
            self.pca = SimplePCA9685(7) 
            self.get_logger().info('PCA9685 initialized on I2C bus 7 at 60Hz (Donkeycar mode)')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize PCA9685: {e}')
            self.pca = None

        # Donkeycar parameters
        self.throttle_channel = 0
        self.steering_channel = 1
        
        # Exact values from Donkeycar myconfig.py
        self.t_center = 370
        self.t_fwd_max = 500
        self.t_rev_max = 220
        
        self.s_center = 375
        self.s_left = 460
        self.s_right = 290

        # Initialize to Neutral
        self.set_throttle(0.0)
        self.set_steering(0.0)

        self.subscription = self.create_subscription(
            Twist,
            'cmd_vel',
            self.listener_callback,
            10)
        
        self.last_msg_time = self.get_clock().now()
        self.timer = self.create_timer(0.1, self.watchdog_callback)
        self.get_logger().info('Motor Driver ready. Using Donkeycar PWM ranges.')

    def set_throttle(self, val):
        if self.pca is None: return
        # val is -1.0 to 1.0
        if val >= 0:
            pwm = int(self.t_center + val * (self.t_fwd_max - self.t_center))
        else:
            pwm = int(self.t_center + val * (self.t_center - self.t_rev_max))
        pwm = max(200, min(550, pwm))
        self.pca.set_pwm(self.throttle_channel, 0, pwm)

    def set_steering(self, val):
        if self.pca is None: return
        # val is -1.0 to 1.0 (ROS: + is left, - is right)
        # Donkeycar: Left is 460, Right is 290. So val=1.0 -> 460, val=-1.0 -> 290
        if val >= 0:
            pwm = int(self.s_center + val * (self.s_left - self.s_center))
        else:
            pwm = int(self.s_center + val * (self.s_center - self.s_right))
        pwm = max(250, min(500, pwm))
        self.pca.set_pwm(self.steering_channel, 0, pwm)

    def listener_callback(self, msg):
        self.last_msg_time = self.get_clock().now()
        # Logging to help user see responsiveness
        if abs(msg.linear.x) > 0.05 or abs(msg.angular.z) > 0.05:
            self.get_logger().info(f'RECEIVING: T={msg.linear.x:.2f}, S={msg.angular.z:.2f}')
        self.set_throttle(msg.linear.x)
        self.set_steering(msg.angular.z)

    def watchdog_callback(self):
        now = self.get_clock().now()
        if (now - self.last_msg_time).nanoseconds > 500_000_000:
            self.set_throttle(0.0)
            self.set_steering(0.0)

def main(args=None):
    rclpy.init(args=args)
    node = MotorDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node.pca:
            node.set_throttle(0.0)
            node.set_steering(0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

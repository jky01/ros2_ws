import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy, Image
import smbus2
import time
import cv2
import threading
from cv_bridge import CvBridge

class SimplePCA9685:
    def __init__(self, bus_num, address=0x40):
        self.bus = smbus2.SMBus(bus_num)
        self.address = address
        self.reset()
        self.set_freq(60)

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

    def set_pwm(self, channel, off):
        self.bus.write_byte_data(self.address, 0x06 + 4*channel, 0)
        self.bus.write_byte_data(self.address, 0x07 + 4*channel, 0)
        self.bus.write_byte_data(self.address, 0x08 + 4*channel, off & 0xFF)
        self.bus.write_byte_data(self.address, 0x09 + 4*channel, off >> 8)

class DonkeyControlNode(Node):
    def __init__(self):
        super().__init__('donkey_control')
        
        # --- Motor Driver ---
        try:
            self.pca = SimplePCA9685(7)
            self.get_logger().info('PCA9685 SUCCESS: Initialized on Bus 7 (60Hz)')
        except Exception as e:
            self.get_logger().error(f'PCA9685 ERROR: {e}')
            self.pca = None

        self.t_center, self.t_fwd_max, self.t_rev_max = 370, 500, 220
        self.s_center, self.s_left, self.s_right = 375, 460, 290
        self.axis_steering, self.axis_fwd_trigger, self.axis_rev_trigger, self.button_toggle = 0, 5, 4, 7

        self.is_enabled = False
        self.last_buttons = []
        self.fwd_calibrated, self.rev_calibrated = False, False

        self.joy_sub = self.create_subscription(Joy, 'joy', self.joy_callback, 10)

        # --- Optimized Camera (Threaded + Accelerated) ---
        self.image_pub = self.create_publisher(Image, 'image_raw', 2) # Low queue to reduce latency
        self.bridge = CvBridge()
        
        # Optimized IMX219 Pipeline with Drop Policy
        gst_pipeline = (
            "nvarguscamerasrc ! "
            "video/x-raw(memory:NVMM), width=(int)640, height=(int)480, format=(string)NV12, framerate=(fraction)30/1 ! "
            "nvvidconv flip-method=0 ! "
            "video/x-raw, width=(int)640, height=(int)480, format=(string)BGRx ! "
            "videoconvert ! "
            "video/x-raw, format=(string)BGR ! "
            "appsink drop=True max-buffers=1 sync=False"
        )
        
        self.cap = cv2.VideoCapture(gst_pipeline, cv2.CAP_GSTREAMER)
        self.latest_frame = None
        self.frame_lock = threading.Lock()
        self.cam_running = True

        if not self.cap.isOpened():
            self.get_logger().error('CAMERA ERROR: Could not open optimized GStreamer pipeline.')
        else:
            self.get_logger().info('CAMERA SUCCESS: Optimized GStreamer active.')
            # Start background thread for high-speed capture
            self.capture_thread = threading.Thread(target=self._capture_loop, daemon=True)
            self.capture_thread.start()
            # ROS Timer for publishing at a steady rate
            self.timer = self.create_timer(1.0/30, self.timer_callback) # 30 FPS

        self.get_logger().info('--- DONKEYCAR LOW-LATENCY NODE READY ---')

    def _capture_loop(self):
        while self.cam_running:
            ret, frame = self.cap.read()
            if ret:
                with self.frame_lock:
                    self.latest_frame = frame
            else:
                time.sleep(0.01)

    def timer_callback(self):
        frame = None
        with self.frame_lock:
            if self.latest_frame is not None:
                frame = self.latest_frame.copy()
                self.latest_frame = None # Consume the frame
        
        if frame is not None:
            try:
                img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                img_msg.header.stamp = self.get_clock().now().to_msg()
                img_msg.header.frame_id = "camera_link"
                self.image_pub.publish(img_msg)
            except Exception as e:
                self.get_logger().error(f'PUB ERROR: {e}')

    def joy_callback(self, msg):
        if not self.last_buttons:
            self.last_buttons = [0] * len(msg.buttons)
            return

        if self.button_toggle < len(msg.buttons):
            if msg.buttons[self.button_toggle] == 1 and self.last_buttons[self.button_toggle] == 0:
                self.is_enabled = not self.is_enabled
                self.get_logger().warn("!!! UNLOCKED !!!" if self.is_enabled else "!!! LOCKED !!!")
                if not self.is_enabled: self.fwd_calibrated, self.rev_calibrated = False, False

        self.last_buttons = list(msg.buttons)

        if not self.fwd_calibrated and msg.axes[self.axis_fwd_trigger] > 0.9: self.fwd_calibrated = True
        if not self.rev_calibrated and msg.axes[self.axis_rev_trigger] > 0.9: self.rev_calibrated = True

        if self.is_enabled:
            steering_val = msg.axes[self.axis_steering] if self.axis_steering < len(msg.axes) else 0.0
            fwd_norm = (1.0 - msg.axes[self.axis_fwd_trigger]) / 2.0 if (self.axis_fwd_trigger < len(msg.axes) and self.fwd_calibrated) else 0.0
            rev_norm = (1.0 - msg.axes[self.axis_rev_trigger]) / 2.0 if (self.axis_rev_trigger < len(msg.axes) and self.rev_calibrated) else 0.0
            throttle_val = fwd_norm if fwd_norm > 0.05 else (-rev_norm if rev_norm > 0.05 else 0.0)
            self.apply_control(throttle_val, steering_val)
        else:
            self.apply_control(0.0, 0.0)

    def apply_control(self, throttle, steering):
        if self.pca is None: return
        t_pwm = int(self.t_center + throttle * (self.t_fwd_max - self.t_center)) if throttle >= 0 else int(self.t_center + throttle * (self.t_center - self.t_rev_max))
        self.pca.set_pwm(0, t_pwm)
        s_pwm = int(self.s_center + steering * (self.s_left - self.s_center)) if steering >= 0 else int(self.s_center + steering * (self.s_center - self.s_right))
        self.pca.set_pwm(1, s_pwm)

def main(args=None):
    rclpy.init(args=args)
    node = DonkeyControlNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.cam_running = False
        if node.cap.isOpened(): node.cap.release()
        if node.pca: node.apply_control(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

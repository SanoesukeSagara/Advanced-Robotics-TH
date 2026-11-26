import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg import String

try:
    import serial
except ImportError:
    print('pyserial missing: sudo apt install python3-serial')
    raise

SAFE_LETTERS = set('ips')   # letters forwarded while allow_motion is False


class ArduinoBridge(Node):

    def __init__(self):
        super().__init__('arduino_bridge')
        port = self.declare_parameter('port', '/dev/ttyACM0').value
        baud = self.declare_parameter('baud', 115200).value
        rate = self.declare_parameter('rate_hz', 50.0).value
        self.invert = self.declare_parameter('invert_yaw', False).value
        self.allow_motion = self.declare_parameter('allow_motion', False).value

        self.ser = serial.Serial(port, baud, timeout=0)
        self.get_logger().info(f'Opened {port} @ {baud}. Waiting for Arduino reset...')
        time.sleep(2.0)   # opening the port resets the Arduino

        self.yaw = None
        self.rx_buffer = ''

        self.create_subscription(Imu, 'imu/data', self.imu_callback, 10)
        self.create_subscription(String, 'arduino_cmd', self.cmd_callback, 10)
        self.create_timer(1.0 / rate, self.send_yaw)
        self.create_timer(0.01, self.read_arduino)
        self.get_logger().info(f'Ready. allow_motion={self.allow_motion}')

    def imu_callback(self, msg):
        q = msg.orientation
        if q.w * q.w + q.x * q.x + q.y * q.y + q.z * q.z < 0.5:
            return
        yaw = math.degrees(math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                                      1.0 - 2.0 * (q.y * q.y + q.z * q.z)))
        self.yaw = -yaw if self.invert else yaw

    def send_yaw(self):
        if self.yaw is not None:
            self.write(f'Y{self.yaw:.2f}\n')

    def cmd_callback(self, msg):
        for ch in msg.data:
            if ch in 'Y\r\n ':
                continue   # 'Y' is reserved for yaw lines
            if not self.allow_motion and ch not in SAFE_LETTERS:
                self.get_logger().warn(f'Blocked {ch!r}: allow_motion is false')
                continue
            self.write(ch)
            self.get_logger().info(f'Sent {ch!r} to the Arduino')

    def write(self, text):
        try:
            self.ser.write(text.encode('ascii'))
        except serial.SerialException as e:
            self.get_logger().error(f'Serial write failed: {e}')

    def read_arduino(self):
        n = self.ser.in_waiting
        if n:
            self.rx_buffer += self.ser.read(n).decode('ascii', errors='replace')
            while '\n' in self.rx_buffer:
                line, self.rx_buffer = self.rx_buffer.split('\n', 1)
                print(f'[arduino] {line.rstrip()}', flush=True)


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ArduinoBridge()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.write('s')   # stop the servos on exit
            node.ser.close()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

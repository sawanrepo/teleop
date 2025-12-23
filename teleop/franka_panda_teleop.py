#use "ros2 run teleop franka_teleop" to run this pacakge
import sys
import termios
import tty
import threading
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool

JOINT_NAMES = [
    'panda_joint1', 'panda_joint2', 'panda_joint3',
    'panda_joint4', 'panda_joint5', 'panda_joint6',
    'panda_joint7', 'panda_finger_joint1', 'panda_finger_joint2'
]

CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
STEP = 0.15
GRIP_STEP = 0.01
SMOOTH_ALPHA = 0.2

INIT_JOINT_POS = [
    0.012,
    -0.5686,
    0.0,
    -2.8106,
    0.0,
    3.0367,
    0.741,
    0.0,
    0.0
]

def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class SmoothJointTeleop(Node):
    def __init__(self):
        super().__init__('smooth_joint_teleop')

        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.create_subscription(Bool, '/teleop_enabled', self.gate_cb, 1)

        self.teleop_enabled = True

        self.current = INIT_JOINT_POS.copy()
        self.target = INIT_JOINT_POS.copy()

        self.timer = self.create_timer(DT, self.control_loop)

        self.key_thread = threading.Thread(target=self.keyboard_loop, daemon=True)
        self.key_thread.start()

        self.get_logger().info("Teleop started with gate")

    def gate_cb(self, msg):
        self.teleop_enabled = msg.data

    def keyboard_loop(self):
        while rclpy.ok():
            key = getch()
            if key == '\x03':
                break

            if key == 'q': self.target[0] += STEP
            elif key == 'a': self.target[0] -= STEP
            elif key == 'w': self.target[1] += STEP
            elif key == 's': self.target[1] -= STEP
            elif key == 'e': self.target[2] += STEP
            elif key == 'd': self.target[2] -= STEP
            elif key == 'r': self.target[3] += STEP
            elif key == 'f': self.target[3] -= STEP
            elif key == 't': self.target[4] += STEP
            elif key == 'g': self.target[4] -= STEP
            elif key == 'y': self.target[5] += STEP
            elif key == 'h': self.target[5] -= STEP
            elif key == 'u': self.target[6] += STEP
            elif key == 'j': self.target[6] -= STEP

            elif key == 'i':
                self.target[7] = min(0.04, self.target[7] + GRIP_STEP)
                self.target[8] = min(0.04, self.target[8] + GRIP_STEP)
            elif key == 'k':
                self.target[7] = max(0.0, self.target[7] - GRIP_STEP)
                self.target[8] = max(0.0, self.target[8] - GRIP_STEP)

    def control_loop(self):
        if not self.teleop_enabled:
            return

        for i in range(len(self.current)):
            self.current[i] += SMOOTH_ALPHA * (self.target[i] - self.current[i])

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = JOINT_NAMES
        msg.position = self.current

        self.pub.publish(msg)


def main():
    rclpy.init()
    node = SmoothJointTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

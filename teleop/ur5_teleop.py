import sys
import termios
import tty
import threading

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


# ===== UR5 JOINT NAMES (MUST MATCH /joint_states) =====
JOINT_NAMES = [
    'shoulder_pan_joint',
    'shoulder_lift_joint',
    'elbow_joint',
    'wrist_1_joint',
    'wrist_2_joint',
    'wrist_3_joint'
]

# ===== CONTROL PARAMS =====
CONTROL_HZ = 20
DT = 1.0 / CONTROL_HZ
STEP = 0.15
SMOOTH_ALPHA = 0.2

# ===== INITIAL JOINT POSITIONS (FROM /joint_states) =====
INIT_JOINT_POS = [
    0.0,        # shoulder_pan_joint
    0.0007,     # shoulder_lift_joint
    0.0002,     # elbow_joint
    0.0,        # wrist_1_joint
    0.0,        # wrist_2_joint
    0.0         # wrist_3_joint
]


def getch():
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


class UR5SmoothTeleop(Node):
    def __init__(self):
        super().__init__('ur5_smooth_joint_teleop')

        self.pub = self.create_publisher(JointState, '/joint_command', 10)
        self.create_subscription(Bool, '/teleop_enabled', self.gate_cb, 1)

        self.teleop_enabled = True

        self.current = INIT_JOINT_POS.copy()
        self.target = INIT_JOINT_POS.copy()

        self.timer = self.create_timer(DT, self.control_loop)

        self.key_thread = threading.Thread(
            target=self.keyboard_loop,
            daemon=True
        )
        self.key_thread.start()

        self.get_logger().info("UR5 smooth joint teleop started")

    def gate_cb(self, msg):
        self.teleop_enabled = msg.data

    def keyboard_loop(self):
        while rclpy.ok():
            key = getch()
            if key == '\x03':
                break

            # Joint 1
            if key == 'q': self.target[0] += STEP
            elif key == 'a': self.target[0] -= STEP

            # Joint 2
            elif key == 'w': self.target[1] += STEP
            elif key == 's': self.target[1] -= STEP

            # Joint 3
            elif key == 'e': self.target[2] += STEP
            elif key == 'd': self.target[2] -= STEP

            # Joint 4
            elif key == 'r': self.target[3] += STEP
            elif key == 'f': self.target[3] -= STEP

            # Joint 5
            elif key == 't': self.target[4] += STEP
            elif key == 'g': self.target[4] -= STEP

            # Joint 6
            elif key == 'y': self.target[5] += STEP
            elif key == 'h': self.target[5] -= STEP

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
    node = UR5SmoothTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()


if __name__ == "__main__":
    main()

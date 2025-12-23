# Teleop Robotics Arm

This project contains scripts to teleoperate robotic arms, specifically the Franka Panda and UR5 arms. It is designed to be easily extensible for additional arms in the future.

## Prerequisites

- ROS 2 (e.g., Humble, Foxy, etc.)
- `colcon` build tool

## Installation

1.  **Source ROS 2**:
    Make sure you have sourced your ROS 2 installation.
    ```bash
    source /opt/ros/humble/setup.bash
    ```
    *(Replace `humble` with your specific ROS 2 distribution if different)*

2.  **Clone and Build**:
    Navigate to your ROS 2 workspace (e.g., `ros2_ws`), clone this repository into the `src` directory, and build it using `colcon`.

    ```bash
    cd ~/ros2_ws/src
    git clone https://github.com/sawanrepo/teleop.git
    cd ~/ros2_ws
    colcon build --packages-select teleop
    ```

3.  **Source the Workspace**:
    After building, source the overlay to make the package available.
    ```bash
    source install/setup.bash
    ```

## Usage

To run the teleoperation scripts, use the `ros2 run` command.

### Franka Panda Teleop

```bash
ros2 run teleop franka_teleop
```

**Controls:**
- **Joint 1**: `q` (+) / `a` (-)
- **Joint 2**: `w` (+) / `s` (-)
- **Joint 3**: `e` (+) / `d` (-)
- **Joint 4**: `r` (+) / `f` (-)
- **Joint 5**: `t` (+) / `g` (-)
- **Joint 6**: `y` (+) / `h` (-)
- **Joint 7**: `u` (+) / `j` (-)
- **Gripper**: `i` (Open/Positive) / `k` (Close/Negative)
- **Quit**: `Ctrl+C`

### UR5 Teleop

```bash
ros2 run teleop ur5_teleop
```

**Controls:**
- **Shoulder Pan**: `q` (+) / `a` (-)
- **Shoulder Lift**: `w` (+) / `s` (-)
- **Elbow**: `e` (+) / `d` (-)
- **Wrist 1**: `r` (+) / `f` (-)
- **Wrist 2**: `t` (+) / `g` (-)
- **Wrist 3**: `y` (+) / `h` (-)
- **Quit**: `Ctrl+C`

## Notes

- Ensure that a robot simulation or hardware interface is running and listening to the `/joint_command` topic (published as `sensor_msgs/msg/JointState`).
- The scripts also subscribe to `/teleop_enabled` (std_msgs/Bool) to gate the control.

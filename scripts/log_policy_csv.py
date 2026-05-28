#!/usr/bin/env python3
"""Record Go2 Gazebo policy command/state topics to CSV.

Run this in a third terminal while Gazebo and rl_sim are running.
"""

import argparse
import csv
import math
import sys
from pathlib import Path
from time import strftime, time

try:
    import rclpy
    from geometry_msgs.msg import Twist
    from rclpy.node import Node
    from rclpy.qos import QoSProfile
    from robot_msgs.msg import RobotCommand, RobotState
    from sensor_msgs.msg import Imu
except ImportError as exc:
    print(
        "Failed to import ROS2 Python modules. Source ROS2 and this workspace first:\n"
        "  source /opt/ros/humble/setup.bash\n"
        "  source install/setup.bash\n"
        f"\nImport error: {exc}",
        file=sys.stderr,
    )
    raise


JOINT_NAMES = [
    "FR_hip_joint",
    "FR_thigh_joint",
    "FR_calf_joint",
    "FL_hip_joint",
    "FL_thigh_joint",
    "FL_calf_joint",
    "RR_hip_joint",
    "RR_thigh_joint",
    "RR_calf_joint",
    "RL_hip_joint",
    "RL_thigh_joint",
    "RL_calf_joint",
]


def finite(value, default=math.nan):
    return value if value is not None else default


def msg_age(now_s, msg_time_s):
    return math.nan if msg_time_s is None else now_s - msg_time_s


def quaternion_to_euler(w, x, y, z):
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)

    sinp = 2.0 * (w * y - z * x)
    if abs(sinp) >= 1.0:
        pitch = math.copysign(math.pi / 2.0, sinp)
    else:
        pitch = math.asin(sinp)

    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return roll, pitch, yaw


class Go2PolicyCsvLogger(Node):
    def __init__(self, args):
        super().__init__("go2_policy_csv_logger")
        self.args = args
        self.start_wall_s = time()
        self.sample_idx = 0
        self.flush_count = 0

        self.last_command = None
        self.last_command_wall_s = None
        self.last_state = None
        self.last_state_wall_s = None
        self.last_imu = None
        self.last_imu_wall_s = None
        self.last_cmd_vel = None
        self.last_cmd_vel_wall_s = None

        output_path = Path(args.output).expanduser() if args.output else self.default_output_path()
        output_path.parent.mkdir(parents=True, exist_ok=True)
        self.output_path = output_path
        self.csv_file = output_path.open("w", newline="")
        self.writer = csv.writer(self.csv_file)
        self.writer.writerow(self.header())

        qos = QoSProfile(depth=20)
        self.create_subscription(RobotCommand, args.command_topic, self.command_callback, qos)
        self.create_subscription(RobotState, args.state_topic, self.state_callback, qos)
        self.create_subscription(Imu, args.imu_topic, self.imu_callback, qos)
        self.create_subscription(Twist, args.cmd_vel_topic, self.cmd_vel_callback, qos)
        self.create_timer(1.0 / args.rate, self.write_row)

        self.get_logger().info(f"Logging Go2 policy CSV to: {self.output_path}")
        self.get_logger().info(
            "Topics: "
            f"command={args.command_topic}, state={args.state_topic}, "
            f"imu={args.imu_topic}, cmd_vel={args.cmd_vel_topic}"
        )

    def default_output_path(self):
        root = Path(__file__).resolve().parents[1]
        timestamp = strftime("%Y%m%d_%H%M%S")
        return root / "debug_logs" / f"go2_policy_topics_{timestamp}.csv"

    def command_callback(self, msg):
        self.last_command = msg
        self.last_command_wall_s = time()

    def state_callback(self, msg):
        self.last_state = msg
        self.last_state_wall_s = time()

    def imu_callback(self, msg):
        self.last_imu = msg
        self.last_imu_wall_s = time()

    def cmd_vel_callback(self, msg):
        self.last_cmd_vel = msg
        self.last_cmd_vel_wall_s = time()

    def header(self):
        header = [
            "sample_idx",
            "wall_time_s",
            "ros_time_s",
            "command_seen",
            "state_seen",
            "imu_seen",
            "cmd_vel_seen",
            "command_age_s",
            "state_age_s",
            "imu_age_s",
            "cmd_vel_age_s",
            "cmd_vel_linear_x",
            "cmd_vel_linear_y",
            "cmd_vel_linear_z",
            "cmd_vel_angular_x",
            "cmd_vel_angular_y",
            "cmd_vel_angular_z",
            "imu_orientation_w",
            "imu_orientation_x",
            "imu_orientation_y",
            "imu_orientation_z",
            "imu_roll",
            "imu_pitch",
            "imu_yaw",
            "imu_angular_velocity_x",
            "imu_angular_velocity_y",
            "imu_angular_velocity_z",
            "imu_linear_acceleration_x",
            "imu_linear_acceleration_y",
            "imu_linear_acceleration_z",
        ]
        for joint in JOINT_NAMES:
            header.extend(
                [
                    f"cmd_{joint}_q",
                    f"cmd_{joint}_dq",
                    f"cmd_{joint}_tau",
                    f"cmd_{joint}_kp",
                    f"cmd_{joint}_kd",
                    f"state_{joint}_q",
                    f"state_{joint}_dq",
                    f"state_{joint}_tau_est",
                    f"error_{joint}_q",
                    f"error_{joint}_dq",
                    f"pd_tau_{joint}",
                ]
            )
        return header

    def motor_command_at(self, index):
        if self.last_command is None or index >= len(self.last_command.motor_command):
            return None
        return self.last_command.motor_command[index]

    def motor_state_at(self, index):
        if self.last_state is None or index >= len(self.last_state.motor_state):
            return None
        return self.last_state.motor_state[index]

    def write_row(self):
        now_wall_s = time()
        now_ros_s = self.get_clock().now().nanoseconds * 1e-9

        cmd_vel = self.last_cmd_vel
        imu = self.last_imu
        imu_roll = math.nan
        imu_pitch = math.nan
        imu_yaw = math.nan
        if imu:
            imu_roll, imu_pitch, imu_yaw = quaternion_to_euler(
                imu.orientation.w,
                imu.orientation.x,
                imu.orientation.y,
                imu.orientation.z,
            )

        row = [
            self.sample_idx,
            now_wall_s - self.start_wall_s,
            now_ros_s,
            int(self.last_command is not None),
            int(self.last_state is not None),
            int(self.last_imu is not None),
            int(self.last_cmd_vel is not None),
            msg_age(now_wall_s, self.last_command_wall_s),
            msg_age(now_wall_s, self.last_state_wall_s),
            msg_age(now_wall_s, self.last_imu_wall_s),
            msg_age(now_wall_s, self.last_cmd_vel_wall_s),
            finite(cmd_vel.linear.x if cmd_vel else None),
            finite(cmd_vel.linear.y if cmd_vel else None),
            finite(cmd_vel.linear.z if cmd_vel else None),
            finite(cmd_vel.angular.x if cmd_vel else None),
            finite(cmd_vel.angular.y if cmd_vel else None),
            finite(cmd_vel.angular.z if cmd_vel else None),
            finite(imu.orientation.w if imu else None),
            finite(imu.orientation.x if imu else None),
            finite(imu.orientation.y if imu else None),
            finite(imu.orientation.z if imu else None),
            imu_roll,
            imu_pitch,
            imu_yaw,
            finite(imu.angular_velocity.x if imu else None),
            finite(imu.angular_velocity.y if imu else None),
            finite(imu.angular_velocity.z if imu else None),
            finite(imu.linear_acceleration.x if imu else None),
            finite(imu.linear_acceleration.y if imu else None),
            finite(imu.linear_acceleration.z if imu else None),
        ]

        for index in range(len(JOINT_NAMES)):
            command = self.motor_command_at(index)
            state = self.motor_state_at(index)
            cmd_q = finite(command.q if command else None)
            cmd_dq = finite(command.dq if command else None)
            cmd_tau = finite(command.tau if command else None)
            cmd_kp = finite(command.kp if command else None)
            cmd_kd = finite(command.kd if command else None)
            state_q = finite(state.q if state else None)
            state_dq = finite(state.dq if state else None)
            state_tau = finite(state.tau_est if state else None)
            error_q = cmd_q - state_q
            error_dq = cmd_dq - state_dq
            pd_tau = cmd_kp * error_q + cmd_kd * error_dq + cmd_tau
            row.extend(
                [
                    cmd_q,
                    cmd_dq,
                    cmd_tau,
                    cmd_kp,
                    cmd_kd,
                    state_q,
                    state_dq,
                    state_tau,
                    error_q,
                    error_dq,
                    pd_tau,
                ]
            )

        self.writer.writerow(row)
        self.sample_idx += 1
        self.flush_count += 1
        if self.flush_count >= self.args.flush_every:
            self.csv_file.flush()
            self.flush_count = 0

    def close(self):
        self.csv_file.flush()
        self.csv_file.close()
        if rclpy.ok():
            self.get_logger().info(f"Saved CSV: {self.output_path}")
        else:
            print(f"Saved CSV: {self.output_path}")


def parse_args():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--rate", type=float, default=50.0, help="CSV sample rate in Hz.")
    parser.add_argument("--output", default="", help="Output CSV path. Defaults to debug_logs/ timestamp.")
    parser.add_argument("--command-topic", default="/robot_joint_controller/command")
    parser.add_argument("--state-topic", default="/robot_joint_controller/state")
    parser.add_argument("--imu-topic", default="/imu")
    parser.add_argument("--cmd-vel-topic", default="/cmd_vel")
    parser.add_argument("--flush-every", type=int, default=25, help="Flush every N rows.")
    return parser.parse_args()


def main():
    args = parse_args()
    rclpy.init()
    node = Go2PolicyCsvLogger(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

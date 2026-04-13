#!/usr/bin/env python3

import csv
import os
from datetime import datetime
from pathlib import Path

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]


class JointVelocityLogger(Node):
    def __init__(self) -> None:
        super().__init__("joint_velocity_logger")

        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter(
            "output_directory",
            os.path.expanduser("~/ros2_ws/data/joint_velocity_logs"),
        )
        self.declare_parameter("output_prefix", "joint_velocity")
        self.declare_parameter("flush_every", 1)

        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.output_directory = Path(str(self.get_parameter("output_directory").value)).expanduser()
        self.output_prefix = str(self.get_parameter("output_prefix").value)
        self.flush_every = max(1, int(self.get_parameter("flush_every").value))

        self.output_directory.mkdir(parents=True, exist_ok=True)
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.output_path = self.output_directory / f"{self.output_prefix}_{timestamp}.txt"
        self.log_file = self.output_path.open("w", newline="")
        self.writer = csv.writer(self.log_file)
        self.writer.writerow(["stamp_sec", *[f"{name}_velocity" for name in JOINT_NAMES]])
        self.log_file.flush()
        self.write_count = 0

        self.create_subscription(JointState, self.joint_state_topic, self.on_joint_state, 100)

        self.get_logger().info(f"Logging joint velocities to: {self.output_path}")

    def on_joint_state(self, msg: JointState) -> None:
        velocities = [float("nan")] * len(JOINT_NAMES)

        if len(msg.name) >= len(JOINT_NAMES):
            name_to_index = {name: idx for idx, name in enumerate(msg.name)}
            if all(name in name_to_index for name in JOINT_NAMES):
                for idx, joint_name in enumerate(JOINT_NAMES):
                    source_index = name_to_index[joint_name]
                    if source_index < len(msg.velocity):
                        velocities[idx] = float(msg.velocity[source_index])
            else:
                for idx in range(min(len(JOINT_NAMES), len(msg.velocity))):
                    velocities[idx] = float(msg.velocity[idx])
        else:
            for idx in range(min(len(JOINT_NAMES), len(msg.velocity))):
                velocities[idx] = float(msg.velocity[idx])

        stamp_sec = float(msg.header.stamp.sec) + float(msg.header.stamp.nanosec) * 1e-9
        self.writer.writerow([stamp_sec, *velocities])
        self.write_count += 1
        if self.write_count % self.flush_every == 0:
            self.log_file.flush()

    def destroy_node(self):
        try:
            if hasattr(self, "log_file") and self.log_file is not None:
                self.log_file.flush()
                self.log_file.close()
        finally:
            return super().destroy_node()


def main() -> None:
    rclpy.init()
    node = JointVelocityLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

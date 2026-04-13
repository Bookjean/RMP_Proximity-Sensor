#!/usr/bin/env python3

import csv
import os
import threading
import time
from datetime import datetime
from typing import List

import rclpy
from geometry_msgs.msg import Point, Pose, PoseStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64MultiArray
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker
from visualization_msgs.msg import MarkerArray


JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]


class RmpDataRecorder(Node):
    def __init__(self) -> None:
        super().__init__("rmp_data_recorder")

        self.declare_parameter("recording_rate", 100.0)
        self.declare_parameter(
            "output_directory",
            os.path.expanduser("~/ros2_ws/data/rmp_datasets"),
        )
        self.declare_parameter("output_prefix", "rmp_dataset")
        self.declare_parameter("mode", "simulation")
        self.declare_parameter("auto_start", True)
        self.declare_parameter("joint_state_topic", "/joint_states")
        self.declare_parameter("command_topic", "/position_controllers/commands")
        self.declare_parameter("goal_position_topic", "/goal_position")
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("ee_pose_topic", "/rmp_ee_pose")
        self.declare_parameter("obstacle_topic", "/obstacles")
        self.declare_parameter("reference_joint_state_topic", "/rb10/reference_joint_states")
        self.declare_parameter("measured_joint_state_topic", "/rb10/measured_joint_states")
        self.declare_parameter("tracking_error_topic", "/rb10/joint_tracking_error_deg")
        self.declare_parameter(
            "range_topics",
            [
                "/proximity_distance1",
                "/proximity_distance2",
                "/proximity_distance3",
                "/proximity_distance4",
            ],
        )
        self.declare_parameter("max_obstacles", 4)

        self.recording_rate = float(self.get_parameter("recording_rate").value)
        self.output_directory = str(self.get_parameter("output_directory").value)
        self.output_prefix = str(self.get_parameter("output_prefix").value)
        self.mode = str(self.get_parameter("mode").value)
        self.auto_start = self._as_bool(self.get_parameter("auto_start").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.goal_position_topic = str(self.get_parameter("goal_position_topic").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.ee_pose_topic = str(self.get_parameter("ee_pose_topic").value)
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.reference_joint_state_topic = str(self.get_parameter("reference_joint_state_topic").value)
        self.measured_joint_state_topic = str(self.get_parameter("measured_joint_state_topic").value)
        self.tracking_error_topic = str(self.get_parameter("tracking_error_topic").value)
        self.range_topics = list(self.get_parameter("range_topics").value)
        self.max_obstacles = int(self.get_parameter("max_obstacles").value)

        os.makedirs(self.output_directory, exist_ok=True)

        self.cb_group = ReentrantCallbackGroup()
        self.data_lock = threading.Lock()
        self.file_lock = threading.Lock()

        self.latest_joint_positions = [float("nan")] * len(JOINT_NAMES)
        self.latest_joint_velocities = [float("nan")] * len(JOINT_NAMES)
        self.latest_command = [float("nan")] * len(JOINT_NAMES)
        self.latest_goal_position = [float("nan")] * 3
        self.latest_goal_pose = [float("nan")] * 7
        self.latest_ee_pose = [float("nan")] * 7
        self.latest_ranges = [float("nan")] * len(self.range_topics)
        self.latest_obstacles = [[float("nan")] * 4 for _ in range(self.max_obstacles)]
        self.latest_reference_joint_positions = [float("nan")] * len(JOINT_NAMES)
        self.latest_measured_joint_positions = [float("nan")] * len(JOINT_NAMES)
        self.latest_tracking_error_deg = [float("nan")] * len(JOINT_NAMES)

        self.prev_joint_positions = None
        self.prev_joint_time = None

        self.is_recording = False
        self.recording_path = None
        self.recording_handle = None
        self.recording_writer = None

        self.create_subscription(
            JointState,
            self.joint_state_topic,
            self.on_joint_state,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Float64MultiArray,
            self.command_topic,
            self.on_command,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Point,
            self.goal_position_topic,
            self.on_goal_position,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            PoseStamped,
            self.goal_pose_topic,
            self.on_goal_pose,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Pose,
            self.ee_pose_topic,
            self.on_ee_pose,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            MarkerArray,
            self.obstacle_topic,
            self.on_obstacles,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            JointState,
            self.reference_joint_state_topic,
            self.on_reference_joint_state,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            JointState,
            self.measured_joint_state_topic,
            self.on_measured_joint_state,
            10,
            callback_group=self.cb_group,
        )
        self.create_subscription(
            Float64MultiArray,
            self.tracking_error_topic,
            self.on_tracking_error,
            10,
            callback_group=self.cb_group,
        )

        self.range_subs = []
        for index, topic in enumerate(self.range_topics):
            self.range_subs.append(
                self.create_subscription(
                    Range,
                    topic,
                    lambda msg, idx=index: self.on_range(msg, idx),
                    10,
                    callback_group=self.cb_group,
                )
            )

        self.start_service = self.create_service(
            Trigger,
            "~/start",
            self.on_start_recording,
            callback_group=self.cb_group,
        )
        self.stop_service = self.create_service(
            Trigger,
            "~/stop",
            self.on_stop_recording,
            callback_group=self.cb_group,
        )

        period = max(1.0 / max(self.recording_rate, 1e-3), 1e-3)
        self.recording_timer = self.create_timer(
            period,
            self.record_once,
            callback_group=self.cb_group,
        )

        self.get_logger().info(f"Recorder mode: {self.mode}")
        self.get_logger().info(f"Joint topic: {self.joint_state_topic}")
        self.get_logger().info(f"Command topic: {self.command_topic}")
        self.get_logger().info(f"EE pose topic: {self.ee_pose_topic}")
        self.get_logger().info(f"Output directory: {self.output_directory}")

        if self.auto_start:
            self.start_recording()

    def on_joint_state(self, msg: JointState) -> None:
        positions = [float("nan")] * len(JOINT_NAMES)
        velocities = [float("nan")] * len(JOINT_NAMES)

        if len(msg.name) >= len(JOINT_NAMES):
            index_map = {name: idx for idx, name in enumerate(msg.name)}
            matched = all(name in index_map for name in JOINT_NAMES)
            if matched:
                for idx, joint_name in enumerate(JOINT_NAMES):
                    source_index = index_map[joint_name]
                    if source_index < len(msg.position):
                        positions[idx] = float(msg.position[source_index])
                    if source_index < len(msg.velocity):
                        velocities[idx] = float(msg.velocity[source_index])
            else:
                self._fill_joint_by_index(msg, positions, velocities)
        else:
            self._fill_joint_by_index(msg, positions, velocities)

        now_time = time.time()
        if any(not self._is_finite(value) for value in velocities):
            if self.prev_joint_positions is not None and self.prev_joint_time is not None:
                dt = now_time - self.prev_joint_time
                if 1e-4 < dt < 1.0:
                    velocities = [
                        (positions[idx] - self.prev_joint_positions[idx]) / dt
                        if self._is_finite(positions[idx]) and self._is_finite(self.prev_joint_positions[idx])
                        else float("nan")
                        for idx in range(len(JOINT_NAMES))
                    ]

        with self.data_lock:
            self.latest_joint_positions = positions
            self.latest_joint_velocities = velocities

        self.prev_joint_positions = positions
        self.prev_joint_time = now_time

    def on_command(self, msg: Float64MultiArray) -> None:
        command = [float("nan")] * len(JOINT_NAMES)
        for idx, value in enumerate(list(msg.data)[: len(JOINT_NAMES)]):
            command[idx] = float(value)
        with self.data_lock:
            self.latest_command = command

    def on_goal_position(self, msg: Point) -> None:
        with self.data_lock:
            self.latest_goal_position = [float(msg.x), float(msg.y), float(msg.z)]
            if not any(self._is_finite(v) for v in self.latest_goal_pose[:3]):
                self.latest_goal_pose[:3] = [float(msg.x), float(msg.y), float(msg.z)]

    def on_goal_pose(self, msg: PoseStamped) -> None:
        pose = msg.pose
        with self.data_lock:
            self.latest_goal_position = [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
            ]
            self.latest_goal_pose = [
                float(pose.position.x),
                float(pose.position.y),
                float(pose.position.z),
                float(pose.orientation.x),
                float(pose.orientation.y),
                float(pose.orientation.z),
                float(pose.orientation.w),
            ]

    def on_ee_pose(self, msg: Pose) -> None:
        with self.data_lock:
            self.latest_ee_pose = [
                float(msg.position.x),
                float(msg.position.y),
                float(msg.position.z),
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            ]

    def on_range(self, msg: Range, index: int) -> None:
        with self.data_lock:
            if index < len(self.latest_ranges):
                self.latest_ranges[index] = float(msg.range)

    def on_obstacles(self, msg: MarkerArray) -> None:
        obstacle_rows = []
        for marker in msg.markers:
            if marker.action == Marker.DELETE:
                continue
            radius = max(float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)) * 0.5
            obstacle_rows.append([
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
                radius,
            ])
            if len(obstacle_rows) >= self.max_obstacles:
                break

        while len(obstacle_rows) < self.max_obstacles:
            obstacle_rows.append([float("nan")] * 4)

        with self.data_lock:
            self.latest_obstacles = obstacle_rows

    def on_reference_joint_state(self, msg: JointState) -> None:
        with self.data_lock:
            self.latest_reference_joint_positions = self._extract_joint_positions(msg)

    def on_measured_joint_state(self, msg: JointState) -> None:
        with self.data_lock:
            self.latest_measured_joint_positions = self._extract_joint_positions(msg)

    def on_tracking_error(self, msg: Float64MultiArray) -> None:
        tracking_error = [float("nan")] * len(JOINT_NAMES)
        for idx, value in enumerate(list(msg.data)[: len(JOINT_NAMES)]):
            tracking_error[idx] = float(value)
        with self.data_lock:
            self.latest_tracking_error_deg = tracking_error

    def on_start_recording(self, request, response):
        del request
        success, message = self.start_recording()
        response.success = success
        response.message = message
        return response

    def on_stop_recording(self, request, response):
        del request
        success, message = self.stop_recording()
        response.success = success
        response.message = message
        return response

    def start_recording(self):
        with self.file_lock:
            if self.is_recording:
                return False, "Already recording"

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.output_prefix}_{self.mode}_{timestamp}.csv"
            self.recording_path = os.path.join(self.output_directory, filename)

            self.recording_handle = open(self.recording_path, "w", newline="")
            self.recording_handle.write(f"# mode,{self.mode}\n")
            self.recording_handle.write(f"# started_at,{datetime.now().isoformat()}\n")
            self.recording_handle.write(f"# joint_state_topic,{self.joint_state_topic}\n")
            self.recording_handle.write(f"# command_topic,{self.command_topic}\n")
            self.recording_handle.write(f"# ee_pose_topic,{self.ee_pose_topic}\n")
            self.recording_handle.write(f"# reference_joint_state_topic,{self.reference_joint_state_topic}\n")
            self.recording_handle.write(f"# measured_joint_state_topic,{self.measured_joint_state_topic}\n")
            self.recording_handle.write(f"# tracking_error_topic,{self.tracking_error_topic}\n")
            self.recording_handle.write(f"# range_topics,{';'.join(self.range_topics)}\n")
            self.recording_writer = csv.writer(self.recording_handle)
            self.recording_writer.writerow(self._header())
            self.recording_handle.flush()
            self.is_recording = True

        message = f"Recording started: {self.recording_path}"
        self.get_logger().info(message)
        return True, message

    def stop_recording(self):
        with self.file_lock:
            if not self.is_recording:
                return False, "Not recording"

            self.is_recording = False
            if self.recording_handle is not None:
                self.recording_handle.write(f"# stopped_at,{datetime.now().isoformat()}\n")
                self.recording_handle.flush()
                self.recording_handle.close()
            path = self.recording_path
            self.recording_handle = None
            self.recording_writer = None
            self.recording_path = None

        message = f"Recording stopped: {path}"
        self.get_logger().info(message)
        return True, message

    def record_once(self) -> None:
        if not self.is_recording:
            return

        with self.data_lock:
            row = [
                datetime.now().isoformat(timespec="milliseconds"),
                f"{time.time():.6f}",
                self.mode,
                *self.latest_joint_positions,
                *self.latest_joint_velocities,
                *self.latest_command,
                *self.latest_goal_position,
                *self.latest_goal_pose,
                *self.latest_ee_pose,
                *self.latest_ranges,
                *self.latest_reference_joint_positions,
                *self.latest_measured_joint_positions,
                *self.latest_tracking_error_deg,
            ]
            for obstacle in self.latest_obstacles:
                row.extend(obstacle)

        with self.file_lock:
            if not self.is_recording or self.recording_writer is None or self.recording_handle is None:
                return
            self.recording_writer.writerow(self._format_row(row))
            self.recording_handle.flush()

    def destroy_node(self):
        if self.is_recording:
            self.stop_recording()
        super().destroy_node()

    def _fill_joint_by_index(
        self,
        msg: JointState,
        positions: List[float],
        velocities: List[float],
    ) -> None:
        for idx in range(min(len(JOINT_NAMES), len(msg.position))):
            positions[idx] = float(msg.position[idx])
        for idx in range(min(len(JOINT_NAMES), len(msg.velocity))):
            velocities[idx] = float(msg.velocity[idx])

    def _extract_joint_positions(self, msg: JointState) -> List[float]:
        positions = [float("nan")] * len(JOINT_NAMES)
        if len(msg.name) >= len(JOINT_NAMES):
            index_map = {name: idx for idx, name in enumerate(msg.name)}
            matched = all(name in index_map for name in JOINT_NAMES)
            if matched:
                for idx, joint_name in enumerate(JOINT_NAMES):
                    source_index = index_map[joint_name]
                    if source_index < len(msg.position):
                        positions[idx] = float(msg.position[source_index])
                return positions

        for idx in range(min(len(JOINT_NAMES), len(msg.position))):
            positions[idx] = float(msg.position[idx])
        return positions

    def _header(self):
        header = [
            "timestamp_iso",
            "timestamp_unix",
            "mode",
        ]
        header.extend([f"q{i + 1}" for i in range(len(JOINT_NAMES))])
        header.extend([f"qd{i + 1}" for i in range(len(JOINT_NAMES))])
        header.extend([f"cmd_q{i + 1}" for i in range(len(JOINT_NAMES))])
        header.extend(["goal_x", "goal_y", "goal_z"])
        header.extend([
            "goal_pose_x",
            "goal_pose_y",
            "goal_pose_z",
            "goal_pose_qx",
            "goal_pose_qy",
            "goal_pose_qz",
            "goal_pose_qw",
        ])
        header.extend([
            "ee_pose_x",
            "ee_pose_y",
            "ee_pose_z",
            "ee_pose_qx",
            "ee_pose_qy",
            "ee_pose_qz",
            "ee_pose_qw",
        ])
        header.extend([f"prox{i + 1}" for i in range(len(self.range_topics))])
        header.extend([f"ref_q{i + 1}" for i in range(len(JOINT_NAMES))])
        header.extend([f"meas_q{i + 1}" for i in range(len(JOINT_NAMES))])
        header.extend([f"ref_minus_meas_deg_q{i + 1}" for i in range(len(JOINT_NAMES))])
        for idx in range(self.max_obstacles):
            header.extend([
                f"obs{idx + 1}_x",
                f"obs{idx + 1}_y",
                f"obs{idx + 1}_z",
                f"obs{idx + 1}_r",
            ])
        return header

    def _as_bool(self, value) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.strip().lower() in {"1", "true", "yes", "on"}
        return bool(value)

    def _format_row(self, row):
        formatted = []
        for value in row:
            if isinstance(value, str):
                formatted.append(value)
            elif self._is_finite(value):
                formatted.append(f"{float(value):.6f}")
            else:
                formatted.append("")
        return formatted

    def _is_finite(self, value) -> bool:
        try:
            return float(value) == float(value) and abs(float(value)) != float("inf")
        except (TypeError, ValueError):
            return False


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RmpDataRecorder()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()

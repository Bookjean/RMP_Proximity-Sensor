#!/usr/bin/env python3

import csv
import math
import os
import threading
import time
from datetime import datetime
from typing import List

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
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
        self.declare_parameter("goal_pose_topic", "/goal_pose")
        self.declare_parameter("ee_pose_topic", "/rmp_ee_pose")
        self.declare_parameter("obstacle_topic", "/obstacles")
        self.declare_parameter("control_point_topic", "/control_points")
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
                "/proximity_distance9",
                "/proximity_distance10",
                "/proximity_distance11",
                "/proximity_distance12",
            ],
        )
        self.declare_parameter("max_obstacles", 8)
        self.declare_parameter("max_control_points", 8)
        self.declare_parameter("collision_rmp_margin", 0.0)
        self.declare_parameter("collision_rmp_metric_modulation_radius", 0.25)
        self.declare_parameter("collision_rmp_metric_scalar", 500.0)
        self.declare_parameter("collision_rmp_metric_exploder_std_dev", 0.02)
        self.declare_parameter("collision_rmp_metric_exploder_eps", 0.001)
        self.declare_parameter("collision_rmp_damping_velocity_gate_length_scale", 0.05)
        self.declare_parameter("estimate_velocity_in_controller", False)
        self.declare_parameter("use_synced_input_velocity_filter", False)
        self.declare_parameter("synced_input_velocity_filter_type", "")
        self.declare_parameter("synced_input_velocity_filter_alpha", 0.0)
        self.declare_parameter("synced_input_velocity_filter_beta", 0.0)
        self.declare_parameter("synced_input_velocity_ratio_tolerance", 0.0)
        self.declare_parameter("use_velocity_feedback_in_solver", True)
        self.declare_parameter("measured_velocity_feedback_blend", 1.0)

        self.recording_rate = float(self.get_parameter("recording_rate").value)
        self.output_directory = str(self.get_parameter("output_directory").value)
        self.output_prefix = str(self.get_parameter("output_prefix").value)
        self.mode = str(self.get_parameter("mode").value)
        self.auto_start = self._as_bool(self.get_parameter("auto_start").value)
        self.joint_state_topic = str(self.get_parameter("joint_state_topic").value)
        self.command_topic = str(self.get_parameter("command_topic").value)
        self.goal_pose_topic = str(self.get_parameter("goal_pose_topic").value)
        self.ee_pose_topic = str(self.get_parameter("ee_pose_topic").value)
        self.obstacle_topic = str(self.get_parameter("obstacle_topic").value)
        self.control_point_topic = str(self.get_parameter("control_point_topic").value)
        self.reference_joint_state_topic = str(self.get_parameter("reference_joint_state_topic").value)
        self.measured_joint_state_topic = str(self.get_parameter("measured_joint_state_topic").value)
        self.tracking_error_topic = str(self.get_parameter("tracking_error_topic").value)
        self.range_topics = list(self.get_parameter("range_topics").value)
        self.max_obstacles = int(self.get_parameter("max_obstacles").value)
        self.max_control_points = int(self.get_parameter("max_control_points").value)
        self.collision_margin = float(self.get_parameter("collision_rmp_margin").value)
        self.collision_metric_modulation_radius = float(
            self.get_parameter("collision_rmp_metric_modulation_radius").value
        )
        self.collision_metric_scalar = float(self.get_parameter("collision_rmp_metric_scalar").value)
        self.collision_metric_exploder_std_dev = float(
            self.get_parameter("collision_rmp_metric_exploder_std_dev").value
        )
        self.collision_metric_exploder_eps = float(
            self.get_parameter("collision_rmp_metric_exploder_eps").value
        )
        self.collision_damping_velocity_gate_length_scale = float(
            self.get_parameter("collision_rmp_damping_velocity_gate_length_scale").value
        )
        self.estimate_velocity_in_controller = self._as_bool(
            self.get_parameter("estimate_velocity_in_controller").value
        )
        self.use_synced_input_velocity_filter = self._as_bool(
            self.get_parameter("use_synced_input_velocity_filter").value
        )
        self.synced_input_velocity_filter_type = str(
            self.get_parameter("synced_input_velocity_filter_type").value
        )
        self.synced_input_velocity_filter_alpha = float(
            self.get_parameter("synced_input_velocity_filter_alpha").value
        )
        self.synced_input_velocity_filter_beta = float(
            self.get_parameter("synced_input_velocity_filter_beta").value
        )
        self.synced_input_velocity_ratio_tolerance = float(
            self.get_parameter("synced_input_velocity_ratio_tolerance").value
        )
        self.use_velocity_feedback_in_solver = self._as_bool(
            self.get_parameter("use_velocity_feedback_in_solver").value
        )
        self.measured_velocity_feedback_blend = float(
            self.get_parameter("measured_velocity_feedback_blend").value
        )

        os.makedirs(self.output_directory, exist_ok=True)

        self.cb_group = ReentrantCallbackGroup()
        self.data_lock = threading.Lock()
        self.file_lock = threading.Lock()

        self.latest_joint_positions = [float("nan")] * len(JOINT_NAMES)
        self.latest_joint_velocities = [float("nan")] * len(JOINT_NAMES)
        self.latest_command = [float("nan")] * len(JOINT_NAMES)
        self.latest_command_velocities = [float("nan")] * len(JOINT_NAMES)
        self.latest_command_accelerations = [float("nan")] * len(JOINT_NAMES)
        self.latest_goal_position = [float("nan")] * 3
        self.latest_goal_pose = [float("nan")] * 7
        self.latest_ee_pose = [float("nan")] * 7
        self.latest_ee_velocity = [float("nan")] * 4
        self.latest_ranges = [float("nan")] * len(self.range_topics)
        self.latest_obstacles = [[float("nan")] * 4 for _ in range(self.max_obstacles)]
        self.latest_control_points = [[float("nan")] * 4 for _ in range(self.max_control_points)]
        self.latest_control_point_velocities = [
            [float("nan")] * 4 for _ in range(self.max_control_points)
        ]
        self.latest_collision_diagnostics = [float("nan")] * 6
        self.latest_reference_joint_positions = [float("nan")] * len(JOINT_NAMES)
        self.latest_measured_joint_positions = [float("nan")] * len(JOINT_NAMES)
        self.latest_tracking_error_deg = [float("nan")] * len(JOINT_NAMES)

        self.prev_joint_positions = None
        self.prev_joint_time = None
        self.prev_command = None
        self.prev_command_velocities = None
        self.prev_command_time = None
        self.prev_ee_position = None
        self.prev_ee_time = None
        self.prev_control_points = None
        self.prev_control_point_time = None

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
            MarkerArray,
            self.control_point_topic,
            self.on_control_points,
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
        self.get_logger().info(f"Control point topic: {self.control_point_topic}")
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
        now_time = time.time()
        command = [float("nan")] * len(JOINT_NAMES)
        for idx, value in enumerate(list(msg.data)[: len(JOINT_NAMES)]):
            command[idx] = float(value)

        command_velocities = [float("nan")] * len(JOINT_NAMES)
        command_accelerations = [float("nan")] * len(JOINT_NAMES)
        if self.prev_command is not None and self.prev_command_time is not None:
            dt = now_time - self.prev_command_time
            if 1e-4 < dt < 1.0:
                command_velocities = [
                    (command[idx] - self.prev_command[idx]) / dt
                    if self._is_finite(command[idx]) and self._is_finite(self.prev_command[idx])
                    else float("nan")
                    for idx in range(len(JOINT_NAMES))
                ]
                if self.prev_command_velocities is not None:
                    command_accelerations = [
                        (command_velocities[idx] - self.prev_command_velocities[idx]) / dt
                        if self._is_finite(command_velocities[idx])
                        and self._is_finite(self.prev_command_velocities[idx])
                        else float("nan")
                        for idx in range(len(JOINT_NAMES))
                    ]

        with self.data_lock:
            self.latest_command = command
            self.latest_command_velocities = command_velocities
            self.latest_command_accelerations = command_accelerations

        self.prev_command = command
        self.prev_command_velocities = command_velocities
        self.prev_command_time = now_time

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
        now_time = time.time()
        position = [
            float(msg.position.x),
            float(msg.position.y),
            float(msg.position.z),
        ]
        ee_velocity = [float("nan")] * 4
        if self.prev_ee_position is not None and self.prev_ee_time is not None:
            dt = now_time - self.prev_ee_time
            if 1e-4 < dt < 1.0:
                linear_velocity = [
                    (position[idx] - self.prev_ee_position[idx]) / dt
                    if self._is_finite(position[idx]) and self._is_finite(self.prev_ee_position[idx])
                    else float("nan")
                    for idx in range(3)
                ]
                if all(self._is_finite(value) for value in linear_velocity):
                    ee_velocity = [
                        *linear_velocity,
                        math.sqrt(sum(value * value for value in linear_velocity)),
                    ]

        with self.data_lock:
            self.latest_ee_pose = [
                *position,
                float(msg.orientation.x),
                float(msg.orientation.y),
                float(msg.orientation.z),
                float(msg.orientation.w),
            ]
            self.latest_ee_velocity = ee_velocity

        self.prev_ee_position = position
        self.prev_ee_time = now_time

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
            self._update_collision_diagnostics_locked()

    def on_control_points(self, msg: MarkerArray) -> None:
        now_time = time.time()
        control_points = [[float("nan")] * 4 for _ in range(self.max_control_points)]
        fallback_index = 0

        for marker in msg.markers:
            if marker.action in (Marker.DELETE, Marker.DELETEALL):
                continue
            if 0 <= marker.id < self.max_control_points:
                index = marker.id
            else:
                while (
                    fallback_index < self.max_control_points
                    and self._is_finite(control_points[fallback_index][0])
                ):
                    fallback_index += 1
                if fallback_index >= self.max_control_points:
                    break
                index = fallback_index

            radius = max(float(marker.scale.x), float(marker.scale.y), float(marker.scale.z)) * 0.5
            control_points[index] = [
                float(marker.pose.position.x),
                float(marker.pose.position.y),
                float(marker.pose.position.z),
                radius,
            ]

        velocities = [[float("nan")] * 4 for _ in range(self.max_control_points)]
        if self.prev_control_points is not None and self.prev_control_point_time is not None:
            dt = now_time - self.prev_control_point_time
            if 1e-4 < dt < 1.0:
                for index in range(self.max_control_points):
                    current = control_points[index]
                    previous = self.prev_control_points[index]
                    if all(self._is_finite(value) for value in current[:3]) and all(
                        self._is_finite(value) for value in previous[:3]
                    ):
                        velocity = [
                            (current[axis] - previous[axis]) / dt
                            for axis in range(3)
                        ]
                        velocities[index] = [
                            *velocity,
                            math.sqrt(sum(value * value for value in velocity)),
                        ]

        with self.data_lock:
            self.latest_control_points = control_points
            self.latest_control_point_velocities = velocities
            self._update_collision_diagnostics_locked()

        self.prev_control_points = control_points
        self.prev_control_point_time = now_time

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
                return True, f"Already recording: {self.recording_path}"

            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.output_prefix}_{self.mode}_{timestamp}.csv"
            self.recording_path = os.path.join(self.output_directory, filename)

            self.recording_handle = open(self.recording_path, "w", newline="")
            self.recording_handle.write(f"# mode,{self.mode}\n")
            self.recording_handle.write(f"# started_at,{datetime.now().isoformat()}\n")
            self.recording_handle.write(f"# joint_state_topic,{self.joint_state_topic}\n")
            self.recording_handle.write(f"# command_topic,{self.command_topic}\n")
            self.recording_handle.write(f"# ee_pose_topic,{self.ee_pose_topic}\n")
            self.recording_handle.write(f"# control_point_topic,{self.control_point_topic}\n")
            self.recording_handle.write(f"# reference_joint_state_topic,{self.reference_joint_state_topic}\n")
            self.recording_handle.write(f"# measured_joint_state_topic,{self.measured_joint_state_topic}\n")
            self.recording_handle.write(f"# tracking_error_topic,{self.tracking_error_topic}\n")
            self.recording_handle.write(f"# range_topics,{';'.join(self.range_topics)}\n")
            self.recording_handle.write(f"# collision_rmp_margin,{self.collision_margin}\n")
            self.recording_handle.write(
                f"# collision_rmp_metric_modulation_radius,{self.collision_metric_modulation_radius}\n"
            )
            self.recording_handle.write(
                f"# collision_rmp_metric_scalar,{self.collision_metric_scalar}\n"
            )
            self.recording_handle.write(
                f"# collision_rmp_metric_exploder_std_dev,{self.collision_metric_exploder_std_dev}\n"
            )
            self.recording_handle.write(
                f"# collision_rmp_metric_exploder_eps,{self.collision_metric_exploder_eps}\n"
            )
            self.recording_handle.write(
                "# collision_rmp_damping_velocity_gate_length_scale,"
                f"{self.collision_damping_velocity_gate_length_scale}\n"
            )
            self.recording_handle.write(
                f"# estimate_velocity_in_controller,{self.estimate_velocity_in_controller}\n"
            )
            self.recording_handle.write(
                f"# use_synced_input_velocity_filter,{self.use_synced_input_velocity_filter}\n"
            )
            self.recording_handle.write(
                f"# synced_input_velocity_filter_type,{self.synced_input_velocity_filter_type}\n"
            )
            self.recording_handle.write(
                f"# synced_input_velocity_filter_alpha,{self.synced_input_velocity_filter_alpha}\n"
            )
            self.recording_handle.write(
                f"# synced_input_velocity_filter_beta,{self.synced_input_velocity_filter_beta}\n"
            )
            self.recording_handle.write(
                "# synced_input_velocity_ratio_tolerance,"
                f"{self.synced_input_velocity_ratio_tolerance}\n"
            )
            self.recording_handle.write(
                f"# use_velocity_feedback_in_solver,{self.use_velocity_feedback_in_solver}\n"
            )
            self.recording_handle.write(
                f"# measured_velocity_feedback_blend,{self.measured_velocity_feedback_blend}\n"
            )
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
                *self.latest_command_velocities,
                *self.latest_command_accelerations,
                *self.latest_goal_position,
                *self.latest_goal_pose,
                *self.latest_ee_pose,
                *self.latest_ee_velocity,
                *(value for point in self.latest_control_points for value in point),
                *(value for point in self.latest_control_point_velocities for value in point),
                *self.latest_collision_diagnostics,
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
        header.extend([f"cmd_qd{i + 1}" for i in range(len(JOINT_NAMES))])
        header.extend([f"cmd_qdd{i + 1}" for i in range(len(JOINT_NAMES))])
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
        header.extend(["ee_vx", "ee_vy", "ee_vz", "ee_speed"])
        for idx in range(self.max_control_points):
            header.extend([
                f"cp{idx + 1}_x",
                f"cp{idx + 1}_y",
                f"cp{idx + 1}_z",
                f"cp{idx + 1}_r",
            ])
        for idx in range(self.max_control_points):
            header.extend([
                f"cp{idx + 1}_vx",
                f"cp{idx + 1}_vy",
                f"cp{idx + 1}_vz",
                f"cp{idx + 1}_speed",
            ])
        header.extend([
            "collision_activation_weight",
            "collision_activation_distance",
            "collision_activation_distance_rate",
            "collision_min_distance",
            "collision_cp_index",
            "collision_obstacle_index",
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

    def _update_collision_diagnostics_locked(self) -> None:
        best_weight = float("nan")
        best_distance = float("nan")
        best_distance_rate = float("nan")
        best_control_point_index = float("nan")
        best_obstacle_index = float("nan")
        min_distance = float("nan")

        for cp_index, control_point in enumerate(self.latest_control_points):
            if not all(self._is_finite(value) for value in control_point):
                continue
            cp_position = control_point[:3]
            cp_radius = control_point[3]
            cp_velocity = self.latest_control_point_velocities[cp_index][:3]

            for obstacle_index, obstacle in enumerate(self.latest_obstacles):
                if not all(self._is_finite(value) for value in obstacle):
                    continue
                delta = [
                    cp_position[axis] - obstacle[axis]
                    for axis in range(3)
                ]
                center_distance = math.sqrt(sum(value * value for value in delta))
                if center_distance <= 1e-9:
                    continue

                clearance = max(
                    center_distance - (cp_radius + obstacle[3]) - self.collision_margin,
                    0.0,
                )
                if not self._is_finite(min_distance) or clearance < min_distance:
                    min_distance = clearance

                delta_hat = [value / center_distance for value in delta]
                distance_rate = float("nan")
                if all(self._is_finite(value) for value in cp_velocity):
                    distance_rate = sum(delta_hat[axis] * cp_velocity[axis] for axis in range(3))

                metric_weight = self._collision_metric_weight(clearance, distance_rate)
                if self._is_finite(metric_weight) and (
                    not self._is_finite(best_weight) or metric_weight > best_weight
                ):
                    best_weight = metric_weight
                    best_distance = clearance
                    best_distance_rate = distance_rate
                    best_control_point_index = float(cp_index)
                    best_obstacle_index = float(obstacle_index)

        self.latest_collision_diagnostics = [
            best_weight,
            best_distance,
            best_distance_rate,
            min_distance,
            best_control_point_index,
            best_obstacle_index,
        ]

    def _collision_metric_weight(self, distance: float, distance_rate: float) -> float:
        radius = self.collision_metric_modulation_radius
        if radius <= 1e-9 or distance > radius:
            return 0.0

        gate = distance * distance / (radius * radius) - 2.0 * distance / radius + 1.0
        denominator = (
            distance / self.collision_metric_exploder_std_dev +
            self.collision_metric_exploder_eps
        )
        if denominator <= 1e-12:
            return float("nan")

        velocity_gate_scale = max(self.collision_damping_velocity_gate_length_scale, 1e-9)
        rate_for_gate = distance_rate if self._is_finite(distance_rate) else 0.0
        sigma = self._sigmoid(rate_for_gate / velocity_gate_scale)
        return self.collision_metric_scalar / denominator * gate * (1.0 - sigma)

    def _sigmoid(self, value: float) -> float:
        if value >= 0.0:
            z = math.exp(-min(value, 60.0))
            return 1.0 / (1.0 + z)
        z = math.exp(max(value, -60.0))
        return z / (1.0 + z)

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

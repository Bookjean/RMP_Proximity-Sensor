#!/usr/bin/env python3
import csv
import math
import os
from dataclasses import dataclass
from datetime import datetime
from typing import Any, Dict, List, Optional, Tuple

import rclpy
from geometry_msgs.msg import Pose, PoseStamped
from rclpy.node import Node
from sensor_msgs.msg import JointState, Range
from std_msgs.msg import Float64MultiArray, UInt8
from visualization_msgs.msg import Marker, MarkerArray


@dataclass
class StampedValue:
    stamp_s: float
    data: Any


class RmpflowTraceLogger(Node):
    DEBUG_FIELDS = [
        "goal_error_m",
        "tcp_z_m",
        "link6_z_m",
        "min_external_clearance_m",
        "min_body_clearance_m",
        "joint_velocity_norm",
        "min_z_safety_triggered",
        "min_link_z_m",
        "min_joint_z_m",
        "min_control_point_z_m",
        "min_body_obstacle_z_m",
    ]

    def __init__(self) -> None:
        super().__init__("rmpflow_trace_logger")

        self.declare_parameter("log_rate_hz", 1.0)
        self.declare_parameter("console_summary", True)
        self.declare_parameter("csv_enabled", True)
        self.declare_parameter("output_directory", "~/ros2_ws/log/rmpflow_trace")
        self.declare_parameter("output_prefix", "rmpflow_trace")
        self.declare_parameter("active_flag_value", 1)
        self.declare_parameter("range_scale", 0.001)
        self.declare_parameter("minimum_hold_distance", 0.05)
        self.declare_parameter("valid_margin", 0.001)
        self.declare_parameter("trigger_distance", 0.29)
        self.declare_parameter("rmp_flag_topic", "/RMP_flag")
        self.declare_parameter("external_goal_topic", "/RMP_goal")
        self.declare_parameter("controller_goal_topic", "/goal_pose")
        self.declare_parameter("joint_state_topic", "/rb10/joint_states")
        self.declare_parameter("command_topic", "/position_controllers/commands")
        self.declare_parameter("target_q_topic", "/target_q")
        self.declare_parameter("target_metric_topic", "/target_metric")
        self.declare_parameter("debug_state_topic", "/rmp_debug_state")
        self.declare_parameter("rmp_ee_pose_topic", "/rmp_ee_pose")
        self.declare_parameter("obstacle_marker_topic", "/obstacles")
        self.declare_parameter("repulsion_metric_marker_topic", "/repulsion_metric_markers")
        self.declare_parameter("tcp_accel_marker_topic", "/tcp_accel_marker")
        self.declare_parameter(
            "range_topics",
            [
                "/proximity_distance1",
                "/proximity_distance2",
                "/proximity_distance3",
                "/proximity_distance4",
                "/proximity_distance5",
                "/proximity_distance6",
                "/proximity_distance7",
                "/proximity_distance8",
                "/proximity_distance9",
                "/proximity_distance10",
                "/proximity_distance11",
                "/proximity_distance12",
            ],
        )

        self.console_summary = self._as_bool(self.get_parameter("console_summary").value)
        self.csv_enabled = self._as_bool(self.get_parameter("csv_enabled").value)
        self.active_flag_value = int(self.get_parameter("active_flag_value").value)
        self.range_scale = float(self.get_parameter("range_scale").value)
        self.minimum_hold_distance = float(self.get_parameter("minimum_hold_distance").value)
        self.valid_margin = float(self.get_parameter("valid_margin").value)
        self.trigger_distance = float(self.get_parameter("trigger_distance").value)
        self.range_topics = list(self.get_parameter("range_topics").value)
        self.range_labels = [self._topic_label(topic) for topic in self.range_topics]

        self.latest: Dict[str, StampedValue] = {}
        self.latest_ranges: Dict[str, StampedValue] = {}
        self.active_obstacle_markers: Dict[Tuple[str, int], Marker] = {}
        self.repulsion_metric_markers: Dict[Tuple[str, int], Marker] = {}
        self.tcp_accel_marker: Optional[StampedValue] = None

        self.csv_file = None
        self.csv_writer = None
        self.csv_path = ""
        self.csv_header = self._make_csv_header()
        if self.csv_enabled:
            output_dir = os.path.expanduser(str(self.get_parameter("output_directory").value))
            os.makedirs(output_dir, exist_ok=True)
            prefix = str(self.get_parameter("output_prefix").value)
            stamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            self.csv_path = os.path.join(output_dir, f"{prefix}_{stamp}.csv")
            self.csv_file = open(self.csv_path, "w", newline="", buffering=1)
            self.csv_writer = csv.DictWriter(self.csv_file, fieldnames=self.csv_header)
            self.csv_writer.writeheader()

        self._create_subscriptions()

        period_s = 1.0 / max(float(self.get_parameter("log_rate_hz").value), 0.1)
        self.timer = self.create_timer(period_s, self._write_snapshot)

        if self.csv_enabled:
            self.get_logger().info(f"RMPFlow trace logging to {self.csv_path}")
        else:
            self.get_logger().info("RMPFlow trace CSV disabled; console summary only")

    def destroy_node(self) -> bool:
        if self.csv_file:
            self.csv_file.close()
        return super().destroy_node()

    def _create_subscriptions(self) -> None:
        self.create_subscription(
            UInt8,
            str(self.get_parameter("rmp_flag_topic").value),
            lambda msg: self._store("rmp_flag", int(msg.data)),
            10,
        )
        self.create_subscription(
            Pose,
            str(self.get_parameter("external_goal_topic").value),
            lambda msg: self._store("external_goal", msg),
            10,
        )
        self.create_subscription(
            PoseStamped,
            str(self.get_parameter("controller_goal_topic").value),
            lambda msg: self._store("controller_goal", msg.pose),
            10,
        )
        self.create_subscription(
            JointState,
            str(self.get_parameter("joint_state_topic").value),
            lambda msg: self._store("joint_state", msg),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("command_topic").value),
            lambda msg: self._store("command_q", list(msg.data)),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("target_q_topic").value),
            lambda msg: self._store("target_q", list(msg.data)),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("target_metric_topic").value),
            lambda msg: self._store("target_metric", list(msg.data)),
            10,
        )
        self.create_subscription(
            Float64MultiArray,
            str(self.get_parameter("debug_state_topic").value),
            lambda msg: self._store("debug_state", list(msg.data)),
            10,
        )
        self.create_subscription(
            Pose,
            str(self.get_parameter("rmp_ee_pose_topic").value),
            lambda msg: self._store("rmp_ee_pose", msg),
            10,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter("obstacle_marker_topic").value),
            self._on_obstacles,
            10,
        )
        self.create_subscription(
            MarkerArray,
            str(self.get_parameter("repulsion_metric_marker_topic").value),
            self._on_repulsion_metrics,
            10,
        )
        self.create_subscription(
            Marker,
            str(self.get_parameter("tcp_accel_marker_topic").value),
            self._on_tcp_accel_marker,
            10,
        )
        for topic in self.range_topics:
            self.create_subscription(
                Range,
                topic,
                lambda msg, topic_name=topic: self._store_range(topic_name, msg),
                10,
            )

    def _store(self, key: str, data: Any) -> None:
        self.latest[key] = StampedValue(self._now_s(), data)

    def _store_range(self, topic: str, msg: Range) -> None:
        self.latest_ranges[topic] = StampedValue(self._now_s(), msg)

    def _on_obstacles(self, msg: MarkerArray) -> None:
        for marker in msg.markers:
            key = (marker.ns, marker.id)
            if marker.action == Marker.DELETEALL:
                self.active_obstacle_markers.clear()
            elif marker.action == Marker.DELETE:
                self.active_obstacle_markers.pop(key, None)
            elif marker.action == Marker.ADD:
                self.active_obstacle_markers[key] = marker
        self._store("obstacle_marker_stamp", True)

    def _on_repulsion_metrics(self, msg: MarkerArray) -> None:
        for marker in msg.markers:
            key = (marker.ns, marker.id)
            if marker.action == Marker.DELETEALL:
                self.repulsion_metric_markers.clear()
            elif marker.action == Marker.DELETE:
                self.repulsion_metric_markers.pop(key, None)
            elif marker.action == Marker.ADD:
                self.repulsion_metric_markers[key] = marker
        self._store("repulsion_metric_stamp", True)

    def _on_tcp_accel_marker(self, msg: Marker) -> None:
        if msg.action == Marker.DELETE or msg.action == Marker.DELETEALL:
            self.tcp_accel_marker = None
            return
        if msg.action == Marker.ADD:
            self.tcp_accel_marker = StampedValue(self._now_s(), msg)

    def _write_snapshot(self) -> None:
        row = self._snapshot_row()
        if self.csv_writer:
            self.csv_writer.writerow(row)
        if self.console_summary:
            self._log_console_summary(row)

    def _snapshot_row(self) -> Dict[str, Any]:
        row = {key: "" for key in self.csv_header}
        row["time_ros_s"] = f"{self._now_s():.6f}"
        row["time_wall_iso"] = datetime.now().isoformat(timespec="milliseconds")

        flag = self._latest_data("rmp_flag")
        if flag is not None:
            row["rmp_flag"] = flag
            row["rmp_active"] = int(flag == self.active_flag_value)
            row["rmp_flag_age_s"] = self._age_s("rmp_flag")

        self._fill_pose(row, "external_goal", "input_goal")
        self._fill_pose(row, "controller_goal", "controller_goal")
        self._fill_pose(row, "rmp_ee_pose", "rmp_ee")
        self._fill_joint_state(row)
        self._fill_vector(row, "target_q", "target_q", 6)
        self._fill_vector(row, "command_q", "command_q", 6)
        self._fill_target_metric(row)
        self._fill_debug_state(row)
        self._fill_marker_summaries(row)
        self._fill_range_values(row)
        return row

    def _fill_pose(self, row: Dict[str, Any], key: str, prefix: str) -> None:
        pose = self._latest_data(key)
        if pose is None:
            return
        row[f"{prefix}_x"] = self._fmt(pose.position.x)
        row[f"{prefix}_y"] = self._fmt(pose.position.y)
        row[f"{prefix}_z"] = self._fmt(pose.position.z)
        row[f"{prefix}_qx"] = self._fmt(pose.orientation.x)
        row[f"{prefix}_qy"] = self._fmt(pose.orientation.y)
        row[f"{prefix}_qz"] = self._fmt(pose.orientation.z)
        row[f"{prefix}_qw"] = self._fmt(pose.orientation.w)
        row[f"{prefix}_age_s"] = self._age_s(key)

    def _fill_joint_state(self, row: Dict[str, Any]) -> None:
        joint_state = self._latest_data("joint_state")
        if joint_state is None:
            return
        row["joint_state_age_s"] = self._age_s("joint_state")
        for index in range(6):
            if index < len(joint_state.name):
                row[f"joint_{index + 1}_name"] = joint_state.name[index]
            if index < len(joint_state.position):
                row[f"joint_{index + 1}_pos_rad"] = self._fmt(joint_state.position[index])
            if index < len(joint_state.velocity):
                row[f"joint_{index + 1}_vel_rad_s"] = self._fmt(joint_state.velocity[index])

    def _fill_vector(self, row: Dict[str, Any], key: str, prefix: str, count: int) -> None:
        values = self._latest_data(key)
        if values is None:
            return
        row[f"{prefix}_age_s"] = self._age_s(key)
        for index in range(count):
            if index < len(values):
                row[f"{prefix}_{index + 1}_rad"] = self._fmt(values[index])

    def _fill_target_metric(self, row: Dict[str, Any]) -> None:
        metric = self._latest_data("target_metric")
        if metric is None:
            return
        row["target_metric_age_s"] = self._age_s("target_metric")
        for index in range(min(9, len(metric))):
            row[f"target_metric_m{index // 3}{index % 3}"] = self._fmt(metric[index])
        if len(metric) >= 9:
            row["target_metric_trace"] = self._fmt(metric[0] + metric[4] + metric[8])
            row["target_metric_frobenius"] = self._fmt(math.sqrt(sum(value * value for value in metric[:9])))

    def _fill_debug_state(self, row: Dict[str, Any]) -> None:
        debug = self._latest_data("debug_state")
        if debug is None:
            return
        row["debug_state_age_s"] = self._age_s("debug_state")
        for index, field in enumerate(self.DEBUG_FIELDS):
            if index < len(debug):
                row[field] = self._fmt(debug[index])

    def _fill_marker_summaries(self, row: Dict[str, Any]) -> None:
        row["obstacle_marker_count"] = len(self.active_obstacle_markers)
        obstacle_frames = sorted(
            marker.text for marker in self.active_obstacle_markers.values() if marker.text
        )
        row["obstacle_marker_frames"] = "|".join(obstacle_frames)

        row["repulsion_metric_dot_count"] = len(self.repulsion_metric_markers)
        if self.repulsion_metric_markers:
            row["repulsion_metric_max_red"] = self._fmt(
                max(marker.color.r for marker in self.repulsion_metric_markers.values())
            )
            row["repulsion_metric_max_alpha"] = self._fmt(
                max(marker.color.a for marker in self.repulsion_metric_markers.values())
            )

        if self.tcp_accel_marker is None:
            return
        marker = self.tcp_accel_marker.data
        row["tcp_accel_marker_age_s"] = self._fmt(self._now_s() - self.tcp_accel_marker.stamp_s)
        start, end = self._marker_start_end(marker)
        if start is None or end is None:
            return
        direction = [end[index] - start[index] for index in range(3)]
        length = math.sqrt(sum(value * value for value in direction))
        row["tcp_accel_marker_length_m"] = self._fmt(length)
        if length > 1e-12:
            row["tcp_accel_dir_x"] = self._fmt(direction[0] / length)
            row["tcp_accel_dir_y"] = self._fmt(direction[1] / length)
            row["tcp_accel_dir_z"] = self._fmt(direction[2] / length)
        row["tcp_accel_start_x"] = self._fmt(start[0])
        row["tcp_accel_start_y"] = self._fmt(start[1])
        row["tcp_accel_start_z"] = self._fmt(start[2])
        row["tcp_accel_end_x"] = self._fmt(end[0])
        row["tcp_accel_end_y"] = self._fmt(end[1])
        row["tcp_accel_end_z"] = self._fmt(end[2])

    def _fill_range_values(self, row: Dict[str, Any]) -> None:
        active_topics = []
        for topic, label in zip(self.range_topics, self.range_labels):
            stamped = self.latest_ranges.get(topic)
            if stamped is None:
                continue
            msg = stamped.data
            raw = float(msg.range)
            usable = self._range_is_usable(msg)
            effective_m = max(raw * self.range_scale, self.minimum_hold_distance)
            triggered = usable and effective_m <= self.trigger_distance
            row[f"{label}_raw"] = self._fmt(raw)
            row[f"{label}_effective_m"] = self._fmt(effective_m)
            row[f"{label}_usable"] = int(usable)
            row[f"{label}_triggered"] = int(triggered)
            row[f"{label}_age_s"] = self._fmt(self._now_s() - stamped.stamp_s)
            if triggered:
                active_topics.append(label)
        row["proximity_triggered_count"] = len(active_topics)
        row["proximity_triggered_topics"] = "|".join(active_topics)

    def _range_is_usable(self, msg: Range) -> bool:
        if not math.isfinite(float(msg.range)):
            return False
        if msg.range < 0.0:
            return False
        return msg.range < (msg.max_range - self.valid_margin)

    def _marker_start_end(self, marker: Marker) -> Tuple[Optional[List[float]], Optional[List[float]]]:
        if len(marker.points) >= 2:
            start = [marker.points[0].x, marker.points[0].y, marker.points[0].z]
            end = [marker.points[1].x, marker.points[1].y, marker.points[1].z]
            return start, end
        position = marker.pose.position
        point = [position.x, position.y, position.z]
        return point, point

    def _make_csv_header(self) -> List[str]:
        header = [
            "time_ros_s",
            "time_wall_iso",
            "rmp_flag",
            "rmp_active",
            "rmp_flag_age_s",
        ]
        for prefix in ["input_goal", "controller_goal", "rmp_ee"]:
            header.extend(
                [
                    f"{prefix}_x",
                    f"{prefix}_y",
                    f"{prefix}_z",
                    f"{prefix}_qx",
                    f"{prefix}_qy",
                    f"{prefix}_qz",
                    f"{prefix}_qw",
                    f"{prefix}_age_s",
                ]
            )
        header.append("joint_state_age_s")
        for index in range(6):
            header.extend(
                [
                    f"joint_{index + 1}_name",
                    f"joint_{index + 1}_pos_rad",
                    f"joint_{index + 1}_vel_rad_s",
                ]
            )
        for prefix in ["target_q", "command_q"]:
            header.append(f"{prefix}_age_s")
            header.extend(f"{prefix}_{index + 1}_rad" for index in range(6))
        header.append("target_metric_age_s")
        header.extend(f"target_metric_m{row}{col}" for row in range(3) for col in range(3))
        header.extend(["target_metric_trace", "target_metric_frobenius", "debug_state_age_s"])
        header.extend(self.DEBUG_FIELDS)
        header.extend(
            [
                "obstacle_marker_count",
                "obstacle_marker_frames",
                "repulsion_metric_dot_count",
                "repulsion_metric_max_red",
                "repulsion_metric_max_alpha",
                "tcp_accel_marker_age_s",
                "tcp_accel_marker_length_m",
                "tcp_accel_dir_x",
                "tcp_accel_dir_y",
                "tcp_accel_dir_z",
                "tcp_accel_start_x",
                "tcp_accel_start_y",
                "tcp_accel_start_z",
                "tcp_accel_end_x",
                "tcp_accel_end_y",
                "tcp_accel_end_z",
                "proximity_triggered_count",
                "proximity_triggered_topics",
            ]
        )
        for label in self.range_labels:
            header.extend(
                [
                    f"{label}_raw",
                    f"{label}_effective_m",
                    f"{label}_usable",
                    f"{label}_triggered",
                    f"{label}_age_s",
                ]
            )
        return header

    def _latest_data(self, key: str) -> Any:
        stamped = self.latest.get(key)
        if stamped is None:
            return None
        return stamped.data

    def _age_s(self, key: str) -> str:
        stamped = self.latest.get(key)
        if stamped is None:
            return ""
        return self._fmt(self._now_s() - stamped.stamp_s)

    def _now_s(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    @staticmethod
    def _fmt(value: Any) -> str:
        try:
            numeric = float(value)
        except (TypeError, ValueError):
            return ""
        if not math.isfinite(numeric):
            return ""
        return f"{numeric:.9g}"

    @staticmethod
    def _topic_label(topic: str) -> str:
        return topic.strip("/").replace("/", "_").replace("-", "_")

    @staticmethod
    def _as_bool(value: Any) -> bool:
        if isinstance(value, bool):
            return value
        if isinstance(value, str):
            return value.lower() in ("1", "true", "yes", "on")
        return bool(value)

    def _log_console_summary(self, row: Dict[str, Any]) -> None:
        target_q = self._compact_vector(row, "target_q", 6)
        command_q = self._compact_vector(row, "command_q", 6)
        triggered = row.get("proximity_triggered_topics", "")
        if not triggered:
            triggered = "none"
        self.get_logger().info(
            "trace "
            f"active={row.get('rmp_active', '')} "
            f"goal_err={row.get('goal_error_m', '')}m "
            f"min_ext_clear={row.get('min_external_clearance_m', '')}m "
            f"sensors={triggered} "
            f"obs={row.get('obstacle_marker_count', '')} "
            f"metric_dots={row.get('repulsion_metric_dot_count', '')} "
            f"tcp_accel_len={row.get('tcp_accel_marker_length_m', '')}m "
            f"target_q=[{target_q}] "
            f"cmd=[{command_q}]"
        )

    @staticmethod
    def _compact_vector(row: Dict[str, Any], prefix: str, count: int) -> str:
        values = []
        for index in range(count):
            value = row.get(f"{prefix}_{index + 1}_rad", "")
            if value == "":
                continue
            try:
                values.append(f"{float(value):.3f}")
            except ValueError:
                continue
        return ", ".join(values)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = RmpflowTraceLogger()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

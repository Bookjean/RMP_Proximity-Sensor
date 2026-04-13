#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def _read_profile_scalar(profile_path: str, key: str, fallback: str) -> str:
    try:
        with open(profile_path, "r", encoding="utf-8") as handle:
            for raw_line in handle:
                stripped = raw_line.strip()
                if not stripped.startswith(f"{key}:"):
                    continue
                _, value = stripped.split(":", 1)
                value = value.split("#", 1)[0].strip()
                return value or fallback
    except OSError:
        pass
    return fallback


def generate_launch_description():
    rmp_pkg = get_package_share_directory("rb10_rmpflow_rviz")
    real_launch_path = os.path.join(rmp_pkg, "launch", "rb10_rmpflow_real.launch.py")
    profile_name = "stage3_target_position_with_cspace_p100_d15.yaml"
    profile_path = os.path.join(rmp_pkg, "config", "rmp_tuning_profiles", profile_name)
    default_target_rmp_accel_p_gain = _read_profile_scalar(
        profile_path,
        "target_rmp_accel_p_gain",
        "100.0",
    )
    default_target_rmp_accel_d_gain = _read_profile_scalar(
        profile_path,
        "target_rmp_accel_d_gain",
        "15.0",
    )

    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")
    record_data = LaunchConfiguration("record_data")
    recording_output_prefix = LaunchConfiguration("recording_output_prefix")
    publish_debug_joint_state_sources = LaunchConfiguration("publish_debug_joint_state_sources")
    measured_position_feedback_blend = LaunchConfiguration("measured_position_feedback_blend")
    measured_velocity_feedback_blend = LaunchConfiguration("measured_velocity_feedback_blend")
    max_joint_accel = LaunchConfiguration("max_joint_accel")
    target_rmp_accel_p_gain = LaunchConfiguration("target_rmp_accel_p_gain")
    target_rmp_accel_d_gain = LaunchConfiguration("target_rmp_accel_d_gain")
    command_guard_max_velocity_rad_s = LaunchConfiguration("command_guard_max_velocity_rad_s")
    enable_realtime = LaunchConfiguration("enable_realtime")
    realtime_priority = LaunchConfiguration("realtime_priority")
    lock_memory = LaunchConfiguration("lock_memory")
    enable_socket_realtime = LaunchConfiguration("enable_socket_realtime")
    socket_realtime_priority = LaunchConfiguration("socket_realtime_priority")

    return LaunchDescription([
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.111.50",
            description="Hostname or IP address of the RB10 controller.",
        ),
        DeclareLaunchArgument(
            "use_rviz",
            default_value="true",
            description="Start RViz.",
        ),
        DeclareLaunchArgument(
            "record_data",
            default_value="false",
            description="Record RMP topics to a dataset file.",
        ),
        DeclareLaunchArgument(
            "recording_output_prefix",
            default_value="target_position_with_cspace",
            description="Prefix for saved dataset filenames when record_data is enabled.",
        ),
        DeclareLaunchArgument(
            "publish_debug_joint_state_sources",
            default_value="true",
            description="Publish reference/measured joint-state debug topics.",
        ),
        DeclareLaunchArgument(
            "measured_position_feedback_blend",
            default_value="0.9",
            description=(
                "Blend factor for solver joint position feedback. "
                "Slightly below 1.0 keeps responsiveness when the robot servo lags."
            ),
        ),
        DeclareLaunchArgument(
            "measured_velocity_feedback_blend",
            default_value="0.04",
            description=(
                "Blend factor for solver joint velocity feedback. "
                "Lower values trust the previous commanded velocity more for faster response."
            ),
        ),
        DeclareLaunchArgument(
            "max_joint_accel",
            default_value="6.0",
            description="Maximum joint acceleration clamp for the dedicated launch.",
        ),
        DeclareLaunchArgument(
            "target_rmp_accel_p_gain",
            default_value=default_target_rmp_accel_p_gain,
            description="Target-attractor proportional acceleration gain for the dedicated launch.",
        ),
        DeclareLaunchArgument(
            "target_rmp_accel_d_gain",
            default_value=default_target_rmp_accel_d_gain,
            description="Target-attractor damping acceleration gain for the dedicated launch.",
        ),
        DeclareLaunchArgument(
            "command_guard_max_velocity_rad_s",
            default_value="1.745329252",
            description=(
                "Command-guard velocity limit for the dedicated launch. "
                "Default is 100 deg/s."
            ),
        ),
        DeclareLaunchArgument(
            "enable_realtime",
            default_value="false",
            description="Run the controller loop as SCHED_FIFO on PREEMPT_RT systems.",
        ),
        DeclareLaunchArgument(
            "realtime_priority",
            default_value="80",
            description="SCHED_FIFO priority used when enable_realtime is true.",
        ),
        DeclareLaunchArgument(
            "lock_memory",
            default_value="false",
            description="Call mlockall for the controller process when realtime is enabled.",
        ),
        DeclareLaunchArgument(
            "enable_socket_realtime",
            default_value="false",
            description="Run the direct RB10 socket send/receive threads as SCHED_FIFO as well.",
        ),
        DeclareLaunchArgument(
            "socket_realtime_priority",
            default_value="60",
            description="SCHED_FIFO priority for the direct RB10 socket threads when enabled.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch_path),
            launch_arguments={
                "robot_ip": robot_ip,
                "use_rviz": use_rviz,
                "use_direct_hardware_backend": "true",
                "startup_move_to_default_pose": "false",
                "rmp_tuning_stage": profile_name,
                "real_joint_state_source": "measured",
                "bridge_publish_rate": "500.0",
                "control_rate": "50.0",
                "servo_t2": "0.1",
                "servo_gain": "0.02",
                "servo_alpha": "0.2",
                "measured_position_feedback_blend": measured_position_feedback_blend,
                "measured_velocity_feedback_blend": measured_velocity_feedback_blend,
                "max_joint_accel": max_joint_accel,
                "estimate_velocity_in_controller": "true",
                "controller_velocity_filter_alpha": "0.10",
                "use_synced_input_velocity_filter": "false",
                "use_velocity_feedback_in_solver": "true",
                "target_rmp_accel_p_gain": target_rmp_accel_p_gain,
                "target_rmp_accel_d_gain": target_rmp_accel_d_gain,
                "command_guard_max_velocity_rad_s": command_guard_max_velocity_rad_s,
                "use_obstacles": "true",
                "use_proximity_bridge": "false",
                "use_tof_ray_visualizer": "false",
                "publish_visualization": "false",
                "publish_rmp_ee_pose": "false",
                "publish_debug_joint_state_sources": publish_debug_joint_state_sources,
                "record_data": record_data,
                "recording_output_prefix": recording_output_prefix,
                "enable_realtime": enable_realtime,
                "realtime_priority": realtime_priority,
                "lock_memory": lock_memory,
                "enable_socket_realtime": enable_socket_realtime,
                "socket_realtime_priority": socket_realtime_priority,
            }.items(),
        ),
    ])

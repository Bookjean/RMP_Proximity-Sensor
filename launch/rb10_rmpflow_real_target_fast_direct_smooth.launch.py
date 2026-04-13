#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    rmp_pkg = get_package_share_directory("rb10_rmpflow_rviz")
    real_launch_path = os.path.join(rmp_pkg, "launch", "rb10_rmpflow_real.launch.py")
    enable_realtime = LaunchConfiguration("enable_realtime")
    realtime_priority = LaunchConfiguration("realtime_priority")
    lock_memory = LaunchConfiguration("lock_memory")
    enable_socket_realtime = LaunchConfiguration("enable_socket_realtime")
    socket_realtime_priority = LaunchConfiguration("socket_realtime_priority")
    bridge_publish_rate = LaunchConfiguration("bridge_publish_rate")
    control_rate = LaunchConfiguration("control_rate")
    estimate_velocity_in_controller = LaunchConfiguration("estimate_velocity_in_controller")
    controller_velocity_filter_alpha = LaunchConfiguration("controller_velocity_filter_alpha")
    use_synced_input_velocity_filter = LaunchConfiguration("use_synced_input_velocity_filter")
    synced_input_velocity_filter_alpha = LaunchConfiguration("synced_input_velocity_filter_alpha")
    synced_input_velocity_filter_beta = LaunchConfiguration("synced_input_velocity_filter_beta")
    synced_input_velocity_filter_type = LaunchConfiguration("synced_input_velocity_filter_type")
    synced_input_velocity_ratio_tolerance = LaunchConfiguration("synced_input_velocity_ratio_tolerance")

    return LaunchDescription([
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
        DeclareLaunchArgument(
            "bridge_publish_rate",
            default_value="100.0",
            description="RB10 direct backend state request rate in Hz.",
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value="100.0",
            description="RMP control loop rate in Hz.",
        ),
        DeclareLaunchArgument(
            "estimate_velocity_in_controller",
            default_value="true",
            description="Use controller-side finite-difference velocity estimation.",
        ),
        DeclareLaunchArgument(
            "controller_velocity_filter_alpha",
            default_value="0.10",
            description="Low-pass blend for the controller-side velocity estimator.",
        ),
        DeclareLaunchArgument(
            "use_synced_input_velocity_filter",
            default_value="false",
            description="Build qd from high-rate state input using a control-rate-synced filter.",
        ),
        DeclareLaunchArgument(
            "synced_input_velocity_filter_alpha",
            default_value="0.35",
            description="Low-pass blend for the high-rate synced input velocity filter.",
        ),
        DeclareLaunchArgument(
            "synced_input_velocity_filter_beta",
            default_value="0.015",
            description="Beta parameter for the high-rate synced input alpha-beta filter.",
        ),
        DeclareLaunchArgument(
            "synced_input_velocity_filter_type",
            default_value="alphabeta",
            description="Synced input velocity filter type: alphabeta or moving_average.",
        ),
        DeclareLaunchArgument(
            "synced_input_velocity_ratio_tolerance",
            default_value="0.05",
            description="Tolerance for matching state-input/control-rate integer multiples.",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(real_launch_path),
            launch_arguments={
                "use_direct_hardware_backend": "true",
                "rmp_tuning_stage": "stage3_target_only.yaml",
                "real_joint_state_source": "measured",
                "bridge_publish_rate": bridge_publish_rate,
                "servo_t2": "0.035",
                "servo_gain": "0.045",
                "servo_alpha": "1.0",
                "measured_position_feedback_blend": "1.0",
                "measured_velocity_feedback_blend": "0.04",
                "estimate_velocity_in_controller": estimate_velocity_in_controller,
                "controller_velocity_filter_alpha": controller_velocity_filter_alpha,
                "use_synced_input_velocity_filter": use_synced_input_velocity_filter,
                "synced_input_velocity_filter_alpha": synced_input_velocity_filter_alpha,
                "synced_input_velocity_filter_beta": synced_input_velocity_filter_beta,
                "synced_input_velocity_filter_type": synced_input_velocity_filter_type,
                "synced_input_velocity_ratio_tolerance": synced_input_velocity_ratio_tolerance,
                "use_velocity_feedback_in_solver": "true",
                "control_rate": control_rate,
                "startup_move_to_default_pose": "false",
                "use_tof_ray_visualizer": "false",
                "publish_visualization": "false",
                "publish_rmp_ee_pose": "false",
                "publish_debug_joint_state_sources": "false",
                "enable_realtime": enable_realtime,
                "realtime_priority": realtime_priority,
                "lock_memory": lock_memory,
                "enable_socket_realtime": enable_socket_realtime,
                "socket_realtime_priority": socket_realtime_priority,
            }.items(),
        ),
    ])

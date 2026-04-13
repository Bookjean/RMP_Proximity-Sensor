#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    rmp_pkg = get_package_share_directory("rb10_rmpflow_rviz")

    urdf_path = os.path.join(rmp_pkg, "urdf", "rb10_1300e.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    params_path = os.path.join(rmp_pkg, "config", "params.yaml")
    rviz_config_path = os.path.join(rmp_pkg, "config", "rb10_rmpflow.rviz")

    robot_ip = LaunchConfiguration("robot_ip")
    use_rviz = LaunchConfiguration("use_rviz")
    cb_simulation = LaunchConfiguration("cb_simulation")
    use_interactive_goal = LaunchConfiguration("use_interactive_goal")
    use_obstacles = LaunchConfiguration("use_obstacles")
    use_proximity_bridge = LaunchConfiguration("use_proximity_bridge")
    record_data = LaunchConfiguration("record_data")
    auto_start_recording = LaunchConfiguration("auto_start_recording")
    recording_rate = LaunchConfiguration("recording_rate")
    recording_output_directory = LaunchConfiguration("recording_output_directory")
    recording_output_prefix = LaunchConfiguration("recording_output_prefix")
    servo_t1 = LaunchConfiguration("servo_t1")
    servo_t2 = LaunchConfiguration("servo_t2")
    servo_gain = LaunchConfiguration("servo_gain")
    servo_alpha = LaunchConfiguration("servo_alpha")
    startup_move_to_default_pose = LaunchConfiguration("startup_move_to_default_pose")
    startup_movej_speed = LaunchConfiguration("startup_movej_speed")
    startup_movej_accel = LaunchConfiguration("startup_movej_accel")
    startup_release_timeout_sec = LaunchConfiguration("startup_release_timeout_sec")
    stop_on_shutdown = LaunchConfiguration("stop_on_shutdown")
    shutdown_action = LaunchConfiguration("shutdown_action")
    use_velocity_filter = LaunchConfiguration("use_velocity_filter")
    velocity_filter_alpha = LaunchConfiguration("velocity_filter_alpha")
    velocity_filter_beta = LaunchConfiguration("velocity_filter_beta")
    joint_velocity_cap_max_velocity = LaunchConfiguration("joint_velocity_cap_max_velocity")
    max_joint_accel = LaunchConfiguration("max_joint_accel")
    measured_velocity_feedback_blend = LaunchConfiguration("measured_velocity_feedback_blend")
    target_rmp_accel_p_gain = LaunchConfiguration("target_rmp_accel_p_gain")
    target_rmp_accel_d_gain = LaunchConfiguration("target_rmp_accel_d_gain")
    bridge_publish_rate = LaunchConfiguration("bridge_publish_rate")
    control_rate = LaunchConfiguration("control_rate")
    rmp_tuning_stage = LaunchConfiguration("rmp_tuning_stage")
    record_joint_velocity = LaunchConfiguration("record_joint_velocity")
    joint_velocity_log_directory = LaunchConfiguration("joint_velocity_log_directory")
    joint_velocity_log_prefix = LaunchConfiguration("joint_velocity_log_prefix")
    tuning_params_path = PathJoinSubstitution(
        [rmp_pkg, "config", "rmp_tuning_profiles", rmp_tuning_stage]
    )

    api_bridge = Node(
        package="rb10_rmpflow_rviz",
        executable="rb10_direct_bridge",
        name="rb10_direct_bridge",
        output="screen",
        parameters=[{
            "robot_ip": robot_ip,
            "simulation_mode": cb_simulation,
            "command_topic": "/position_controllers/commands",
            "joint_state_topic": "/joint_states",
            "publish_rate": bridge_publish_rate,
            "servo_t1": servo_t1,
            "servo_t2": servo_t2,
            "servo_gain": servo_gain,
            "servo_alpha": servo_alpha,
            "startup_move_to_default_pose": startup_move_to_default_pose,
            "startup_movej_speed": startup_movej_speed,
            "startup_movej_accel": startup_movej_accel,
            "startup_release_timeout_sec": startup_release_timeout_sec,
            "stop_on_shutdown": stop_on_shutdown,
            "shutdown_action": shutdown_action,
            "use_velocity_filter": use_velocity_filter,
            "velocity_filter_alpha": velocity_filter_alpha,
            "velocity_filter_beta": velocity_filter_beta,
        }],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 100.0,
        }],
    )

    rmpflow_controller = Node(
        package="rb10_rmpflow_rviz",
        executable="rmpflow_controller",
        name="rmpflow_controller",
        output="screen",
        parameters=[
            params_path,
            {
                "backend_mode": "joint_command_topics",
                "joint_state_topic": "/joint_states",
                "position_command_topic": "/position_controllers/commands",
                "publish_joint_states": False,
                "joint_velocity_cap_max_velocity": joint_velocity_cap_max_velocity,
                "max_joint_accel": max_joint_accel,
                "measured_velocity_feedback_blend": measured_velocity_feedback_blend,
                "target_rmp_accel_p_gain": target_rmp_accel_p_gain,
                "target_rmp_accel_d_gain": target_rmp_accel_d_gain,
                "control_rate": control_rate,
            },
            tuning_params_path,
        ],
    )

    interactive_goal = Node(
        package="rb10_rmpflow_rviz",
        executable="interactive_goal",
        name="interactive_goal",
        output="screen",
        parameters=[params_path],
        condition=IfCondition(use_interactive_goal),
    )

    obstacle_manager = Node(
        package="rb10_rmpflow_rviz",
        executable="obstacle_manager",
        name="obstacle_manager",
        output="screen",
        condition=IfCondition(PythonExpression([
            '"', use_obstacles, '" == "true" and "',
            use_proximity_bridge, '" != "true"',
        ])),
        parameters=[params_path],
    )

    proximity_obstacle_bridge = Node(
        package="rb10_rmpflow_rviz",
        executable="proximity_obstacle_bridge",
        name="proximity_obstacle_bridge",
        output="screen",
        condition=IfCondition(use_proximity_bridge),
        parameters=[params_path],
    )

    tof_ray_visualizer = Node(
        package="rb10_rmpflow_rviz",
        executable="tof_ray_visualizer",
        name="tof_ray_visualizer",
        output="screen",
        parameters=[{
            "publish_rate": 20.0,
            "max_range": 0.2,
            "min_range": 0.02,
            "sensor_face_width": 0.25,
            "sensor_face_height": 0.25,
            "sensor_grid_resolution": 7,
            "edge_range_ratio": 0.6,
            "edge_falloff_power": 2.0,
        }],
    )

    data_recorder = Node(
        package="rb10_rmpflow_rviz",
        executable="rmp_data_recorder.py",
        name="rmp_data_recorder",
        output="screen",
        condition=IfCondition(record_data),
        parameters=[{
            "mode": "real",
            "auto_start": auto_start_recording,
            "recording_rate": recording_rate,
            "output_directory": recording_output_directory,
            "output_prefix": recording_output_prefix,
            "joint_state_topic": "/joint_states",
            "command_topic": "/position_controllers/commands",
            "goal_position_topic": "/goal_position",
            "goal_pose_topic": "/goal_pose",
            "obstacle_topic": "/obstacles",
        }],
    )


    joint_velocity_logger = Node(
        package="rb10_rmpflow_rviz",
        executable="joint_velocity_logger.py",
        name="joint_velocity_logger",
        output="screen",
        condition=IfCondition(record_joint_velocity),
        parameters=[{
            "joint_state_topic": "/joint_states",
            "output_directory": joint_velocity_log_directory,
            "output_prefix": joint_velocity_log_prefix,
            "flush_every": 1,
        }],
    )

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        condition=IfCondition(use_rviz),
        arguments=["-d", rviz_config_path] if os.path.exists(rviz_config_path) else [],
    )

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
            "cb_simulation",
            default_value="false",
            description="Use the RB controller box in simulation mode.",
        ),
        DeclareLaunchArgument(
            "use_interactive_goal",
            default_value="true",
            description="Start the interactive goal publisher.",
        ),
        DeclareLaunchArgument(
            "use_obstacles",
            default_value="true",
            description="Enable the interactive obstacle manager.",
        ),
        DeclareLaunchArgument(
            "use_proximity_bridge",
            default_value="false",
            description="Use external proximity topics to build obstacle markers.",
        ),
        DeclareLaunchArgument(
            "bridge_publish_rate",
            default_value="500.0",
            description="RB10 state receive/publish rate in Hz before filtering.",
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value="100.0",
            description="RMP control loop rate in Hz.",
        ),
        DeclareLaunchArgument(
            "rmp_tuning_stage",
            default_value="stage1_all_off.yaml",
            description=(
                "RMP tuning preset file in config/rmp_tuning_profiles. "
                "Use stage1_all_off.yaml, stage2_cspace_only.yaml, or stage3_target_only.yaml."
            ),
        ),
        DeclareLaunchArgument(
            "record_data",
            default_value="false",
            description="Record RMP topics to a dataset file.",
        ),
        DeclareLaunchArgument(
            "record_joint_velocity",
            default_value="false",
            description="Log /joint_states velocity values to a txt file.",
        ),
        DeclareLaunchArgument(
            "joint_velocity_log_directory",
            default_value=os.path.expanduser("~/ros2_ws/data/joint_velocity_logs"),
            description="Directory for joint velocity txt logs.",
        ),
        DeclareLaunchArgument(
            "joint_velocity_log_prefix",
            default_value="joint_velocity",
            description="Prefix for joint velocity txt logs.",
        ),
        DeclareLaunchArgument(
            "auto_start_recording",
            default_value="true",
            description="Start recording immediately when the recorder node launches.",
        ),
        DeclareLaunchArgument(
            "recording_rate",
            default_value="100.0",
            description="Dataset recording rate in Hz.",
        ),
        DeclareLaunchArgument(
            "recording_output_directory",
            default_value=os.path.expanduser("~/ros2_ws/data/rmp_datasets"),
            description="Directory for saved dataset files.",
        ),
        DeclareLaunchArgument(
            "recording_output_prefix",
            default_value="rmp_dataset",
            description="Prefix for saved dataset filenames.",
        ),
        DeclareLaunchArgument(
            "servo_t1",
            default_value="0.002",
            description="ServoJ interpolation time for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "servo_t2",
            default_value="0.03",
            description="ServoJ smoothing time for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "servo_gain",
            default_value="0.06",
            description="ServoJ gain for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "servo_alpha",
            default_value="0.45",
            description="ServoJ low-pass filter gain for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "startup_move_to_default_pose",
            default_value="true",
            description="Move to the configured default joint pose before enabling joint-state publication.",
        ),
        DeclareLaunchArgument(
            "startup_movej_speed",
            default_value="20.0",
            description="Joint speed for the startup move_j to the default pose.",
        ),
        DeclareLaunchArgument(
            "startup_movej_accel",
            default_value="20.0",
            description="Joint acceleration for the startup move_j to the default pose.",
        ),
        DeclareLaunchArgument(
            "startup_release_timeout_sec",
            default_value="12.0",
            description="How long to wait for the startup move_j to settle before releasing joint-state publication.",
        ),
        DeclareLaunchArgument(
            "stop_on_shutdown",
            default_value="true",
            description="Send a stop command to the RB10 when the launch exits.",
        ),
        DeclareLaunchArgument(
            "shutdown_action",
            default_value="halt",
            description="Stop action to send on shutdown: halt or pause.",
        ),
        DeclareLaunchArgument(
            "use_velocity_filter",
            default_value="false",
            description="Use the alpha-beta filter when estimating /joint_states velocity from position.",
        ),
        DeclareLaunchArgument(
            "velocity_filter_alpha",
            default_value="0.5",
            description="Alpha parameter for the RB10 joint velocity alpha-beta filter.",
        ),
        DeclareLaunchArgument(
            "velocity_filter_beta",
            default_value="0.015",
            description="Beta parameter for the RB10 joint velocity alpha-beta filter.",
        ),
        DeclareLaunchArgument(
            "joint_velocity_cap_max_velocity",
            default_value="1.0",
            description="RMP joint velocity cap used in the real-robot topic backend.",
        ),
        DeclareLaunchArgument(
            "max_joint_accel",
            default_value="12.0",
            description="RMP maximum joint acceleration used in the real-robot topic backend.",
        ),
        DeclareLaunchArgument(
            "measured_velocity_feedback_blend",
            default_value="0.2",
            description=(
                "Blend factor for solver joint velocity feedback. "
                "1.0 uses only measured velocity, 0.0 uses only previous command velocity."
            ),
        ),
        DeclareLaunchArgument(
            "target_rmp_accel_p_gain",
            default_value="300.0",
            description="RMP target acceleration proportional gain for the real-robot topic backend.",
        ),
        DeclareLaunchArgument(
            "target_rmp_accel_d_gain",
            default_value="100.0",
            description="RMP target acceleration damping gain for the real-robot topic backend.",
        ),
        api_bridge,
        robot_state_publisher,
        rmpflow_controller,
        data_recorder,
        joint_velocity_logger,
        interactive_goal,
        obstacle_manager,
        proximity_obstacle_bridge,
        tof_ray_visualizer,
        rviz,
    ])

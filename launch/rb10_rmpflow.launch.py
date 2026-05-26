#!/usr/bin/env python3
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _validate_params_file(context):
    params_path = LaunchConfiguration("params_file").perform(context)
    if not os.path.isfile(params_path):
        raise RuntimeError(
            f'Launch argument "params_file" must point to an existing YAML file: {params_path}'
        )
    return []


def generate_launch_description():
    rmp_pkg = get_package_share_directory("rb10_rmpflow_rviz")
    config_dir = os.path.join(rmp_pkg, "config")
    default_params_path = os.path.join(config_dir, "params.yaml")

    urdf_path = os.path.join(rmp_pkg, "urdf", "rb10_1300e.urdf")
    with open(urdf_path, "r") as f:
        robot_description = f.read()

    rviz_config_path = os.path.join(config_dir, "rb10_rmpflow.rviz")

    robot_ip = LaunchConfiguration("robot_ip")
    params_file = LaunchConfiguration("params_file")
    use_rviz = LaunchConfiguration("use_rviz")
    cb_simulation = LaunchConfiguration("cb_simulation")
    use_direct_hardware_backend = LaunchConfiguration("use_direct_hardware_backend")
    real_joint_state_source = LaunchConfiguration("real_joint_state_source")
    raw_joint_state_topic = LaunchConfiguration("raw_joint_state_topic")
    normalized_joint_state_topic = LaunchConfiguration("normalized_joint_state_topic")
    start_local_robot_state_publisher = LaunchConfiguration("start_local_robot_state_publisher")
    publish_debug_joint_state_sources = LaunchConfiguration("publish_debug_joint_state_sources")
    start_api_bridge = LaunchConfiguration("start_api_bridge")
    start_rmpflow_bridge = LaunchConfiguration("start_rmpflow_bridge")
    use_interactive_goal = LaunchConfiguration("use_interactive_goal")
    use_obstacles = LaunchConfiguration("use_obstacles")
    use_proximity_bridge = LaunchConfiguration("use_proximity_bridge")
    record_data = LaunchConfiguration("record_data")
    auto_start_recording = LaunchConfiguration("auto_start_recording")
    recording_rate = LaunchConfiguration("recording_rate")
    recording_output_directory = LaunchConfiguration("recording_output_directory")
    recording_output_prefix = LaunchConfiguration("recording_output_prefix")
    recorder_range_topics = [
        "/proximity_distance1",
        "/proximity_distance2",
        "/proximity_distance3",
        "/proximity_distance4",
        "/proximity_distance9",
        "/proximity_distance10",
        "/proximity_distance11",
        "/proximity_distance12",
    ]
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
    enable_realtime = LaunchConfiguration("enable_realtime")
    realtime_priority = LaunchConfiguration("realtime_priority")
    lock_memory = LaunchConfiguration("lock_memory")
    enable_socket_realtime = LaunchConfiguration("enable_socket_realtime")
    socket_realtime_priority = LaunchConfiguration("socket_realtime_priority")
    use_velocity_filter = LaunchConfiguration("use_velocity_filter")
    velocity_filter_alpha = LaunchConfiguration("velocity_filter_alpha")
    velocity_filter_beta = LaunchConfiguration("velocity_filter_beta")
    bridge_max_command_step_deg = LaunchConfiguration("bridge_max_command_step_deg")
    bridge_max_command_velocity_deg_s = LaunchConfiguration("bridge_max_command_velocity_deg_s")
    bridge_large_command_jump_warn_deg = LaunchConfiguration("bridge_large_command_jump_warn_deg")
    measured_position_feedback_blend = LaunchConfiguration("measured_position_feedback_blend")
    measured_velocity_feedback_blend = LaunchConfiguration("measured_velocity_feedback_blend")
    use_synced_input_velocity_filter = LaunchConfiguration("use_synced_input_velocity_filter")
    synced_input_velocity_filter_alpha = LaunchConfiguration("synced_input_velocity_filter_alpha")
    synced_input_velocity_filter_beta = LaunchConfiguration("synced_input_velocity_filter_beta")
    synced_input_velocity_ratio_tolerance = LaunchConfiguration("synced_input_velocity_ratio_tolerance")
    estimate_velocity_in_controller = LaunchConfiguration("estimate_velocity_in_controller")
    controller_velocity_filter_alpha = LaunchConfiguration("controller_velocity_filter_alpha")
    use_velocity_feedback_in_solver = LaunchConfiguration("use_velocity_feedback_in_solver")
    command_guard_max_step_rad = LaunchConfiguration("command_guard_max_step_rad")
    command_guard_max_velocity_rad_s = LaunchConfiguration("command_guard_max_velocity_rad_s")
    bridge_publish_rate = LaunchConfiguration("bridge_publish_rate")
    control_rate = LaunchConfiguration("control_rate")
    visualization_rate = LaunchConfiguration("visualization_rate")
    publish_visualization = LaunchConfiguration("publish_visualization")
    publish_rmp_ee_pose = LaunchConfiguration("publish_rmp_ee_pose")
    publish_target_metric = LaunchConfiguration("publish_target_metric")
    target_metric_topic = LaunchConfiguration("target_metric_topic")
    use_rmpflow_trace_logger = LaunchConfiguration("use_rmpflow_trace_logger")
    rmpflow_trace_log_rate = LaunchConfiguration("rmpflow_trace_log_rate")
    rmpflow_trace_log_directory = LaunchConfiguration("rmpflow_trace_log_directory")
    rmpflow_trace_console_summary = LaunchConfiguration("rmpflow_trace_console_summary")
    record_joint_velocity = LaunchConfiguration("record_joint_velocity")
    joint_velocity_log_directory = LaunchConfiguration("joint_velocity_log_directory")
    joint_velocity_log_prefix = LaunchConfiguration("joint_velocity_log_prefix")
    controller_parameters = {
        "robot_ip": robot_ip,
        "simulation_mode": cb_simulation,
        "real_joint_state_source": real_joint_state_source,
        "hardware_data_request_rate": bridge_publish_rate,
        "joint_state_topic": normalized_joint_state_topic,
        "rmp_flag_gate_enabled": True,
        "rmp_flag_topic": "/RMP_flag",
        "rmp_active_flag_value": 1,
        "position_command_topic": "/position_controllers/commands",
        "publish_target_q": True,
        "target_q_topic": "/target_q",
        "publish_target_metric": publish_target_metric,
        "target_metric_topic": target_metric_topic,
        "publish_joint_states": ParameterValue(
            PythonExpression([
                'True if "', use_direct_hardware_backend, '" == "true" else False'
            ]),
            value_type=bool,
        ),
        "backend_mode": ParameterValue(
            PythonExpression([
                '"rb10_direct_api" if "', use_direct_hardware_backend,
                '" == "true" else "joint_command_topics"'
            ]),
            value_type=str,
        ),
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
        "enable_realtime": enable_realtime,
        "realtime_priority": realtime_priority,
        "lock_memory": lock_memory,
        "enable_socket_realtime": enable_socket_realtime,
        "socket_realtime_priority": socket_realtime_priority,
        "measured_position_feedback_blend": measured_position_feedback_blend,
        "measured_velocity_feedback_blend": measured_velocity_feedback_blend,
        "use_synced_input_velocity_filter": use_synced_input_velocity_filter,
        "synced_input_velocity_filter_alpha": synced_input_velocity_filter_alpha,
        "synced_input_velocity_filter_beta": synced_input_velocity_filter_beta,
        "synced_input_velocity_filter_type": "alpha-beta",
        "synced_input_velocity_ratio_tolerance": synced_input_velocity_ratio_tolerance,
        "estimate_velocity_in_controller": estimate_velocity_in_controller,
        "controller_velocity_filter_alpha": controller_velocity_filter_alpha,
        "use_velocity_feedback_in_solver": use_velocity_feedback_in_solver,
        "publish_debug_joint_state_sources": publish_debug_joint_state_sources,
        "command_guard_max_step_rad": command_guard_max_step_rad,
        "command_guard_max_velocity_rad_s": command_guard_max_velocity_rad_s,
        "control_rate": control_rate,
        "visualization_rate": visualization_rate,
        "publish_visualization": publish_visualization,
        "publish_rmp_ee_pose": publish_rmp_ee_pose,
    }

    api_bridge = Node(
        package="rb10_rmpflow_rviz",
        executable="rb10_api_bridge.py",
        name="rb10_api_bridge",
        output="screen",
        condition=IfCondition(PythonExpression([
            '"', use_direct_hardware_backend, '" != "true" and "',
            start_api_bridge, '" == "true"',
        ])),
        parameters=[{
            "robot_ip": robot_ip,
            "simulation_mode": cb_simulation,
            "command_topic": "/position_controllers/commands",
            "joint_state_topic": "/joint_states",
            "real_joint_state_source": real_joint_state_source,
            "publish_rate": bridge_publish_rate,
            "servo_t1": servo_t1,
            "servo_t2": servo_t2,
            "servo_gain": servo_gain,
            "servo_alpha": servo_alpha,
            "max_command_step_deg": bridge_max_command_step_deg,
            "max_command_velocity_deg_s": bridge_max_command_velocity_deg_s,
            "large_command_jump_warn_deg": bridge_large_command_jump_warn_deg,
            "stop_on_shutdown": stop_on_shutdown,
            "shutdown_action": shutdown_action,
        }],
    )

    joint_state_adapter = Node(
        package="rb10_rmpflow_rviz",
        executable="joint_state_adapter",
        name="joint_state_adapter",
        output="screen",
        parameters=[{
            "input_topic": raw_joint_state_topic,
            "output_topic": normalized_joint_state_topic,
            "source_joint_names": ["joint_1", "joint_2", "joint_3", "joint_4", "joint_5", "joint_6"],
            "target_joint_names": ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"],
        }],
    )

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        condition=IfCondition(start_local_robot_state_publisher),
        parameters=[{
            "robot_description": robot_description,
            "publish_frequency": 100.0,
        }],
        remappings=[
            ("/joint_states", normalized_joint_state_topic),
        ],
    )

    rmpflow_controller = Node(
        package="rb10_rmpflow_rviz",
        executable="rmpflow_controller",
        name="rmpflow_controller",
        output="screen",
        parameters=[
            params_file,
            controller_parameters,
        ],
    )

    rmpflow_bridge = Node(
        package="rb10_rmpflow_rviz",
        executable="rmpflow_bridge",
        name="rmpflow_bridge",
        output="screen",
        condition=IfCondition(start_rmpflow_bridge),
        parameters=[{
            "flag_topic": "/RMP_flag",
            "goal_topic": "/RMP_goal",
            "controller_goal_topic": "/goal_pose",
            "controller_command_topic": "/position_controllers/commands",
            "target_q_topic": "/target_q",
            "forward_target_q": False,
            "goal_frame_id": "base_link",
            "active_flag_value": 1,
            "command_forward_delay_ms": 100,
        }],
    )

    interactive_goal = Node(
        package="rb10_rmpflow_rviz",
        executable="interactive_goal",
        name="interactive_goal",
        output="screen",
        parameters=[
            params_file,
            {
                "joint_state_topic": normalized_joint_state_topic,
                "lock_orientation_to_tcp": False,
            },
        ],
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
        parameters=[params_file],
    )

    proximity_obstacle_bridge = Node(
        package="rb10_rmpflow_rviz",
        executable="proximity_obstacle_bridge",
        name="proximity_obstacle_bridge",
        output="screen",
        condition=IfCondition(use_proximity_bridge),
        parameters=[
            params_file,
            {
                "rmp_flag_gate_enabled": True,
                "rmp_flag_topic": "/RMP_flag",
                "rmp_active_flag_value": 1,
            },
        ],
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
            "joint_state_topic": normalized_joint_state_topic,
            "command_topic": "/position_controllers/commands",
            "goal_pose_topic": "/goal_pose",
            "obstacle_topic": "/obstacles",
            "range_topics": recorder_range_topics,
            "max_obstacles": len(recorder_range_topics),
            "reference_joint_state_topic": "/rb10/reference_joint_states",
            "measured_joint_state_topic": "/rb10/measured_joint_states",
            "tracking_error_topic": "/rb10/joint_tracking_error_deg",
        }],
    )


    joint_velocity_logger = Node(
        package="rb10_rmpflow_rviz",
        executable="joint_velocity_logger.py",
        name="joint_velocity_logger",
        output="screen",
        condition=IfCondition(record_joint_velocity),
        parameters=[{
            "joint_state_topic": normalized_joint_state_topic,
            "output_directory": joint_velocity_log_directory,
            "output_prefix": joint_velocity_log_prefix,
            "flush_every": 1,
        }],
    )

    rmpflow_trace_logger = Node(
        package="rb10_rmpflow_rviz",
        executable="rmpflow_trace_logger.py",
        name="rmpflow_trace_logger",
        output="screen",
        condition=IfCondition(use_rmpflow_trace_logger),
        parameters=[{
            "log_rate_hz": rmpflow_trace_log_rate,
            "console_summary": rmpflow_trace_console_summary,
            "output_directory": rmpflow_trace_log_directory,
            "rmp_flag_topic": "/RMP_flag",
            "external_goal_topic": "/RMP_goal",
            "controller_goal_topic": "/goal_pose",
            "joint_state_topic": normalized_joint_state_topic,
            "command_topic": "/position_controllers/commands",
            "target_q_topic": "/target_q",
            "target_metric_topic": target_metric_topic,
            "debug_state_topic": "/rmp_debug_state",
            "rmp_ee_pose_topic": "/rmp_ee_pose",
            "obstacle_marker_topic": "/obstacles",
            "repulsion_metric_marker_topic": "/repulsion_metric_markers",
            "tcp_accel_marker_topic": "/tcp_accel_marker",
            "range_scale": 0.001,
            "minimum_hold_distance": 0.05,
            "trigger_distance": 0.29,
            "range_topics": [
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
            "params_file",
            default_value=default_params_path,
            description="Absolute path to the controller params YAML.",
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
            "use_direct_hardware_backend",
            default_value="false",
            description=(
                "Bypass the ROS command bridge and let rmpflow_controller send RB10 API servo "
                "commands directly while keeping RViz/debug publication alive."
            ),
        ),
        DeclareLaunchArgument(
            "real_joint_state_source",
            default_value="measured",
            description=(
                "Joint position source for /joint_states in real mode: "
                "measured uses encoder angle, reference uses controller reference angle."
            ),
        ),
        DeclareLaunchArgument(
            "raw_joint_state_topic",
            default_value="/joint_states",
            description="Incoming raw joint_states topic to normalize for RViz and the controller.",
        ),
        DeclareLaunchArgument(
            "normalized_joint_state_topic",
            default_value="/rb10/joint_states",
            description="Internal normalized joint_states topic with base~wrist3 naming.",
        ),
        DeclareLaunchArgument(
            "start_local_robot_state_publisher",
            default_value="true",
            description=(
                "Start the local robot_state_publisher. Keep false on the compute PC when another "
                "machine already publishes /robot_description, /tf, and /tf_static."
            ),
        ),
        DeclareLaunchArgument(
            "publish_debug_joint_state_sources",
            default_value="true",
            description=(
                "Publish reference/measured joint-state debug topics and their tracking error "
                "for vibration-source diagnosis."
            ),
        ),
        DeclareLaunchArgument(
            "start_api_bridge",
            default_value="false",
            description=(
                "Start the local RB10 API bridge when using the joint-command-topics backend. "
                "Set false on the compute PC when another machine will forward q commands to the robot."
            ),
        ),
        DeclareLaunchArgument(
            "start_rmpflow_bridge",
            default_value="true",
            description=(
                "Start the topic bridge that gates RMP execution with /RMP_flag, forwards "
                "/RMP_goal to the controller, and republishes /target_q."
            ),
        ),
        DeclareLaunchArgument(
            "use_interactive_goal",
            default_value="false",
            description="Start the interactive goal publisher.",
        ),
        DeclareLaunchArgument(
            "use_obstacles",
            default_value="true",
            description="Enable the interactive obstacle manager.",
        ),
        DeclareLaunchArgument(
            "use_proximity_bridge",
            default_value="true",
            description="Use external proximity topics to build obstacle markers.",
        ),
        DeclareLaunchArgument(
            "bridge_publish_rate",
            default_value="500.0",
            description="RB10 state receive/publish rate in Hz before filtering.",
        ),
        DeclareLaunchArgument(
            "control_rate",
            default_value="50.0",
            description="RMP control loop rate in Hz.",
        ),
        DeclareLaunchArgument(
            "visualization_rate",
            default_value="20.0",
            description="Controller-side RViz/debug publication rate in Hz.",
        ),
        DeclareLaunchArgument(
            "publish_visualization",
            default_value="true",
            description=(
                "Publish controller visualization topics such as goal markers, control points, "
                "body obstacle markers, end-effector pose, and debug state."
            ),
        ),
        DeclareLaunchArgument(
            "publish_rmp_ee_pose",
            default_value="true",
            description="Publish the controller's internal RMP end-effector pose topic.",
        ),
        DeclareLaunchArgument(
            "publish_target_metric",
            default_value="true",
            description="Publish the controller target RMP 3x3 leaf metric as Float64MultiArray.",
        ),
        DeclareLaunchArgument(
            "target_metric_topic",
            default_value="/target_metric",
            description="Topic name for the target RMP metric.",
        ),
        DeclareLaunchArgument(
            "use_rmpflow_trace_logger",
            default_value="true",
            description="Start a trace logger that records RMP inputs, sensor ranges, metrics, and commands.",
        ),
        DeclareLaunchArgument(
            "rmpflow_trace_log_rate",
            default_value="1.0",
            description="Trace logger snapshot rate in Hz.",
        ),
        DeclareLaunchArgument(
            "rmpflow_trace_log_directory",
            default_value=os.path.expanduser("~/ros2_ws/log/rmpflow_trace"),
            description="Directory for RMPFlow trace CSV logs.",
        ),
        DeclareLaunchArgument(
            "rmpflow_trace_console_summary",
            default_value="true",
            description="Print one-line trace summaries to the launch terminal.",
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
            default_value="false",
            description="Keep recorder idle until the experiment runner starts it.",
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
            default_value="target_only_tuning",
            description="Prefix for saved dataset filenames.",
        ),
        DeclareLaunchArgument(
            "servo_t1",
            default_value="0.002",
            description="ServoJ interpolation time for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "servo_t2",
            default_value="0.1",
            description="ServoJ smoothing time for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "servo_gain",
            default_value="0.02",
            description="ServoJ gain for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "servo_alpha",
            default_value="0.4",
            description="ServoJ low-pass filter gain for the internal RB10 bridge.",
        ),
        DeclareLaunchArgument(
            "bridge_max_command_step_deg",
            default_value="1.0",
            description="Hard per-cycle joint-step limit for the Python RB10 bridge to avoid hardware safety trips.",
        ),
        DeclareLaunchArgument(
            "bridge_max_command_velocity_deg_s",
            default_value="100.0",
            description="Hard joint-velocity limit used by the Python RB10 bridge command guard.",
        ),
        DeclareLaunchArgument(
            "bridge_large_command_jump_warn_deg",
            default_value="2.0",
            description="Warn when the requested bridge command jumps more than this far from the current joint reference.",
        ),
        DeclareLaunchArgument(
            "startup_move_to_default_pose",
            default_value="false",
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
            "measured_position_feedback_blend",
            default_value="1.0",
            description=(
                "Blend factor for solver joint position feedback. "
                "1.0 uses only measured/reference joint position, 0.0 uses only the previous "
                "commanded joint position."
            ),
        ),
        DeclareLaunchArgument(
            "measured_velocity_feedback_blend",
            default_value="0.35",
            description=(
                "Blend factor for solver joint velocity feedback. "
                "1.0 uses only measured velocity, 0.0 uses only previous command velocity."
            ),
        ),
        DeclareLaunchArgument(
            "use_synced_input_velocity_filter",
            default_value="false",
            description=(
                "Use the high-rate direct-backend state input to build a control-rate-synchronized "
                "joint velocity estimate before feeding qd to the solver. "
                "For direct backend use, pair this with estimate_velocity_in_controller:=false."
            ),
        ),
        DeclareLaunchArgument(
            "synced_input_velocity_filter_alpha",
            default_value="0.35",
            description=(
                "Low-pass blend for the synchronized high-rate input velocity filter. "
                "1.0 keeps the windowed raw velocity, smaller values smooth it."
            ),
        ),
        DeclareLaunchArgument(
            "synced_input_velocity_filter_beta",
            default_value="0.02",
            description=(
                "Beta parameter for the fixed alpha-beta synchronized high-rate input velocity filter."
            ),
        ),
        DeclareLaunchArgument(
            "synced_input_velocity_ratio_tolerance",
            default_value="0.05",
            description=(
                "Tolerance used when matching hardware_data_request_rate/control_rate to an integer "
                "sample multiple for the synchronized input velocity filter."
            ),
        ),
        DeclareLaunchArgument(
            "estimate_velocity_in_controller",
            default_value="false",
            description=(
                "Estimate joint velocity inside the 100 Hz controller loop from joint position "
                "instead of consuming the bridge-published /joint_states velocity directly."
            ),
        ),
        DeclareLaunchArgument(
            "controller_velocity_filter_alpha",
            default_value="0.25",
            description=(
                "Low-pass blend used by the controller-side joint velocity estimator. "
                "1.0 keeps the raw 100 Hz finite-difference velocity, smaller values smooth it."
            ),
        ),
        DeclareLaunchArgument(
            "use_velocity_feedback_in_solver",
            default_value="true",
            description=(
                "Pass joint velocity to the RMP solver. Set false to test position-only feedback "
                "by zeroing qd before the solve step."
            ),
        ),
        DeclareLaunchArgument(
            "command_guard_max_step_rad",
            default_value="0.00436332313",
            description=(
                "Maximum per-cycle joint position correction applied by the controller command "
                "guard, in radians."
            ),
        ),
        DeclareLaunchArgument(
            "command_guard_max_velocity_rad_s",
            default_value="2.5",
            description=(
                "Maximum joint velocity allowed by the controller command guard, in radians per "
                "second."
            ),
        ),
        OpaqueFunction(function=_validate_params_file),
        api_bridge,
        joint_state_adapter,
        robot_state_publisher,
        rmpflow_controller,
        rmpflow_bridge,
        data_recorder,
        joint_velocity_logger,
        rmpflow_trace_logger,
        interactive_goal,
        obstacle_manager,
        proximity_obstacle_bridge,
        rviz,
    ])

#!/usr/bin/env python3
import math
import os
import signal
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray


JOINT_NAMES = ["base", "shoulder", "elbow", "wrist1", "wrist2", "wrist3"]


def _as_bool(value):
    if isinstance(value, bool):
        return value
    if isinstance(value, str):
        return value.strip().lower() in {"1", "true", "yes", "on"}
    return bool(value)



class RB10APIBridge(Node):
    def __init__(self):
        super().__init__('rb10_api_bridge')

        self.declare_parameter('robot_ip', '192.168.111.50')
        self.declare_parameter('simulation_mode', True)
        self.declare_parameter('command_topic', '/position_controllers/commands')
        self.declare_parameter('joint_state_topic', '/joint_states')
        self.declare_parameter('real_joint_state_source', 'measured')
        self.declare_parameter('publish_rate', 100.0)
        self.declare_parameter('servo_t1', 0.002)
        self.declare_parameter('servo_t2', 0.1)
        self.declare_parameter('servo_gain', 0.02)
        self.declare_parameter('servo_alpha', 0.2)
        self.declare_parameter('max_command_step_deg', 0.25)
        self.declare_parameter('max_command_velocity_deg_s', 25.0)
        self.declare_parameter('large_command_jump_warn_deg', 2.0)
        self.declare_parameter('stop_on_shutdown', True)
        self.declare_parameter('shutdown_action', 'halt')

        self.robot_ip = self.get_parameter('robot_ip').value
        self.simulation_mode = _as_bool(self.get_parameter('simulation_mode').value)
        self.command_topic = self.get_parameter('command_topic').value
        self.joint_state_topic = self.get_parameter('joint_state_topic').value
        self.real_joint_state_source = str(
            self.get_parameter('real_joint_state_source').value
        ).strip().lower()
        self.publish_rate = float(self.get_parameter('publish_rate').value)
        self.servo_t1 = float(self.get_parameter('servo_t1').value)
        self.servo_t2 = float(self.get_parameter('servo_t2').value)
        self.servo_gain = float(self.get_parameter('servo_gain').value)
        self.servo_alpha = float(self.get_parameter('servo_alpha').value)
        self.max_command_step_deg = max(float(self.get_parameter('max_command_step_deg').value), 0.01)
        self.max_command_velocity_deg_s = max(
            float(self.get_parameter('max_command_velocity_deg_s').value),
            self.max_command_step_deg * max(self.publish_rate, 1.0),
        )
        self.large_command_jump_warn_deg = max(
            float(self.get_parameter('large_command_jump_warn_deg').value),
            self.max_command_step_deg,
        )
        self.stop_on_shutdown = _as_bool(self.get_parameter('stop_on_shutdown').value)
        self.shutdown_action = str(self.get_parameter('shutdown_action').value).strip().lower()

        self.api = self._load_api()
        self.connected = False
        self.prev_joint_positions = None
        self.prev_timestamp = None
        self._last_error = 0.0
        self._last_conn_warn = 0.0
        self._last_guard_warn = 0.0
        self._stop_requested = False
        self._signal_handled = False

        self._connect_robot()

        self.joint_state_pub = self.create_publisher(JointState, self.joint_state_topic, 10)

        self.create_subscription(Float64MultiArray, self.command_topic, self._command_cb, 10)
        self.create_timer(1.0 / max(self.publish_rate, 1.0), self._publish_joint_states)
        signal.signal(signal.SIGINT, self._handle_signal)
        signal.signal(signal.SIGTERM, self._handle_signal)

        mode_label = 'SIMULATION' if self.simulation_mode else 'REAL'
        self.get_logger().info(
            f'RB10 api bridge connected to {self.robot_ip} in {mode_label} mode '
            f'({self.command_topic} -> {self.joint_state_topic})'
        )

    def _load_api(self):
        script_dir = os.path.dirname(os.path.abspath(__file__))
        if script_dir not in sys.path:
            sys.path.insert(0, script_dir)
        try:
            from api.cobot import (
                ToCB,
                DisConnectToCB,
                CobotInit,
                SetProgramMode,
                PG_MODE,
                IsDataSockConnect,
                IsCommandSockConnect,
                GetCurrentSplitedJoint,
                GetCurrentMeasuredSplitedJoint,
                IsPause,
                SendCOMMAND,
                CMD_TYPE,
                MotionPause,
                MotionHalt,
                MoveJB_Clear,
                MovePB_Clear,
                MoveITPL_Clear,
            )
        except Exception as exc:
            raise RuntimeError(
                'Bundled api.cobot import failed from the RMP package. '
                f'Expected sibling api module next to rb10_api_bridge.py. error: {exc}'
            )
        return {
            'ToCB': ToCB,
            'DisConnectToCB': DisConnectToCB,
            'CobotInit': CobotInit,
            'SetProgramMode': SetProgramMode,
            'PG_MODE': PG_MODE,
            'IsDataSockConnect': IsDataSockConnect,
            'IsCommandSockConnect': IsCommandSockConnect,
            'GetCurrentSplitedJoint': GetCurrentSplitedJoint,
            'GetCurrentMeasuredSplitedJoint': GetCurrentMeasuredSplitedJoint,
            'IsPause': IsPause,
            'SendCOMMAND': SendCOMMAND,
            'CMD_TYPE': CMD_TYPE,
            'MotionPause': MotionPause,
            'MotionHalt': MotionHalt,
            'MoveJB_Clear': MoveJB_Clear,
            'MovePB_Clear': MovePB_Clear,
            'MoveITPL_Clear': MoveITPL_Clear,
        }

    def _connect_robot(self):
        self.get_logger().info(f'Connecting to RB10 at {self.robot_ip}')
        try:
            self.api['ToCB'](self.robot_ip)
            self.api['CobotInit']()
            mode = self.api['PG_MODE'].SIMULATION if self.simulation_mode else self.api['PG_MODE'].REAL
            self.api['SetProgramMode'](mode)
            self.connected = True
        except Exception as exc:
            raise RuntimeError(f'Failed to connect via api.cobot: {exc}')

    def _get_current_joint_source(self):
        if self.real_joint_state_source == 'measured':
            return self.api['GetCurrentMeasuredSplitedJoint']()
        return self.api['GetCurrentSplitedJoint']()

    def _command_cb(self, msg):
        if not self.connected:
            return
        if len(msg.data) < 6:
            self.get_logger().warn(f'Received {len(msg.data)} joint commands, expected 6')
            return
        try:
            try:
                if self.api['IsPause']():
                    return
            except Exception:
                pass
            if not self.api['IsDataSockConnect']() or not self.api['IsCommandSockConnect']():
                if time.time() - self._last_conn_warn > 5.0:
                    self.get_logger().warn('RB10 sockets not connected')
                    self._last_conn_warn = time.time()
                return
            current_joints_deg = self._get_current_joint_source()
            if not current_joints_deg or len(current_joints_deg) < 6:
                return
            if any(j is None for j in current_joints_deg[:6]):
                return

            joints_deg = [math.degrees(float(j)) for j in msg.data[:6]]
            max_step_deg = max(
                self.max_command_step_deg,
                self.max_command_velocity_deg_s / max(self.publish_rate, 1.0),
            )
            clamped = False
            severe_jump = False
            safe_joints_deg = []
            for desired_deg, current_deg in zip(joints_deg, current_joints_deg[:6]):
                delta_deg = desired_deg - float(current_deg)
                if abs(delta_deg) > self.large_command_jump_warn_deg:
                    severe_jump = True
                limited_deg = float(current_deg) + max(-max_step_deg, min(max_step_deg, delta_deg))
                if abs(limited_deg - desired_deg) > 1e-9:
                    clamped = True
                safe_joints_deg.append(limited_deg)

            if (clamped or severe_jump) and time.time() - self._last_guard_warn > 1.0:
                self.get_logger().warn(
                    'Servo command guard limited the per-cycle joint step to '
                    f'{max_step_deg:.3f} deg to avoid triggering robot safety'
                )
                self._last_guard_warn = time.time()

            cmd = (
                'move_servo_j(jnt[' + ','.join(f'{j:.3f}' for j in safe_joints_deg) + '],' +
                f'{self.servo_t1},{self.servo_t2},{self.servo_gain},{self.servo_alpha})'
            )
            self.api['SendCOMMAND'](cmd, self.api['CMD_TYPE'].MOVE)
        except Exception as exc:
            if time.time() - self._last_error > 1.0:
                self.get_logger().error(f'Error sending command to RB10: {exc}')
                self._last_error = time.time()

    def _stop_robot_motion(self):
        if self._stop_requested or not self.connected or not self.stop_on_shutdown:
            return
        self._stop_requested = True

        action = self.shutdown_action if self.shutdown_action in {'halt', 'pause'} else 'halt'
        first_name = 'MotionHalt' if action == 'halt' else 'MotionPause'
        second_name = 'MotionPause' if action == 'halt' else 'MotionHalt'

        first_error = None
        try:
            self.api[first_name]()
            self.get_logger().info(f'Sent {first_name}() to RB10 during shutdown')
        except Exception as exc:
            first_error = exc

        time.sleep(0.05)
        self._clear_motion_buffers()
        time.sleep(0.05)

        try:
            self.api[first_name]()
            self.get_logger().info(f'Sent {first_name}() to RB10 during shutdown (post-clear)')
            return
        except Exception as exc:
            self.get_logger().warn(f'{first_name}() retry failed during shutdown: {exc}')

        try:
            self.api[second_name]()
            self.get_logger().info(f'Sent {second_name}() to RB10 during shutdown')
        except Exception as exc:
            self.get_logger().warn(
                f'Failed to stop RB10 motion during shutdown: {first_name}={first_error}, {second_name}={exc}'
            )

    def _clear_motion_buffers(self):
        for clear_name in ('MoveITPL_Clear', 'MovePB_Clear', 'MoveJB_Clear'):
            try:
                self.api[clear_name]()
                self.get_logger().info(f'Sent {clear_name}() to RB10 during shutdown')
            except Exception as exc:
                self.get_logger().warn(f'{clear_name}() failed during shutdown: {exc}')

    def _handle_signal(self, signum, frame):
        if self._signal_handled:
            return
        self._signal_handled = True

        try:
            self._stop_robot_motion()
        except Exception as exc:
            self.get_logger().warn(f'RB10 signal-stop sequence failed: {exc}')

        self.connected = False
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        except Exception:
            pass

        try:
            if rclpy.ok():
                rclpy.shutdown()
        except Exception:
            pass

    def _publish_joint_states(self):
        if not self.connected:
            return
        try:
            if not self.api['IsDataSockConnect']():
                return
            jnts = self._get_current_joint_source()
            if not jnts or len(jnts) < 6:
                return
            if any(j is None for j in jnts[:6]):
                return
            current_timestamp = self.get_clock().now()
            positions = [math.radians(float(j)) for j in jnts[:6]]
            velocities = [0.0] * 6
            if self.prev_joint_positions is not None and self.prev_timestamp is not None:
                dt = (current_timestamp - self.prev_timestamp).nanoseconds / 1e9
                if 0.001 < dt < 1.0:
                    velocities = [
                        (positions[i] - self.prev_joint_positions[i]) / dt
                        for i in range(6)
                    ]
            self.prev_joint_positions = positions.copy()
            self.prev_timestamp = current_timestamp
            msg = JointState()
            msg.header.stamp = current_timestamp.to_msg()
            msg.name = JOINT_NAMES
            msg.position = positions
            msg.velocity = velocities
            self.joint_state_pub.publish(msg)
        except Exception as exc:
            if time.time() - self._last_error > 1.0:
                self.get_logger().error(f'Error publishing joint state: {exc}')
                self._last_error = time.time()

    def destroy_node(self):
        try:
            signal.signal(signal.SIGINT, signal.SIG_IGN)
            signal.signal(signal.SIGTERM, signal.SIG_IGN)
        except Exception:
            pass

        if self.connected:
            try:
                self._stop_robot_motion()
            except Exception as exc:
                self.get_logger().warn(f'RB10 shutdown stop sequence failed: {exc}')
            try:
                self.api['DisConnectToCB']()
            except BaseException:
                pass
            self.connected = False
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = RB10APIBridge()
        rclpy.spin(node)
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

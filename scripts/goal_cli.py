#!/usr/bin/env python3

import argparse
import sys
import time

import rclpy
from geometry_msgs.msg import Point, PoseStamped
from rclpy.node import Node


class GoalCliNode(Node):
    def __init__(self, point_topic: str, pose_topic: str) -> None:
        super().__init__("goal_cli")
        self.point_pub = self.create_publisher(Point, point_topic, 10)
        self.pose_pub = self.create_publisher(PoseStamped, pose_topic, 10)

    def wait_for_subscribers(self, use_pose: bool, timeout_sec: float) -> bool:
        publisher = self.pose_pub if use_pose else self.point_pub
        deadline = time.monotonic() + max(timeout_sec, 0.0)
        while time.monotonic() < deadline and rclpy.ok():
            if publisher.get_subscription_count() > 0:
                return True
            rclpy.spin_once(self, timeout_sec=0.1)
        return publisher.get_subscription_count() > 0


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Publish a terminal-entered goal for the RB10 RMPflow stack."
    )
    parser.add_argument("x", type=float, help="Goal X in base_link")
    parser.add_argument("y", type=float, help="Goal Y in base_link")
    parser.add_argument("z", type=float, help="Goal Z in base_link")
    parser.add_argument("--qx", type=float, help="Optional goal orientation x")
    parser.add_argument("--qy", type=float, help="Optional goal orientation y")
    parser.add_argument("--qz", type=float, help="Optional goal orientation z")
    parser.add_argument("--qw", type=float, help="Optional goal orientation w")
    parser.add_argument(
        "--frame-id",
        default="base_link",
        help="Frame id used when publishing a pose goal",
    )
    parser.add_argument(
        "--point-topic",
        default="/goal_position_cmd",
        help="Point command topic consumed by interactive_goal",
    )
    parser.add_argument(
        "--pose-topic",
        default="/goal_pose_cmd",
        help="Pose command topic consumed by interactive_goal",
    )
    parser.add_argument(
        "--repeat",
        type=int,
        default=10,
        help="How many times to publish the command",
    )
    parser.add_argument(
        "--rate",
        type=float,
        default=10.0,
        help="Publish rate in Hz while repeating",
    )
    parser.add_argument(
        "--wait-for-subscriber",
        type=float,
        default=2.0,
        help="How long to wait for the interactive_goal subscriber before publishing",
    )
    args = parser.parse_args()

    quat_values = [args.qx, args.qy, args.qz, args.qw]
    if any(value is not None for value in quat_values) and not all(
        value is not None for value in quat_values
    ):
        parser.error("Provide either all of --qx/--qy/--qz/--qw or none of them.")
    if args.repeat < 1:
        parser.error("--repeat must be at least 1.")
    if args.rate <= 0.0:
        parser.error("--rate must be greater than 0.")
    return args


def main() -> int:
    args = parse_args()
    use_pose = args.qx is not None

    rclpy.init()
    node = GoalCliNode(args.point_topic, args.pose_topic)

    subscriber_found = node.wait_for_subscribers(use_pose, args.wait_for_subscriber)
    if not subscriber_found:
        node.get_logger().warning(
            "No subscriber detected on the goal command topic yet; publishing anyway."
        )

    period_sec = 1.0 / args.rate
    try:
        for repeat_index in range(args.repeat):
            if use_pose:
                msg = PoseStamped()
                msg.header.frame_id = args.frame_id
                msg.header.stamp = node.get_clock().now().to_msg()
                msg.pose.position.x = args.x
                msg.pose.position.y = args.y
                msg.pose.position.z = args.z
                msg.pose.orientation.x = args.qx
                msg.pose.orientation.y = args.qy
                msg.pose.orientation.z = args.qz
                msg.pose.orientation.w = args.qw
                node.pose_pub.publish(msg)
            else:
                msg = Point()
                msg.x = args.x
                msg.y = args.y
                msg.z = args.z
                node.point_pub.publish(msg)

            if repeat_index + 1 < args.repeat:
                time.sleep(period_sec)
    finally:
        node.destroy_node()
        rclpy.shutdown()

    if use_pose:
        print(
            f"Published pose goal to {args.pose_topic}: "
            f"position=({args.x:.3f}, {args.y:.3f}, {args.z:.3f}) "
            f"orientation=({args.qx:.4f}, {args.qy:.4f}, {args.qz:.4f}, {args.qw:.4f})"
        )
    else:
        print(
            f"Published point goal to {args.point_topic}: "
            f"({args.x:.3f}, {args.y:.3f}, {args.z:.3f})"
        )
    return 0


if __name__ == "__main__":
    sys.exit(main())

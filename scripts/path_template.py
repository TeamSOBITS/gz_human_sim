#!/usr/bin/env python3
"""Publish a canned shape (circle/square) as a PoseArray on a human's cmd_path.

Generic exercise/demo tool for ActorCommandPlugin's path-follow mode: any
world using gz_human_sim can point this at a spawned human's namespaced
cmd_path topic and have it walk the shape, without depending on any
particular arena's layout (compare sobits_gazebo_worlds' own
send_multi_waypoints.py, which *is* arena-specific -- it reads hand-placed
waypoint coordinates from that package's own config).
"""

import math

import rclpy
from geometry_msgs.msg import Pose, PoseArray
from rclpy.node import Node


def _circle_waypoints(center_x, center_y, radius, num_waypoints, clockwise):
    direction = -1.0 if clockwise else 1.0
    waypoints = []
    for i in range(num_waypoints):
        angle = direction * i * (2.0 * math.pi / num_waypoints)
        # Face tangent to the circle, in the direction of travel.
        tangent_angle = angle + direction * (math.pi / 2.0)
        waypoints.append((
            center_x + radius * math.cos(angle),
            center_y + radius * math.sin(angle),
            tangent_angle,
        ))
    return waypoints


def _square_waypoints(center_x, center_y, side_length, clockwise):
    half = side_length / 2.0
    corners = [
        (center_x + half, center_y + half),
        (center_x - half, center_y + half),
        (center_x - half, center_y - half),
        (center_x + half, center_y - half),
    ]
    if clockwise:
        corners = [corners[0]] + list(reversed(corners[1:]))
    waypoints = []
    for i, (x, y) in enumerate(corners):
        next_x, next_y = corners[(i + 1) % len(corners)]
        yaw = math.atan2(next_y - y, next_x - x)
        waypoints.append((x, y, yaw))
    return waypoints


class PathTemplatePublisher(Node):
    """Publish one canned-shape PoseArray to a cmd_path topic, then idle."""

    def __init__(self) -> None:
        super().__init__('path_template_publisher')

        self.declare_parameter('shape', 'circle')
        self.declare_parameter('path_topic', '/cmd_path')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('center_x', 0.0)
        self.declare_parameter('center_y', 0.0)
        self.declare_parameter('radius', 2.0)
        self.declare_parameter('side_length', 3.0)
        self.declare_parameter('num_waypoints', 12)
        self.declare_parameter('clockwise', False)

        self.shape = self.get_parameter('shape').value
        path_topic = self.get_parameter('path_topic').value
        self.frame_id = self.get_parameter('frame_id').value
        self.center_x = float(self.get_parameter('center_x').value)
        self.center_y = float(self.get_parameter('center_y').value)
        self.radius = float(self.get_parameter('radius').value)
        self.side_length = float(self.get_parameter('side_length').value)
        self.num_waypoints = int(self.get_parameter('num_waypoints').value)
        self.clockwise = bool(self.get_parameter('clockwise').value)

        if self.shape not in ('circle', 'square'):
            raise RuntimeError(
                f"Unsupported shape '{self.shape}'. Use 'circle' or 'square'.")

        self.publisher = self.create_publisher(PoseArray, path_topic, 10)
        # Give the ros_gz_bridge parameter_bridge (spawn_human.launch.py's
        # actor_command_bridge) a moment to finish matching before we
        # publish -- a PoseArray sent before it's subscribed is just lost.
        self.timer = self.create_timer(1.0, self._publish_once)
        self.get_logger().info(
            f"Publishing a {self.shape} path to {path_topic} in 1s...")

    def _publish_once(self) -> None:
        self.timer.cancel()

        if self.shape == 'circle':
            waypoints = _circle_waypoints(
                self.center_x, self.center_y, self.radius, self.num_waypoints,
                self.clockwise)
        else:
            waypoints = _square_waypoints(
                self.center_x, self.center_y, self.side_length, self.clockwise)

        msg = PoseArray()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.frame_id
        for x, y, yaw in waypoints:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.orientation.z = math.sin(yaw * 0.5)
            pose.orientation.w = math.cos(yaw * 0.5)
            msg.poses.append(pose)

        self.publisher.publish(msg)
        self.get_logger().info(
            f"Published {len(waypoints)} waypoints ({self.shape}).")


def main() -> None:
    rclpy.init()
    node = PathTemplatePublisher()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()

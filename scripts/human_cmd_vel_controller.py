#!/usr/bin/env python3

import math

import rclpy
from geometry_msgs.msg import Pose, Twist
from rclpy.node import Node
from ros_gz_interfaces.srv import SetEntityPose


class HumanCmdVelController(Node):
    def __init__(self) -> None:
        super().__init__("human_cmd_vel_controller")

        self.declare_parameter("world_name", "rcjo2025_arena")
        self.declare_parameter("model_name", "gz_human")
        self.declare_parameter("cmd_vel_topic", "/gz_human/cmd_vel")
        self.declare_parameter("initial_x", -2.0)
        self.declare_parameter("initial_y", 1.5)
        self.declare_parameter("initial_z", 0.0)
        self.declare_parameter("initial_yaw", 0.0)
        self.declare_parameter("update_rate_hz", 20.0)

        self.world_name = self.get_parameter("world_name").value
        self.model_name = self.get_parameter("model_name").value
        self.cmd_vel_topic = self.get_parameter("cmd_vel_topic").value
        self.x = float(self.get_parameter("initial_x").value)
        self.y = float(self.get_parameter("initial_y").value)
        self.z = float(self.get_parameter("initial_z").value)
        self.yaw = float(self.get_parameter("initial_yaw").value)
        update_rate = float(self.get_parameter("update_rate_hz").value)

        self.current_twist = Twist()

        self.client = self.create_client(SetEntityPose, f"/world/{self.world_name}/set_pose")
        self.get_logger().info(f"Waiting for /world/{self.world_name}/set_pose service...")
        self.client.wait_for_service()

        self.sub_cmd_vel = self.create_subscription(
            Twist, self.cmd_vel_topic, self.cmd_vel_callback, 10
        )
        self.timer = self.create_timer(1.0 / update_rate, self.update_pose)
        self.get_logger().info(
            f"Controlling {self.model_name} from {self.cmd_vel_topic}"
        )

    def cmd_vel_callback(self, msg: Twist) -> None:
        self.current_twist = msg

    def yaw_to_quaternion(self):
        half = self.yaw * 0.5
        return 0.0, 0.0, math.sin(half), math.cos(half)

    def set_pose(self) -> None:
        request = SetEntityPose.Request()
        request.entity.name = self.model_name

        pose = Pose()
        pose.position.x = self.x
        pose.position.y = self.y
        pose.position.z = self.z
        qx, qy, qz, qw = self.yaw_to_quaternion()
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz
        pose.orientation.w = qw
        request.pose = pose

        self.client.call_async(request)

    def update_pose(self) -> None:
        dt = float(self.timer.timer_period_ns) * 1e-9

        self.yaw += self.current_twist.angular.z * dt
        cos_yaw = math.cos(self.yaw)
        sin_yaw = math.sin(self.yaw)

        # Treat cmd_vel in body frame, like standard teleop.
        self.x += (
            self.current_twist.linear.x * cos_yaw
            - self.current_twist.linear.y * sin_yaw
        ) * dt
        self.y += (
            self.current_twist.linear.x * sin_yaw
            + self.current_twist.linear.y * cos_yaw
        ) * dt

        self.set_pose()


def main() -> None:
    rclpy.init()
    node = HumanCmdVelController()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()

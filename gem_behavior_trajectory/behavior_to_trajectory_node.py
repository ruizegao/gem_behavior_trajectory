import math

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Float32
from nav_msgs.msg import Odometry, Path
from geometry_msgs.msg import PoseStamped


class BehaviorToTrajectoryNode(Node):
    def __init__(self):
        super().__init__("behavior_to_trajectory_node")

        self.current_behavior = "STOP"
        self.current_speed_cmd = 0.0
        self.current_odom = None

        self.create_subscription(
            String,
            "/behavior/state",
            self.behavior_callback,
            10,
        )

        self.create_subscription(
            Odometry,
            "/odometry",
            self.odom_callback,
            10,
        )

        self.path_pub = self.create_publisher(Path, "/planning/local_path", 10)
        self.speed_pub = self.create_publisher(Float32, "/planning/target_speed", 10)

        self.timer = self.create_timer(0.1, self.timer_callback)

        self.get_logger().info("Behavior-to-trajectory node initialized.")

    def behavior_callback(self, msg: String):
        self.current_behavior = msg.data.upper()

    def odom_callback(self, msg: Odometry):
        self.current_odom = msg

    def timer_callback(self):
        if self.current_odom is None:
            return

        if self.current_behavior == "GO":
            path = self.generate_straight_path()
            target_speed = 1.0

        elif self.current_behavior == "DETOUR_LEFT":
            path = self.generate_detour_path(side="left")
            target_speed = 0.6

        elif self.current_behavior == "DETOUR_RIGHT":
            path = self.generate_detour_path(side="right")
            target_speed = 0.6

        else:
            path = self.generate_stop_path()
            target_speed = 0.0

        self.path_pub.publish(path)

        speed_msg = Float32()
        speed_msg.data = target_speed
        self.speed_pub.publish(speed_msg)

    def generate_straight_path(self):
        local_waypoints = [
            (2.0, 0.0),
            (4.0, 0.0),
            (6.0, 0.0),
            (8.0, 0.0),
        ]
        return self.local_waypoints_to_path(local_waypoints)

    def generate_stop_path(self):
        local_waypoints = [
            (0.0, 0.0),
        ]
        return self.local_waypoints_to_path(local_waypoints)

    def generate_detour_path(self, side="left"):
        sign = 1.0 if side == "left" else -1.0

        local_waypoints = [
            (1.0, 0.0),
            (2.5, 0.5 * sign),
            (4.0, 1.0 * sign),
            (6.0, 1.0 * sign),
            (8.0, 0.5 * sign),
            (10.0, 0.0),
        ]

        return self.local_waypoints_to_path(local_waypoints)

    def local_waypoints_to_path(self, local_waypoints):
        odom = self.current_odom

        x0 = odom.pose.pose.position.x
        y0 = odom.pose.pose.position.y
        yaw = self.quaternion_to_yaw(odom.pose.pose.orientation)

        path = Path()
        path.header.stamp = self.get_clock().now().to_msg()
        path.header.frame_id = odom.header.frame_id

        for lx, ly in local_waypoints:
            gx = x0 + math.cos(yaw) * lx - math.sin(yaw) * ly
            gy = y0 + math.sin(yaw) * lx + math.cos(yaw) * ly

            pose = PoseStamped()
            pose.header = path.header
            pose.pose.position.x = gx
            pose.pose.position.y = gy
            pose.pose.position.z = 0.0
            pose.pose.orientation = odom.pose.pose.orientation

            path.poses.append(pose)

        return path

    @staticmethod
    def quaternion_to_yaw(q):
        siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
        cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        return math.atan2(siny_cosp, cosy_cosp)


def main(args=None):
    rclpy.init(args=args)
    node = BehaviorToTrajectoryNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
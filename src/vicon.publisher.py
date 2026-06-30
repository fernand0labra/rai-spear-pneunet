#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose

from mocap4r2_msgs.msg import RigidBodies


class RigidBodiesToOdometry(Node):
    def __init__(self):
        super().__init__("rigid_bodies_to_odometry")

        # Names to extract from the rigidbodies list
        self.declare_parameter(
            "target_names",
            ["softdrone-base.softdrone-base", "softdrone-tip.softdrone-tip"],
        )
        self.declare_parameter(
            "output_topics",
            ["/odom/softdrone/base", "/odom/softdrone/tip"],
        )

        self.target_names = list(self.get_parameter("target_names").value)
        self.output_topics = list(self.get_parameter("output_topics").value)

        if len(self.target_names) != len(self.output_topics):
            raise ValueError("target_names and output_topics must have the same length")

        self.odom_publishers = {
            name: self.create_publisher(Odometry, topic, 10)
            for name, topic in zip(self.target_names, self.output_topics)
        }

        self.subscription = self.create_subscription(
            RigidBodies,
            "/rigid_bodies",
            self.rigid_bodies_callback,
            10,
        )

        self.get_logger().info(
            f"Listening on /rigid_bodies and publishing odometry for: {self.target_names}"
        )

    def rigid_bodies_callback(self, msg: RigidBodies):
        # The incoming message appears to have:
        # msg.header
        # msg.rigidbodies -> list of rigid body objects
        for body in msg.rigidbodies:
            name = body.rigid_body_name

            if name not in self.odom_publishers:
                continue

            odom = Odometry()
            odom.header = msg.header
            odom.child_frame_id = name

            # Copy pose
            odom.pose.pose.position.x = body.pose.position.x
            odom.pose.pose.position.y = body.pose.position.y
            odom.pose.pose.position.z = body.pose.position.z

            odom.pose.pose.orientation.x = body.pose.orientation.x
            odom.pose.pose.orientation.y = body.pose.orientation.y
            odom.pose.pose.orientation.z = body.pose.orientation.z
            odom.pose.pose.orientation.w = body.pose.orientation.w

            # Optional: clear twist if your source does not provide it
            odom.twist.twist.linear.x = 0.0
            odom.twist.twist.linear.y = 0.0
            odom.twist.twist.linear.z = 0.0
            odom.twist.twist.angular.x = 0.0
            odom.twist.twist.angular.y = 0.0
            odom.twist.twist.angular.z = 0.0

            self.odom_publishers[name].publish(odom)


def main():
    rclpy.init()
    node = RigidBodiesToOdometry()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
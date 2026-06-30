#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomSaver(Node):
    def __init__(self):
        super().__init__('odom_coordinate_saver')

        self.fx = open('x.txt', 'w')
        self.fy = open('y.txt', 'w')
        self.fz = open('z.txt', 'w')

        self.subscription = self.create_subscription(
            Odometry,
            '/odom/softdrone/tip',
            self.odom_callback,
            10
        )

        self.get_logger().info('Subscribed to /odom/softdrone/tip')

    def odom_callback(self, msg: Odometry):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        z = msg.pose.pose.position.z

        self.fx.write(f'{x}\n')
        self.fy.write(f'{y}\n')
        self.fz.write(f'{z}\n')

        self.fx.flush()
        self.fy.flush()
        self.fz.flush()

    def close_files(self):
        self.fx.close()
        self.fy.close()
        self.fz.close()


def main():
    rclpy.init()
    node = OdomSaver()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.close_files()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
#!/usr/bin/env python3

import math
import threading
from collections import deque

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation


class OdomPlotter(Node):
    def __init__(self):
        super().__init__('odom_plotter')

        # One buffer per topic
        self.data = {
            '/odom/softdrone/base': {
                't': deque(maxlen=5000),
                'x': deque(maxlen=5000),
                'y': deque(maxlen=5000),
                'z': deque(maxlen=5000),
                'last_pos': None,
                'start_time': None,
            },
            '/odom/softdrone/tip': {
                't': deque(maxlen=5000),
                'x': deque(maxlen=5000),
                'y': deque(maxlen=5000),
                'z': deque(maxlen=5000),
                'last_pos': None,
                'start_time': None,
            }
        }

        self.start_values = {
            '/odom/softdrone/base': [0.0, 0.0, 0.0],
            '/odom/softdrone/tip': [0.0, 0.0, 0.0],
        }

        self.sub_base = self.create_subscription(
            Odometry,
            '/odom/softdrone/base',
            self.make_callback('/odom/softdrone/base'),
            10
        )

        self.sub_tip = self.create_subscription(
            Odometry,
            '/odom/softdrone/tip',
            self.make_callback('/odom/softdrone/tip'),
            10
        )

    def make_callback(self, topic_name):
        def callback(msg: Odometry):
            # Extract position
            p = msg.pose.pose.position
            x, y, z = p.x - self.start_values[topic_name][0], p.y - self.start_values[topic_name][1], p.z - self.start_values[topic_name][2]

            # Skip invalid / empty values
            if any(math.isnan(v) or math.isinf(v) for v in (x, y, z)):
                return

            # If you want to treat exact zeros as "empty", keep this:
            # Comment it out if zero is a valid position for your bag.
            if p.x == 0.0 and p.y == 0.0 and p.z == 0.0:
                return

            sample = self.data[topic_name]

            # Initialize time reference
            now = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
            if sample['start_time'] is None:
                sample['start_time'] = now

            rel_t = now - sample['start_time']

            # Skip jumps larger than 0.01 from the last accepted sample
            last_pos = sample['last_pos']
            if last_pos is not None:
                dx = abs(x - last_pos[0])
                dy = abs(y - last_pos[1])
                dz = abs(z - last_pos[2])

                # if dx > 0.05 or dy > 0.05 or dz > 0.05:
                #     return

            # Accept sample
            sample['t'].append(rel_t)
            sample['x'].append(x)
            sample['y'].append(y)
            sample['z'].append(z)
            sample['last_pos'] = (x, y, z)

        return callback


def main():
    rclpy.init()
    node = OdomPlotter()

    # Spin ROS in a background thread
    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    plt.style.use('default')
    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(12, 8))
    fig.suptitle('Odometry Position Over Time')

    lines = {}
    colors = {
        '/odom/softdrone/base': 'tab:blue',
        '/odom/softdrone/tip': 'tab:orange',
    }

    limits_low = [-1.0, 5.0, 1.0]
    limits_high = [0.0, 6.0, 2.0]
    for i, axis_name in enumerate(['x', 'y', 'z']):
        ax = axes[i]
        ax.set_ylabel(axis_name)
        # ax.set_ylim(-0.05, 0.05)
        ax.grid(True)

        for topic in node.data.keys():
            line, = ax.plot([], [], label=topic, color=colors[topic])
            lines[(topic, axis_name)] = line

        ax.legend(loc='upper right')

    axes[-1].set_xlabel('Time (s)')

    def update(_):
        for topic, sample in node.data.items():
            t = list(sample['t'])
            xs = list(sample['x'])
            ys = list(sample['y'])
            zs = list(sample['z'])

            lines[(topic, 'x')].set_data(t, xs)
            lines[(topic, 'y')].set_data(t, ys)
            lines[(topic, 'z')].set_data(t, zs)

        for ax in axes:
            ax.relim()
            ax.autoscale_view()

        return list(lines.values())

    ani = FuncAnimation(fig, update, interval=100)
    plt.tight_layout()
    plt.show()

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
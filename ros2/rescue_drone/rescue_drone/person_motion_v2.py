import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3

import math


class PersonMotionV2(Node):

    def __init__(self):
        super().__init__('person_motion_v2')

        self.get_logger().info(
            '🏃 V2 Person Motion Tracker Starting...'
        )

        # Previous person position
        self.previous_x = None
        self.previous_y = None
        self.previous_time = None

        # Smoothed velocity
        self.velocity_x = 0.0
        self.velocity_y = 0.0

        # Smoothing factor
        # Smaller = smoother but slower response
        self.alpha = 0.25

        # Subscribe to tracked person
        self.target_sub = self.create_subscription(
            Point,
            '/rescue_drone/person_center_v2',
            self.target_callback,
            10
        )

        # Publish person velocity
        self.velocity_pub = self.create_publisher(
            Vector3,
            '/rescue_drone/person_velocity_v2',
            10
        )

        self.get_logger().info(
            '🎯 Waiting for tracked person...'
        )

    def target_callback(self, msg):

        current_time = self.get_clock().now()

        current_x = msg.x
        current_y = msg.y

        # First detection
        if self.previous_x is None:

            self.previous_x = current_x
            self.previous_y = current_y
            self.previous_time = current_time

            self.get_logger().info(
                f'👤 First position: '
                f'({current_x:.0f}, {current_y:.0f})'
            )

            return

        # Calculate elapsed time
        dt = (
            current_time - self.previous_time
        ).nanoseconds / 1e9

        # Ignore invalid time intervals
        if dt <= 0.0 or dt > 2.0:
            self.previous_x = current_x
            self.previous_y = current_y
            self.previous_time = current_time
            return

        # Position change
        dx = current_x - self.previous_x
        dy = current_y - self.previous_y

        # Raw image velocity
        raw_vx = dx / dt
        raw_vy = dy / dt

        # Exponential smoothing
        self.velocity_x = (
            self.alpha * raw_vx
            + (1.0 - self.alpha) * self.velocity_x
        )

        self.velocity_y = (
            self.alpha * raw_vy
            + (1.0 - self.alpha) * self.velocity_y
        )

        # Overall image speed
        speed = math.sqrt(
            self.velocity_x ** 2
            + self.velocity_y ** 2
        )

        # Publish velocity
        velocity_msg = Vector3()

        velocity_msg.x = self.velocity_x
        velocity_msg.y = self.velocity_y
        velocity_msg.z = speed

        self.velocity_pub.publish(
            velocity_msg
        )

        # Direction
        if speed < 10:
            direction = 'STATIONARY'

        elif abs(self.velocity_x) > abs(self.velocity_y):

            if self.velocity_x > 0:
                direction = 'MOVING RIGHT'
            else:
                direction = 'MOVING LEFT'

        else:

            if self.velocity_y > 0:
                direction = 'MOVING DOWN'
            else:
                direction = 'MOVING UP'

        self.get_logger().info(
            f'🏃 PERSON MOTION | '
            f'vx={self.velocity_x:+.1f} px/s | '
            f'vy={self.velocity_y:+.1f} px/s | '
            f'speed={speed:.1f} px/s | '
            f'{direction}'
        )

        # Save current position
        self.previous_x = current_x
        self.previous_y = current_y
        self.previous_time = current_time


def main(args=None):

    rclpy.init(args=args)

    node = PersonMotionV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

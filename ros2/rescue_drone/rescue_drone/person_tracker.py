import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point


class PersonTracker(Node):

    def __init__(self):
        super().__init__('person_tracker')

        self.subscription = self.create_subscription(
            Point,
            '/rescue_drone/person_center',
            self.person_callback,
            10
        )

        self.get_logger().info(
            '🎯 Person Tracker Started'
        )

        self.get_logger().info(
            'Waiting for YOLO person position...'
        )

    def person_callback(self, msg):

        center_x = msg.x
        center_y = msg.y
        confidence = msg.z

        # Camera width = 1920 pixels
        camera_center = 1920 / 2

        # Dead-zone around the center
        left_limit = camera_center - 200
        right_limit = camera_center + 200

        if center_x < left_limit:

            direction = 'LEFT'

        elif center_x > right_limit:

            direction = 'RIGHT'

        else:

            direction = 'CENTER'

        self.get_logger().info(
            f'👤 PERSON | '
            f'x={center_x:.0f} '
            f'y={center_y:.0f} '
            f'confidence={confidence:.2f} '
            f'→ MOVE {direction}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PersonTracker()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
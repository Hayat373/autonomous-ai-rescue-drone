import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3


class InterceptControllerV2(Node):

    def __init__(self):
        super().__init__('intercept_controller_v2')

        self.get_logger().info('🧠 V2.4 Intercept Controller Starting...')
        self.get_logger().info('🚁 DRY-RUN MODE — Drone will NOT move')

        self.predicted_x = None
        self.predicted_y = None

        self.velocity_x = 0.0
        self.velocity_y = 0.0

        self.last_prediction_time = None

        # Camera resolution
        self.image_width = 1920
        self.image_height = 1080

        # Horizontal center of camera
        self.image_center_x = self.image_width / 2

        # Dead zone around center
        self.center_tolerance = 150

        # Prediction freshness
        self.prediction_timeout = 1.0

        self.prediction_sub = self.create_subscription(
            Point,
            '/rescue_drone/predicted_person_v2',
            self.prediction_callback,
            10
        )

        self.velocity_sub = self.create_subscription(
            Vector3,
            '/rescue_drone/person_velocity_v2',
            self.velocity_callback,
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.make_decision
        )

        self.get_logger().info(
            '🎯 Waiting for predicted person position...'
        )

    def prediction_callback(self, msg):

        self.predicted_x = msg.x
        self.predicted_y = msg.y

        self.last_prediction_time = self.get_clock().now()

    def velocity_callback(self, msg):

        self.velocity_x = msg.x
        self.velocity_y = msg.y

    def make_decision(self):

        if self.predicted_x is None:
            self.get_logger().info(
                '🔍 SEARCHING — no predicted person'
            )
            return

        # Check prediction freshness
        if self.last_prediction_time is not None:

            age = (
                self.get_clock().now()
                - self.last_prediction_time
            ).nanoseconds / 1e9

            if age > self.prediction_timeout:

                self.get_logger().warn(
                    '⚠️ Prediction is stale — HOLD'
                )

                self.print_decision(
                    'HOLD',
                    'Prediction lost/stale'
                )

                return

        # Calculate horizontal error
        error_x = self.predicted_x - self.image_center_x

        # Determine horizontal movement
        if error_x < -self.center_tolerance:

            horizontal_action = 'MOVE LEFT'

        elif error_x > self.center_tolerance:

            horizontal_action = 'MOVE RIGHT'

        else:

            horizontal_action = 'CENTERED'

        # Determine person motion
        speed = (
            self.velocity_x ** 2
            + self.velocity_y ** 2
        ) ** 0.5

        if speed < 10:

            motion = 'STATIONARY'

        elif abs(self.velocity_x) > abs(self.velocity_y):

            if self.velocity_x > 0:
                motion = 'MOVING RIGHT'
            else:
                motion = 'MOVING LEFT'

        else:

            if self.velocity_y > 0:
                motion = 'MOVING DOWN'
            else:
                motion = 'MOVING UP'

        # Determine forward behavior
        #
        # The current V2 camera does not yet provide
        # a reliable predicted 3D target distance here.
        # Therefore we deliberately do NOT command
        # forward movement yet.

        if horizontal_action == 'CENTERED':

            forward_action = 'HOLD'

        else:

            forward_action = 'WAIT'

        # Print complete decision
        self.get_logger().info(
            '\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '🧠 V2.4 INTERCEPT DECISION\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'👤 Predicted person: '
            f'({self.predicted_x:.0f}, {self.predicted_y:.0f})\n'
            f'🎯 Image center: '
            f'({self.image_center_x:.0f}, {self.image_height/2:.0f})\n'
            f'📐 Horizontal error: '
            f'{error_x:+.0f} px\n'
            f'🏃 Person motion: {motion}\n'
            f'💨 Speed: {speed:.1f} px/s\n'
            f'↔️ Horizontal action: {horizontal_action}\n'
            f'➡️ Forward action: {forward_action}\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

    def print_decision(self, action, reason):

        self.get_logger().info(
            '\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '🧠 V2.4 INTERCEPT DECISION\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'🎯 ACTION: {action}\n'
            f'📋 REASON: {reason}\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )


def main(args=None):

    rclpy.init(args=args)

    node = InterceptControllerV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

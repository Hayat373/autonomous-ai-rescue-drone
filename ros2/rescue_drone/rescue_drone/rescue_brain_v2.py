import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3


class RescueBrainV2(Node):

    def __init__(self):
        super().__init__('rescue_brain_v2')

        self.get_logger().info('🧠 V2.6 RESCUE BRAIN STARTING...')
        self.get_logger().info('🚁 DRY-RUN MODE — PX4 WILL NOT MOVE')

        self.horizontal_error = None
        self.distance = None

        self.velocity_x = 0.0
        self.velocity_y = 0.0

        self.last_target_time = None

        # Subscribe to 3D target
        self.target_sub = self.create_subscription(
            Vector3,
            '/rescue_drone/target_3d_v2',
            self.target_callback,
            10
        )

        # Decision loop
        self.timer = self.create_timer(
            0.5,
            self.make_decision
        )

        self.get_logger().info(
            '🎯 Waiting for 3D rescue target...'
        )

    def target_callback(self, msg):

        self.horizontal_error = msg.x
        self.distance = msg.z

        self.last_target_time = self.get_clock().now()

    def make_decision(self):

        if self.horizontal_error is None:
            self.get_logger().info(
                '🔍 SEARCHING — no target'
            )
            return

        # Check target freshness
        age = (
            self.get_clock().now()
            - self.last_target_time
        ).nanoseconds / 1e9

        if age > 1.5:

            self.print_decision(
                'HOLD',
                'Target data is stale'
            )

            return

        error = self.horizontal_error
        distance = self.distance

        # --------------------------------
        # HORIZONTAL CONTROL
        # --------------------------------

        if error < -150:

            horizontal = 'MOVE LEFT'

        elif error > 150:

            horizontal = 'MOVE RIGHT'

        else:

            horizontal = 'CENTERED'

        # --------------------------------
        # FORWARD CONTROL
        # --------------------------------

        if distance > 8.0:

            forward = 'MOVE FORWARD'

        elif distance > 4.0:

            forward = 'MOVE FORWARD SLOW'

        elif distance > 2.5:

            forward = 'APPROACH SLOWLY'

        else:

            forward = 'STOP'

        # --------------------------------
        # DELIVERY DECISION
        # --------------------------------

        if distance <= 2.5 and abs(error) <= 150:

            mission_action = 'DELIVER PACKAGE'

        elif distance <= 2.5:

            mission_action = 'CENTER ON PERSON'

        else:

            mission_action = 'APPROACH PERSON'

        # --------------------------------
        # COMPLETE DECISION
        # --------------------------------

        self.get_logger().info(
            '\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '🧠 V2.6 AUTONOMOUS RESCUE BRAIN\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'🎯 Horizontal error: {error:+.0f} px\n'
            f'📏 Distance: {distance:.2f} m\n'
            f'↔️ Horizontal: {horizontal}\n'
            f'➡️ Forward: {forward}\n'
            f'🚑 Mission: {mission_action}\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )

    def print_decision(self, action, reason):

        self.get_logger().info(
            '\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '🧠 V2.6 AUTONOMOUS RESCUE BRAIN\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'🚁 ACTION: {action}\n'
            f'📋 REASON: {reason}\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )


def main(args=None):

    rclpy.init(args=args)

    node = RescueBrainV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

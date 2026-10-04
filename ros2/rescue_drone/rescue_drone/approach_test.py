import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3


class ApproachTest(Node):

    def __init__(self):
        super().__init__('approach_test')

        self.person_x = None
        self.person_y = None
        self.distance = None
        self.confidence = None

        self.target_sub = self.create_subscription(
            Vector3,
            '/rescue_drone/rescue_target',
            self.target_callback,
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.control_loop
        )

        self.get_logger().info('======================================')
        self.get_logger().info('🚁 RESCUE APPROACH DRY RUN')
        self.get_logger().info('⚠️ DRONE MOVEMENT IS DISABLED')
        self.get_logger().info('======================================')

    def target_callback(self, msg):

        self.person_x = msg.x
        self.person_y = msg.y
        self.distance = msg.z

        self.last_target_time = self.get_clock().now()

    def control_loop(self):

        if self.person_x is None or self.distance is None:
            self.get_logger().info(
                '🔎 SEARCHING | No rescue target'
            )
            return

        # ------------------------------------------------
        # IMAGE CENTER
        # ------------------------------------------------

        image_center = 960.0

        error_x = self.person_x - image_center

        # ------------------------------------------------
        # CENTERING
        # ------------------------------------------------

        if error_x < -120:

            yaw_command = 'YAW LEFT'

        elif error_x > 120:

            yaw_command = 'YAW RIGHT'

        else:

            yaw_command = 'CENTERED'

        # ------------------------------------------------
        # DISTANCE CONTROL
        # ------------------------------------------------

        if self.distance > 6.0:

            distance_state = 'FAR'
            forward_speed = 0.50

        elif self.distance > 3.0:

            distance_state = 'APPROACH'
            forward_speed = 0.30

        elif self.distance > 2.5:

            distance_state = 'SLOW APPROACH'
            forward_speed = 0.15

        else:

            distance_state = 'DELIVERY RANGE'
            forward_speed = 0.0

        # ------------------------------------------------
        # FINAL ACTION
        # ------------------------------------------------

        if yaw_command != 'CENTERED':

            action = yaw_command
            forward_speed = 0.0

        elif distance_state == 'DELIVERY RANGE':

            action = 'HOVER / DELIVERY'

        else:

            action = 'MOVE FORWARD'

        # ------------------------------------------------
        # OUTPUT
        # ------------------------------------------------

        self.get_logger().info(
            f'👤 PERSON | '
            f'x={self.person_x:.0f} | '
            f'distance={self.distance:.2f}m | '
            f'error={error_x:+.0f} | '
            f'state={distance_state} | '
            f'action={action} | '
            f'forward={forward_speed:.2f} m/s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = ApproachTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
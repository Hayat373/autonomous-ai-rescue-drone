import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Vector3
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand
)


class RescueFlightV2(Node):

    def __init__(self):
        super().__init__('rescue_flight_v2')

        self.get_logger().info('🚁 V2.7 RESCUE FLIGHT CONTROLLER')
        self.get_logger().info('⚠️ PX4 SIMULATION MODE')

        self.target_error = None
        self.target_distance = None

        self.last_target_time = None

        # Home/hover position
        self.home_x = 0.0
        self.home_y = 0.0
        self.hover_z = -3.0

        # Current commanded position
        self.target_x = 0.0
        self.target_y = 0.0
        self.target_z = -3.0

        # Safety limits
        self.max_step = 0.5
        self.max_distance = 20.0

        # Target subscriber
        self.target_sub = self.create_subscription(
            Vector3,
            '/rescue_drone/target_3d_v2',
            self.target_callback,
            10
        )

        # PX4 publishers
        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info(
            '🎯 Waiting for rescue target...'
        )

    def target_callback(self, msg):

        self.target_error = msg.x
        self.target_distance = msg.z

        self.last_target_time = self.get_clock().now()

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = (
            self.get_clock().now()
            .nanoseconds // 1000
        )

        self.offboard_pub.publish(msg)

    def publish_setpoint(self):

        msg = TrajectorySetpoint()

        msg.position[0] = self.target_x
        msg.position[1] = self.target_y
        msg.position[2] = self.target_z

        msg.yaw = 0.0

        msg.timestamp = (
            self.get_clock().now()
            .nanoseconds // 1000
        )

        self.setpoint_pub.publish(msg)

    def send_command(self, command):

        msg = VehicleCommand()

        msg.command = command
        msg.param1 = 1.0
        msg.param2 = 6.0

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        msg.timestamp = (
            self.get_clock().now()
            .nanoseconds // 1000
        )

        self.command_pub.publish(msg)

    def control_loop(self):

        # Always keep publishing offboard mode
        self.publish_offboard_mode()

        # No target yet
        if self.target_error is None:

            self.publish_setpoint()

            return

        # Check target freshness
        age = (
            self.get_clock().now()
            - self.last_target_time
        ).nanoseconds / 1e9

        if age > 1.5:

            self.get_logger().warn(
                '⚠️ TARGET LOST — HOLDING POSITION'
            )

            self.publish_setpoint()

            return

        error = self.target_error
        distance = self.target_distance

        # --------------------------------
        # HORIZONTAL MOVEMENT
        # --------------------------------

        if error > 150:

            self.target_y -= self.max_step

            direction = 'RIGHT'

        elif error < -150:

            self.target_y += self.max_step

            direction = 'LEFT'

        else:

            direction = 'CENTERED'

        # --------------------------------
        # FORWARD MOVEMENT
        # --------------------------------

        if distance > 8.0:

            self.target_x += self.max_step

            forward = 'FORWARD FAST'

        elif distance > 4.0:

            self.target_x += 0.25

            forward = 'FORWARD SLOW'

        elif distance > 2.5:

            self.target_x += 0.10

            forward = 'APPROACH'

        else:

            forward = 'STOP'

        # Safety clamp
        self.target_x = max(
            -self.max_distance,
            min(self.max_distance, self.target_x)
        )

        self.target_y = max(
            -self.max_distance,
            min(self.max_distance, self.target_y)
        )

        self.target_z = self.hover_z

        self.publish_setpoint()

        self.get_logger().info(
            f'🚁 FLIGHT COMMAND | '
            f'DIRECTION={direction} | '
            f'{forward} | '
            f'target=({self.target_x:.2f}, '
            f'{self.target_y:.2f}, '
            f'{self.target_z:.2f})'
        )


def main(args=None):

    rclpy.init(args=args)

    node = RescueFlightV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

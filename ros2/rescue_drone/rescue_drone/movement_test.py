import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)


class MovementTest(Node):
    def __init__(self):
        super().__init__('movement_test')

        # PX4 status and position outputs use BEST_EFFORT QoS.
        feedback_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10,
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10,
        )
        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10,
        )

        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_cb,
            feedback_qos,
        )
        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_cb,
            feedback_qos,
        )

        self.armed = False
        self.nav_state = -1
        self.preflight_ok = False
        self.position_received = False

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0
        self.home_x = 0.0
        self.home_y = 0.0

        self.phase = 'PREPARE'
        self.count = 0
        self.last_log_phase = ''

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('🚁 MOVEMENT TEST STARTED')
        self.get_logger().info('Mission: TAKEOFF → MOVE 1m → RETURN → LAND')
        self.get_logger().info('Feedback subscriptions use BEST_EFFORT QoS')
        self.get_logger().info('Waiting for PX4 status and position...')

    def status_cb(self, msg):
        self.armed = (
            msg.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )
        self.nav_state = msg.nav_state
        self.preflight_ok = msg.pre_flight_checks_pass

    def position_cb(self, msg):
        self.x = float(msg.x)
        self.y = float(msg.y)
        self.z = float(msg.z)
        self.position_received = True

    def publish_offboard_mode(self):
        msg = OffboardControlMode()
        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.mode_pub.publish(msg)

    def publish_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.yaw = 0.0
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000
        self.command_pub.publish(msg)

    def stop_test(self, reason):
        self.get_logger().error(reason)
        self.get_logger().error(
            'Test stopped. No further movement commands will be sent.'
        )
        self.timer.cancel()

    def control_loop(self):
        self.publish_offboard_mode()

        # Do not command flight until feedback is available.
        if not self.position_received or self.nav_state < 0:
            return

        self.count += 1

        if self.phase == 'PREPARE':
            if not self.preflight_ok:
                self.stop_test(
                    'PX4 preflight checks are not passing. Check PX4/QGC.'
                )
                return

            # Stream setpoints for at least five seconds before OFFBOARD.
            self.publish_setpoint(self.x, self.y, -2.0)

            if self.count % 10 == 0:
                self.get_logger().info(
                    f'Preparing OFFBOARD: {self.count}/50 | '
                    f'nav_state={self.nav_state}'
                )

            if self.count >= 50:
                self.home_x = self.x
                self.home_y = self.y
                self.get_logger().info('Requesting OFFBOARD mode')
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                    1.0,
                    6.0,
                )
                self.phase = 'WAIT_OFFBOARD'
                self.count = 0
            return

        if self.phase == 'WAIT_OFFBOARD':
            self.publish_setpoint(self.home_x, self.home_y, -2.0)

            if self.nav_state == 14:
                self.get_logger().info(
                    'OFFBOARD confirmed. Requesting arm.'
                )
                self.send_command(
                    VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                    1.0,
                )
                self.phase = 'WAIT_ARM'
                self.count = 0
            elif self.count >= 50:
                self.stop_test(
                    f'OFFBOARD was not accepted; nav_state={self.nav_state}. '
                    'Check PX4 status and command acknowledgements.'
                )
            return

        if self.phase == 'WAIT_ARM':
            self.publish_setpoint(self.home_x, self.home_y, -2.0)

            if self.armed and self.nav_state == 14:
                self.get_logger().info(
                    'ARM confirmed. Hovering at 2 metres.'
                )
                self.phase = 'HOVER'
                self.count = 0
            elif self.count >= 50:
                self.stop_test(
                    f'Arming not confirmed; armed={self.armed}, '
                    f'nav_state={self.nav_state}.'
                )
            return

        if self.phase == 'HOVER':
            self.publish_setpoint(self.home_x, self.home_y, -2.0)

            if self.count >= 40:
                self.phase = 'MOVE'
                self.count = 0
                self.get_logger().info(
                    '➡️ Moving 1 metre along world +X'
                )
            return

        if self.phase == 'MOVE':
            self.publish_setpoint(self.home_x + 1.0, self.home_y, -2.0)

            if self.count % 10 == 0:
                self.get_logger().info(
                    f'MOVE | position=({self.x:.2f}, '
                    f'{self.y:.2f}, {self.z:.2f})'
                )

            if self.count >= 50:
                self.phase = 'RETURN'
                self.count = 0
                self.get_logger().info('↩️ Returning to starting position')
            return

        if self.phase == 'RETURN':
            self.publish_setpoint(self.home_x, self.home_y, -2.0)

            if self.count % 10 == 0:
                self.get_logger().info(
                    f'RETURN | position=({self.x:.2f}, '
                    f'{self.y:.2f}, {self.z:.2f})'
                )

            if self.count >= 50:
                self.get_logger().info('🛬 Requesting LAND mode')
                self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
                self.phase = 'LAND'
                self.count = 0
            return

        if self.phase == 'LAND':
            # Let PX4's landing controller take over.
            # Keep sending the last position setpoint while it transitions.
            self.publish_setpoint(self.home_x, self.home_y, -2.0)

            if self.count % 20 == 0:
                self.get_logger().info(
                    f'LANDING | armed={self.armed} | '
                    f'position=({self.x:.2f}, {self.y:.2f}, {self.z:.2f})'
                )


def main(args=None):
    rclpy.init(args=args)
    node = MovementTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

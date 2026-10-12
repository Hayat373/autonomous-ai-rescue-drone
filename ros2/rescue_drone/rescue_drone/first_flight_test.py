import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus
)


class FirstFlightTest(Node):

    def __init__(self):
        super().__init__('first_flight_test')

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

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            10
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            10
        )

        self.x = 0.0
        self.y = 0.0
        self.z = 0.0

        self.nav_state = 0
        self.armed = False

        self.counter = 0

        self.get_logger().info(
            '🚁 FIRST CONTROLLED FLIGHT TEST'
        )

        self.get_logger().info(
            '📋 TAKEOFF → MOVE 1m → RETURN → LAND'
        )

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

    def position_callback(self, msg):

        self.x = msg.x
        self.y = msg.y
        self.z = msg.z

    def status_callback(self, msg):

        self.nav_state = msg.nav_state

        self.armed = (
            msg.arming_state ==
            VehicleStatus.ARMING_STATE_ARMED
        )

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.offboard_pub.publish(msg)

    def publish_setpoint(self, x, y, z):

        msg = TrajectorySetpoint()

        msg.position[0] = float(x)
        msg.position[1] = float(y)
        msg.position[2] = float(z)

        msg.yaw = 0.0

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.setpoint_pub.publish(msg)

    def send_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.command = command

        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.command_pub.publish(msg)

    def control_loop(self):

        self.counter += 1

        # --------------------------------
        # ALWAYS publish OFFBOARD heartbeat
        # --------------------------------

        self.publish_offboard_mode()

        # --------------------------------
        # ALWAYS publish a valid setpoint
        # --------------------------------

        if self.counter < 50:

            # PREPARE OFFBOARD
            # Hold at current/home position

            self.publish_setpoint(
                0.0,
                0.0,
                -3.0
            )

            if self.counter % 10 == 0:

                self.get_logger().info(
                    f'🟡 Preparing OFFBOARD '
                    f'({self.counter}/50)'
                )

            return

        # --------------------------------
        # REQUEST OFFBOARD
        # --------------------------------

        if self.counter == 50:

            self.get_logger().info(
                '🟠 REQUESTING OFFBOARD'
            )

            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

        # --------------------------------
        # ARM
        # --------------------------------

        if self.counter == 60:

            self.get_logger().info(
                '🟢 REQUESTING ARM'
            )

            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0
            )

        # --------------------------------
        # TAKEOFF / HOVER
        # --------------------------------

        if self.counter < 120:

            self.publish_setpoint(
                0.0,
                0.0,
                -3.0
            )

            phase = 'TAKEOFF/HOVER'

        # --------------------------------
        # MOVE +X 1 METER
        # --------------------------------

        elif self.counter < 180:

            self.publish_setpoint(
                1.0,
                0.0,
                -3.0
            )

            phase = 'MOVE +X'

        # --------------------------------
        # RETURN HOME
        # --------------------------------

        elif self.counter < 240:

            self.publish_setpoint(
                0.0,
                0.0,
                -3.0
            )

            phase = 'RETURN HOME'

        # --------------------------------
        # LAND
        # --------------------------------

        elif self.counter == 240:

            self.get_logger().info(
                '🛬 REQUESTING LAND'
            )

            self.send_command(
                VehicleCommand.VEHICLE_CMD_NAV_LAND
            )

            phase = 'LAND'

        else:

            self.publish_setpoint(
                0.0,
                0.0,
                -3.0
            )

            phase = 'LANDING'

        self.get_logger().info(
            f'🚁 {phase} | '
            f'pos=({self.x:.2f}, '
            f'{self.y:.2f}, '
            f'{self.z:.2f}) | '
            f'armed={self.armed} | '
            f'nav_state={self.nav_state}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = FirstFlightTest()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

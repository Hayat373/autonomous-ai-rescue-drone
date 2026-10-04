import rclpy
from rclpy.node import Node
from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint, VehicleCommand


class HoverTest(Node):

    def __init__(self):
        super().__init__('hover_test')

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

        self.counter = 0

        self.timer = self.create_timer(0.1, self.control_loop)

        self.get_logger().info('🚁 Hover test controller started')

    def control_loop(self):

        # Tell PX4 we are controlling position
        offboard = OffboardControlMode()
        offboard.position = True
        offboard.velocity = False
        offboard.acceleration = False
        offboard.attitude = False
        offboard.body_rate = False
        offboard.timestamp = self.get_clock().now().nanoseconds // 1000

        self.offboard_pub.publish(offboard)

        # Hold position 3 meters above home
        setpoint = TrajectorySetpoint()

        setpoint.position = [0.0, 0.0, -3.0]

        setpoint.velocity = [float('nan')] * 3
        setpoint.acceleration = [float('nan')] * 3
        setpoint.jerk = [float('nan')] * 3

        setpoint.yaw = 0.0
        setpoint.yawspeed = float('nan')

        setpoint.timestamp = self.get_clock().now().nanoseconds // 1000

        self.setpoint_pub.publish(setpoint)

        self.counter += 1

        # Send OFFBOARD command after some setpoints
        if self.counter == 20:
            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                param1=1.0,
                param2=6.0
            )

            self.get_logger().info('🟢 Requesting OFFBOARD mode')

        # Arm after OFFBOARD request
        if self.counter == 30:
            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                param1=1.0
            )

            self.get_logger().info('🚁 ARM command sent')

    def send_command(self, command, param1=0.0, param2=0.0):

        msg = VehicleCommand()

        msg.command = command
        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True
        msg.timestamp = self.get_clock().now().nanoseconds // 1000

        self.command_pub.publish(msg)


def main(args=None):

    rclpy.init(args=args)

    node = HoverTest()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from px4_msgs.msg import OffboardControlMode, TrajectorySetpoint


class CoordinateTest(Node):

    def __init__(self):
        super().__init__('coordinate_test')

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

        self.step = 0
        self.counter = 0

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('🧭 COORDINATE TEST STARTED')
        self.get_logger().info('⚠️ SETPOINT TEST ONLY')
        self.get_logger().info('⚠️ DO NOT ARM')

    def control_loop(self):

        self.counter += 1

        # Publish offboard mode
        mode = OffboardControlMode()
        mode.position = True
        mode.timestamp = self.get_clock().now().nanoseconds // 1000

        self.offboard_pub.publish(mode)

        # Every 50 cycles = 5 seconds
        if self.counter >= 50:
            self.counter = 0
            self.step += 1

        target = TrajectorySetpoint()

        # Keep altitude at -3 m
        target.position[2] = -3.0

        if self.step == 0:
            # HOME
            target.position[0] = 0.0
            target.position[1] = 0.0

        elif self.step == 1:
            # +X
            target.position[0] = 0.5
            target.position[1] = 0.0

        elif self.step == 2:
            # HOME
            target.position[0] = 0.0
            target.position[1] = 0.0

        elif self.step == 3:
            # +Y
            target.position[0] = 0.0
            target.position[1] = 0.5

        else:
            # HOME
            target.position[0] = 0.0
            target.position[1] = 0.0

        target.yaw = 0.0
        target.timestamp = self.get_clock().now().nanoseconds // 1000

        self.setpoint_pub.publish(target)

        self.get_logger().info(
            f'🧭 STEP={self.step} | '
            f'X={target.position[0]:+.1f} | '
            f'Y={target.position[1]:+.1f} | '
            f'Z={target.position[2]:+.1f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = CoordinateTest()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

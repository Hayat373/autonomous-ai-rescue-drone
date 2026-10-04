import rclpy
from rclpy.node import Node

from px4_msgs.msg import VehicleOdometry

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)


class PositionReader(Node):

    def __init__(self):
        super().__init__('position_reader')

        # QoS settings matching the PX4 odometry publisher
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        self.subscription = self.create_subscription(
            VehicleOdometry,
            '/fmu/out/vehicle_odometry',
            self.odometry_callback,
            qos_profile
        )

        self.get_logger().info(
            '🚁 Rescue Drone Position Reader Started'
        )

    def odometry_callback(self, msg):

        x = msg.position[0]
        y = msg.position[1]
        z = msg.position[2]

        self.get_logger().info(
            f'📍 Drone Position → '
            f'X: {x:.2f} | '
            f'Y: {y:.2f} | '
            f'Z: {z:.2f}'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PositionReader()

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

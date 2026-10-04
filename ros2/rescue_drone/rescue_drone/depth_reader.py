import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import numpy as np


class DepthReader(Node):

    def __init__(self):
        super().__init__('depth_reader')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            10
        )

        self.get_logger().info('🌊 Depth Reader Started')
        self.get_logger().info('📏 Looking for valid depth...')

    def depth_callback(self, msg):

        try:
            depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='32FC1'
            )

            height, width = depth_image.shape

            # Center of image
            cx = width // 2
            cy = height // 2

            # Look at a 40x40 area around the center
            region = depth_image[
                cy - 20:cy + 20,
                cx - 20:cx + 20
            ]

            # Keep only valid depth values
            valid_depths = region[
                np.isfinite(region) &
                (region > 0.1) &
                (region < 100.0)
            ]

            if len(valid_depths) == 0:
                self.get_logger().warning(
                    '⚠️ No valid depth in center area'
                )
                return

            # Use median to reduce noise
            depth = float(np.median(valid_depths))

            self.get_logger().info(
                f'📏 CENTER DISTANCE = {depth:.2f} meters '
                f'| valid pixels = {len(valid_depths)}'
            )

        except Exception as e:
            self.get_logger().error(
                f'Depth error: {e}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = DepthReader()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
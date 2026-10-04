import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point, Vector3
from cv_bridge import CvBridge

import numpy as np


class PersonDistance(Node):

    def __init__(self):
        super().__init__('person_distance')

        self.bridge = CvBridge()

        self.person_x = None
        self.person_y = None
        self.person_confidence = None

        self.depth_image = None

        # YOLO person center
        self.person_sub = self.create_subscription(
            Point,
            '/rescue_drone/person_center',
            self.person_callback,
            10
        )

        # Depth camera
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            10
        )

        # Final target information
        self.target_pub = self.create_publisher(
            Vector3,
            '/rescue_drone/rescue_target',
            10
        )

        self.timer = self.create_timer(
            0.5,
            self.calculate_distance
        )

        self.get_logger().info(
            '📏 PERSON DISTANCE NODE STARTED'
        )

        self.get_logger().info(
            '🎯 Waiting for YOLO person + depth data...'
        )

    def person_callback(self, msg):

        self.person_x = int(msg.x)
        self.person_y = int(msg.y)
        self.person_confidence = float(msg.z)

    def depth_callback(self, msg):

        try:

            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='32FC1'
            )

        except Exception as e:

            self.get_logger().error(
                f'❌ Depth conversion error: {e}'
            )

    def calculate_distance(self):

        if self.person_x is None:
            return

        if self.depth_image is None:
            return

        height, width = self.depth_image.shape

        # Convert YOLO RGB coordinates
        # 1920x1080 → depth 640x480

        depth_x = int(
            self.person_x * width / 1920.0
        )

        depth_y = int(
            self.person_y * height / 1080.0
        )

        # Keep coordinates inside image

        depth_x = max(
            0,
            min(width - 1, depth_x)
        )

        depth_y = max(
            0,
            min(height - 1, depth_y)
        )

        # Small area around person

        radius = 10

        x1 = max(
            0,
            depth_x - radius
        )

        x2 = min(
            width,
            depth_x + radius
        )

        y1 = max(
            0,
            depth_y - radius
        )

        y2 = min(
            height,
            depth_y + radius
        )

        region = self.depth_image[
            y1:y2,
            x1:x2
        ]

        # Keep valid depth

        valid_depths = region[
            np.isfinite(region) &
            (region > 0.1) &
            (region < 100.0)
        ]

        if len(valid_depths) == 0:

            self.get_logger().warning(
                '⚠️ No valid depth at person'
            )

            return

        # Median gives a more stable measurement

        distance = float(
            np.median(valid_depths)
        )

        # Publish target

        target = Vector3()

        target.x = float(self.person_x)
        target.y = float(self.person_y)
        target.z = float(distance)

        self.target_pub.publish(target)

        self.get_logger().info(
            f'🎯 TARGET | '
            f'x={self.person_x} '
            f'y={self.person_y} '
            f'distance={distance:.2f} m'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PersonDistance()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point, Vector3
from sensor_msgs.msg import Image

from cv_bridge import CvBridge

import numpy as np


class Target3DV2(Node):

    def __init__(self):
        super().__init__('target_3d_v2')

        self.get_logger().info('🎯 V2.5 3D Target Estimator Starting...')
        self.get_logger().info('🚁 DRY-RUN MODE — Drone will NOT move')

        self.bridge = CvBridge()

        self.predicted_x = None
        self.predicted_y = None

        self.velocity_x = 0.0
        self.velocity_y = 0.0

        self.depth_image = None

        # RGB camera resolution
        self.rgb_width = 1920
        self.rgb_height = 1080

        # Depth camera resolution
        self.depth_width = 640
        self.depth_height = 480

        # Camera horizontal center
        self.camera_center_x = self.rgb_width / 2

        # Subscribe to predicted person
        self.prediction_sub = self.create_subscription(
            Point,
            '/rescue_drone/predicted_person_v2',
            self.prediction_callback,
            10
        )

        # Subscribe to person velocity
        self.velocity_sub = self.create_subscription(
            Vector3,
            '/rescue_drone/person_velocity_v2',
            self.velocity_callback,
            10
        )

        # Subscribe to depth camera
        self.depth_sub = self.create_subscription(
            Image,
            '/depth_camera',
            self.depth_callback,
            10
        )

        # Run estimator
        self.timer = self.create_timer(
            0.5,
            self.estimate_target
        )

        # Publish 3D target
        self.target_pub = self.create_publisher(
            Vector3,
            '/rescue_drone/target_3d_v2',
            10
        )

        self.get_logger().info(
            '🎯 Waiting for prediction + depth data...'
        )

    def prediction_callback(self, msg):

        self.predicted_x = msg.x
        self.predicted_y = msg.y

    def velocity_callback(self, msg):

        self.velocity_x = msg.x
        self.velocity_y = msg.y

    def depth_callback(self, msg):

        try:
            self.depth_image = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='passthrough'
            )

        except Exception as e:

            self.get_logger().error(
                f'❌ Depth conversion error: {repr(e)}'
            )

    def estimate_target(self):

        if self.predicted_x is None:
            self.get_logger().info(
                '🔍 Waiting for predicted person...'
            )
            return

        if self.depth_image is None:
            self.get_logger().info(
                '📷 Waiting for depth image...'
            )
            return

        # Convert RGB coordinates to depth coordinates
        depth_x = int(
            self.predicted_x
            * self.depth_width
            / self.rgb_width
        )

        depth_y = int(
            self.predicted_y
            * self.depth_height
            / self.rgb_height
        )

        # Keep coordinates inside image
        depth_x = max(
            0,
            min(self.depth_width - 1, depth_x)
        )

        depth_y = max(
            0,
            min(self.depth_height - 1, depth_y)
        )

        # Small region around predicted target
        radius = 10

        x1 = max(0, depth_x - radius)
        x2 = min(self.depth_width, depth_x + radius)

        y1 = max(0, depth_y - radius)
        y2 = min(self.depth_height, depth_y + radius)

        region = self.depth_image[y1:y2, x1:x2]

        if region.size == 0:
            return

        # Keep only valid depth values
        valid = region[
            np.isfinite(region)
            & (region > 0.2)
            & (region < 50.0)
        ]

        if valid.size < 20:

            self.get_logger().warn(
                '⚠️ Not enough valid depth pixels'
            )

            return

        # Median is more stable than single pixel
        distance = float(
            np.median(valid)
        )

        # Horizontal camera error
        horizontal_error = (
            self.predicted_x
            - self.camera_center_x
        )

        # Approximate horizontal angle
        # Camera horizontal FOV is approximately 90 degrees
        horizontal_angle = (
            horizontal_error
            / self.camera_center_x
        ) * 45.0

        # Create 3D target message
        target = Vector3()

        # x = horizontal image error
        target.x = float(horizontal_error)

        # y = predicted vertical pixel position
        target.y = float(self.predicted_y)

        # z = estimated distance
        target.z = distance

        self.target_pub.publish(target)

        # Determine direction
        if horizontal_error < -150:

            direction = 'LEFT'

        elif horizontal_error > 150:

            direction = 'RIGHT'

        else:

            direction = 'CENTER'

        # Determine distance state
        if distance > 8.0:

            distance_state = 'FAR'

        elif distance > 4.0:

            distance_state = 'MEDIUM'

        elif distance > 2.5:

            distance_state = 'NEAR'

        else:

            distance_state = 'DELIVERY RANGE'

        self.get_logger().info(
            '\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            '🎯 V2.5 3D RESCUE TARGET\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━\n'
            f'🔮 Predicted pixel: '
            f'({self.predicted_x:.0f}, {self.predicted_y:.0f})\n'
            f'📷 Depth pixel: '
            f'({depth_x}, {depth_y})\n'
            f'📏 Distance: '
            f'{distance:.2f} m\n'
            f'↔️ Horizontal error: '
            f'{horizontal_error:+.0f} px\n'
            f'📐 Approx angle: '
            f'{horizontal_angle:+.1f}°\n'
            f'🎯 Direction: {direction}\n'
            f'📍 Distance state: {distance_state}\n'
            f'🏃 Person velocity: '
            f'({self.velocity_x:+.1f}, '
            f'{self.velocity_y:+.1f}) px/s\n'
            '━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━'
        )


def main(args=None):

    rclpy.init(args=args)

    node = Target3DV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

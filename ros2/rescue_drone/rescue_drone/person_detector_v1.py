import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from geometry_msgs.msg import Point

from ultralytics import YOLO

import cv2
import numpy as np

from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy
)


class PersonDetector(Node):

    def __init__(self):
        super().__init__('person_detector')

        self.get_logger().info(
            '🤖 YOLO11 Rescue Person Tracker Starting...'
        )

        # -----------------------------------------
        # OpenCV <-> ROS 2
        # -----------------------------------------

        self.bridge = CvBridge()

        # Load YOLO11 nano
        self.model = YOLO('yolo11n.pt')

        # -----------------------------------------
        # Camera topic
        # -----------------------------------------

        camera_topic = (
            '/world/default/model/x500_depth_0/'
            'link/camera_link/sensor/IMX214/image'
        )

        # -----------------------------------------
        # Annotated image publisher
        #
        # BEST_EFFORT works better with
        # sensor/image viewers such as RQT.
        # -----------------------------------------

        image_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.image_pub = self.create_publisher(
            Image,
            '/rescue_drone/detection_image',
            image_qos
        )

        self.person_center_pub = self.create_publisher(
            Point,
            '/rescue_drone/person_center',
            10
        )

        # -----------------------------------------
        # Camera subscription
        # -----------------------------------------

        camera_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        self.subscription = self.create_subscription(
            Image,
            camera_topic,
            self.image_callback,
            camera_qos
        )

        self.frame_count = 0

        self.get_logger().info(
            '📷 Waiting for camera images...'
        )

        self.get_logger().info(
            '🎯 Looking for people...'
        )

    def image_callback(self, msg):

        try:

            # =========================================
            # 1. ROS IMAGE -> OpenCV
            # =========================================

            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            frame = np.asarray(
                frame,
                dtype=np.uint8
            )

            self.frame_count += 1

            # =========================================
            # 2. CREATE SMALL IMAGE FOR YOLO
            #
            # Original camera:
            # 1920 x 1080
            #
            # YOLO:
            # 640 x 360
            #
            # This makes CPU inference much faster.
            # =========================================

            original_height, original_width = frame.shape[:2]

            inference_width = 640
            inference_height = 360

            small_frame = cv2.resize(
                frame,
                (
                    inference_width,
                    inference_height
                )
            )

            # =========================================
            # 3. YOLO PERSON DETECTION
            # =========================================

            results = self.model(
                small_frame,
                imgsz=640,
                conf=0.5,
                classes=[0],
                verbose=False
            )

            person_found = False

            # =========================================
            # 4. PROCESS DETECTIONS
            # =========================================

            for result in results:

                for box in result.boxes:

                    confidence = float(
                        box.conf[0]
                    )

                    # Coordinates from 640x360 image
                    sx1, sy1, sx2, sy2 = map(
                        int,
                        box.xyxy[0]
                    )

                    # ---------------------------------
                    # Convert coordinates back to
                    # original 1920x1080 image
                    # ---------------------------------

                    scale_x = (
                        original_width /
                        inference_width
                    )

                    scale_y = (
                        original_height /
                        inference_height
                    )

                    x1 = int(sx1 * scale_x)
                    y1 = int(sy1 * scale_y)
                    x2 = int(sx2 * scale_x)
                    y2 = int(sy2 * scale_y)

                    # Keep coordinates inside image
                    x1 = max(
                        0,
                        min(x1, original_width - 1)
                    )

                    y1 = max(
                        0,
                        min(y1, original_height - 1)
                    )

                    x2 = max(
                        0,
                        min(x2, original_width - 1)
                    )

                    y2 = max(
                        0,
                        min(y2, original_height - 1)
                    )

                    # ---------------------------------
                    # Person center
                    # ---------------------------------

                    center_x = int(
                        (x1 + x2) / 2
                    )

                    center_y = int(
                        (y1 + y2) / 2
                    )

                    center_msg = Point()
                    center_msg.x = float(center_x)          
                    center_msg.y = float(center_y)
                    center_msg.z = float(confidence)

                    self.person_center_pub.publish(center_msg)

                    person_found = True

                    # =================================
                    # GREEN BOUNDING BOX
                    # =================================

                    cv2.rectangle(
                        frame,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        5
                    )

                    # =================================
                    # RED CENTER POINT
                    # =================================

                    cv2.circle(
                        frame,
                        (center_x, center_y),
                        12,
                        (0, 0, 255),
                        -1
                    )

                    # =================================
                    # PERSON LABEL
                    # =================================

                    cv2.putText(
                        frame,
                        f'PERSON {confidence:.2f}',
                        (
                            x1,
                            max(y1 - 15, 35)
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        1.2,
                        (0, 255, 0),
                        3
                    )

                    # =================================
                    # CENTER COORDINATES
                    # =================================

                    center_text = (
                        f'CENTER '
                        f'({center_x}, {center_y})'
                    )

                    cv2.putText(
                        frame,
                        center_text,
                        (
                            x1,
                            min(
                                y2 + 40,
                                original_height - 10
                            )
                        ),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.9,
                        (0, 255, 255),
                        2
                    )

                    # =================================
                    # LOG DETECTION
                    # =================================

                    self.get_logger().info(
                        f'👤 PERSON TRACKED | '
                        f'confidence={confidence:.2f} | '
                        f'center=({center_x},{center_y})'
                    )

                    # Track only the first person
                    break

                if person_found:
                    break

            # =========================================
            # 5. STATUS TEXT
            # =========================================

            if person_found:

                cv2.putText(
                    frame,
                    'RESCUE TARGET FOUND',
                    (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.3,
                    (0, 255, 0),
                    4
                )

            else:

                cv2.putText(
                    frame,
                    'SEARCHING FOR PERSON...',
                    (40, 70),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.3,
                    (0, 0, 255),
                    4
                )

            # =========================================
            # 6. MAKE IMAGE CONTIGUOUS
            # =========================================

            frame = np.ascontiguousarray(
                frame,
                dtype=np.uint8
            )

            # =========================================
            # 7. CREATE ROS IMAGE MANUALLY
            #
            # This avoids the KeyError(16) issue.
            # =========================================

            output_msg = Image()

            output_msg.header = msg.header

            output_msg.height = frame.shape[0]
            output_msg.width = frame.shape[1]

            output_msg.encoding = 'bgr8'

            output_msg.is_bigendian = 0

            output_msg.step = (
                frame.shape[1] * 3
            )

            output_msg.data = frame.tobytes()

            # =========================================
            # 8. PUBLISH ANNOTATED IMAGE
            # =========================================

            self.image_pub.publish(
                output_msg
            )

        except Exception as e:

            self.get_logger().error(
                f'❌ Detection error: {repr(e)}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = PersonDetector()

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
import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Image
from geometry_msgs.msg import Point

from cv_bridge import CvBridge

from ultralytics import YOLO

import cv2
import traceback


class PersonTrackerV2(Node):

    def __init__(self):
        super().__init__('person_tracker_v2')

        self.get_logger().info(
            '🚁 V2 Person Tracker Starting...'
        )

        self.get_logger().info(
            '🎯 YOLO11 + ByteTrack'
        )

        self.bridge = CvBridge()

        # YOLO model
        self.model = YOLO('yolo11n.pt')

        # Prevent overlapping camera callbacks
        self.processing = False

        # Camera
        self.image_sub = self.create_subscription(
            Image,
            '/world/default/model/x500_depth_0/link/camera_link/sensor/IMX214/image',
            self.image_callback,
            10
        )

        # Tracked person center
        self.person_center_pub = self.create_publisher(
            Point,
            '/rescue_drone/person_center_v2',
            10
        )

        # Annotated image
        self.image_pub = self.create_publisher(
            Image,
            '/rescue_drone/tracking_image_v2',
            10
        )

        self.get_logger().info(
            '📷 Waiting for camera images...'
        )

    def image_callback(self, msg):

        # Skip this frame if previous frame is still processing
        if self.processing:
            return

        self.processing = True

        try:

            # Convert ROS image to OpenCV
            frame = self.bridge.imgmsg_to_cv2(
                msg,
                desired_encoding='bgr8'
            )

            original_height, original_width = frame.shape[:2]

            # Resize for CPU performance
            small_frame = cv2.resize(
                frame,
                (640, 360)
            )

            # YOLO + ByteTrack
            results = self.model.track(
                source=small_frame,
                persist=True,
                tracker='bytetrack.yaml',
                classes=[0],
                conf=0.35,
                verbose=False
            )

            annotated = small_frame.copy()

            found_person = False

            if results and results[0].boxes is not None:

                boxes = results[0].boxes

                for box in boxes:

                    if box.cls is None:
                        continue

                    cls = int(box.cls[0])

                    # Person class
                    if cls != 0:
                        continue

                    confidence = float(box.conf[0])

                    # Bounding box
                    x1, y1, x2, y2 = map(
                        int,
                        box.xyxy[0].tolist()
                    )

                    # Tracking ID
                    track_id = -1

                    if box.id is not None:
                        track_id = int(box.id[0])

                    # Center
                    center_x = int((x1 + x2) / 2)
                    center_y = int((y1 + y2) / 2)

                    # Convert to original 1920x1080 coordinates
                    scale_x = original_width / 640.0
                    scale_y = original_height / 360.0

                    original_center_x = int(
                        center_x * scale_x
                    )

                    original_center_y = int(
                        center_y * scale_y
                    )

                    # Draw bounding box
                    cv2.rectangle(
                        annotated,
                        (x1, y1),
                        (x2, y2),
                        (0, 255, 0),
                        2
                    )

                    # Center point
                    cv2.circle(
                        annotated,
                        (center_x, center_y),
                        5,
                        (0, 0, 255),
                        -1
                    )

                    # Label
                    label = (
                        f'PERSON ID:{track_id} '
                        f'{confidence:.2f}'
                    )

                    cv2.putText(
                        annotated,
                        label,
                        (x1, max(y1 - 10, 20)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.6,
                        (0, 255, 0),
                        2
                    )

                    # Center text
                    cv2.putText(
                        annotated,
                        f'CENTER ({original_center_x}, '
                        f'{original_center_y})',
                        (x1, min(y2 + 20, 350)),
                        cv2.FONT_HERSHEY_SIMPLEX,
                        0.5,
                        (255, 255, 0),
                        2
                    )

                    found_person = True

                    # Publish target
                    center_msg = Point()

                    center_msg.x = float(
                        original_center_x
                    )

                    center_msg.y = float(
                        original_center_y
                    )

                    center_msg.z = float(
                        confidence
                    )

                    self.person_center_pub.publish(
                        center_msg
                    )

                    self.get_logger().info(
                        f'👤 PERSON TRACKED | '
                        f'ID={track_id} | '
                        f'confidence={confidence:.2f} | '
                        f'center='
                        f'({original_center_x},'
                        f'{original_center_y})'
                    )

                    # Track only first person
                    break

            if not found_person:

                self.get_logger().info(
                    '🔍 Searching for person...'
                )

            # Publish annotated image manually
            annotated_msg = Image()

            annotated_msg.height = annotated.shape[0]
            annotated_msg.width = annotated.shape[1]
            annotated_msg.encoding = 'bgr8'
            annotated_msg.is_bigendian = 0
            annotated_msg.step = annotated.shape[1] * 3
            annotated_msg.data = annotated.tobytes()

            self.image_pub.publish(annotated_msg)

        except Exception as e:

            self.get_logger().error(
                f'❌ Tracker error: {repr(e)}'
            )

            self.get_logger().error(
                traceback.format_exc()
            )

        finally:

            self.processing = False


def main(args=None):

    rclpy.init(args=args)

    node = PersonTrackerV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

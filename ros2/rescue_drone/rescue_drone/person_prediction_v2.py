import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Point
from geometry_msgs.msg import Vector3


class PersonPredictionV2(Node):

    def __init__(self):
        super().__init__('person_prediction_v2')

        self.get_logger().info(
            '🔮 V2 Person Prediction Starting...'
        )

        # Current position
        self.current_x = None
        self.current_y = None

        # Current velocity
        self.velocity_x = 0.0
        self.velocity_y = 0.0

        # How far into the future we predict
        self.prediction_time = 2.0

        # Position subscriber
        self.position_sub = self.create_subscription(
            Point,
            '/rescue_drone/person_center_v2',
            self.position_callback,
            10
        )

        # Velocity subscriber
        self.velocity_sub = self.create_subscription(
            Vector3,
            '/rescue_drone/person_velocity_v2',
            self.velocity_callback,
            10
        )

        # Predicted position publisher
        self.prediction_pub = self.create_publisher(
            Point,
            '/rescue_drone/predicted_person_v2',
            10
        )

        # Calculate prediction periodically
        self.timer = self.create_timer(
            0.2,
            self.predict
        )

        self.get_logger().info(
            '🎯 Waiting for person position and velocity...'
        )

    def position_callback(self, msg):

        self.current_x = msg.x
        self.current_y = msg.y

    def velocity_callback(self, msg):

        self.velocity_x = msg.x
        self.velocity_y = msg.y

    def predict(self):

        # Need both position and velocity
        if self.current_x is None:
            return

        # Future position
        predicted_x = (
            self.current_x
            + self.velocity_x * self.prediction_time
        )

        predicted_y = (
            self.current_y
            + self.velocity_y * self.prediction_time
        )

        # Keep prediction inside camera image
        predicted_x = max(
            0.0,
            min(1920.0, predicted_x)
        )

        predicted_y = max(
            0.0,
            min(1080.0, predicted_y)
        )

        # Publish predicted position
        prediction_msg = Point()

        prediction_msg.x = predicted_x
        prediction_msg.y = predicted_y
        prediction_msg.z = self.prediction_time

        self.prediction_pub.publish(
            prediction_msg
        )

        self.get_logger().info(
            f'🔮 PREDICTION | '
            f'current=({self.current_x:.0f},'
            f'{self.current_y:.0f}) | '
            f'velocity=({self.velocity_x:+.1f},'
            f'{self.velocity_y:+.1f}) | '
            f'future=({predicted_x:.0f},'
            f'{predicted_y:.0f}) | '
            f'lookahead={self.prediction_time:.1f}s'
        )


def main(args=None):

    rclpy.init(args=args)

    node = PersonPredictionV2()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

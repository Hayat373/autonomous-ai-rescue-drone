import math
import rclpy

from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleAttitude,
    VehicleStatus,
)


class RescueMission(Node):

    def __init__(self):
        super().__init__('rescue_mission')

        # --------------------------------------------------
        # STATE
        # --------------------------------------------------

        self.person_x = None
        self.person_y = None
        self.person_distance = None
        self.last_target_time = None

        self.pos_x = 0.0
        self.pos_y = 0.0
        self.pos_z = 0.0

        self.yaw = 0.0

        self.armed = False
        self.offboard = False

        self.home_x = None
        self.home_y = None

        self.mission_state = 'TAKEOFF'

        self.counter = 0
        self.delivery_counter = 0

        # --------------------------------------------------
        # PX4 QoS
        # --------------------------------------------------

        px4_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )

        # --------------------------------------------------
        # PUBLISHERS
        # --------------------------------------------------

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

        # --------------------------------------------------
        # SUBSCRIBERS
        # --------------------------------------------------

        self.target_sub = self.create_subscription(
            Vector3,
            '/rescue_drone/rescue_target',
            self.target_callback,
            10
        )

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            px4_qos
        )

        self.attitude_sub = self.create_subscription(
            VehicleAttitude,
            '/fmu/out/vehicle_attitude',
            self.attitude_callback,
            px4_qos
        )

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            px4_qos
        )

        # --------------------------------------------------
        # TIMER
        # --------------------------------------------------

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        self.get_logger().info('==========================================')
        self.get_logger().info('🚁 AUTONOMOUS AI RESCUE MISSION')
        self.get_logger().info('==========================================')
        self.get_logger().info('⚠️ REAL PX4 COMMANDS ENABLED')
        self.get_logger().info('🚁 TAKEOFF → APPROACH → DELIVERY → HOME')
        self.get_logger().info('==========================================')

    # ======================================================
    # TARGET
    # ======================================================

    def target_callback(self, msg):

        self.person_x = msg.x
        self.person_y = msg.y
        self.person_distance = msg.z

        self.last_target_time = self.get_clock().now()

    # ======================================================
    # POSITION
    # ======================================================

    def position_callback(self, msg):

        self.pos_x = msg.x
        self.pos_y = msg.y
        self.pos_z = msg.z

        if self.home_x is None:
            self.home_x = self.pos_x
            self.home_y = self.pos_y

    # ======================================================
    # ATTITUDE
    # ======================================================

    def attitude_callback(self, msg):

        q = msg.q

        # PX4 quaternion:
        # q[0] = w
        # q[1] = x
        # q[2] = y
        # q[3] = z

        w = q[0]
        x = q[1]
        y = q[2]
        z = q[3]

        self.yaw = math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z)
        )

    # ======================================================
    # STATUS
    # ======================================================

    def status_callback(self, msg):

        # PX4 arming state
        self.armed = (
            msg.arming_state ==
            VehicleStatus.ARMING_STATE_ARMED
        )

        self.offboard = (
            msg.nav_state ==
            VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    # ======================================================
    # OFFBOARD
    # ======================================================

    def publish_offboard_mode(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        self.offboard_pub.publish(msg)

    # ======================================================
    # POSITION SETPOINT
    # ======================================================

    def publish_position(self, x, y, z):

        msg = TrajectorySetpoint()

        msg.position[0] = float(x)
        msg.position[1] = float(y)
        msg.position[2] = float(z)

        msg.yaw = float(self.yaw)

        self.setpoint_pub.publish(msg)

    # ======================================================
    # COMMAND
    # ======================================================

    def command(self, command, param1=0.0, param2=0.0):

        msg = VehicleCommand()

        msg.command = command

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.command_pub.publish(msg)

    # ======================================================
    # ARM
    # ======================================================

    def arm(self):

        self.command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0
        )

        self.get_logger().info('🔓 ARM COMMAND SENT')

    # ======================================================
    # OFFBOARD COMMAND
    # ======================================================

    def enable_offboard(self):

        self.command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

        self.get_logger().info(
            '🎮 OFFBOARD COMMAND SENT'
        )

    # ======================================================
    # LAND
    # ======================================================

    def land(self):

        self.command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND
        )

        self.get_logger().info(
            '🛬 LAND COMMAND SENT'
        )

    # ======================================================
    # CONTROL LOOP
    # ======================================================

    def control_loop(self):

        self.publish_offboard_mode()

        self.counter += 1

        # --------------------------------------------------
        # PHASE 1: SEND SETPOINTS BEFORE OFFBOARD
        # --------------------------------------------------

        if self.counter < 20:

            self.publish_position(
                self.home_x if self.home_x is not None else 0.0,
                self.home_y if self.home_y is not None else 0.0,
                -3.0
            )

            return

        # --------------------------------------------------
        # PHASE 2: OFFBOARD
        # --------------------------------------------------

        if self.counter == 20:

            self.enable_offboard()

            return

        # --------------------------------------------------
        # PHASE 3: ARM
        # --------------------------------------------------

        if self.counter == 30:

            self.arm()

            return

        # --------------------------------------------------
        # SAFETY: HOME POSITION
        # --------------------------------------------------

        if self.home_x is None:

            return

        # --------------------------------------------------
        # TAKEOFF
        # --------------------------------------------------

        if self.mission_state == 'TAKEOFF':

            self.publish_position(
                self.home_x,
                self.home_y,
                -3.0
            )

            altitude = -self.pos_z

            self.get_logger().info(
                f'🚁 TAKEOFF | altitude={altitude:.2f}m'
            )

            if altitude >= 2.7:

                self.mission_state = 'SEARCH'

                self.get_logger().info(
                    '🔎 TAKEOFF COMPLETE → SEARCHING'
                )

            return

        # --------------------------------------------------
        # SEARCH
        # --------------------------------------------------

        if self.mission_state == 'SEARCH':

            self.publish_position(
                self.pos_x,
                self.pos_y,
                -3.0
            )

            if self.person_distance is not None:

                self.mission_state = 'APPROACH'

                self.get_logger().info(
                    '👤 PERSON FOUND → APPROACH'
                )

            return

        # --------------------------------------------------
        # APPROACH
        # --------------------------------------------------

        if self.mission_state == 'APPROACH':

            if self.person_x is None:
                return

            if self.person_distance is None:
                return

            # ----------------------------------------------
            # TARGET LOST SAFETY
            # ----------------------------------------------

            if self.last_target_time is not None:

                age = (
                    self.get_clock().now()
                    - self.last_target_time
                ).nanoseconds / 1e9

                if age > 2.0:

                    self.get_logger().warn(
                        '⚠️ TARGET LOST → HOLD'
                    )

                    self.publish_position(
                        self.pos_x,
                        self.pos_y,
                        -3.0
                    )

                    return

            # ----------------------------------------------
            # IMAGE ERROR
            # ----------------------------------------------

            image_center = 960.0

            error_x = (
                self.person_x -
                image_center
            )

            # ----------------------------------------------
            # SMALL YAW CORRECTION
            # ----------------------------------------------

            if abs(error_x) > 150:

                if error_x > 0:

                    yaw_change = -0.10

                    direction = 'RIGHT'

                else:

                    yaw_change = 0.10

                    direction = 'LEFT'

                target_yaw = self.yaw + yaw_change

                # keep yaw in -pi → pi
                target_yaw = math.atan2(
                    math.sin(target_yaw),
                    math.cos(target_yaw)
                )

                msg = TrajectorySetpoint()

                msg.position[0] = float(self.pos_x)
                msg.position[1] = float(self.pos_y)
                msg.position[2] = -3.0

                msg.yaw = float(target_yaw)

                self.setpoint_pub.publish(msg)

                self.get_logger().info(
                    f'↪️ TURN {direction} | '
                    f'pixel error={error_x:+.0f}'
                )

                return

            # ----------------------------------------------
            # PERSON CENTERED
            # ----------------------------------------------

            # DELIVERY RANGE

            if self.person_distance <= 2.5:

                self.mission_state = 'DELIVERY'

                self.delivery_counter = 0

                self.get_logger().info(
                    '📦 DELIVERY RANGE REACHED'
                )

                return

            # ----------------------------------------------
            # MOVE FORWARD
            # ----------------------------------------------

            if self.person_distance > 6.0:

                forward = 0.50

            elif self.person_distance > 3.0:

                forward = 0.30

            else:

                forward = 0.15

            # Body forward → NED
            vx = forward * math.cos(self.yaw)
            vy = forward * math.sin(self.yaw)

            target_x = self.pos_x + vx
            target_y = self.pos_y + vy

            self.publish_position(
                target_x,
                target_y,
                -3.0
            )

            self.get_logger().info(
                f'🚁 APPROACH | '
                f'distance={self.person_distance:.2f}m | '
                f'forward={forward:.2f}m/s'
            )

            return

        # --------------------------------------------------
        # DELIVERY
        # --------------------------------------------------

        if self.mission_state == 'DELIVERY':

            self.publish_position(
                self.pos_x,
                self.pos_y,
                -3.0
            )

            self.delivery_counter += 1

            if self.delivery_counter % 10 == 0:

                self.get_logger().info(
                    '📦 EMERGENCY PACKAGE DELIVERED ✓'
                )

            if self.delivery_counter >= 50:

                self.mission_state = 'RETURN_HOME'

                self.get_logger().info(
                    '🏠 RETURNING HOME'
                )

            return

        # --------------------------------------------------
        # RETURN HOME
        # --------------------------------------------------

        if self.mission_state == 'RETURN_HOME':

            self.publish_position(
                self.home_x,
                self.home_y,
                -3.0
            )

            distance_home = math.sqrt(
                (self.pos_x - self.home_x) ** 2 +
                (self.pos_y - self.home_y) ** 2
            )

            self.get_logger().info(
                f'🏠 RETURN HOME | '
                f'distance={distance_home:.2f}m'
            )

            if distance_home < 0.8:

                self.mission_state = 'LAND'

                self.get_logger().info(
                    '🛬 HOME REACHED → LAND'
                )

            return

        # --------------------------------------------------
        # LAND
        # --------------------------------------------------

        if self.mission_state == 'LAND':

            self.land()

            self.mission_state = 'LANDED'

            return

        # --------------------------------------------------
        # LANDED
        # --------------------------------------------------

        if self.mission_state == 'LANDED':

            return


def main(args=None):

    rclpy.init(args=args)

    node = RescueMission()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        pass

    finally:

        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
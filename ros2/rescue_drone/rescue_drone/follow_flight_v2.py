import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import Vector3
from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleStatus,
    VehicleLocalPosition,
)


class FollowFlightV2(Node):
    def __init__(self):
        super().__init__('follow_flight_v2')

        self.get_logger().warn(
            'V2 FOLLOW TEST — Gazebo simulation only; slow movement enabled'
        )

        self.feedback_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.BEST_EFFORT,
        )

        self.position = None
        self.status = None
        self.target = None

        self.home_x = None
        self.home_y = None
        self.home_z = None

        self.takeoff_z = None
        self.phase = 'WAIT'
        self.phase_started = self.get_clock().now()
        self.start_time = None
        self.last_target_time = None
        self.last_mode_request = None
        self.last_arm_request = None

        self.offboard_pub = self.create_publisher(
            OffboardControlMode, '/fmu/in/offboard_control_mode', 10
        )
        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint, '/fmu/in/trajectory_setpoint', 10
        )
        self.command_pub = self.create_publisher(
            VehicleCommand, '/fmu/in/vehicle_command', 10
        )

        self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            self.feedback_qos,
        )
        self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            self.feedback_qos,
        )
        self.create_subscription(
            Vector3,
            '/rescue_drone/target_3d_v2',
            self.target_callback,
            10,
        )

        self.timer = self.create_timer(0.1, self.control_loop)
        self.get_logger().info(
            'Waiting for PX4 position, vehicle status, and AI target...'
        )

    def now_us(self):
        return self.get_clock().now().nanoseconds // 1000

    def seconds_since(self, stamp):
        return (self.get_clock().now() - stamp).nanoseconds / 1e9

    def position_callback(self, msg):
        self.position = msg

    def status_callback(self, msg):
        self.status = msg

    def target_callback(self, msg):
        if math.isfinite(msg.x) and math.isfinite(msg.z) and msg.z > 0.2:
            self.target = msg
            self.last_target_time = self.get_clock().now()

    def is_armed(self):
        return (
            self.status is not None
            and self.status.arming_state == VehicleStatus.ARMING_STATE_ARMED
        )

    def is_offboard(self):
        return (
            self.status is not None
            and self.status.nav_state == VehicleStatus.NAVIGATION_STATE_OFFBOARD
        )

    def set_phase(self, phase):
        self.phase = phase
        self.phase_started = self.get_clock().now()
        self.get_logger().warn(f'MISSION PHASE: {phase}')

    def publish_mode(self, position=False, velocity=False):
        msg = OffboardControlMode()
        msg.position = position
        msg.velocity = velocity
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.timestamp = self.now_us()
        self.offboard_pub.publish(msg)

    def publish_position_setpoint(self, x, y, z):
        msg = TrajectorySetpoint()
        msg.position = [float(x), float(y), float(z)]
        msg.velocity = [float('nan')] * 3
        msg.acceleration = [float('nan')] * 3
        msg.yaw = 0.0
        msg.timestamp = self.now_us()
        self.setpoint_pub.publish(msg)

    def publish_velocity_setpoint(self, vx, vy):
        msg = TrajectorySetpoint()
        msg.position = [float('nan')] * 3
        msg.velocity = [float(vx), float(vy), 0.0]
        msg.acceleration = [float('nan')] * 3
        msg.yaw = float('nan')
        msg.timestamp = self.now_us()
        self.setpoint_pub.publish(msg)

    def send_command(self, command, param1=0.0, param2=0.0):
        msg = VehicleCommand()
        msg.command = command
        msg.param1 = float(param1)
        msg.param2 = float(param2)
        msg.target_system = 1
        msg.target_component = 1
        msg.source_system = 1
        msg.source_component = 1
        msg.from_external = True
        msg.timestamp = self.now_us()
        self.command_pub.publish(msg)

    def request_offboard(self):
        now = self.get_clock().now()
        if (
            self.last_mode_request is None
            or self.seconds_since(self.last_mode_request) >= 2.0
        ):
            self.send_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE, 1.0, 6.0
            )
            self.last_mode_request = now
            self.get_logger().info('Requested PX4 OFFBOARD mode')

    def request_arm(self):
        now = self.get_clock().now()
        if (
            self.last_arm_request is None
            or self.seconds_since(self.last_arm_request) >= 2.0
        ):
            self.send_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0
            )
            self.last_arm_request = now
            self.get_logger().info('Requested arming')

    def control_loop(self):
        if self.position is None or self.status is None:
            return

        if not self.position.xy_valid or not self.position.z_valid:
            return

        current_x = float(self.position.x)
        current_y = float(self.position.y)
        current_z = float(self.position.z)

        if self.phase == 'WAIT':
            if self.target is None or self.last_target_time is None:
                return
            if self.seconds_since(self.last_target_time) > 3.0:
                return

            self.home_x = current_x
            self.home_y = current_y
            self.home_z = current_z
            self.takeoff_z = self.home_z - 2.0
            self.get_logger().info(
                f'Home recorded: x={self.home_x:.2f}, '
                f'y={self.home_y:.2f}, z={self.home_z:.2f}'
            )
            self.set_phase('PREPARE')

        if self.phase == 'PREPARE':
            self.publish_mode(position=True)
            self.publish_position_setpoint(
                self.home_x, self.home_y, self.takeoff_z
            )

            if self.seconds_since(self.phase_started) >= 3.0:
                self.request_offboard()

            if self.is_offboard():
                self.request_arm()

            if self.is_offboard() and self.is_armed():
                self.start_time = self.get_clock().now()
                self.set_phase('TAKEOFF')

            elif self.seconds_since(self.phase_started) > 20.0:
                self.get_logger().error(
                    'PX4 did not enter armed OFFBOARD mode. '
                    'Stopping the mission; check PX4/QGroundControl.'
                )
                self.set_phase('ABORT')

        elif self.phase == 'TAKEOFF':
            self.publish_mode(position=True)
            self.publish_position_setpoint(
                self.home_x, self.home_y, self.takeoff_z
            )

            self.get_logger().info(
                f'TAKEOFF | z={current_z:.2f} m | '
                f'target z={self.takeoff_z:.2f} m',
                throttle_duration_sec=1.0,
            )

            # In PX4 NED coordinates, negative z is upward.
            if current_z <= self.takeoff_z + 0.35:
                self.set_phase('FOLLOW')
            elif self.seconds_since(self.phase_started) > 35.0:
                self.get_logger().error(
                    'Takeoff timed out. Requesting landing.'
                )
                self.set_phase('LAND')

        elif self.phase == 'FOLLOW':
            if not self.is_armed():
                self.get_logger().error(
                    'Drone unexpectedly disarmed; stopping follow control.'
                )
                self.set_phase('ABORT')
                return

            if not self.is_offboard():
                self.get_logger().error(
                    'OFFBOARD mode lost; stopping follow control.'
                )
                self.set_phase('ABORT')
                return

            self.publish_mode(velocity=True)

            vx = 0.0
            vy = 0.0
            target_age = (
                self.seconds_since(self.last_target_time)
                if self.last_target_time is not None
                else 999.0
            )

            if self.target is not None and target_age <= 3.0:
                error_x = float(self.target.x)
                distance = float(self.target.z)

                # Conservative forward speed based on estimated distance.
                if distance > 8.0:
                    vx = 0.15
                elif distance > 4.5:
                    vx = 0.10
                elif distance > 3.5:
                    vx = 0.05

                # Image-right is provisionally mapped to negative world Y.
                # Verify this direction in Gazebo before continuing.
                if error_x > 150.0:
                    vy = -0.10
                elif error_x < -150.0:
                    vy = 0.10

                self.get_logger().info(
                    f'FOLLOW | distance={distance:.2f} m | '
                    f'image error={error_x:+.0f} px | '
                    f'vx={vx:+.2f} | vy={vy:+.2f}',
                    throttle_duration_sec=1.0,
                )
            else:
                self.get_logger().warn(
                    'AI target stale — hovering',
                    throttle_duration_sec=2.0,
                )

            self.publish_velocity_setpoint(vx, vy)

            if self.start_time is not None:
                if self.seconds_since(self.start_time) >= 20.0:
                    self.set_phase('RETURN_HOME')

        elif self.phase == 'RETURN_HOME':
            self.publish_mode(position=True)
            self.publish_position_setpoint(
                self.home_x, self.home_y, self.takeoff_z
            )

            distance_home = math.hypot(
                current_x - self.home_x, current_y - self.home_y
            )
            self.get_logger().info(
                f'RETURN HOME | horizontal distance={distance_home:.2f} m',
                throttle_duration_sec=1.0,
            )

            if distance_home < 0.35 and abs(current_z - self.takeoff_z) < 0.5:
                self.set_phase('LAND')

            elif self.seconds_since(self.phase_started) > 35.0:
                self.get_logger().error(
                    'Return timed out; requesting landing at current location.'
                )
                self.set_phase('LAND')

        elif self.phase == 'LAND':
            # Send the land command once; do not keep publishing OFFBOARD
            # setpoints that could fight the landing mode.
            self.send_command(VehicleCommand.VEHICLE_CMD_NAV_LAND)
            self.set_phase('LANDING')
            self.get_logger().warn('Landing command sent; monitoring PX4')

        elif self.phase == 'LANDING':
            if not self.is_armed():
                self.get_logger().info('Drone disarmed; landing complete.')
                self.set_phase('LANDED')
            elif self.seconds_since(self.phase_started) > 30.0:
                self.get_logger().warn(
                    'Still armed after landing request. '
                    'Check QGroundControl and land manually if needed.'
                )

        elif self.phase == 'ABORT':
            # Stop issuing movement commands. Use QGroundControl to land
            # if the vehicle remains armed.
            if self.is_armed():
                self.get_logger().error(
                    'ABORT: vehicle still armed. Use QGroundControl Land.'
                )
            else:
                self.get_logger().info('ABORT: vehicle is disarmed.')

        elif self.phase in ('LANDED',):
            pass


def main(args=None):
    rclpy.init(args=args)
    node = FollowFlightV2()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()

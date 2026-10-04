import rclpy
from rclpy.node import Node

from px4_msgs.msg import (
    VehicleCommand,
    OffboardControlMode,
    TrajectorySetpoint
)

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy


class FlightController(Node):

    def __init__(self):
        super().__init__('flight_controller')

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=10
        )

        # Publishers
        self.offboard_control_mode_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            qos_profile
        )

        self.trajectory_setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            qos_profile
        )

        self.vehicle_command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            qos_profile
        )

        # 10 Hz control loop
        self.timer = self.create_timer(0.1, self.control_loop)

        # Mission state
        self.counter = 0
        self.armed = False
        self.offboard_enabled = False

        # Current mission waypoint
        self.current_waypoint = 0

        # How long we have stayed near a waypoint
        self.waypoint_timer = 0

        self.get_logger().info(
            '🚁 Autonomous Waypoint Controller Started'
        )

    # ---------------------------------------------------------
    # PX4 timestamp
    # ---------------------------------------------------------

    def timestamp(self):
        return int(
            self.get_clock().now().nanoseconds / 1000
        )

    # ---------------------------------------------------------
    # Offboard control mode
    # ---------------------------------------------------------

    def publish_offboard_control_mode(self):

        msg = OffboardControlMode()

        msg.timestamp = self.timestamp()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False
        msg.thrust_and_torque = False
        msg.direct_actuator = False

        self.offboard_control_mode_pub.publish(msg)

    # ---------------------------------------------------------
    # Waypoint selection
    # ---------------------------------------------------------

    def get_waypoint(self):

        # Waypoint 0:
        # Takeoff and hover at home
        if self.current_waypoint == 0:
            return 0.0, 0.0, -3.0

        # Waypoint 1:
        # Fly 5 meters forward
        elif self.current_waypoint == 1:
            return 5.0, 0.0, -3.0

        # Waypoint 2:
        # Fly 5 meters sideways
        elif self.current_waypoint == 2:
            return 5.0, 5.0, -3.0

        # Waypoint 3:
        # Return home
        elif self.current_waypoint == 3:
            return 0.0, 0.0, -3.0

        # Waypoint 4:
        # Stay at home
        else:
            return 0.0, 0.0, -3.0

    # ---------------------------------------------------------
    # Publish trajectory
    # ---------------------------------------------------------

    def publish_position_setpoint(self):

        x, y, z = self.get_waypoint()

        msg = TrajectorySetpoint()

        msg.timestamp = self.timestamp()

        msg.position[0] = x
        msg.position[1] = y
        msg.position[2] = z

        msg.yaw = 0.0

        self.trajectory_setpoint_pub.publish(msg)

    # ---------------------------------------------------------
    # Send PX4 command
    # ---------------------------------------------------------

    def send_vehicle_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.timestamp = self.timestamp()

        msg.command = command

        msg.param1 = param1
        msg.param2 = param2

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        self.vehicle_command_pub.publish(msg)

    # ---------------------------------------------------------
    # Mission progression
    # ---------------------------------------------------------

    def update_mission(self):

        # Wait 5 seconds at each waypoint
        self.waypoint_timer += 1

        if self.waypoint_timer >= 50:

            if self.current_waypoint < 3:

                self.current_waypoint += 1

                self.waypoint_timer = 0

                if self.current_waypoint == 1:
                    self.get_logger().info(
                        '➡️ Moving to Point 1: (5, 0, -3)'
                    )

                elif self.current_waypoint == 2:
                    self.get_logger().info(
                        '➡️ Moving to Point 2: (5, 5, -3)'
                    )

                elif self.current_waypoint == 3:
                    self.get_logger().info(
                        '🏠 Returning HOME: (0, 0, -3)'
                    )

            else:

                self.get_logger().info(
                    '✅ Mission complete — hovering at HOME'
                )

                self.current_waypoint = 4

    # ---------------------------------------------------------
    # Main control loop
    # ---------------------------------------------------------

    def control_loop(self):

        # These MUST be continuously published.
        self.publish_offboard_control_mode()
        self.publish_position_setpoint()

        self.counter += 1

        # -----------------------------------------------------
        # Prepare Offboard mode
        # -----------------------------------------------------

        if self.counter == 20 and not self.offboard_enabled:

            self.get_logger().info(
                '🟡 Switching to OFFBOARD mode...'
            )

            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
                1.0,
                6.0
            )

            self.offboard_enabled = True

        # -----------------------------------------------------
        # Arm
        # -----------------------------------------------------

        if self.counter == 30 and not self.armed:

            self.get_logger().info(
                '🟢 Sending ARM command...'
            )

            self.send_vehicle_command(
                VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
                1.0,
                0.0
            )

            self.armed = True

        # -----------------------------------------------------
        # Mission starts after takeoff
        # -----------------------------------------------------

        if self.counter > 80:

            self.update_mission()


def main(args=None):

    rclpy.init(args=args)

    node = FlightController()

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
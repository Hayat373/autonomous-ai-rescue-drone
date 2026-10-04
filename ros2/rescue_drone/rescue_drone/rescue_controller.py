#!/usr/bin/env python3

import rclpy

from rclpy.node import Node
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    HistoryPolicy,
)

from px4_msgs.msg import (
    OffboardControlMode,
    TrajectorySetpoint,
    VehicleCommand,
    VehicleLocalPosition,
    VehicleStatus,
)


class RescueController(Node):

    def __init__(self):
        super().__init__('rescue_controller')

        # ============================================================
        # FLIGHT SETTINGS
        # ============================================================

        self.takeoff_altitude = -3.0

        self.hover_seconds = 10.0

        # ============================================================
        # PX4 LOCAL POSITION QoS
        # ============================================================

        position_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ============================================================
        # PX4 VEHICLE STATUS QoS
        # ============================================================

        status_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=5
        )

        # ============================================================
        # LOCAL POSITION SUBSCRIBER
        # ============================================================

        self.position_sub = self.create_subscription(
            VehicleLocalPosition,
            '/fmu/out/vehicle_local_position_v1',
            self.position_callback,
            position_qos
        )

        # ============================================================
        # VEHICLE STATUS SUBSCRIBER
        # ============================================================

        self.status_sub = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status_v4',
            self.status_callback,
            status_qos
        )

        # ============================================================
        # OFFBOARD CONTROL MODE
        # ============================================================

        self.offboard_pub = self.create_publisher(
            OffboardControlMode,
            '/fmu/in/offboard_control_mode',
            10
        )

        # ============================================================
        # TRAJECTORY SETPOINT
        # ============================================================

        self.setpoint_pub = self.create_publisher(
            TrajectorySetpoint,
            '/fmu/in/trajectory_setpoint',
            10
        )

        # ============================================================
        # VEHICLE COMMAND
        # ============================================================

        self.command_pub = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        # ============================================================
        # DRONE POSITION
        # ============================================================

        self.drone_x = 0.0
        self.drone_y = 0.0
        self.drone_z = 0.0

        self.position_received = False

        # ============================================================
        # PX4 STATUS
        # ============================================================

        self.nav_state = None
        self.arming_state = None

        # ============================================================
        # MISSION STATE
        # ============================================================

        self.state = "WAITING"

        self.state_start_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        # ============================================================
        # COUNTER
        # ============================================================

        self.counter = 0

        self.offboard_sent = False
        self.arm_sent = False
        self.land_sent = False
        self.disarm_sent = False

        # ============================================================
        # TIMER
        # ============================================================

        self.timer = self.create_timer(
            0.1,
            self.control_loop
        )

        # ============================================================
        # STARTUP
        # ============================================================

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            "🚁 RESCUE DRONE FLIGHT TEST"
        )

        self.get_logger().info(
            "=================================================="
        )

        self.get_logger().info(
            "🎯 Mission: TAKEOFF → HOVER → LAND"
        )

        self.get_logger().info(
            "📏 Takeoff altitude: 3 meters"
        )

        self.get_logger().info(
            "⏱️ Hover time: 10 seconds"
        )

        self.get_logger().info(
            "⏳ Waiting for PX4..."
        )

    # ================================================================
    # POSITION CALLBACK
    # ================================================================

    def position_callback(self, msg):

        self.drone_x = float(msg.x)
        self.drone_y = float(msg.y)
        self.drone_z = float(msg.z)

        self.position_received = True

    # ================================================================
    # VEHICLE STATUS CALLBACK
    # ================================================================

    def status_callback(self, msg):

        self.nav_state = msg.nav_state
        self.arming_state = msg.arming_state

    # ================================================================
    # CHANGE STATE
    # ================================================================

    def change_state(self, new_state):

        if self.state == new_state:
            return

        old_state = self.state

        self.state = new_state

        self.state_start_time = (
            self.get_clock().now().nanoseconds / 1e9
        )

        self.get_logger().info(
            f"🔄 STATE: {old_state} → {new_state}"
        )

    # ================================================================
    # STATE TIMER
    # ================================================================

    def state_elapsed_time(self):

        now = (
            self.get_clock().now().nanoseconds / 1e9
        )

        return now - self.state_start_time

    # ================================================================
    # PUBLISH OFFBOARD CONTROL MODE
    # ================================================================

    def publish_offboard_control(self):

        msg = OffboardControlMode()

        msg.position = True
        msg.velocity = False
        msg.acceleration = False
        msg.attitude = False
        msg.body_rate = False

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.offboard_pub.publish(msg)

    # ================================================================
    # PUBLISH POSITION SETPOINT
    # ================================================================

    def publish_setpoint(
        self,
        x,
        y,
        z
    ):

        msg = TrajectorySetpoint()

        msg.position = [
            float(x),
            float(y),
            float(z)
        ]

        msg.velocity = [
            float("nan"),
            float("nan"),
            float("nan")
        ]

        msg.acceleration = [
            float("nan"),
            float("nan"),
            float("nan")
        ]

        msg.jerk = [
            float("nan"),
            float("nan"),
            float("nan")
        ]

        msg.yaw = float("nan")
        msg.yawspeed = float("nan")

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.setpoint_pub.publish(msg)

    # ================================================================
    # SEND VEHICLE COMMAND
    # ================================================================

    def send_command(
        self,
        command,
        param1=0.0,
        param2=0.0
    ):

        msg = VehicleCommand()

        msg.command = command

        msg.param1 = float(param1)
        msg.param2 = float(param2)

        msg.target_system = 1
        msg.target_component = 1

        msg.source_system = 1
        msg.source_component = 1

        msg.from_external = True

        msg.timestamp = (
            self.get_clock().now().nanoseconds // 1000
        )

        self.command_pub.publish(msg)

    # ================================================================
    # SET OFFBOARD MODE
    # ================================================================

    def set_offboard_mode(self):

        self.get_logger().info(
            "🕹️ REQUESTING OFFBOARD MODE"
        )

        self.send_command(
            VehicleCommand.VEHICLE_CMD_DO_SET_MODE,
            1.0,
            6.0
        )

        self.offboard_sent = True

    # ================================================================
    # ARM
    # ================================================================

    def arm(self):

        self.get_logger().info(
            "🔑 ARMING DRONE"
        )

        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            1.0,
            0.0
        )

        self.arm_sent = True

    # ================================================================
    # LAND
    # ================================================================

    def land(self):

        self.get_logger().info(
            "🛬 REQUESTING LAND"
        )

        self.send_command(
            VehicleCommand.VEHICLE_CMD_NAV_LAND,
            0.0,
            0.0
        )

        self.land_sent = True

    # ================================================================
    # DISARM
    # ================================================================

    def disarm(self):

        self.get_logger().info(
            "🔓 DISARMING DRONE"
        )

        self.send_command(
            VehicleCommand.VEHICLE_CMD_COMPONENT_ARM_DISARM,
            0.0,
            0.0
        )

        self.disarm_sent = True

    # ================================================================
    # WAITING
    # ================================================================

    def state_waiting(self):

        # We need a valid PX4 position before doing anything.

        if not self.position_received:

            if self.counter % 20 == 0:

                self.get_logger().info(
                    "⏳ Waiting for PX4 local position..."
                )

            return

        self.get_logger().info(
            f"📍 PX4 POSITION RECEIVED | "
            f"x={self.drone_x:.2f} "
            f"y={self.drone_y:.2f} "
            f"z={self.drone_z:.2f}"
        )

        self.change_state("PRE_OFFBOARD")

    # ================================================================
    # PRE-OFFBOARD
    # ================================================================

    def state_pre_offboard(self):

        # Keep sending setpoints before entering OFFBOARD.
        #
        # PX4 requires a stream of setpoints before accepting
        # OFFBOARD mode.

        self.publish_setpoint(
            self.drone_x,
            self.drone_y,
            self.takeoff_altitude
        )

        elapsed = self.state_elapsed_time()

        if elapsed >= 3.0:

            self.get_logger().info(
                "✅ OFFBOARD SETPOINT STREAM READY"
            )

            self.change_state("OFFBOARD")

    # ================================================================
    # OFFBOARD
    # ================================================================

    def state_offboard(self):

        self.publish_setpoint(
            self.drone_x,
            self.drone_y,
            self.takeoff_altitude
        )

        if not self.offboard_sent:

            self.set_offboard_mode()

            self.get_logger().info(
                "🕹️ OFFBOARD COMMAND SENT"
            )

            self.change_state("ARMING")

    # ================================================================
    # ARMING
    # ================================================================

    def state_arming(self):

        self.publish_setpoint(
            self.drone_x,
            self.drone_y,
            self.takeoff_altitude
        )

        if not self.arm_sent:

            self.arm()

            self.get_logger().info(
                "🚁 ARM COMMAND SENT"
            )

            self.change_state("TAKEOFF")

    # ================================================================
    # TAKEOFF
    # ================================================================

    def state_takeoff(self):

        # Keep the drone directly above its current XY position.

        self.publish_setpoint(
            self.drone_x,
            self.drone_y,
            self.takeoff_altitude
        )

        altitude = abs(self.drone_z)

        if self.counter % 20 == 0:

            self.get_logger().info(
                f"🚁 TAKING OFF | "
                f"Altitude={altitude:.2f} m"
            )

        # PX4 NED:
        #
        # Ground = 0
        # 1 meter = -1
        # 3 meters = -3

        if self.drone_z <= -2.7:

            self.get_logger().info(
                "✅ TARGET ALTITUDE REACHED"
            )

            self.get_logger().info(
                "🛑 STARTING HOVER"
            )

            self.change_state("HOVER")

    # ================================================================
    # HOVER
    # ================================================================

    def state_hover(self):

        self.publish_setpoint(
            self.drone_x,
            self.drone_y,
            self.takeoff_altitude
        )

        elapsed = self.state_elapsed_time()

        if self.counter % 20 == 0:

            self.get_logger().info(
                f"🛑 HOVERING | "
                f"time={elapsed:.1f}s | "
                f"altitude={abs(self.drone_z):.2f}m"
            )

        if elapsed >= self.hover_seconds:

            self.get_logger().info(
                "✅ HOVER TEST COMPLETE"
            )

            self.change_state("LANDING")

    # ================================================================
    # LANDING
    # ================================================================

    def state_landing(self):

        # Send LAND only once.

        if not self.land_sent:

            self.land()

            self.get_logger().info(
                "🛬 LAND COMMAND SENT"
            )

        # Keep publishing a safe position setpoint while PX4
        # transitions into landing.

        self.publish_setpoint(
            self.drone_x,
            self.drone_y,
            0.0
        )

        # Detect when drone is close to ground.

        if self.drone_z >= -0.2:

            self.get_logger().info(
                "🛬 DRONE REACHED GROUND"
            )

            self.change_state("DISARMING")

    # ================================================================
    # DISARMING
    # ================================================================

    def state_disarming(self):

        if not self.disarm_sent:

            self.disarm()

            self.get_logger().info(
                "🔓 DISARM COMMAND SENT"
            )

        self.change_state("COMPLETE")

    # ================================================================
    # COMPLETE
    # ================================================================

    def state_complete(self):

        # Keep the drone at ground level.

        self.publish_setpoint(
            self.drone_x,
            self.drone_y,
            0.0
        )

        if self.counter % 50 == 0:

            self.get_logger().info(
                "=================================================="
            )

            self.get_logger().info(
                "✅ FLIGHT TEST COMPLETE"
            )

            self.get_logger().info(
                f"📍 Final position: "
                f"x={self.drone_x:.2f} "
                f"y={self.drone_y:.2f} "
                f"z={self.drone_z:.2f}"
            )

            self.get_logger().info(
                "=================================================="
            )

    # ================================================================
    # MAIN CONTROL LOOP
    # ================================================================

    def control_loop(self):

        self.counter += 1

        # ------------------------------------------------------------
        # Always publish OFFBOARD control mode
        # ------------------------------------------------------------

        self.publish_offboard_control()

        # ------------------------------------------------------------
        # STATE MACHINE
        # ------------------------------------------------------------

        if self.state == "WAITING":

            self.state_waiting()

        elif self.state == "PRE_OFFBOARD":

            self.state_pre_offboard()

        elif self.state == "OFFBOARD":

            self.state_offboard()

        elif self.state == "ARMING":

            self.state_arming()

        elif self.state == "TAKEOFF":

            self.state_takeoff()

        elif self.state == "HOVER":

            self.state_hover()

        elif self.state == "LANDING":

            self.state_landing()

        elif self.state == "DISARMING":

            self.state_disarming()

        elif self.state == "COMPLETE":

            self.state_complete()


# ====================================================================
# MAIN
# ====================================================================

def main(args=None):

    rclpy.init(args=args)

    node = RescueController()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:

        node.get_logger().info(
            "🛑 Flight controller stopped"
        )

    finally:

        node.destroy_node()

        rclpy.shutdown()


# ====================================================================
# ENTRY POINT
# ====================================================================

if __name__ == "__main__":

    main()
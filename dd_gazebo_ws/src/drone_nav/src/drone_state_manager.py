import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from px4_msgs.msg import VehicleCommand, VehicleCommandAck, VehicleStatus
from enum import Enum


class DroneStateManager(Node):
    class State(Enum):
        Idle = 0
        Arming = 1
        SettingMode = 2
        TakingOff = 3
        Hovering = 4
        Landing = 5
        WaitUntilDisarmed = 6

    def __init__(self):
        super().__init__('drone_state_manager_node')

        # Subscribe to the "signal" topic
        self.signal_subscription = self.create_subscription(
            String,
            'signal',
            self.signal_callback,
            10
        )

        self.get_logger().info("DroneStateManager activated, waiting for signals 'A' or 'B'")

        # Publishers and Subscribers
        self.vehicle_command_publisher = self.create_publisher(
            VehicleCommand,
            '/fmu/in/vehicle_command',
            10
        )

        self.vehicle_command_ack_subscription = self.create_subscription(
            VehicleCommandAck,
            '/fmu/out/vehicle_command_ack',
            self.vehicle_command_ack_callback,
            10
        )

        self.vehicle_status_subscription = self.create_subscription(
            VehicleStatus,
            '/fmu/out/vehicle_status',
            self.vehicle_status_callback,
            10
        )

        # Initialize state variables
        self.current_state = self.State.Idle
        self.is_armed = False
        self.awaiting_ack_command = None  # Tracks which command we're waiting for acknowledgment

        # Define MAVLink command constants
        self.MAV_CMD_COMPONENT_ARM_DISARM = 400
        self.MAV_CMD_DO_SET_MODE = 176
        self.MAV_CMD_NAV_TAKEOFF = 22
        self.MAV_CMD_NAV_LAND = 21

        # MAVLink result constants
        self.VEHICLE_RESULT_ACCEPTED = 0

        # Arming state constant
        self.ARMING_STATE_ARMED = 2

    def signal_callback(self, msg):
        if msg.data == "A":
            self.get_logger().info("Signal 'A' received, initiating takeoff sequence")
            self.run_state(self.State.Arming)
        elif msg.data == "B":
            self.get_logger().info("Signal 'B' received, initiating landing")
            self.run_state(self.State.Landing)
        else:
            self.get_logger().warn(f"Unknown signal received: '{msg.data}'")

    def run_state(self, state):
        if state == self.State.Arming:
            self.arm()
        elif state == self.State.SettingMode:
            self.set_offboard_mode()
        elif state == self.State.TakingOff:
            self.send_takeoff_command()
        elif state == self.State.Landing:
            self.land()
        elif state == self.State.WaitUntilDisarmed:
            self.wait_until_disarmed()
        else:
            self.get_logger().warn("Unknown state")

    def arm(self):
        if self.current_state != self.State.Idle:
            self.get_logger().warn("Cannot arm: Drone is not in Idle state")
            return

        self.get_logger().info("Sending arm command")
        cmd = VehicleCommand()
        cmd.command = self.MAV_CMD_COMPONENT_ARM_DISARM
        cmd.param1 = 1.0  # Arm
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(cmd)
        self.current_state = self.State.Arming
        self.awaiting_ack_command = self.MAV_CMD_COMPONENT_ARM_DISARM

    def set_offboard_mode(self):
        if self.current_state != self.State.Arming:
            self.get_logger().warn("Cannot set mode: Drone is not in Arming state")
            return

        self.get_logger().info("Setting offboard mode")
        cmd = VehicleCommand()
        cmd.command = self.MAV_CMD_DO_SET_MODE
        cmd.param1 = 1.0  # Base mode
        cmd.param2 = 6.0  # Custom mode: PX4_CUSTOM_MAIN_MODE_OFFBOARD
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000
        self.vehicle_command_publisher.publish(cmd)
        self.current_state = self.State.SettingMode
        self.awaiting_ack_command = self.MAV_CMD_DO_SET_MODE

    def send_takeoff_command(self):
        if self.current_state != self.State.SettingMode:
            self.get_logger().warn("Cannot take off: Drone is not in SettingMode state")
            return

        self.get_logger().info("Sending takeoff command")
        cmd = VehicleCommand()
        cmd.command = self.MAV_CMD_NAV_TAKEOFF
        cmd.param7 = 10.0  # Desired altitude in meters
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000  # Microseconds
        self.vehicle_command_publisher.publish(cmd)
        self.current_state = self.State.TakingOff
        self.awaiting_ack_command = self.MAV_CMD_NAV_TAKEOFF

    def land(self):
        if self.current_state != self.State.Hovering and self.current_state != self.State.TakingOff:
            self.get_logger().warn("Cannot land: Drone is not in Hovering or TakingOff state")
            return

        self.get_logger().info("Sending land command")
        cmd = VehicleCommand()
        cmd.command = self.MAV_CMD_NAV_LAND
        cmd.target_system = 1
        cmd.target_component = 1
        cmd.source_system = 1
        cmd.source_component = 1
        cmd.from_external = True
        cmd.timestamp = self.get_clock().now().nanoseconds // 1000  # Microseconds
        self.vehicle_command_publisher.publish(cmd)
        self.current_state = self.State.Landing
        self.awaiting_ack_command = self.MAV_CMD_NAV_LAND

    def wait_until_disarmed(self):
        if self.current_state != self.State.Landing:
            self.get_logger().warn("Cannot wait until disarmed: Drone is not in Landing state")
            return

        self.get_logger().info("Waiting until drone is disarmed")
        self.current_state = self.State.WaitUntilDisarmed

    def vehicle_command_ack_callback(self, msg):
        if msg.command != self.awaiting_ack_command:
            # Ignore acknowledgments for commands we're not tracking
            return

        if msg.result == self.VEHICLE_RESULT_ACCEPTED:
            if msg.command == self.MAV_CMD_COMPONENT_ARM_DISARM:
                self.get_logger().info("Arm command accepted")
                self.run_state(self.State.SettingMode)
            elif msg.command == self.MAV_CMD_DO_SET_MODE:
                self.get_logger().info("Set mode command accepted")
                self.run_state(self.State.TakingOff)
            elif msg.command == self.MAV_CMD_NAV_TAKEOFF:
                self.get_logger().info("Takeoff command accepted")
                self.current_state = self.State.Hovering
            elif msg.command == self.MAV_CMD_NAV_LAND:
                self.get_logger().info("Landing command accepted")
                self.run_state(self.State.WaitUntilDisarmed)
            else:
                self.get_logger().warn(f"Received acknowledgment for unknown command: {msg.command}")
        else:
            self.get_logger().error(f"Command {msg.command} rejected with result {msg.result}")
            self.current_state = self.State.Idle

        # Reset awaiting acknowledgment command
        self.awaiting_ack_command = None

    def vehicle_status_callback(self, msg):
        # Update arming state
        self.is_armed = (msg.arming_state == self.ARMING_STATE_ARMED)
        if not self.is_armed and self.current_state == self.State.WaitUntilDisarmed:
            self.get_logger().info("Drone disarmed, operation complete")
            self.current_state = self.State.Idle

    def on_activate(self):
        self.get_logger().info("DroneStateManager node activated")

    def on_deactivate(self):
        self.get_logger().info("DroneStateManager node deactivated")


def main(args=None):
    rclpy.init(args=args)
    drone_state_manager = DroneStateManager()
    try:
        rclpy.spin(drone_state_manager)
    except KeyboardInterrupt:
        pass
    finally:
        drone_state_manager.destroy_node()
        rclpy.shutdown()

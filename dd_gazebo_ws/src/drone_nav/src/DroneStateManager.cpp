#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "px4_msgs/msg/vehicle_command_ack.hpp"
#include "px4_msgs/msg/vehicle_status.hpp"
#include "std_msgs/msg/string.hpp"
#include <memory>

using namespace std::chrono_literals;

class DroneStateManager : public rclcpp::Node {
public:
    DroneStateManager()
    : Node("drone_state_manager_node")
    {
        // Publishers and Subscribers
        command_publisher_ = this->create_publisher<px4_msgs::msg::VehicleCommand>(
            "/fmu/in/vehicle_command", 10);

        command_ack_subscription_ = this->create_subscription<px4_msgs::msg::VehicleCommandAck>(
            "/fmu/out/vehicle_command_ack", 10,
            std::bind(&DroneStateManager::vehicleCommandAckCallback, this, std::placeholders::_1));

        vehicle_status_subscription_ = this->create_subscription<px4_msgs::msg::VehicleStatus>(
            "/fmu/out/vehicle_status", 10,
            std::bind(&DroneStateManager::vehicleStatusCallback, this, std::placeholders::_1));

        signal_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "signal", 10,
            std::bind(&DroneStateManager::signalCallback, this, std::placeholders::_1));

        // Initialize state variables
        current_state_ = State::Idle;
        is_armed_ = false;
        awaiting_ack_command_ = 0;

        RCLCPP_INFO(this->get_logger(), "DroneStateManager activated, waiting for signals 'A' or 'B'");

        // Initialize timer for checking disarming state
        timer_ = this->create_wall_timer(
            500ms, std::bind(&DroneStateManager::timerCallback, this));
    }

private:
    enum class State {
        Idle,
        Arming,
        SettingMode,
        TakingOff,
        Hovering,
        Landing,
        WaitUntilDisarmed
    };

    // MAVLink command and result constants
    static constexpr uint16_t MAV_CMD_COMPONENT_ARM_DISARM = 400;
    static constexpr uint16_t MAV_CMD_DO_SET_MODE = 176;
    static constexpr uint16_t MAV_CMD_NAV_TAKEOFF = 22;
    static constexpr uint16_t MAV_CMD_NAV_LAND = 21;
    static constexpr uint8_t VEHICLE_RESULT_ACCEPTED = 0;
    static constexpr uint8_t ARMING_STATE_ARMED = 2;

    void signalCallback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "A") {
            RCLCPP_INFO(this->get_logger(), "Signal 'A' received, initiating takeoff sequence");
            runState(State::Arming);
        } else if (msg->data == "B") {
            RCLCPP_INFO(this->get_logger(), "Signal 'B' received, initiating landing");
            runState(State::Landing);
        } else {
            RCLCPP_WARN(this->get_logger(), "Unknown signal received: '%s'", msg->data.c_str());
        }
    }

    void runState(State state)
    {
        switch (state) {
            case State::Arming:
                arm();
                break;
            case State::SettingMode:
                setOffboardMode();
                break;
            case State::TakingOff:
                sendTakeoffCommand();
                break;
            case State::Landing:
                land();
                break;
            case State::WaitUntilDisarmed:
                waitUntilDisarmed();
                break;
            default:
                RCLCPP_WARN(this->get_logger(), "Unknown state");
                break;
        }
    }

    void arm()
    {
        if (current_state_ != State::Idle) {
            RCLCPP_WARN(this->get_logger(), "Cannot arm: Drone is not in Idle state");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sending arm command");
        auto cmd = px4_msgs::msg::VehicleCommand();
        cmd.command = MAV_CMD_COMPONENT_ARM_DISARM;
        cmd.param1 = 1.0;  // Arm
        fillVehicleCommand(cmd);
        command_publisher_->publish(cmd);
        current_state_ = State::Arming;
        awaiting_ack_command_ = MAV_CMD_COMPONENT_ARM_DISARM;
    }

    void setOffboardMode()
    {
        if (current_state_ != State::Arming) {
            RCLCPP_WARN(this->get_logger(), "Cannot set mode: Drone is not in Arming state");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Setting offboard mode");
        auto cmd = px4_msgs::msg::VehicleCommand();
        cmd.command = MAV_CMD_DO_SET_MODE;
        cmd.param1 = 1.0;  // Base mode
        cmd.param2 = 6.0;  // Custom mode: PX4_CUSTOM_MAIN_MODE_OFFBOARD
        fillVehicleCommand(cmd);
        command_publisher_->publish(cmd);
        current_state_ = State::SettingMode;
        awaiting_ack_command_ = MAV_CMD_DO_SET_MODE;
    }

    void sendTakeoffCommand()
    {
        if (current_state_ != State::SettingMode) {
            RCLCPP_WARN(this->get_logger(), "Cannot take off: Drone is not in SettingMode state");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sending takeoff command");
        auto cmd = px4_msgs::msg::VehicleCommand();
        cmd.command = MAV_CMD_NAV_TAKEOFF;
        cmd.param7 = 10.0;  // Desired altitude in meters
        fillVehicleCommand(cmd);
        command_publisher_->publish(cmd);
        current_state_ = State::TakingOff;
        awaiting_ack_command_ = MAV_CMD_NAV_TAKEOFF;
    }

    void land()
    {
        if (current_state_ != State::Hovering && current_state_ != State::TakingOff) {
            RCLCPP_WARN(this->get_logger(), "Cannot land: Drone is not in Hovering or TakingOff state");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Sending land command");
        auto cmd = px4_msgs::msg::VehicleCommand();
        cmd.command = MAV_CMD_NAV_LAND;
        fillVehicleCommand(cmd);
        command_publisher_->publish(cmd);
        current_state_ = State::Landing;
        awaiting_ack_command_ = MAV_CMD_NAV_LAND;
    }

    void waitUntilDisarmed()
    {
        if (current_state_ != State::Landing) {
            RCLCPP_WARN(this->get_logger(), "Cannot wait until disarmed: Drone is not in Landing state");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Waiting until drone is disarmed");
        current_state_ = State::WaitUntilDisarmed;
    }

    void vehicleCommandAckCallback(const px4_msgs::msg::VehicleCommandAck::SharedPtr msg)
    {
        if (msg->command != awaiting_ack_command_) {
            // Ignore acknowledgments for commands we're not tracking
            return;
        }

        if (msg->result == VEHICLE_RESULT_ACCEPTED) {
            if (msg->command == MAV_CMD_COMPONENT_ARM_DISARM) {
                RCLCPP_INFO(this->get_logger(), "Arm command accepted");
                runState(State::SettingMode);
            } else if (msg->command == MAV_CMD_DO_SET_MODE) {
                RCLCPP_INFO(this->get_logger(), "Set mode command accepted");
                runState(State::TakingOff);
            } else if (msg->command == MAV_CMD_NAV_TAKEOFF) {
                RCLCPP_INFO(this->get_logger(), "Takeoff command accepted");
                current_state_ = State::Hovering;
            } else if (msg->command == MAV_CMD_NAV_LAND) {
                RCLCPP_INFO(this->get_logger(), "Landing command accepted");
                runState(State::WaitUntilDisarmed);
            } else {
                RCLCPP_WARN(this->get_logger(), "Received acknowledgment for unknown command: %u", msg->command);
            }
        } else {
            RCLCPP_ERROR(this->get_logger(), "Command %u rejected with result %u", msg->command, msg->result);
            current_state_ = State::Idle;
        }

        // Reset awaiting acknowledgment command
        awaiting_ack_command_ = 0;
    }

    void vehicleStatusCallback(const px4_msgs::msg::VehicleStatus::SharedPtr msg)
    {
        // Update arming state
        is_armed_ = (msg->arming_state == ARMING_STATE_ARMED);
        if (!is_armed_ && current_state_ == State::WaitUntilDisarmed) {
            RCLCPP_INFO(this->get_logger(), "Drone disarmed, operation complete");
            current_state_ = State::Idle;
        }
    }

    void timerCallback()
    {
        // This timer can be used to perform periodic checks or actions if necessary
        // For now, it's just a placeholder
    }

    void fillVehicleCommand(px4_msgs::msg::VehicleCommand &cmd)
    {
        cmd.target_system = 1;
        cmd.target_component = 1;
        cmd.source_system = 1;
        cmd.source_component = 1;
        cmd.from_external = true;
        cmd.timestamp = this->get_clock()->now().nanoseconds() / 1000;  // Timestamp in microseconds
    }

    // Class members
    rclcpp::Publisher<px4_msgs::msg::VehicleCommand>::SharedPtr command_publisher_;
    rclcpp::Subscription<px4_msgs::msg::VehicleCommandAck>::SharedPtr command_ack_subscription_;
    rclcpp::Subscription<px4_msgs::msg::VehicleStatus>::SharedPtr vehicle_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_subscription_;
    rclcpp::TimerBase::SharedPtr timer_;

    State current_state_;
    bool is_armed_;
    uint16_t awaiting_ack_command_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto drone_state_manager = std::make_shared<DroneStateManager>();
    rclcpp::spin(drone_state_manager);
    rclcpp::shutdown();
    return 0;
}

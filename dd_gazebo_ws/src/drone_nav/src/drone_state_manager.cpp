#include "rclcpp/rclcpp.hpp"
#include "px4_msgs/msg/vehicle_command.hpp"
#include "std_msgs/msg/string.hpp"

using namespace px4_msgs::msg;

class DroneStateManager : public rclcpp::Node {
public:
    DroneStateManager()
    : Node("drone_state_manager"),
      current_state_(State::Idle)
    {
        // Publisher to send VehicleCommand messages to PX4
        vehicle_command_publisher_ = this->create_publisher<VehicleCommand>(
            "/fmu/in/vehicle_command", rclcpp::QoS(10));

        // Subscription to external signals
        signal_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "drone_state_signal", 10,
            std::bind(&DroneStateManager::signal_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Drone State Manager initialized. Waiting for signals.");
    }

private:
    enum class State {
        Idle,
        TakingOff,
        Flying,
        Landing
    };

    void signal_callback(const std_msgs::msg::String::SharedPtr msg) {
        std::string signal = msg->data;

        if (signal == "takeoff" && current_state_ == State::Idle) {
            RCLCPP_INFO(this->get_logger(), "Received 'takeoff' signal.");
            takeoff();
        } else if (signal == "fly" && current_state_ == State::TakingOff) {
            RCLCPP_INFO(this->get_logger(), "Received 'fly' signal.");
            change_state_to(State::Flying);
            // Add flight operations here if needed
        } else if (signal == "land" && current_state_ == State::Flying) {
            RCLCPP_INFO(this->get_logger(), "Received 'land' signal.");
            land();
        } else {
            RCLCPP_WARN(this->get_logger(), "Received invalid signal '%s' in state '%s'.",
                        signal.c_str(), state_to_string(current_state_).c_str());
        }
    }

    void takeoff() {
        // Arm the vehicle
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_COMPONENT_ARM_DISARM, 1.0);
        RCLCPP_INFO(this->get_logger(), "Sent arm command.");

        // Wait briefly to ensure the vehicle is armed
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // Send takeoff command
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_TAKEOFF, NAN, NAN, NAN, NAN, NAN, NAN, 10.0);
        RCLCPP_INFO(this->get_logger(), "Sent takeoff command.");

        change_state_to(State::TakingOff);
    }

    void land() {
        // Send land command
        send_vehicle_command(VehicleCommand::VEHICLE_CMD_NAV_LAND);
        RCLCPP_INFO(this->get_logger(), "Sent land command.");

        change_state_to(State::Landing);
    }

    void send_vehicle_command(uint16_t command, float param1 = NAN, float param2 = NAN,
                              float param3 = NAN, float param4 = NAN, float param5 = NAN,
                              float param6 = NAN, float param7 = NAN) {
        auto msg = VehicleCommand();
        msg.timestamp = now().nanoseconds() / 1000;  // PX4 expects microseconds
        msg.param1 = param1;
        msg.param2 = param2;
        msg.param3 = param3;
        msg.param4 = param4;
        msg.param5 = param5;
        msg.param6 = param6;
        msg.param7 = param7;
        msg.command = command;
        msg.target_system = 1;
        msg.target_component = 1;
        msg.source_system = 1;
        msg.source_component = 1;
        msg.from_external = true;

        vehicle_command_publisher_->publish(msg);
    }

    void change_state_to(State new_state) {
        current_state_ = new_state;
        RCLCPP_INFO(this->get_logger(), "State changed to: %s", state_to_string(new_state).c_str());
    }

    std::string state_to_string(State state) {
        switch (state) {
            case State::Idle:
                return "Idle";
            case State::TakingOff:
                return "TakingOff";
            case State::Flying:
                return "Flying";
            case State::Landing:
                return "Landing";
            default:
                return "Unknown";
        }
    }

    // Class members
    rclcpp::Publisher<VehicleCommand>::SharedPtr vehicle_command_publisher_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr signal_subscription_;

    State current_state_;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto drone_state_manager = std::make_shared<DroneStateManager>();
    rclcpp::spin(drone_state_manager);
    rclcpp::shutdown();
    return 0;
}

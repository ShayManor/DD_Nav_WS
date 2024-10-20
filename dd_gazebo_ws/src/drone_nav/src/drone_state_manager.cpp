#include "rclcpp/rclcpp.hpp"
#include "px4_ros2/mode_executor_base.hpp"
#include "px4_ros2/mode_base.hpp"
#include "std_msgs/msg/string.hpp"

class DroneStateManager : public px4_ros2::ModeExecutorBase {
public:
    DroneStateManager(rclcpp::Node & node)
    : ModeExecutorBase(node, px4_ros2::ModeExecutorBase::Settings{}),
      _node(node)
    {
        // Subscribe to the "signal" topic
        _signal_subscription = _node.create_subscription<std_msgs::msg::String>(
            "signal", 10,
            std::bind(&DroneStateManager::signalCallback, this, std::placeholders::_1));

        RCLCPP_INFO(_node.get_logger(), "DroneStateManager activated, waiting for signals 'A' or 'B'");
    }

    enum class State {
        Idle,
        TakingOff,
        Hovering,
        Landing,
        WaitUntilDisarmed,
    };

    void onActivate() override {
        // Node activation logic
        RCLCPP_INFO(_node.get_logger(), "DroneStateManager node activated");
    }

    void onDeactivate(DeactivateReason reason) override {
        // Node deactivation logic
        RCLCPP_INFO(_node.get_logger(), "DroneStateManager node deactivated");
    }

private:
    void signalCallback(const std_msgs::msg::String::SharedPtr msg) {
        if (msg->data == "A") {
            RCLCPP_INFO(_node.get_logger(), "Signal 'A' received, initiating takeoff and hover");
            runState(State::TakingOff, px4_ros2::Result::Success);
        } else if (msg->data == "B") {
            RCLCPP_INFO(_node.get_logger(), "Signal 'B' received, initiating landing");
            runState(State::Landing, px4_ros2::Result::Success);
        } else {
            RCLCPP_WARN(_node.get_logger(), "Unknown signal received: '%s'", msg->data.c_str());
        }
    }

    void runState(State state, px4_ros2::Result previous_result) {
        if (previous_result != px4_ros2::Result::Success) {
            RCLCPP_ERROR(_node.get_logger(), "State %i: previous state failed: %s", (int)state, resultToString(previous_result));
            return;
        }

        switch (state) {
            case State::Reset:
                // Reset logic if needed
                break;

            case State::TakingOff:
                takeoff([this](px4_ros2::Result result) {
                    if (result == px4_ros2::Result::Success) {
                        RCLCPP_INFO(_node.get_logger(), "Takeoff successful, now hovering");
                        // Optionally transition to Hovering state
                    } else {
                        RCLCPP_ERROR(_node.get_logger(), "Takeoff failed: %s", resultToString(result));
                    }
                });
                break;

            case State::Landing:
                land([this](px4_ros2::Result result) {
                    if (result == px4_ros2::Result::Success) {
                        RCLCPP_INFO(_node.get_logger(), "Landing initiated, waiting until disarmed");
                        runState(State::WaitUntilDisarmed, px4_ros2::Result::Success);
                    } else {
                        RCLCPP_ERROR(_node.get_logger(), "Landing failed: %s", resultToString(result));
                    }
                });
                break;

            case State::WaitUntilDisarmed:
                waitUntilDisarmed([this](px4_ros2::Result result) {
                    if (result == px4_ros2::Result::Success) {
                        RCLCPP_INFO(_node.get_logger(), "Drone disarmed, operation complete");
                    } else {
                        RCLCPP_ERROR(_node.get_logger(), "Disarm failed: %s", resultToString(result));
                    }
                });
                break;

            default:
                RCLCPP_WARN(_node.get_logger(), "Unknown state");
                break;
        }
    }

    // Class members
    rclcpp::Node & _node;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr _signal_subscription;
};

int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("drone_state_manager_node");
    auto drone_state_manager = std::make_shared<DroneStateManager>(*node);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
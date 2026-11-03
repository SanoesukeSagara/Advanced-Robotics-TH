#include <chrono>
#include <memory>
#include <functional>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using std::placeholders::_1;

class MovementSubscriber : public rclcpp::Node
{
public:
    MovementSubscriber()
    : Node("movement_subscriber")
    {
        // Subscribe to the Python publisher's topic
        subscription_py_ = this->create_subscription<std_msgs::msg::String>(
            "py_movement_commands",
            10,
            std::bind(
                &MovementSubscriber::py_listener_callback,
                this,
                _1
            )
        );

        // Subscribe to the C++ publisher's topic
        subscription_cpp_ = this->create_subscription<std_msgs::msg::String>(
            "cpp_movement_commands",
            10,
            std::bind(
                &MovementSubscriber::cpp_listener_callback,
                this,
                _1
            )
        );

        RCLCPP_INFO(
            this->get_logger(),
            "Movement CPP Subscriber Node Initialized."
        );
    }

private:

    // Callback for Python publisher messages
    void py_listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received Python movement command: '%s'",
            msg->data.c_str()
        );

        process_movement_command(msg->data);
    }

    // Callback for C++ publisher messages
    void cpp_listener_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Received C++ movement command: '%s'",
            msg->data.c_str()
        );

        process_movement_command(msg->data);
    }

    // Process the received movement command
    void process_movement_command(const std::string & command)
    {
        if (command == "Move Forward") {
            move_forward();
        }
        else if (command == "Turn Left") {
            turn_left();
        }
        else if (command == "Turn Right") {
            turn_right();
        }
        else {
            RCLCPP_WARN(
                this->get_logger(),
                "Unknown movement command: '%s'",
                command.c_str()
            );
        }
    }

    // Simulate robot movement
    void move_forward()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Robot is moving forward."
        );
    }

    void turn_left()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Robot is turning left."
        );
    }

    void turn_right()
    {
        RCLCPP_INFO(
            this->get_logger(),
            "Robot is turning right."
        );
    }

    // Subscription handles
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_py_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_cpp_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto subscriber_node = std::make_shared<MovementSubscriber>();

    rclcpp::spin(subscriber_node);

    rclcpp::shutdown();

    return 0;
}

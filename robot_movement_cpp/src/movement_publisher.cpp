#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

class MovementPublisher : public rclcpp::Node
{
public:
    MovementPublisher()
    : Node("movement_publisher")
    {
        // Create a publisher
        publisher_ = this->create_publisher<std_msgs::msg::String>(
            "cpp_movement_commands", 10);

        // Create a timer that fires every 2 seconds
        timer_ = this->create_wall_timer(
            2s,
            std::bind(&MovementPublisher::timer_callback, this));

        RCLCPP_INFO(
            this->get_logger(),
            "Movement CPP Publisher Node Initialized.");
    }

private:
    void timer_callback()
    {
        // Create a String message
        auto msg = std_msgs::msg::String();

        msg.data = "Move Forward";

        // Publish the message
        publisher_->publish(msg);

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing cpp command: '%s'",
            msg.data.c_str());
    }

    // Publisher and timer
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

    rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto publisher_node = std::make_shared<MovementPublisher>();

    rclcpp::spin(publisher_node);
    rclcpp::shutdown();

    return 0;
}

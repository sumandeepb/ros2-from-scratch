#include "my_robot_interfaces/msg/hardware_status.hpp"
#include "rclcpp/rclcpp.hpp"

class CustomPublisherNode : public rclcpp::Node {
public:
    CustomPublisherNode() : Node("custom_publisher") {
        custom_publisher_ = this->create_publisher<my_robot_interfaces::msg::HardwareStatus>("hardware_status", 10);
        timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                         std::bind(&CustomPublisherNode::publishMessage, this));
        RCLCPP_INFO(this->get_logger(), "Custom publisher has been started.");
    }

    void publishMessage() {
        auto msg = my_robot_interfaces::msg::HardwareStatus();
        msg.temperature = 35.0;
        custom_publisher_->publish(msg);
    }

private:
    rclcpp::Publisher<my_robot_interfaces::msg::HardwareStatus>::SharedPtr custom_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CustomPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

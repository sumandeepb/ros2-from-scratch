#include "example_interfaces/msg/int64.hpp"
#include "rclcpp/rclcpp.hpp"

class NumberPublisherNode : public rclcpp::Node {
public:
    NumberPublisherNode() : Node("number_publisher") {
        number_ = 1;
        number_publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        number_timer_ = this->create_wall_timer(std::chrono::seconds(1),
                                                std::bind(&NumberPublisherNode::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number publisher has been started.");
    }

    void publishNumber() {
        auto msg = example_interfaces::msg::Int64();
        msg.data = number_;
        number_publisher_->publish(msg);
        RCLCPP_INFO(this->get_logger(), "Number: %d", number_);
        // number_++;
    }

private:
    int number_;
    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr number_publisher_;
    rclcpp::TimerBase::SharedPtr number_timer_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<NumberPublisherNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

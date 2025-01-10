#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

class TurtleControllerNode : public rclcpp::Node {
public:
    TurtleControllerNode() : Node("turtle_controller") {
        pose_subscriber_ = this->create_subscription<turtlesim::msg::Pose>("/turtle1/pose", 10, std::bind(&TurtleControllerNode::callbackPose, this, std::placeholders::_1));
        twist_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/turtle1/cmd_vel", 10);
    }

private:
    rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr twist_publisher_;

    void callbackPose(turtlesim::msg::Pose pose) {
        auto cmd = geometry_msgs::msg::Twist();
        if (pose.x < 5.5) {
            cmd.linear.x = 1.0;
            cmd.angular.z = 1.0;
        } else {
            cmd.linear.x = 2.0;
            cmd.angular.z = 2.0;
        }
        twist_publisher_->publish(cmd);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

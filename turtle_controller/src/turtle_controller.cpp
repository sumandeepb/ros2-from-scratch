#include "geometry_msgs/msg/twist.hpp"
#include "my_robot_interfaces/srv/activate_turtle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/set_pen.hpp"

using namespace std::chrono_literals;
using Pose = turtlesim::msg::Pose;
using Twist = geometry_msgs::msg::Twist;
using SetPen = turtlesim::srv::SetPen;
using ActivateTurtle = my_robot_interfaces::srv::ActivateTurtle;

class TurtleControllerNode : public rclcpp::Node {
public:
    TurtleControllerNode() : Node("turtle_controller") {
        activate_turtle_service_ = this->create_service<ActivateTurtle>("/turtle1/activate_turtle",
                                                                        std::bind(&TurtleControllerNode::callbackActivateTurtle,
                                                                                  this, std::placeholders::_1, std::placeholders::_2));
        pose_subscriber_ = this->create_subscription<Pose>("/turtle1/pose", 10,
                                                           std::bind(&TurtleControllerNode::callbackPose,
                                                                     this,
                                                                     std::placeholders::_1));
        twist_publisher_ = this->create_publisher<Twist>("/turtle1/cmd_vel", 10);
        set_pen_client_ = this->create_client<SetPen>("/turtle1/set_pen");
        RCLCPP_INFO(this->get_logger(), "Turtle controller has been started.");
    }

private:
    rclcpp::Service<ActivateTurtle>::SharedPtr activate_turtle_service_;
    rclcpp::Subscription<Pose>::SharedPtr pose_subscriber_;
    rclcpp::Publisher<Twist>::SharedPtr twist_publisher_;
    rclcpp::Client<SetPen>::SharedPtr set_pen_client_;
    float previous_x_ = 0.0;
    bool is_active = true;

    void callbackActivateTurtle(const ActivateTurtle::Request::SharedPtr request, const ActivateTurtle::Response::SharedPtr response) {
        if (!is_active && request->activate) {
            response->message = "Starting the turtle";
        } else if (is_active && !request->activate) {
            response->message = "Stopping the turtle";
        } else {
            response->message = "No action on the turtle";
        }
        is_active = request->activate;
    }

    void callbackPose(Pose pose) {
        if (is_active) {
            auto cmd = Twist();
            if (pose.x < 5.5) {
                cmd.linear.x = 1.0;
                cmd.angular.z = 1.0;
            } else {
                cmd.linear.x = 2.0;
                cmd.angular.z = 2.0;
            }
            twist_publisher_->publish(cmd);

            // check position
            if (pose.x > 5.5 && previous_x_ <= 5.5) {
                previous_x_ = pose.x;
                RCLCPP_INFO(this->get_logger(), "Set color to red.");
                callSetPen(255, 0, 0);
            } else if (pose.x <= 5.5 && previous_x_ > 5.5) {
                previous_x_ = pose.x;
                RCLCPP_INFO(this->get_logger(), "Set color to green.");
                callSetPen(0, 255, 0);
            }
        }
    }

    void callSetPen(uint8_t r, uint8_t g, uint8_t b) {
        while (!set_pen_client_->wait_for_service(1s)) {
            RCLCPP_WARN(this->get_logger(), "Waiting for the server...");
        }
        auto request = std::make_shared<SetPen::Request>();
        request->r = r;
        request->g = g;
        request->b = b;
        set_pen_client_->async_send_request(request,
                                            std::bind(&TurtleControllerNode::callbackSetPenResponse,
                                                      this,
                                                      std::placeholders::_1));
    }

    void callbackSetPenResponse(rclcpp::Client<SetPen>::SharedFuture future) {
        auto response = future.get();
        RCLCPP_INFO(this->get_logger(), "Successfully changed pen color");
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<TurtleControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

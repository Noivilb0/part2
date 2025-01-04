#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include <iostream>
#include <thread>
#include <chrono>

class UINode : public rclcpp::Node {
public:
    UINode() : Node("ui_node") {
        pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);
    }

    void sendCommand(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;

        // Publish command for 1 second
        for (int i = 0; i < 10; ++i) {
            pub_->publish(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop the robot
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        pub_->publish(cmd);
    }

private:
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<UINode>();

    while (rclcpp::ok()) {
        double linear, angular;
        std::cout << "Enter linear velocity: ";
        std::cin >> linear;
        std::cout << "Enter angular velocity: ";
        std::cin >> angular;

        node->sendCommand(linear, angular);
    }

    rclcpp::shutdown();
    return 0;
}



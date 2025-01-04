#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

class BoundaryCheckNode : public rclcpp::Node {
public:
    BoundaryCheckNode() : Node("boundary_check_node") {
        // Subscriber for robot odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&BoundaryCheckNode::odomCallback, this, std::placeholders::_1));

        // Publisher to send velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Define boundary limits
        x_min_ = 1.0;
        x_max_ = 9.0;
        y_min_ = 1.0;
        y_max_ = 9.0;

        in_bounds_ = true; // Assume robot starts within bounds
    }

    void spin() {
        rclcpp::Rate rate(10); // 10 Hz
        while (rclcpp::ok()) {
            checkBoundary();
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

    void sendCommand(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;

        // Publish command for 1 second unless out of bounds
        for (int i = 0; i < 10 && in_bounds_; ++i) {
            cmd_pub_->publish(cmd);
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        // Stop the robot
        cmd.linear.x = 0.0;
        cmd.angular.z = 0.0;
        cmd_pub_->publish(cmd);
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    double x_min_, x_max_, y_min_, y_max_;
    nav_msgs::msg::Odometry robot_odom_;
    bool in_bounds_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_odom_ = *msg;
    }

    void checkBoundary() {
        double x = robot_odom_.pose.pose.position.x;
        double y = robot_odom_.pose.pose.position.y;

        if (x < x_min_ || x > x_max_ || y < y_min_ || y > y_max_) {
            if (in_bounds_) {
                stopRobot();
                RCLCPP_WARN(this->get_logger(), "Robot is out of bounds! Movement stopped.");
                in_bounds_ = false;
            }
        } else {
            in_bounds_ = true; // Reset in_bounds_ flag if robot is within bounds
        }
    }

    void stopRobot() {
        geometry_msgs::msg::Twist stop_msg;
        cmd_pub_->publish(stop_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto boundary_check_node = std::make_shared<BoundaryCheckNode>();

    std::thread boundary_thread([&]() { boundary_check_node->spin(); });

    // Main loop for user commands
    while (rclcpp::ok()) {
        double linear, angular;
        std::cout << "Enter linear velocity (m/s): ";
        std::cin >> linear;
        std::cout << "Enter angular velocity (rad/s): ";
        std::cin >> angular;

        if (boundary_check_node->in_bounds_) {
            boundary_check_node->sendCommand(linear, angular);
            std::cout << "Command sent. Use CTRL+C to stop.\n";
        } else {
            std::cout << "Robot is out of bounds. Adjust position manually to resume control.\n";
        }
    }

    boundary_thread.join();
    rclcpp::shutdown();
    return 0;
}


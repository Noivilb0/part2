#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <iostream>
#include <thread>
#include <atomic>

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode()
        : Node("robot_control_node"),
          linear_velocity_(0.0),
          angular_velocity_(0.0),
          in_bounds_(true) {
        // Subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RobotControlNode::odomCallback, this, std::placeholders::_1));

        // Publisher for velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Boundary limits
        x_min_ = 1.0;
        x_max_ = 9.0;
        y_min_ = 1.0;
        y_max_ = 9.0;
    }

    void run() {
        std::thread input_thread([this]() { getUserInput(); });

        rclcpp::Rate rate(10); // 10 Hz
        while (rclcpp::ok()) {
            if (in_bounds_) {
                publishCommand();
            } else {
                stopRobot();
            }
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }

        input_thread.join();
    }

private:
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    double x_min_, x_max_, y_min_, y_max_;
    std::atomic<double> linear_velocity_;
    std::atomic<double> angular_velocity_;
    nav_msgs::msg::Odometry robot_odom_;
    std::atomic<bool> in_bounds_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_odom_ = *msg;
        double x = robot_odom_.pose.pose.position.x;
        double y = robot_odom_.pose.pose.position.y;

        // Check boundaries
        if (x < x_min_ || x > x_max_ || y < y_min_ || y > y_max_) {
            in_bounds_ = false;
        } else {
            in_bounds_ = true;
        }
    }

    void publishCommand() {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear_velocity_;
        cmd.angular.z = angular_velocity_;
        cmd_pub_->publish(cmd);
    }

    void stopRobot() {
        geometry_msgs::msg::Twist stop_msg;
        stop_msg.linear.x = 0.0;
        stop_msg.angular.z = 0.0;
        cmd_pub_->publish(stop_msg);

        RCLCPP_WARN_ONCE(this->get_logger(), "Robot out of bounds! Stopping.");
    }

    void getUserInput() {
        while (rclcpp::ok()) {
            double linear, angular;
            std::cout << "Enter linear velocity (m/s): ";
            std::cin >> linear;
            std::cout << "Enter angular velocity (rad/s): ";
            std::cin >> angular;

            linear_velocity_ = linear;
            angular_velocity_ = angular;

            std::cout << "Commands will be published until boundary conditions are met.\n";
        }
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotControlNode>();
    node->run();
    rclcpp::shutdown();
    return 0;
}


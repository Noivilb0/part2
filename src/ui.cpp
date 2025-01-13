#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_srvs/srv/set_bool.hpp"
#include "geometry_msgs/msg/point.hpp"
#include <iostream>
#include <thread>
#include <atomic>

class RobotControlNode : public rclcpp::Node {
public:
    RobotControlNode()
        : Node("robot_control_node"),
          linear_velocity_(0.0),
          angular_velocity_(0.0),
          in_bounds_(true),
          stop_robot_(false) {
        // Subscriber for odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&RobotControlNode::odomCallback, this, std::placeholders::_1));

        // Publisher for velocity commands
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Publisher for position in feet
        position_pub_ = this->create_publisher<geometry_msgs::msg::Point>("/position_feet", 10);

        // Service for stopping/restarting the robot
        control_service_ = this->create_service<std_srvs::srv::SetBool>(
            "control_robot", std::bind(&RobotControlNode::controlRobotCallback, this, std::placeholders::_1, std::placeholders::_2));

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
            if (in_bounds_ && !stop_robot_) {
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
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        // Convert position to feet
        geometry_msgs::msg::Point position_feet;
        position_feet.x = x * 3.28;
        position_feet.y = y * 3.28;
        position_feet.z = 0.0;

        // Publish position in feet
        position_pub_->publish(position_feet);

        // Check if the robot is within bounds
        in_bounds_ = (x >= x_min_ && x <= x_max_ && y >= y_min_ && y <= y_max_);
    }

    void controlRobotCallback(const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
                              std::shared_ptr<std_srvs::srv::SetBool::Response> response) {
        stop_robot_ = request->data;
        response->success = true;
    }

    void publishCommand() {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = linear_velocity_;
        cmd_msg.angular.z = angular_velocity_;
        cmd_pub_->publish(cmd_msg);
    }

    void stopRobot() {
        auto cmd_msg = geometry_msgs::msg::Twist();
        cmd_msg.linear.x = 0.0;
        cmd_msg.angular.z = 0.0;
        cmd_pub_->publish(cmd_msg);
    }

    void getUserInput() {
        // Implementation for user input
    }

    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr position_pub_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr control_service_;

    double linear_velocity_;
    double angular_velocity_;
    double x_min_, x_max_, y_min_, y_max_;
    std::atomic<bool> in_bounds_;
    std::atomic<bool> stop_robot_;
};
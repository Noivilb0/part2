#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "std_msgs/msg/float32.hpp"
#include <cmath>
#include <iostream>
#include <string>
#include <thread>
#include <chrono>

class DistanceNode : public rclcpp::Node {
public:
    DistanceNode() : Node("distance_node") {
        // Subscriber for robot odometry
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/odom", 10, std::bind(&DistanceNode::odomCallback, this, std::placeholders::_1));

        // Publisher for distance (single robot monitoring)
        distance_pub_ = this->create_publisher<std_msgs::msg::Float32>("/robot/distance", 10);

        // Publisher to stop the robot
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Initialize threshold and boundary values
        distance_threshold_ = 1.0;
        boundary_min_ = 1.0;
        boundary_max_ = 10.0;
    }

    void spin() {
        rclcpp::Rate rate(10); // 10 Hz
        while (rclcpp::ok()) {
            publishDistance();
            checkProximity();
            rclcpp::spin_some(this->get_node_base_interface());
            rate.sleep();
        }
    }

    void sendCommand(double linear, double angular) {
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = linear;
        cmd.angular.z = angular;

        // Publish command for 1 second
        for (int i = 0; i < 10; ++i) {
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
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr distance_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;

    nav_msgs::msg::Odometry robot_odom_;

    double distance_threshold_;
    double boundary_min_;
    double boundary_max_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        robot_odom_ = *msg;
    }

    void publishDistance() {
        std_msgs::msg::Float32 distance_msg;
        distance_msg.data = calculateDistance(robot_odom_);
        distance_pub_->publish(distance_msg);
    }

    double calculateDistance(const nav_msgs::msg::Odometry& odom) {
        // Assuming the origin as a reference point
        double dx = odom.pose.pose.position.x;
        double dy = odom.pose.pose.position.y;
        return std::sqrt(dx * dx + dy * dy);
    }

    void checkProximity() {
        double distance = calculateDistance(robot_odom_);

        // Stop the robot if it is close to the boundary or below the threshold
        if (distance < distance_threshold_ || isOutOfBounds(robot_odom_)) {
            stopRobot();
            RCLCPP_WARN(this->get_logger(), "Robot is too close to a boundary or threshold. Movement stopped.");
        }
    }

    bool isOutOfBounds(const nav_msgs::msg::Odometry& odom) {
        return odom.pose.pose.position.x < boundary_min_ || odom.pose.pose.position.x > boundary_max_ ||
               odom.pose.pose.position.y < boundary_min_ || odom.pose.pose.position.y > boundary_max_;
    }

    void stopRobot() {
        geometry_msgs::msg::Twist stop_msg;
        cmd_pub_->publish(stop_msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto distance_node = std::make_shared<DistanceNode>();

    std::thread distance_thread([&]() { distance_node->spin(); });

    // Main loop for user commands
    while (rclcpp::ok()) {
        double linear, angular;
        std::cout << "Enter linear velocity: ";
        std::cin >> linear;
        std::cout << "Enter angular velocity: ";
        std::cin >> angular;

        distance_node->sendCommand(linear, angular);
    }

    distance_thread.join();
    rclcpp::shutdown();
    return 0;
}


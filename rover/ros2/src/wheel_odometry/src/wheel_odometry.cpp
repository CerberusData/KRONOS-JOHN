#include "wheel_odometry/wheel_odometry.hpp"

WheelOdometry::WheelOdometry(const rclcpp::NodeOptions & options)
: Node("odometry", options)
{
    RCLCPP_INFO(this->get_logger(), "Odometry constructor");

    /* Publishers */
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odometry/odometry", 50);
    wheel_odom_global_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/wheel_odometry/global_odometry", 50);

    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&WheelOdometry::PubTimerCb, this));
}

void WheelOdometry::PubTimerCb()
{
    std::cout << "Hola" << std::endl;
}
#include "wheel_odometry/wheel_odometry.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto odom_node = std::make_shared<WheelOdometry>(options);

    RCLCPP_WARN(odom_node->get_logger(), "Init Wheel Odometry");
    rclcpp::spin(odom_node);
    rclcpp::shutdown();

    return 0;
}
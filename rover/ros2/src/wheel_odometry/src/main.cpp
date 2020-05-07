#include "wheel_odometry/wheel_odometry.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto Odom_node = std::make_shared<WheelOdometry>(options);

    RCLCPP_WARN(Odom_node->get_logger(), "Init Wheel Odometry");
    // while (rclcpp::ok())
    // {
    rclcpp::spin(Odom_node);
    // }
    rclcpp::shutdown();

    return 0;
}
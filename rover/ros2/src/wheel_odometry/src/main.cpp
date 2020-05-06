#include "wheel_odometry/wheel_odometry.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto WheelOdom_node = std::make_shared<WheelOdometry>(options);

    RCLCPP_WARN(WheelOdom_node->get_logger(), "Init Wheel Odometry");
    while (rclcpp::ok())
    {
        rclcpp::spin(WheelOdom_node);
    }
    rclcpp::shutdown();

    return 0;
}
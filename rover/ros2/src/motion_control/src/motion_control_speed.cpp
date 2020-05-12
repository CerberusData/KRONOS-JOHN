#include "motion_control/speed_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    auto speed_node = std::make_shared<SpeedController>(options);
    
    auto period = std::chrono::milliseconds(100);
    rclcpp::Rate r(period);

    RCLCPP_WARN(speed_node->get_logger(), "Init Speed Controller");
    
    while(rclcpp::ok())
    {
        speed_node->Controller();
        rclcpp::spin_some(speed_node);
        r.sleep();
    }
    rclcpp::shutdown();

    return 0;
}
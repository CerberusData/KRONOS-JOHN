#include "motion_control/speed_controller.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto speed_node = std::make_shared<SpeedController>(options);
    
    RCLCPP_WARN(speed_node->get_logger(), "Init Speed Controller");
    rclcpp:spin(speed_node);
    rclcpp::shutdown();

    return 0;
}
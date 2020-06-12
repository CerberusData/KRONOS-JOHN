#include "supervisor/supervisor.hpp"

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;

    auto supervisor_node = std::make_shared<Supervisor>(options);

    RCLCPP_WARN(supervisor_node->get_logger(), "Init Supervisor");

    rclcpp::spin(supervisor_node);
    rclcpp::shutdown();

    return 0;
}
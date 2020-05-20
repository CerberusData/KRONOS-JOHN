#include "can_test/can_test.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor;



    std::string str_name = "vcan0";
    const char* int_name_ = str_name.c_str();

    auto can_driver = new CANDriver(int_name_);
    // auto can_driver = std::make_shared<CANDriver>(int_name_);
    auto test_node = std::make_shared<CANTest>(options, can_driver);

    RCLCPP_WARN(test_node->get_logger(), "Init Test");
    executor.add_node(test_node);
    executor.spin();
    rclcpp::shutdown();

    return 0;

}
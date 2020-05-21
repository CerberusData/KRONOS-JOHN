#include "can_test/can_test.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor;

    /* CAN Driver creation */
    std::string Mystr = "can0";
    const char *interface_name_ = Mystr.c_str();    
    // auto can_dvr_ = new CANDriver(interface_name_);
    
    auto can_dvr_ = std::make_shared<CANDriver>(interface_name_);
    auto test_node = std::make_shared<CANTest>(options, can_dvr_);

    RCLCPP_WARN(test_node->get_logger(), "Init Test");

    executor.add_node(test_node);
    executor.spin();
    
    rclcpp::shutdown();
    return 0;

}
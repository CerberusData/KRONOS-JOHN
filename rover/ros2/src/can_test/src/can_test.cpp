#include "can_test/can_test.hpp"

CANTest::CANTest(const rclcpp::NodeOptions & options, CANDriver* can_driver)
: Node("can_test", options)
{


    leds_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/test/leds", 10, std::bind(&CANTest::LedsCb, this, _1));
    battery_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/test/battery", 10, std::bind(&CANTest::BatteryCb, this, _1));
    chassis_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/test/chassis", 10, std::bind(&CANTest::ChassisCb, this, _1));

    can_dvr_ = can_driver;
    Configuration();
}


void CANTest::LedsCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Leds Callback");
}

void CANTest::BatteryCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Battery Callback");
}

void CANTest::ChassisCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Chassis Callback");
}

void CANTest::Configuration()
{
    RCLCPP_INFO(this->get_logger(), "Initial configuration");
    // can_dvr_->CANWrite(CHASSIS_ADDRESS, 8, data);


}
void CANTest::SendLeds()
{

}

void CANTest::SendBattery()
{

}

void CANTest::SendChassis()
{


}


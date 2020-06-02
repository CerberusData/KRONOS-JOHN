/*
    - File name: battery.cpp
    - This file defines members and member functions for the battery in ROS
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#include "canlink/modules/battery.hpp"
 
Battery::Battery(const rclcpp::NodeOptions & options, std::shared_ptr<CANDriver> can_driver)
: Node("battery", options)
{
    RCLCPP_INFO(this->get_logger(), "Battery init");
    /* Publishers */
    battery_pub_ = this->create_publisher<sensor_msgs::msg::BatteryState>("/canlink/battery/data", 10);
    battery_status_pub_ = this->create_publisher<std_msgs::msg::String>("/canlink/battery/status", 10);
    battery_FR_pub_ = this->create_publisher<std_msgs::msg::Float32>("/canlink/battery/voltage_FR", 10);

    auto battery_status_msg = std::make_unique<std_msgs::msg::String>();

    if (can_driver)
    {
        battery_status_msg->data = "OK";
    }
    else
    {
        battery_status_msg->data = "CAN bus connection Error";
    }
    battery_status_pub_->publish(std::move(battery_status_msg));
}

void Battery::PublishBatteryStatus(uint8_t integer_msg, uint8_t decimal_msg, float current)
{
    /* Battery data */ 
    auto battery_msg = std::make_unique<sensor_msgs::msg::BatteryState>();
    battery_msg->header.stamp = this->now();
    battery_msg->voltage = (float)integer_msg + (float)(decimal_msg / 100.0);
    battery_msg->current = current;
    battery_pub_->publish(std::move(battery_msg));

    /* Battery status */ 
    auto battery_status_msg = std::make_unique<std_msgs::msg::String>();
    battery_status_msg->data = "OK";
    battery_status_pub_->publish(std::move(battery_status_msg));

    /* Battery data for FR */ 
    auto battery_msg_FR = std::make_unique<std_msgs::msg::Float32>();
    battery_msg_FR->data = (float)integer_msg + (float)(decimal_msg / 100.0);
    battery_FR_pub_->publish(std::move(battery_msg_FR));
}

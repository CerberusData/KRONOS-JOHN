/*
    - File name: battery.h
    - This library defines members and member functions for the battery module's communication through CAN
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#ifndef CAN_BATTERY_H_INCLUDED
#define CAN_BATTERY_H_INCLUDED

#include <memory>
#include <vector>
#include <chrono>

#include "rclcpp/rclcpp.hpp"
#include "canlink/socketCAN.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"
#include "sensor_msgs/msg/battery_state.hpp"

class Battery : public rclcpp::Node
{
    public:
        Battery(const rclcpp::NodeOptions & options, CANDriver *can_driver);
        ~Battery(){};

        /* Public functions */
        void PublishBatteryStatus(uint8_t integer_msg, uint8_t decimal_msg, float current);

    private:
        /* Publishers */
        rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub_;
        rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr battery_FR_pub_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr battery_status_pub_;
        
        /* Objects */ 
        CANDriver *can_driver_;
};
#endif

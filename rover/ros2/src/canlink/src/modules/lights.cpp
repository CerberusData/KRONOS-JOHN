/*
    - File name: lights.cpp
    - This library defines members and member functions for the battery module's communication through CAN
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/

#include "canlink/modules/lights.hpp"

Lights::Lights(const rclcpp::NodeOptions & options, 
    std::shared_ptr<CANDriver> can_driver) : Node("lights", options)
{
    RCLCPP_DEBUG(this->get_logger(), "Lights init");
    can_driver_ = can_driver;
    
    uint8_t data[3] = {LIGHTS_ADDRESS, led1_sts_, led2_sts_}; 
    if (can_driver_)
    {
        can_driver_->CANWrite(CHASSIS_ADDRESS, 3, data);
    }
    else
    {
        can_driver_->CANWrite(CHASSIS_ADDRESS, 3, data);
    }
}


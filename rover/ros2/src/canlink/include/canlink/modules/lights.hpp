/*
    - File name: lights.h
    - This library defines members and member functions for the battery module's communication through CAN
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#ifndef CAN_LIGHTS_H_INCLUDED
#define CAN_LIGHTS_H_INCLUDED

#include <memory>
#include <utility>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "canlink/socketCAN.hpp"
#include "utils/console.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float32.hpp"

#define LIGHTS_ADDRESS 0x0B
#define CHASSIS_ADDRESS 0x45A

class Lights : public rclcpp::Node
{
    public:
        Lights(const rclcpp::NodeOptions & options, 
            std::shared_ptr<CANDriver> can_driver);
        ~Lights(){};
 
    private: 
        /* Modules */
        std::shared_ptr<CANDriver> can_driver_;
        
        /* Enviroment variables */
        uint8_t led1_sts_ = getEnv("LIGHT1_STATUS", 0x00);
        uint8_t led2_sts_ = getEnv("LIGHT1_STATUS", 0x00);
};
#endif  /* End of CAN_LIGHTS_H_INCLUDED */

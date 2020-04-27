/*
    - File name: CANCabin.hpp
    - This file defines members and member functions for the battery in ROS
    - By: Andres Rengifo
    - Email: andres@kiwicampus.com
*/
 
#ifndef CAN_CABIN_H_INCLUDED
#define CAN_CABIN_H_INCLUDED

#include <memory>
#include <vector>
#include <string>
#include <chrono>

#include <rclcpp/rclcpp.hpp>

/* Custom libraries */
#include "canlink/socketCAN.hpp"
#include "utils/console.hpp"

/* ROS2 Messages */
#include "std_msgs/msg/string.hpp"
 
/* Modules */
#include "canlink/modules/lock_system.hpp"

class CANCabin : public rclcpp::Node
{
    public:
        CANCabin(const rclcpp::NodeOptions & options, CANDriver *can_driver);
        ~CANCabin(){};

        /* Functions*/
        void StartCANCabinRead();
    
    private:
        /* Modules */
        LockSystem *lock_system_;
        CANDriver *can_driver_;

        /* Member functions */
        void PublishCANCabinInfo(struct can_frame *frame);
        
        /* Thread to Listen CAN port */
        std::thread read_thread_; 
        
        /* Double check */
        // const char *interface_name_ = getEnv("CANLINK_CABIN_INTERFACE", "can1").c_str();
        std::string Mystr = "can1";
        const char *interface_name_ = Mystr.c_str();

        rclcpp::TimerBase::SharedPtr pub_timer_; // RMV
        void Pub_Cb_();  //RMV
};
#endif  /* End of CAN_CABIN_H_INCLUDED */ 
 

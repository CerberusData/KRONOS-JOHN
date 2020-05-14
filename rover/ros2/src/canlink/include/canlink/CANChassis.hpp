/*
    - File name: CANChassis.hpp
    - This library defines members and member functions for the CANnode
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#ifndef CAN_ROS_H_INCLUDED
#define CAN_ROS_H_INCLUDED

#include <memory>
#include <vector>
#include <chrono>
#include <thread>

#include "rclcpp/rclcpp.hpp"

#include "canlink/modules/chassis.hpp"
#include "canlink/modules/battery.hpp"
#include "canlink/modules/lights.hpp"

#include "std_msgs/msg/string.hpp"

 
class CANChassis : public rclcpp::Node
{
    public:
        CANChassis(const rclcpp::NodeOptions & options, CANDriver *can_driver);
        ~CANChassis(){};

        /* Functions */
        void StartCANBusRead();

        
    private:
        /* Publishers */
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr can_dvr_status_pub_;
        
        /* Publishers Messages */
        std_msgs::msg::String can_dvr_status_msg_;

        /* Modules */
        Battery* battery_;
        Chassis* chassis_;
        Lights* lights_;
        CANDriver* can_driver_;

        /* Member functions */
        void PublishCANInfo(struct can_frame *frame);

        /* Thread to Listen CAN port */
        std::thread read_thread_;

        /* Double check */
        // const char *interface_name_ = getEnv("CANLINK_CHASSIS_INTERFACE", "can0").c_str();
        std::string Mystr = "can0";
        const char *interface_name_ = Mystr.c_str();

        //rclcpp::TimerBase::SharedPtr pub_timer_; // RMV
        //void Pub_Cb_();  // RMV

};
#endif  /* End of CAN_CHASSIS_H_INCLUDED */ 
 
 

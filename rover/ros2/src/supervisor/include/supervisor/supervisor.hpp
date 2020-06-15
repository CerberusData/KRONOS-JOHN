#ifndef SUPERVISOR_H_INCLUDED
#define SUPERVISOR_H_INCLUDED

#include <memory>
#include <utility>

#include "rclcpp/rclcpp.hpp"

// ROS 2 Messages
#include "std_msgs/msg/bool.hpp"

// ROS 2 Services
#include "std_srvs/srv/set_bool.hpp"


using std::placeholders::_1;

class Supervisor : public rclcpp::Node
{
    public:
        Supervisor(rclcpp::NodeOptions & options);
        ~Supervisor(){};

    private:
        // Subscribers
        rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
        
        // Clients
        rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_clt_;

        // Subscribers callbacks
        void ArmCb(const std_msgs::msg::Bool::SharedPtr msg);
};


#endif


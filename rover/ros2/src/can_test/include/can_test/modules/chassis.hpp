/*
    - File name:chassis.h
    - This library defines members and member functions for the CAN communication with the chassis
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#ifndef CAN_CHASSIS_H_INCLUDED
#define CAN_CHASSIS_H_INCLUDED

#include <memory>
#include <utility>
#include <vector>
#include <math.h>

/* ROS2 Default */
#include <rclcpp/rclcpp.hpp>

/* Custom libraries */
#include "can_test/socketCAN.hpp"

/* ROS2 Services */
#include "std_srvs/srv/set_bool.hpp"
 
using std::placeholders::_1;
using std::placeholders::_2;
using std::placeholders::_3;

class Chassis : public rclcpp::Node
{
    public:
        /* Constructor */
        Chassis(const rclcpp::NodeOptions & options);
        ~Chassis(){};

    private:
        /* Services */
        rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr arm_srv_;

        /* Services callbacks */
        bool ArmCb(
            const std::shared_ptr<rmw_request_id_t> request_header,
            const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
            std::shared_ptr<std_srvs::srv::SetBool::Response> response);

};
#endif  /* End of CAN_CHASSIS_H_INCLUDED */ 

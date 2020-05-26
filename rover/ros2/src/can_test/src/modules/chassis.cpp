/*
    - File name: chassis.cpp.
    - This library defines members and member functions for the CAN communication with the chassis
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
#include "can_test/modules/chassis.hpp"

Chassis::Chassis(const rclcpp::NodeOptions & options)
: Node("chassis", options)
{   
    RCLCPP_INFO(this->get_logger(), "Chassis init");

    /* Services */
    arm_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/canlink/chassis/arm", std::bind(&Chassis::ArmCb, this, _1, _2, _3));
}

/* ------------------------------------------------------------------------- */
/* Services callbacks */
bool Chassis::ArmCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    bool arm_req = request->data;

    if ((arm_req == true))
    {
        /*
            Arming command
        */
        RCLCPP_INFO(this->get_logger(), "----- Arming chassis -----");
        response->success = true;
        response->message = "Armed";
    }

    if ((arm_req == false))
    {
        /*
            Disarming command
        */
        RCLCPP_INFO(this->get_logger(), "----- Disarming Chassis -----");
        response->success = true;
        response->message = "Disarmed";
    }        
    return true;
}

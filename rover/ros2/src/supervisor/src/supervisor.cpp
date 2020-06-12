#include "supervisor/supervisor.hpp"

Supervisor::Supervisor(rclcpp::NodeOptions & options)
: Node("supervisor", options)
{
    RCLCPP_INFO(this->get_logger(), "Supervisor init");

    // Subscribers
    arm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/local_client/arm", 10, std::bind(&Supervisor::ArmCb, this, _1)
    );

    // Clients
    arm_clt_ = this->create_client<std_srvs::srv::SetBool>(
        "/canlink/chassis/arm" 
    );

}
    
void Supervisor::ArmCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    /*
        Description:
            - Callback for the subscriber "/local_client/arm"
            - Handles the arm request from the local client to call the 
            corresponding service.
    */

    RCLCPP_DEBUG(this->get_logger(), "Client request");
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = msg->data;
    arm_clt_->async_send_request(request);
}
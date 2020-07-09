#include "supervisor/supervisor.hpp"

Supervisor::Supervisor(rclcpp::NodeOptions &options)
    : Node("supervisor", options)
{
    RCLCPP_INFO(this->get_logger(), "Supervisor init");

    // Subscribers
    arm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/local_client/arm", 10, std::bind(&Supervisor::ArmCb, this, _1));

    // Clients
    arm_clt_ = this->create_client<std_srvs::srv::SetBool>(
        "/canlink/chassis/arm");

    // Parameters client (To can_test Node)
    parameters_client_ = std::make_shared<rclcpp::SyncParametersClient>(
        this, "can_test");

    while (!parameters_client_->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
    }
    
    RCLCPP_DEBUG(this->get_logger(), "Listing parameters...");

    auto parameters = parameters_client_->get_parameters({"davidson"});


    // Get a few of the parameters just set.
    for (auto &parameter : parameters)
    {
        /*
            List Parameter name, type and value
        */
        std::cout << "Parameter name: " << parameter.get_name() << std::endl;
        std::cout << "Parameter value ("<< parameter.get_type_name() << "): " 
            << parameter.value_to_string() << std::endl;
        can_test_param_ = parameter.get_value<double>();
    }
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
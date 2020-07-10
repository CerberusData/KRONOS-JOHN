#include "supervisor/supervisor.hpp"

Supervisor::Supervisor(rclcpp::NodeOptions &options)
    : Node("supervisor", options)
{
    RCLCPP_INFO(this->get_logger(), "Supervisor init");

    // Subscribers
    if (local_client_ == true)
    {
        arm_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/local_client/arm", 
            10, 
            std::bind(&Supervisor::ArmCb, this, _1));
    }

    chs_status_sub_ = this->create_subscription<usr_msgs::msg::ChassisState>(
        "/canlink/chassis/status", 
        10, 
        std::bind(&Supervisor::ChassisStateCb, this, _1));

    // ToDo (Kmilo7204) Subscriber to handle the chassis state

    // Clients
    arm_clt_ = this->create_client<std_srvs::srv::SetBool>(
        "/canlink/chassis/arm");

    // Parameters client (To can_test Node)
    param_clt_ = std::make_shared<rclcpp::SyncParametersClient>(
        this, "can_test");

    // Parameters client reader
    ParamsClientReader(param_clt_);
    
    // Memory allocation for can_test_params
    can_test_params_.reserve(1);
    can_test_params_ = param_clt_->get_parameters({"davidson"});
    
    // Parameters extraction
    ParamsExtractor(can_test_params_, false);
}

void Supervisor::ParamsClientReader(
    const rclcpp::SyncParametersClient::SharedPtr param_clt)
{
    while (!param_clt->wait_for_service(std::chrono::seconds(1)))
    {
        if (!rclcpp::ok())
        {
            RCLCPP_ERROR(
                this->get_logger(), "Interrupted while waiting for the service. Exiting.");
            rclcpp::shutdown();
        }

        RCLCPP_INFO(this->get_logger(), "Service not available, waiting again...");
    }

}

void Supervisor::ParamsExtractor(
    std::vector<rclcpp::Parameter> param_vct, bool debug)
{
    RCLCPP_INFO(this->get_logger(), "Listing parameters...");
    // Iterate through the parameters vector to extract the value
    for (auto &parameter : param_vct)
    {
        if (debug == true)
        {
            std::cout << "Parameter name: " << parameter.get_name() << std::endl;
            std::cout << "Parameter value ("<< parameter.get_type_name() << "): " 
                << parameter.value_to_string() << std::endl;
        }

        // Extract the desired value
        davidson_param_ = parameter.get_value<double>();
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

    RCLCPP_DEBUG(this->get_logger(), "Local client request");
    auto request = std::make_shared<std_srvs::srv::SetBool::Request>();
    request->data = msg->data;
    arm_clt_->async_send_request(request);

}

void Supervisor::ChassisStateCb(const usr_msgs::msg::ChassisState::SharedPtr msg)
{
    chassis_state_ = msg;
}
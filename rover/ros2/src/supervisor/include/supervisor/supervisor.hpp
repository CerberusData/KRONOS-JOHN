#ifndef SUPERVISOR_H_INCLUDED
#define SUPERVISOR_H_INCLUDED

#include <memory>
#include <chrono>
#include <utility>

#include "rclcpp/rclcpp.hpp"

// ROS 2 Messages
#include "std_msgs/msg/bool.hpp"

// Custom Messages
#include "usr_msgs/msg/chassis_state.hpp"

// ROS 2 Services
#include "std_srvs/srv/set_bool.hpp"

// Custom libraries
#include "utils/console.hpp"

using std::placeholders::_1;


class Supervisor : public rclcpp::Node
{
public:
    Supervisor(rclcpp::NodeOptions & options);
    ~Supervisor(){};

private:
    // Subscribers
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr arm_sub_;
    rclcpp::Subscription<usr_msgs::msg::ChassisState>::SharedPtr chs_status_sub_;
    
    // Clients
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr arm_clt_;
    rclcpp::SyncParametersClient::SharedPtr param_clt_; 

private:
    // Subscribers callbacks
    void ArmCb(const std_msgs::msg::Bool::SharedPtr msg);
    void ChassisStateCb(const usr_msgs::msg::ChassisState::SharedPtr msg);

private:
    // Member functions
    void ParamsClientReader(
        const rclcpp::SyncParametersClient::SharedPtr param_clt);
    void ParamsExtractor(
        std::vector<rclcpp::Parameter> param_vct, bool debug=false);
    
private: 
    std::vector<rclcpp::Parameter> can_test_params_;
    std::shared_ptr<usr_msgs::msg::ChassisState> chassis_state_;

private:
    // ToDo (Kmilo7204) setter and getter
    double davidson_param_ = 0.0;

private:
    // Environment variables
    bool local_client_ = getEnv("LOCAL_CLIENT", false);

};


#endif


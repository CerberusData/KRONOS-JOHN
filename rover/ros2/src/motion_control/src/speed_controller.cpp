#include "motion_control/speed_controller.hpp"

SpeedController::SpeedController(rclcpp::NodeOptions & options)
: Node("speed_controller", options)
{
    RCLCPP_INFO(this->get_logger(), "Speed Controller constructor");

    /* Publishers */
    output_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/motion_control/speed_controller/output", 10);

    /* Subscribers */ 
    driving_cmd_fr_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/freedom_client/reference_commands", 10, std::bind(&SpeedController::CommandsCb, this, _1));

    // ros2 topic pub --once /freedom_client/reference_commands geometry_msgs/msg/TwistStamped "{linear: {x: 1.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    // ros2 topic pub --once /freedom_client/reference_commands geometry_msgs/msg/TwistStamped "{linear: {x: -1.5, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
    // ros2 topic pub --once /freedom_client/reference_commands geometry_msgs/msg/TwistStamped "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}"
    // ros2 topic pub --once /freedom_client/reference_commands geometry_msgs/msg/TwistStamped "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"
    // ros2 topic pub --once /freedom_client/reference_commands geometry_msgs/msg/TwistStamped "{linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}"

    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -1.0}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

    // ros2 topic pub --once /test/leds std_msgs/msg/Bool "{data: true}"
    // ros2 topic pub --once /test/config std_msgs/msg/Bool "{data: true}"
    // ros2 topic pub --once /test/chassis std_msgs/msg/Bool "{data: true}"
    // ros2 service call /test/arm std_srvs/srv/SetBool "{data: true}"
    // ros2 service call /canlink/chassis/arm std_srvs/srv/SetBool "{data: true}"
    // ros2 service call /canlink/chassis/arm std_srvs/srv/SetBool "{data: false}"


}

void SpeedController::CommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    RCLCPP_WARN(this->get_logger(), "Commands Callback");
    auto commands_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    /* Control commands assignation */
    commands_msg->header.stamp = this->now();
    commands_msg->twist.linear.x = msg->twist.linear.x;
    commands_msg->twist.linear.y = 0.0f; 
    commands_msg->twist.linear.z = 0.0f;
    commands_msg->twist.angular.x = 0.0f;
    commands_msg->twist.angular.y = 0.0f; 
    commands_msg->twist.angular.z = msg->twist.angular.z;

    output_cmd_pub_->publish(std::move(commands_msg));
}

float SpeedController::ThrottlePID(float vx_ref)
{
    if (vx_ref == 0.0f)
    {
        return 0.0f;
    }
    return 0.0f;
}

void SpeedController::Controller()
{
    RCLCPP_INFO(this->get_logger(), "Controller function");
}
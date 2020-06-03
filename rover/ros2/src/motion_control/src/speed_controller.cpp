

#include "motion_control/speed_controller.hpp"

SpeedController::SpeedController(rclcpp::NodeOptions & options)
: Node("speed_controller", options)
{
    RCLCPP_INFO(this->get_logger(), "Speed Controller constructor");

    // Publishers
    output_cmd_pub_ = this->create_publisher<geometry_msgs::msg::TwistStamped>(
        "/motion_control/speed_controller/output", 10);

    // Subscribers
    driving_cmd_fr_sub_ = this->create_subscription<geometry_msgs::msg::TwistStamped>(
        "/freedom_client/joystick_commands", 10, std::bind(&SpeedController::CommandsCb, this, _1));
    odometry_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "/wheel_odometry/global_odometry", 50, std::bind(&SpeedController::OdometryCb, this, _1));

    prev_time_ = this->now();

    // Soft Speed object
    linear_soft_spline = std::make_shared<SoftSpeedSpline>(0.8f);
}

void SpeedController::OdometryCb(const nav_msgs::msg::Odometry::SharedPtr msg)
{
    /*
        - Odometry callback to extract the current velocity and pose comming from 
        Wheel odometry
        - We are using global wheel odometry, but I need to check if we should 
        use the local one.
    */
    robot_twist_.twist = msg->twist.twist;

    double roll = 0.0f;
    double pitch = 0.0f;
    tf2::Quaternion q(msg->pose.pose.orientation.x,
                    msg->pose.pose.orientation.y,
                    msg->pose.pose.orientation.z,
                    msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    m.getRPY(roll, pitch, bot_yaw_);

    if (first_yaw_value == false)
    {
        yaw_set_point_ = bot_yaw_;
        first_yaw_value = true;
    }
}

void SpeedController::CommandsCb(const geometry_msgs::msg::TwistStamped::SharedPtr msg)
{
    // Reference commands for the robot motion
    reference_cmd_.header.stamp = this->now();
    reference_cmd_.twist = msg->twist;
}

float SpeedController::ThrottlePID(float ref_vx, float cur_vx, double dt)
{
    /*
        (Reference, Current, dt)
    */

    if (ref_vx == 0.0f)
    {
        return 0.0f;
    }

    if (throttle_ctrl_ == true)
    {
        float smooth_ff = 1.0;
        float e_k = ref_vx - cur_vx;
        float prop_error = e_k;
        float der_error = (dt != 0.0f) ? (e_k - e_k1_) / dt : 0.0f;
        int_error_ += int_error_ * dt;
        e_k1_ = e_k;

        // Smoothing the predictive term when changing direction
        if (((prev_ref_vx_ < 0.0f) && (ref_vx - prev_ref_vx_ > 0.0f)) 
            || ((prev_ref_vx_ > 0.0f) && (ref_vx - prev_ref_vx_ < 0.0f)))
        {   
            smooth_ff = 0.5f;
        }

        // PID command with predictive term (FF) and reactive term (FB)
        float u_cmd = (smooth_ff * kff_thr_ * cur_vx) 
            + (kp_thr_ * (prop_error + (ki_thr_ * int_error_) + (kd_thr_ * der_error)));
        prev_ref_vx_ = ref_vx;
        return u_cmd;
    }

    else
    {
        return ref_vx;
    }
}

void SpeedController::Controller()
{
    RCLCPP_DEBUG(this->get_logger(), "Controller function");

    auto output_cmd_msg = std::make_unique<geometry_msgs::msg::TwistStamped>();

    float lin_vx = reference_cmd_.twist.linear.x;
    float ang_wz = reference_cmd_.twist.angular.z;

    rclcpp::Time curr_time = this->now();
    double dt = (curr_time - prev_time_).seconds();
    prev_time_ = this->now();

    // ToDo: Smart break conditions for handling the speed scaling factor
    // ToDo: Implementation of soft speed curve

    // (reference, acc_factor)
    float vx_ref = linear_soft_spline->CalculateSoftSpeed(lin_vx, 1.0f);

    // (reference, current, dt)
    output_cmd_msg->twist.linear.x = ThrottlePID(vx_ref, robot_twist_.twist.linear.x, dt);
    output_cmd_msg->twist.angular.z = ang_wz;

    output_cmd_pub_->publish(std::move(output_cmd_msg));
}

    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 1.0}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: -1.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.3, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -0.5}}}"
    // ros2 topic pub -r 10 /motion_control/speed_controller/output geometry_msgs/msg/TwistStamped "{twist: {linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}}"

    // ros2 service call /canlink/chassis/arm std_srvs/srv/SetBool "{data: true}"
    
    // ros2 service call /canlink/chassis/arm std_srvs/srv/SetBool "{data: false}"
    // ros2 service call /test/arm std_srvs/srv/SetBool "{data: true}"

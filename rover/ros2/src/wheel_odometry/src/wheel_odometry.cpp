#include "wheel_odometry/wheel_odometry.hpp"
#define PI 3.1516

WheelOdometry::WheelOdometry(const rclcpp::NodeOptions & options)
: Node("odometry", options)
{
    RCLCPP_INFO(this->get_logger(), "Odometry constructor");

    /* Publishers */
    wheel_odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wheel_odometry/odometry", 50);
    wheel_odom_global_pub_ = this->create_publisher<nav_msgs::msg::Odometry>(
        "/wheel_odometry/global_odometry", 50);

    /* Subscribers */
    imu_sub_ = this->create_subscription<sensor_msgs::msg::Imu>(
        "/imu/data_bot", 10, std::bind(&WheelOdometry::ImuCb, this, _1));
    motor_status_sub_ = this->create_subscription<usr_msgs::msg::Motors>(
        "/canlink/chassis/motors_status", 10, std::bind(&WheelOdometry::MotorStatusCb, this, _1));
    movement_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/imu/move_state", 10, std::bind(&WheelOdometry::MovementCb, this, _1));

    /* Services */
    restart_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/wheel_odometry/restart", std::bind(&WheelOdometry::RestartCb, this, _1, _2, _3));

    prev_time_ = this->now();
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&WheelOdometry::PubTimerCb, this));
}

/* Callbacks */
bool WheelOdometry::RestartCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    /* 
        This service is in charge of restarting the odometry values and set the
        IMU offset for the new movement.
    */
    (void) request_header;
    (void) request;

    wheel_odom_msg_.pose.pose.position.x = 0.0f;
    wheel_odom_msg_.pose.pose.position.y = 0.0f;
    wheel_odom_msg_.pose.pose.position.z = 0.0f;
    
    imu_yaw_offset_ = imu_yaw_;

    /* Quaternion representation */
    tf2::Quaternion quat;
    quat.setRPY(0.0f, 0.0f, 0.0f);

    /* Quaternion assignation */
    wheel_odom_msg_.pose.pose.orientation.x = quat[0];
    wheel_odom_msg_.pose.pose.orientation.y = quat[1];
    wheel_odom_msg_.pose.pose.orientation.z = quat[2];
    wheel_odom_msg_.pose.pose.orientation.w = quat[3];
    wheel_odom_pub_->publish(wheel_odom_msg_);

    /* Service response */
    response->success = true;
    response->message = "Odometry Restarted";
    
    return true;
} 

void WheelOdometry::MovementCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    /* 
        Callback for the topic /imu/move_state. It represents if the Kiwibot is
        moving or not based on the IMU data.
    */
    imu_state_ = msg->data;
}

void WheelOdometry::MotorStatusCb(const usr_msgs::msg::Motors::SharedPtr msg)
{
    /*
        Callbak for the topic canlink/chassis/motors_status. Custom message con-
        taining the following elements:
          - header
          - rpm
          - current
          - error_status
        
        Located at usr_msgs/msg/canlink
    */
    motors_rpm_ = msg->rpm;
    motors_curr_ = msg->current;
}

void WheelOdometry::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    /*
        Callback for the topic /imu/data_bot. It updates a Quaternion to extract
        the current eualer angles.
    */

    tf2::Quaternion q(msg->orientation.x, 
                    msg->orientation.y,
                    msg->orientation.z, 
                    msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(imu_roll_, imu_pitch_, imu_yaw_);
    RCLCPP_INFO(this->get_logger(), "Pitch: %0.4f", imu_pitch_);
    RCLCPP_INFO(this->get_logger(), "Roll: %0.4f", imu_roll_);
    RCLCPP_INFO(this->get_logger(), "Yaw: %0.4f", imu_yaw_);

    imu_omega_ = msg->angular_velocity.z;

    if (imu_published_ == false)
    {
        /*
            It checks if the IMU has published messages.
        */
        imu_published_ = true;
    }
}

/* Functions */
float WheelOdometry::CalculateSlipFactor(float kin_omega, float imu_omega)
{
    float alpha = 1.0f;
    if (kin_omega != 0.0f)
    {
        alpha = 1.0f - ((kin_omega - imu_omega) / 2.0f); 
        alpha = alpha < 0.0f ? 0.0f : alpha;
        alpha = alpha > 1.0f ? 1.0f : alpha;
    }

    return alpha;
}

void WheelOdometry::CalculateOdometry()
{
    /*
        Function to calculate the Wheel Odometry (Local and Global)
    */

    /* Wheels linear velocities */
    float _FR_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[0]) / 60.0f; 
    float _RR_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[1]) / 60.0f; 
    float _RL_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[2]) / 60.0f; 
    float _FL_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[3]) / 60.0f; 

    /* Left and Right linear velocities */
    float _R_vel = -(_FR_vel + _RR_vel) / 2.0f;  
    float _L_vel = (_FL_vel + _RL_vel) / 2.0f;
    float _X_vel = (_R_vel + _L_vel) / 2.0f;
    float _kin_omega = (_R_vel - _L_vel) / bot_track_;

    rclcpp::Time curr_time = this->now();
    double dt = (curr_time - prev_time_).seconds();
    prev_time_ = this->now();

    /* Filling a Quaternion with the current euler angles */
    tf2::Quaternion quat;
    quat.setRPY(imu_roll_, imu_pitch_, imu_yaw_ - imu_yaw_offset_);

    /* Header assignation */
    wheel_odom_msg_.header.stamp = this->now();
    wheel_odom_msg_.header.frame_id = "odom";
    wheel_odom_msg_.child_frame_id = "base_link";
    /* Quaternion assignation */
    wheel_odom_msg_.pose.pose.orientation.x = quat[0];
    wheel_odom_msg_.pose.pose.orientation.y = quat[1];
    wheel_odom_msg_.pose.pose.orientation.z = quat[2];
    wheel_odom_msg_.pose.pose.orientation.w = quat[3];
    /* Speeds in X and Y axes */
    float _X_dot = _X_vel * cos(imu_yaw_ - imu_yaw_offset_);
    float _Y_dot = _X_vel * sin(imu_yaw_ - imu_yaw_offset_);
    /* Adding displacement in [m] to the current message */
    wheel_odom_msg_.pose.pose.position.x += _X_dot * dt;
    wheel_odom_msg_.pose.pose.position.y += _Y_dot * dt;
    wheel_odom_msg_.pose.pose.position.z = 0.0f;
    wheel_odom_msg_.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                                        0, 0.1, 0, 0, 0, 0,
                                        0, 0, 0.2, 0, 0, 0,
                                        0, 0, 0, 0.001, 0, 0,
                                        0, 0, 0, 0, 0.001, 0,
                                        0, 0, 0, 0, 0, 0.002};
    /* Velocities assignation */
    wheel_odom_msg_.twist.twist.linear.x = _X_vel;
    wheel_odom_msg_.twist.twist.linear.y = 0.0f;
    wheel_odom_msg_.twist.twist.linear.z = 0.0f;
    wheel_odom_msg_.twist.twist.angular.x = 0.0f;
    wheel_odom_msg_.twist.twist.angular.y = 0.0f;
    wheel_odom_msg_.twist.twist.angular.z = imu_omega_;

    /* Testing purposes */
    wheel_odom_msg_.twist.covariance = {0.0000001, 0, 0, 0, 0, 0,
                                    0, 0.000000001, 0, 0, 0, 0,
                                    0, 0, 0.00000002, 0, 0, 0,
                                    0, 0, 0, 0.01, 0, 0,
                                    0, 0, 0, 0, 0.01, 0,
                                    0, 0, 0, 0, 0, 0.002};

    /* Twist covariance tunning (Velocities) */
    // if ((imu_state_ == true) && _X_vel == 0.0)
    // { //Robot is moving but wheels not (e.g. robot is being carried)
    //     /* 
    //         When robot is moving, but wheels not, hence we should avoid adding
    //         values to the linear components.
    //     */
    //     wheel_odom_msg_.twist.covariance = {999, 0, 0, 0, 0, 0,
    //                                         0, 999, 0, 0, 0, 0,
    //                                         0, 0, 999, 0, 0, 0,
    //                                         0, 0, 0, 0.001, 0, 0,
    //                                         0, 0, 0, 0, 0.001, 0,
    //                                         0, 0, 0, 0, 0, 0.002};
    // }
    // else
    // {
    //     /* 
    //         When robot is moving and wheel are moving, so we trust the readings
    //         in all the components.
    //     */
    //     wheel_odom_msg_.twist.covariance = {0.0000001, 0, 0, 0, 0, 0,
    //                                         0, 0.000000001, 0, 0, 0, 0,
    //                                         0, 0, 0.00000002, 0, 0, 0,
    //                                         0, 0, 0, 0.01, 0, 0,
    //                                         0, 0, 0, 0, 0.01, 0,
    //                                         0, 0, 0, 0, 0, 0.002};
    // }

    /* Pose covariance tunning (Positions) */
    

    /* Calculations for Global Wheel Odometry */
    float _alpha = CalculateSlipFactor(_kin_omega, imu_omega_);
    float _X_vel_corrected = _X_vel * _alpha;

    /* Filling a global Quaternion with the current euler angles */
    tf2::Quaternion quat_global;
    quat_global.setRPY(imu_roll_, imu_pitch_, imu_yaw_);

    /* Header assignation */
    global_wheel_odom_msg_.header.stamp = this->now();
    global_wheel_odom_msg_.header.frame_id = "odom";
    global_wheel_odom_msg_.child_frame_id = "base_link";
    /* Quaternion assignation */
    global_wheel_odom_msg_.pose.pose.orientation.x = quat_global[0];
    global_wheel_odom_msg_.pose.pose.orientation.y = quat_global[1];
    global_wheel_odom_msg_.pose.pose.orientation.z = quat_global[2];
    global_wheel_odom_msg_.pose.pose.orientation.w = quat_global[3];
    /* Speeds in X and Y axes */
    float _X_dot_global = _X_vel_corrected * cos(imu_yaw_ - imu_yaw_offset_);
    float _Y_dot_global = _X_vel_corrected * sin(imu_yaw_ - imu_yaw_offset_);
    /* Adding displacement in [m] to the global message */
    global_wheel_odom_msg_.pose.pose.position.x += _X_dot_global * dt;
    global_wheel_odom_msg_.pose.pose.position.y += _Y_dot_global * dt;
    global_wheel_odom_msg_.pose.pose.position.z = 0.0f;
    global_wheel_odom_msg_.pose.covariance = wheel_odom_msg_.pose.covariance;
    /* Velocities assignation */
    global_wheel_odom_msg_.twist.twist.linear.x = _X_vel_corrected;
    global_wheel_odom_msg_.twist.twist.linear.y = 0.0f;
    global_wheel_odom_msg_.twist.twist.linear.z = 0.0f;
    global_wheel_odom_msg_.twist.twist.angular.x = 0.0f;
    global_wheel_odom_msg_.twist.twist.angular.y = 0.0f;
    global_wheel_odom_msg_.twist.twist.angular.z = imu_omega_;
    global_wheel_odom_msg_.twist.covariance = wheel_odom_msg_.twist.covariance;

    if (imu_published_ == true)
    {
        wheel_odom_pub_->publish(wheel_odom_msg_);
        wheel_odom_global_pub_->publish(global_wheel_odom_msg_);
    } 
}

void WheelOdometry::PubTimerCb()
{
    CalculateOdometry();
}
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
        "/canlink/chassis/motors_status", 10, std::bind(&WheelOdometry::MovementCb, this, _1));

    /* Services */
    restart_srv_ = this->create_service<std_srvs::srv::SetBool>(
        "/wheel_odometry/restart", std::bind(&WheelOdometry::RestartCb, this, _1, _2, _3));

    prev_time_ = this->now();
    pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(50), std::bind(&WheelOdometry::PubTimerCb, this));
}

bool WheelOdometry::RestartCb(
    const std::shared_ptr<rmw_request_id_t> request_header,
    const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
    std::shared_ptr<std_srvs::srv::SetBool::Response> response)
{
    
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

    response->success = true;
    response->message = "Odometry Restarted";
    
    return true;
} 

void WheelOdometry::MovementCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    imu_state_ = msg->data;
}

void WheelOdometry::MotorStatusCb(const usr_msgs::msg::Motors::SharedPtr msg)
{
    motors_rpm_ = msg->rpm;
    motors_curr_ = msg->current;
}

void WheelOdometry::ImuCb(const sensor_msgs::msg::Imu::SharedPtr msg)
{
    tf2::Quaternion q(msg->orientation.x, 
                    msg->orientation.y,
                    msg->orientation.z, 
                    msg->orientation.w);

    tf2::Matrix3x3 m(q);
    m.getRPY(imu_roll_, imu_pitch_, imu_yaw_);

    imu_omega_ = msg->angular_velocity.z;

    if (imu_published_ == false)
    {
        imu_published_ = true;
    }
}

void WheelOdometry::CalculateOdometry()
{
    /* Wheels linear velocities */
    float _FR_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[0]) / 60.0f; 
    float _RR_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[1]) / 60.0f; 
    float _RL_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[2]) / 60.0f; 
    float _FL_vel = (2.0 * PI * wheel_rad_ * motors_rpm_[3]) / 60.0f; 

    /* Left and Right linear velocities */
    float _R_vel = -(_FR_vel + _RR_vel) / 2.0f;     /* DOuble check these values */
    float _L_vel = (_FL_vel + _RL_vel) / 2.0f;
    float _X_vel = (_R_vel + _L_vel) / 2.0f;
    // float _Z_omg = (_R_vel - _L_vel) / bot_track_;

    rclcpp::Time curr_time = this->now();
    double dt = (curr_time - prev_time_).seconds();
    prev_time_ = this->now();

    wheel_odom_msg_.header.stamp = this->now();
    wheel_odom_msg_.header.frame_id = "odom";
    wheel_odom_msg_.child_frame_id = "base_link";


    tf2::Quaternion quat;
    quat.setRPY(imu_roll_, imu_pitch_, imu_yaw_ - imu_yaw_offset_);
    
    /* Quaternion assignation */
    wheel_odom_msg_.pose.pose.orientation.x = quat[0];
    wheel_odom_msg_.pose.pose.orientation.y = quat[1];
    wheel_odom_msg_.pose.pose.orientation.z = quat[2];
    wheel_odom_msg_.pose.pose.orientation.w = quat[3];
    
    /* Speeds in X and Y axes */
    float _X_dot = _X_vel * cos(imu_yaw_ - imu_yaw_offset_);
    float _Y_dot = _X_vel * sin(imu_yaw_ - imu_yaw_offset_);

    /* Adding displacement in [m] to the current message */
    wheel_odom_msg_.pose.pose.position.x = _X_dot * dt;
    wheel_odom_msg_.pose.pose.position.y = _Y_dot * dt;
    wheel_odom_msg_.pose.pose.position.z = 0.0f;
    
    /* Velocities assignation */
    wheel_odom_msg_.twist.twist.linear.x = _X_vel;
    wheel_odom_msg_.twist.twist.linear.y = 0.0f;
    wheel_odom_msg_.twist.twist.linear.z = 0.0f;

    wheel_odom_msg_.twist.twist.angular.x = 0.0f;
    wheel_odom_msg_.twist.twist.angular.y = 0.0f;
    wheel_odom_msg_.twist.twist.angular.z = imu_omega_;

    /* Twist covariance tunning (Velocities) */
    if ((imu_state_ == true) && _X_vel == 0.0)
    { //Robot is moving but wheels not (e.g. robot is being carried)
        /* 
            When robot is moving, but wheels not, hence we should avoid adding
            values to the linear components.
        */
        wheel_odom_msg_.twist.covariance = {999, 0, 0, 0, 0, 0,
                                            0, 999, 0, 0, 0, 0,
                                            0, 0, 999, 0, 0, 0,
                                            0, 0, 0, 0.001, 0, 0,
                                            0, 0, 0, 0, 0.001, 0,
                                            0, 0, 0, 0, 0, 0.002};
    }
    else
    {
        /* 
            When robot is moving and wheel are moving, so we trust the readings
            in all the components.
        */
        wheel_odom_msg_.twist.covariance = {0.0000001, 0, 0, 0, 0, 0,
                                            0, 0.000000001, 0, 0, 0, 0,
                                            0, 0, 0.00000002, 0, 0, 0,
                                            0, 0, 0, 0.01, 0, 0,
                                            0, 0, 0, 0, 0.01, 0,
                                            0, 0, 0, 0, 0, 0.002};
    }

    /* Pose covariance tunning (Positions) */
    wheel_odom_msg_.pose.covariance = {0.1, 0, 0, 0, 0, 0,
                                        0, 0.1, 0, 0, 0, 0,
                                        0, 0, 0.2, 0, 0, 0,
                                        0, 0, 0, 0.001, 0, 0,
                                        0, 0, 0, 0, 0.001, 0,
                                        0, 0, 0, 0, 0, 0.002};

    if (imu_published_ == true)
    {
        wheel_odom_pub_->publish(wheel_odom_msg_);
    } 
    else
    {
        RCLCPP_ERROR(this->get_logger(), "IMU is not publishing to Wheel Odometry");
    }

}

void WheelOdometry::PubTimerCb()
{
    CalculateOdometry();
}
/*
    - File name: lock_system.cpp
    - This library defines members and member functions for the battery module's communication through CAN
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#include "canlink/modules/lock_system.hpp"
 
LockSystem::LockSystem(const rclcpp::NodeOptions & options, CANDriver *can_driver)
: Node("lock_system", options)
{
    RCLCPP_INFO(this->get_logger(), "Lock System init");
    can_driver_ = can_driver;

    /* Publishers */
    lock_system_status_pub_ = this->create_publisher<std_msgs::msg::Bool>("/canlink/lockSystem/status", 10);
    lock_system_toggle_door_pub_ = this->create_publisher<std_msgs::msg::Bool>("/rover_teleop/openDoor", 10);
 
    /* Subscribers */
    lock_system_open_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/canlink/lockSystem/open", 10, std::bind(&LockSystem::OpenLockCb, this, _1));
    lock_system_request_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
        "/canlink/lockSystem/request", 10, std::bind(&LockSystem::RequestLockStatusCb, this, _1));
}
 
void LockSystem::OpenLockCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Open Lock Callback! ");
    bool open_lock = msg->data;
    if(open_lock)
    {
        uint8_t data[8] = {OPEN_LOCK_ID, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
        can_driver_->CANWrite(LOCK_SYSTEM_ADDRESS, 8, data);
    }
}

void LockSystem::RequestLockStatusCb(const std_msgs::msg::Bool::SharedPtr msg)
{
    RCLCPP_INFO(this->get_logger(), "Request Lock Callback! ");
    bool request_ls = msg->data;
    if(request_ls)
    {
        uint8_t data[8] = {LOCK_STATUS_ID, REQUEST_LOCK_STATUS, 0x0, 0x0, 0x0, 0x0, 0x0, 0x0};
        can_driver_->CANWrite(LOCK_SYSTEM_ADDRESS, 8, data);
    }
}
 
void LockSystem::PublishLockStatus(struct can_frame *frame)
{
    RCLCPP_INFO(this->get_logger(), "Publish Lock Status");
    auto lock_system_status_ = std::make_unique<std_msgs::msg::Bool>();
    /* Lock is closed */
    lock_system_status_->data = frame->data[2] == 1;
    lock_system_status_pub_->publish(std::move(lock_system_status_));
}
 
void LockSystem::PublishToggleDoor(void)
{
    RCLCPP_INFO(this->get_logger(), "Push Toogle Door");
    bool door_state = false;
    // n_->param<bool>("/rover_teleop/openDoor",door_state, false);

    auto lock_sys_toggle_msg_ = std::make_unique<std_msgs::msg::Bool>();
    lock_sys_toggle_msg_->data = !door_state; 
    lock_system_toggle_door_pub_->publish(std::move(lock_sys_toggle_msg_));
}
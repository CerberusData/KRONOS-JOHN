/*
    - File name: can_cabin.cpp
    - This file defines members and member functions for the battery in ROS
    - By: Juan David Galvis
    - Email: juangalvis@kiwicampus.com
*/
 
#include "canlink/CANCabin.hpp"
 
CANCabin::CANCabin(const rclcpp::NodeOptions & options, CANDriver* can_driver, std::shared_ptr<LockSystem> lock_system)
: Node("can_cabin", options)
{
    RCLCPP_INFO(this->get_logger(), "CAN Cabin Initializer");
    /* CAN Driver Instantiation - Open socket */
    try
    {
        can_driver_ = can_driver;

        /* Module instantiation */
        lock_system_ = lock_system;
    }
    catch (const std::system_error &error)
    {
        /* Module instantiation */
        lock_system_ = nullptr;

        char buff[100];
        snprintf(buff, sizeof(buff), "ERROR reading CAN device: %s", error.what());
        std::string errorStr(buff);
    }
    read_thread_ = std::thread(&CANCabin::StartCANCabinRead, this);
    
    // pub_timer_ = this->create_wall_timer(std::chrono::milliseconds(500), std::bind(&CANCabin::Pub_Cb_, this)); 
}

/*
void CANCabin::Pub_Cb_()
{
    // RCLCPP_WARN(this->get_logger(), "Timer Cabin! ");
    rclcpp::Time time = this->now();
    RCLCPP_INFO(this->get_logger(), "Timer Cabin! ");
}
*/

void CANCabin::PublishCANCabinInfo(struct can_frame *frame)
{
    switch (frame->data[0])
    {
        case LOCK_STATUS_ID:   
            lock_system_->PublishLockStatus(frame);
        break;

        case TOGGLE_DOOR_ID:
            lock_system_->PublishToggleDoor();
        break;
 
        default:
        {
            RCLCPP_INFO(this->get_logger(), "CAN Cabin Invalid message");
            break;
        }
    }
}

void CANCabin::StartCANCabinRead()
{
    RCLCPP_INFO(this->get_logger(), "CAN bus thread initialization");
    struct can_frame *frame;
    while(true)
    {
        frame = can_driver_->ReadSocket();
        if (frame)
        {
            if ((frame->can_id & 0xFFF) == KIWIBOT_ADDRESS)
            {
                PublishCANCabinInfo(frame);
            }
        }
        else
        {
            RCLCPP_INFO(this->get_logger(), "Impossible to read CAN bus");
        }  
    }
}

int main(int argc, char * argv[])
{
    /* 
        - Nodes that are running subscribers or timmers must be added to the executor as
          it is going to spin the node. This same node should be passed as an argument to
          the constructor of the main object (Cabin) in order to avoid copying the node.

        - Nodes spinning:
          + CAN Cabin - Main module
          + Lock System - Submodule
    */
    
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    rclcpp::executors::SingleThreadedExecutor executor; 

    /* CAN Driver creation */
    std::string Mystr = "can1";
    const char *interface_name_ = Mystr.c_str();    
    auto can_dvr_ = new CANDriver(interface_name_);

    /* Nodes definition */ 
    auto lock_system = std::make_shared<LockSystem>(options, can_dvr_);  /* Lock System */
    auto CANCabin_node = std::make_shared<CANCabin>(options, can_dvr_, lock_system);  /* Cabin CAN */
    
    /* Filling executor */     
    executor.add_node(CANCabin_node);
    executor.add_node(lock_system);

    RCLCPP_WARN(CANCabin_node->get_logger(), "Init Cabin");

    /* Executor spinning */
    executor.spin();
    
    rclcpp::shutdown();
    return 0;
}
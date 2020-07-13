#include <chrono>

// ROS 2 Default
#include "rclcpp/rclcpp.hpp"

// ROS 2 messages
#include "std_msgs/msg/float32.hpp"

// Custom messages
//#include "usr_msgs/msg/best_message.hpp"

using std::placeholders::_1;

class Control : public rclcpp::Node
{
public: 
    Control(const rclcpp::NodeOptions & options);
    ~Control(){};

// private:
    /*
        Name here both publishers, one with the std_msgs [Float32] and other 
        with usr_msgs [BestMessage]

        Find more information about creating publishers:
            https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/
        
        Example:
            rclcpp::Publisher<message_type>::SharedPtr pub_name;
    */


private: 
    rclcpp::TimerBase::SharedPtr pub_tmr_;

private:
    void PubTimerCb();

};


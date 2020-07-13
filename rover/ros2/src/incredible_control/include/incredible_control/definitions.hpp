#include <chrono>
#include <vector>
#include <math.h>

// ROS 2 Default
#include "rclcpp/rclcpp.hpp"

// ROS 2 messages
#include "std_msgs/msg/float32.hpp"

// Custom messages
//#include "usr_msgs/msg/robot_position.hpp"

using std::placeholders::_1;

class Control : public rclcpp::Node
{
public: 
    Control(const rclcpp::NodeOptions & options, double angle);
    ~Control(){};


// private:
    /*
        Name here both publishers, one with the std_msgs [Float32] and other 
        with usr_msgs [RobotPosition]

        Find more information about creating publishers:
            https://index.ros.org/doc/ros2/Tutorials/Writing-A-Simple-Cpp-Publisher-And-Subscriber/
        
        Example:
            rclcpp::Publisher<message_type>::SharedPtr pub_name;
    */

// Timers
private: 
    rclcpp::TimerBase::SharedPtr pub_tmr_;

// Member functions
private:
    
    /*
        Publisher time. This is initially setup at 500ms
    */
    void PubTimerCb();
    
    /*
        This function will calculate the Robot position based on the unit circle
    */
    std::vector<double> CalculatePosition(double angle);

// Member attributes 
private:
    double m_angle;
    std::vector<double> m_pos = {0.0, 0.0};
};


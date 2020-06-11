#ifndef SOFT_SPEED_SPLINE_H_INCLUDED    
#define SOFT_SPEED_SPLINE_H_INCLUDED
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "utils/console.hpp"

enum
{
    steady = 1,
    acceleration = 2
};

class SoftSpeedSpline
{
    public:
    SoftSpeedSpline(float avg_acc);
    ~SoftSpeedSpline(){};

    float CalculateSoftSpeed(float reference_vel, float acc_factor);

    private:
    void SplineCoefficients(float init_v, float final_v, float init_acc);
    float SoftSpeedValue(double curr_time);

    rclcpp::Clock clock_;
    rclcpp::Time start_time_;
    rclcpp::Time current_time_;    


    bool soft_speed_ = getEnv("SOFT_SPEED_ENABLE", true);
    bool manual_cmd_ = false;
    float spline_params[3][3];
    float avg_acc_ = 0.0;
    float out_vel_ = 0.0;
    float target_vel_ = 0.0;
    double time_1_ = 0.0;
    double time_2_ = 0.0;
    int curve_stage_ = steady;


};

#endif
#include "motion_control/soft_speed_spline.hpp"

SoftSpeedSpline::SoftSpeedSpline(float avg_acc) 
: avg_acc_(avg_acc)
{
    start_time_ = clock_.now();
}

float SoftSpeedSpline::CalculateSoftSpeed(float reference_vel, float acc_factor)
{
    (void)acc_factor;
    if (soft_speed_ == true)
    {
        // if (std::abs(reference_vel - out_vel_) >= 0.28f)
        // {
        //     manual_cmd_ = true;
        // }

        // if (manual_cmd_ == true)
        // {
        switch (curve_stage_)
        {
            case steady:
                /*
                    - This case represents 2 conditions *Stopped" or "Steady". 
                    - Check if the Robot is not moving (Zero velocity) or if 
                    it has reached the target (Is in a steady state).
                    - When a change in the reference velocity is detected, 
                    we jump into the if condition.
                */
                if (std::abs(reference_vel - out_vel_) != 0.0f)
                {
                    target_vel_ = reference_vel;
                    curve_stage_ = acceleration;
                    start_time_ = clock_.now();
                    // Output speed becomes the initial velocity (Last velocity)
                    SplineCoefficients(out_vel_, target_vel_, 0.0f);
                }

                return out_vel_;

            case acceleration:
                // Check if the Robot is in pure acceleration.
                if (std::abs(reference_vel - target_vel_) < 0.001f)
                {
                    current_time_ = clock_.now();
                    out_vel_ = SoftSpeedValue((current_time_ - start_time_).seconds());
                    return out_vel_;
                }
                
                // If the robot has reached the reference and is in "Steady"
                else
                {
                    curve_stage_ = steady;
                    out_vel_ = reference_vel;
                    return out_vel_;
                }

            default:
                return out_vel_;
        }
        // }

        // Reference velocity is coming from Waypoint
        // else
        // {
        //     curve_stage_ = steady;
        //     out_vel_ = reference_vel;
        //     return out_vel_;
        // }
    }

    else
    {
        return reference_vel;
    }
}


float SoftSpeedSpline::SoftSpeedValue(double curr_time)
{
    float soft_speed = 0.0;
    // 1st acceleration
    if ((curr_time >= 0.0) && (curr_time < time_1_))
    {
        soft_speed = 3 * spline_params[0][2] * pow(curr_time, 2) 
            + 2 * spline_params[0][1] * curr_time + spline_params[0][0];
    }

    // 2nd acceleration
    else if ((curr_time >= time_1_) && (curr_time < time_1_ + time_2_))
    {
        soft_speed = 3 * spline_params[1][2] * pow(curr_time - time_1_, 2) 
            + (2 * spline_params[1][1] * (curr_time - time_1_)) + spline_params[1][0];
    }

    // Final velocity
    else
    {
        soft_speed = 3 * spline_params[2][2] * pow(curr_time, 2) 
            + 2 * spline_params[2][1] * curr_time + spline_params[2][0];
    }

    return soft_speed;
}

void SoftSpeedSpline::SplineCoefficients(float init_v, float final_v, float init_acc)
{
    double final_t = std::abs(final_v - init_v) / avg_acc_;
    time_1_ = final_t / 3.0;
    time_2_ = final_t / 3.0;
    // 1st acceleration
    spline_params[0][0] = init_v;
    spline_params[0][1] = init_acc / 2.0;
    spline_params[0][2] = (2.0 * (final_v - init_v - (init_acc * time_1_) 
        - (init_acc * time_2_))) / (6.0 * time_1_ * (time_1_ + time_2_));
    // 2nd acceleration
    spline_params[1][0] = (2.0 * ((final_v * time_1_) + (init_v * time_2_)
        + (init_acc * time_1_ * time_2_))) / (2.0 * (time_1_ + time_2_));
    spline_params[1][1] = (2.0 * (final_v - init_v) - (init_acc * time_1_)) 
        / (2.0 * (time_1_ + time_2_));
    spline_params[1][2] = -(2.0 * (final_v - init_v) - (init_acc * time_1_)) 
        / (6.0 * time_2_ * (time_1_ + time_2_));
    // Final velocity
    spline_params[2][0] = final_v;
    spline_params[2][1] = 0.0;
    spline_params[2][2] = 0.0;
}

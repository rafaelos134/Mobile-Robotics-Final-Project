#include "cinematica.hpp"
#include <cmath>


WheelVelocities calculate_kinematics(double v, double w) {
    double wheelRadius_ = 0.05; 
    double wheelSeparation_ = 0.22; 

    double maxV_ = 0.5;
    double maxW_ = 0.5;

    // Limita v e w
    if (v > maxV_) v = maxV_;
    else if (v < -maxV_) v = -maxV_;

    if (w > maxW_) w = maxW_;
    else if (w < -maxW_) w = -maxW_;


    // cinematica
    double w_right = ((2.0 * v) + (w * wheelSeparation_)) / (2.0 * wheelRadius_ );
    double w_left  = ((2.0 * v) - (w * wheelSeparation_)) / (2.0 * wheelRadius_ );

    WheelVelocities result;
    result.left_rad_s = w_left;
    result.right_rad_s = w_right;

    return result;
}
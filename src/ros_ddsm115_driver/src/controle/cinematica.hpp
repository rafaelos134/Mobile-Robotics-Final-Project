#ifndef CINEMATICA_HPP
#define CINEMATICA_HPP


struct WheelVelocities {
    double left_rad_s;
    double right_rad_s;
    double maxSpeed_;
    double wheelRadius_;   
    double wheelSeparation_;
};


WheelVelocities calculate_kinematics(double v, double w);

#endif
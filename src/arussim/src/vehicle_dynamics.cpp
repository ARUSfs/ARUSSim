# include "arussim/vehicle_dynamics.hpp"

VehicleDynamics::VehicleDynamics(){
    x_ = 0;
    y_ = 0;
    yaw_ = 0;
    vx_ = 0;
    input_delta_ = 0;
    input_acc_ = 0;
    dt_ = 0.01;
}

void VehicleDynamics::update_simulation(double input_delta, 
                                        double input_acc, 
                                        double dt){
    input_delta_ = input_delta;
    input_acc_ = input_acc;
    dt_ = dt;

    calculate_dynamics();
    integrate_dynamics();
}

void VehicleDynamics::calculate_dynamics(){
    x_dot_ = vx_ * std::cos(yaw_);
    y_dot_ = vx_ * std::sin(yaw_);
    yaw_dot_ = vx_ / kWheelBase * std::atan(input_delta_);
    vx_dot_ = calculate_fx() / kMass;
}

void VehicleDynamics::integrate_dynamics(){
    x_ += x_dot_ * dt_;
    y_ += y_dot_ * dt_;
    yaw_ += yaw_dot_ * dt_;
    vx_ += vx_dot_ * dt_;
    if(vx_ < 0){
        vx_ = 0;
    }
}

double VehicleDynamics::calculate_fx(){
    double Drag = 0.5*kAirDensity*kCDA*pow(vx_,2);
    double longitudinal_force = std::clamp(kMass * input_acc_, kMinFx, kMaxFx) - Drag - kRollingResistance;

    return longitudinal_force;
}
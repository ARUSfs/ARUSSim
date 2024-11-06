# include "arussim/vehicle_dynamics.hpp"

VehicleDynamics::VehicleDynamics(){}

void VehicleDynamics::update_simulation(double& x, 
                                        double& y, 
                                        double& yaw, 
                                        double& vx, 
                                        double input_delta, 
                                        double input_acc, 
                                        double dt,
                                        double kMass,
                                        double kWheelBase){
    calculate_dynamics(vx, yaw, input_delta, input_acc, kMass, kWheelBase);
    integrate_dynamics(x, y, yaw, vx, dt);
}

void VehicleDynamics::calculate_dynamics(double vx, double yaw, double input_delta, double input_acc, double kMass, double kWheelBase){
    x_dot_ = vx * std::cos(yaw);
    y_dot_ = vx * std::sin(yaw);
    yaw_dot_ = vx / kWheelBase * std::atan(input_delta);
    vx_dot_ = calculate_fx(vx, input_acc, kMass) / kMass;
}

void VehicleDynamics::integrate_dynamics(double& x, double& y, double& yaw, double& vx, double dt){
    x += x_dot_ * dt;
    y += y_dot_ * dt;
    yaw += yaw_dot_ * dt;
    vx += vx_dot_ * dt;
    if(vx < 0){
        vx = 0;
    }
}

double VehicleDynamics::calculate_fx(double vx, double input_acc, double kMass){
    double Drag = 0.5*kAirDensity*kCDA*pow(vx,2);
    double longitudinal_force = std::clamp(kMass * input_acc, kMinFx, kMaxFx) - Drag - kRollingResistance;

    return longitudinal_force;
}
# include "arussim/vehicle_dynamics.hpp"

VehicleDynamics::VehicleDynamics(){}

void VehicleDynamics::update_simulation(double& x, 
                                        double& y, 
                                        double& yaw, 
                                        double& vx, 
                                        double input_delta, 
                                        double input_acc, 
                                        double dt){
    calculate_dynamics(vx, input_delta);
    integrate_dynamics(x, y, yaw, vx, input_acc, dt);
}

void VehicleDynamics::calculate_dynamics(double vx, double input_delta){
    x_dot_ = vx * std::cos(input_delta);
    y_dot_ = vx * std::sin(input_delta);
    yaw_dot_ = vx / kWheelBase * std::atan(input_delta);
 
}

void VehicleDynamics::integrate_dynamics(double& x, double& y, double& yaw, double& vx, 
                                            double input_acc, double dt){
    x += x_dot_ * dt;
    y += y_dot_ * dt;
    yaw += yaw_dot_ * dt;
    vx += input_acc * dt;
}
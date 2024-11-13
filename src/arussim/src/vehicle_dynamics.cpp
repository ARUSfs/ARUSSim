# include "arussim/vehicle_dynamics.hpp"

VehicleDynamics::VehicleDynamics(){
    x_ = 0;
    y_ = 0;
    yaw_ = 0;
    vx_ = 0;
    vy_ = 0;
    r_ = 0;

    ax_ = 0;
    ay_ = 0;
    
    input_delta_ = 0;
    input_acc_ = 0;
    dt_ = 0.001;
}

void VehicleDynamics::update_simulation(double input_delta, 
                                        double input_acc, 
                                        double dt){
    input_delta_ = input_delta;
    input_acc_ = input_acc;
    dt_ = dt;

    update_torque_cmd();

    calculate_dynamics();
    integrate_dynamics();
}

void VehicleDynamics::calculate_dynamics(){
    
    calculate_tire_slip();
    calculate_tire_loads();

    Tire_force force_fl = calculate_tire_forces(tire_slip_.alpha_fl_, tire_slip_.lambda_fl_, tire_loads_.fl_);
    Tire_force force_fr = calculate_tire_forces(tire_slip_.alpha_fr_, tire_slip_.lambda_fr_, tire_loads_.fr_);
    Tire_force force_rl = calculate_tire_forces(tire_slip_.alpha_rl_, tire_slip_.lambda_rl_, tire_loads_.rl_);
    Tire_force force_rr = calculate_tire_forces(tire_slip_.alpha_rr_, tire_slip_.lambda_rr_, tire_loads_.rr_);

    double fy_front = force_fl.fy + force_fr.fy;
    double fy_rear = force_rl.fy + force_rr.fy;

    ax_ = (calculate_fx() - fy_front * std::sin(delta_)) / kMass ;
    ay_ = (fy_front * std::cos(delta_) + fy_rear) / kMass;

    x_dot_ = vx_ * std::cos(yaw_);
    y_dot_ = vx_ * std::sin(yaw_);

    vx_dot_ = ax_ + r_ * vy_;
    vy_dot_ = ay_ - r_ * vx_;
    r_dot_ = (fy_front*std::cos(delta_)*kLf - fy_rear*kLr) / kIzz;

    delta_ = input_delta_;

    // Tire angular acceleration
    w_fl_dot_ = (torque_cmd_.fl_ - force_fl.fx / kTireDynRadius) / kTireInertia;
    w_fr_dot_ = (torque_cmd_.fr_ - force_fr.fx / kTireDynRadius) / kTireInertia;
    w_rl_dot_ = (torque_cmd_.rl_ - force_rl.fx / kTireDynRadius) / kTireInertia;
    w_rr_dot_ = (torque_cmd_.rr_ - force_rr.fx / kTireDynRadius) / kTireInertia;
}

void VehicleDynamics::integrate_dynamics(){

    // Simple Euler method for now
    x_ += x_dot_ * dt_;
    y_ += y_dot_ * dt_;
    yaw_ += r_ * dt_;
    vx_ += vx_dot_ * dt_;
    vy_ += vy_dot_ *dt_;
    r_ += r_dot_ * dt_;

    kinematic_correction();

    wheel_speed_.fl_ += w_fl_dot_ * dt_;
    wheel_speed_.fr_ += w_fr_dot_ * dt_;
    wheel_speed_.rl_ += w_rl_dot_ * dt_;
    wheel_speed_.rr_ += w_rr_dot_ * dt_;

    if(vx_ < 0){
        vx_ = 0;
    }
}

double VehicleDynamics::calculate_fx(){
    double Drag = 0.5*kAirDensity*kCDA*pow(vx_,2);
    double longitudinal_force = std::clamp(kMass * input_acc_, kMinFx, kMaxFx) - Drag - kRollingResistance;

    return longitudinal_force;
}

void VehicleDynamics::calculate_tire_loads(){

    double load_transfer_ay = kMass * kHCog * ay_ / kTrackWidth;

    tire_loads_.fl_ = kStaticLoadFront - (1 - kMassDistributionRear) * load_transfer_ay;
    tire_loads_.fr_ = kStaticLoadFront + (1 - kMassDistributionRear) * load_transfer_ay;
    tire_loads_.rl_ = kStaticLoadFront - kMassDistributionRear * load_transfer_ay;
    tire_loads_.rr_ = kStaticLoadFront + kMassDistributionRear * load_transfer_ay;

    if(tire_loads_.fl_ < 0){tire_loads_.fl_ = 0;}
    if(tire_loads_.fr_ < 0){tire_loads_.fr_ = 0;}
    if(tire_loads_.rl_ < 0){tire_loads_.rl_ = 0;}
    if(tire_loads_.rr_ < 0){tire_loads_.rr_ = 0;}

}

void VehicleDynamics::calculate_tire_slip(){
    double vy_front = vy_ + kLf * r_;
    double vy_rear = vy_ - kLr * r_;
    double vx_left = vx_ - r_ * kTrackWidth / 2;
    double vx_right = vx_ + r_ * kTrackWidth / 2;

    if(vx_ > 1){
        tire_slip_.alpha_fl_ = std::atan(vy_front / vx_left) - delta_;
        tire_slip_.alpha_fr_ = std::atan(vy_front / vx_right) - delta_;
        tire_slip_.alpha_rl_ = std::atan(vy_rear / vx_left);
        tire_slip_.alpha_rr_ = std::atan(vy_rear / vx_right);
    }

    double vx_fl = std::sqrt(vy_front*vy_front + vx_left*vx_left) * std::cos(tire_slip_.alpha_fl_);
    double vx_fr = std::sqrt(vy_front*vy_front + vx_right*vx_right) * std::cos(tire_slip_.alpha_fr_);
    double vx_rl = std::sqrt(vy_rear*vy_rear + vx_left*vx_left) * std::cos(tire_slip_.alpha_rl_);
    double vx_rr = std::sqrt(vy_rear*vy_rear + vx_right*vx_right) * std::cos(tire_slip_.alpha_rr_);

    double eps = 0.01;

    tire_slip_.lambda_fl_ = kTireDynRadius * wheel_speed_.fl_ / (vx_fl + eps) - 1;
    tire_slip_.lambda_fr_ = kTireDynRadius * wheel_speed_.fr_ / (vx_fr + eps) - 1;
    tire_slip_.lambda_rl_ = kTireDynRadius * wheel_speed_.rl_ / (vx_rl + eps) - 1;
    tire_slip_.lambda_rr_ = kTireDynRadius * wheel_speed_.rr_ / (vx_rr + eps) - 1;

}

VehicleDynamics::Tire_force VehicleDynamics::calculate_tire_forces(double slip_angle, double slip_ratio, double tire_load){

    double fy_pure = tire_load * kCamberStiffness * slip_angle;
    double fx_pure = tire_load * kCamberStiffness * slip_ratio;

    Tire_force tire_force;
    tire_force.fy = fy_pure;
    tire_force.fx = fx_pure;

    return tire_force;
}

void VehicleDynamics::kinematic_correction(){
    double lambda = 1 - std::clamp((vx_ - 2.0)/(5.0 - 2.0),0.0,1.0);
    double r_kinematic = std::tan(delta_) * vx_ / kWheelBase;
    double vy_kinematic = r_kinematic * kLr;

    r_ = lambda * r_kinematic + (1 - lambda) * r_;
    vy_ = lambda * vy_kinematic + (1 - lambda) * vy_;
}

void VehicleDynamics::update_torque_cmd(){
    double total_fx_cmd = input_acc_ * kMass;
    torque_cmd_.fl_ = 0.2 * total_fx_cmd / kTireDynRadius;
    torque_cmd_.fr_ = 0.2 * total_fx_cmd / kTireDynRadius;
    torque_cmd_.rl_ = 0.3 * total_fx_cmd / kTireDynRadius;
    torque_cmd_.rr_ = 0.3 * total_fx_cmd / kTireDynRadius;
}
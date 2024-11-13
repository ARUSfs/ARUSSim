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

    calculate_dynamics();
    integrate_dynamics();
}

void VehicleDynamics::calculate_dynamics(){
    
    calculate_tire_slip();
    calculate_tire_loads();
    double fy_front, fy_rear;
    calculate_tire_forces(fy_front, fy_rear);

    ax_ = (calculate_fx() - fy_front * std::sin(delta_)) / kMass ;
    ay_ = (fy_front * std::cos(delta_) + fy_rear) / kMass;

    x_dot_ = vx_ * std::cos(yaw_);
    y_dot_ = vx_ * std::sin(yaw_);

    vx_dot_ = ax_ + r_ * vy_;
    vy_dot_ = ay_ - r_ * vx_;
    r_dot_ = (fy_front*std::cos(delta_)*kLf - fy_rear*kLr) / kIzz;

    delta_ = input_delta_;
}

void VehicleDynamics::integrate_dynamics(){

    x_ += x_dot_ * dt_;
    y_ += y_dot_ * dt_;
    yaw_ += r_ * dt_;
    vx_ += vx_dot_ * dt_;
    vy_ += vy_dot_ *dt_;
    r_ += r_dot_ * dt_;

    kinematic_correction();

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
    if(vx_ > 1){
        tire_slip_.alpha_fl_ = std::atan((vy_ + kLf * r_)/(vx_ - r_ * kTrackWidth / 2)) - delta_;
        tire_slip_.alpha_fr_ = std::atan((vy_ + kLf * r_)/(vx_ + r_ * kTrackWidth / 2)) - delta_;
        tire_slip_.alpha_rl_ = std::atan((vy_ - kLr * r_)/(vx_ - r_ * kTrackWidth / 2));
        tire_slip_.alpha_rr_ = std::atan((vy_ - kLr * r_)/(vx_ + r_ * kTrackWidth / 2));
    }
}

void VehicleDynamics::calculate_tire_forces(double &fy_front, double &fy_rear){

    double fy_fl = tire_loads_.fl_ * kCamberStiffness * tire_slip_.alpha_fl_;
    double fy_fr = tire_loads_.fr_ * kCamberStiffness * tire_slip_.alpha_fr_;
    double fy_rl = tire_loads_.rl_ * kCamberStiffness * tire_slip_.alpha_rl_;
    double fy_rr = tire_loads_.rr_ * kCamberStiffness * tire_slip_.alpha_rr_;

    fy_front = fy_fl + fy_fr;
    fy_rear = fy_rl + fy_rr;

}

void VehicleDynamics::kinematic_correction(){
    double lambda = 1 - std::clamp((vx_ - 2.0)/(5.0 - 2.0),0.0,1.0);
    double r_kinematic = std::tan(delta_) * vx_ / kWheelBase;
    double vy_kinematic = r_kinematic * kLr;

    r_ = lambda * r_kinematic + (1 - lambda) * r_;
    vy_ = lambda * vy_kinematic + (1 - lambda) * vy_;
}
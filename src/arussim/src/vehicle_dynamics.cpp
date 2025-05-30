#include "arussim/vehicle_dynamics.hpp"

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

    delta_ = 0;
    delta_v_ = 0;
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

    force.fy = force_fl.fy;
    force.fx = force_fl.fx;

    double fy_front = force_fl.fy + force_fr.fy;
    double fy_rear = force_rl.fy + force_rr.fy;

    ax_ = calculate_fx(force_fl, force_fr, force_rl, force_rr) / kMass;
    double total_fy = fy_front * std::cos(delta_) + fy_rear;
    total_fy += (force_fl.fx + force_fr.fx) * std::sin(delta_);
    ay_ = total_fy / kMass;

    x_dot_ = vx_ * std::cos(yaw_) - vy_ * std::sin(yaw_);
    y_dot_ = vx_ * std::sin(yaw_) + vy_ * std::cos(yaw_);

    vx_dot_ = ax_ + r_ * vy_;
    vy_dot_ = ay_ - r_ * vx_;

    double mz_lateral = fy_front * std::cos(delta_) * kLf - fy_rear * kLr;
    mz_lateral += (force_fl.fy - force_fr.fy) * std::sin(delta_) * kTrackWidth/2;
    double mz_longitudinal = ((force_fr.fx - force_fl.fx) * std::cos(delta_) + force_rr.fx - force_rl.fx) * kTrackWidth/2;
    mz_longitudinal += (force_fl.fx + force_fr.fx) * std::sin(delta_) * kLf;

    double total_mz = mz_lateral + mz_longitudinal;

    r_dot_ = total_mz / kIzz;

    // Tire angular acceleration
    w_fl_dot_ = (torque_cmd_.fl_ - force_fl.fx * kTireDynRadius) / kTireInertia;
    w_fr_dot_ = (torque_cmd_.fr_ - force_fr.fx * kTireDynRadius) / kTireInertia;
    w_rl_dot_ = (torque_cmd_.rl_ - force_rl.fx * kTireDynRadius) / kTireInertia;
    w_rr_dot_ = (torque_cmd_.rr_ - force_rr.fx * kTireDynRadius) / kTireInertia;

    delta_dot_ = std::clamp(delta_v_, -kSteeringVMax, kSteeringVMax);
    delta_v_dot_ = - kCoefDelta * delta_ - kCoefV * delta_v_ + kCoefInput * input_delta_;
    delta_v_dot_ = std::clamp(delta_v_dot_, -kSteeringAMax, kSteeringAMax);

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

    delta_ += delta_dot_ * dt_ + 0.5 * delta_v_dot_ * dt_ * dt_;
    delta_v_ += delta_v_dot_ * dt_;

    if(vx_ < 0){
        vx_ = 0;
    }
}

double VehicleDynamics::calculate_fx(Tire_force force_fl, Tire_force force_fr, Tire_force force_rl, Tire_force force_rr){

    double longitudinal_tire_force = (force_fl.fx + force_fr.fx) * std::cos(delta_) + force_rl.fx + force_rr.fx;
    longitudinal_tire_force -= (force_fl.fy + force_fr.fy) * std::sin(delta_);

    double aero_drag = 0.5 * kAirDensity * kCDA * vx_*vx_;

    double longitudinal_force = longitudinal_tire_force - aero_drag - kRollingResistance;

    return longitudinal_force;
}

void VehicleDynamics::calculate_tire_loads(){

    double load_transfer_ay = kMass * kHCog * ay_ / kTrackWidth;
    double load_transfer_ax = kMass * kHCog * ax_ / kWheelBase;

    tire_loads_.fl_ = kStaticLoadFront - (1 - kMassDistributionRear) * load_transfer_ay - load_transfer_ax/2;
    tire_loads_.fr_ = kStaticLoadFront + (1 - kMassDistributionRear) * load_transfer_ay - load_transfer_ax/2;
    tire_loads_.rl_ = kStaticLoadFront - kMassDistributionRear * load_transfer_ay + load_transfer_ax/2;
    tire_loads_.rr_ = kStaticLoadFront + kMassDistributionRear * load_transfer_ay + load_transfer_ax/2;

    double aero_lift = 0.5 * kAirDensity * kCLA * vx_*vx_;
    double aero_drag = 0.5 * kAirDensity * kCDA * vx_*vx_;

    tire_loads_.fl_ += kCOPx * aero_lift / 2 - (kCOPy - kHCog) * aero_drag;
    tire_loads_.fr_ += kCOPx * aero_lift / 2 - (kCOPy - kHCog) * aero_drag;
    tire_loads_.rl_ += (1 - kCOPx) * aero_lift / 2 + (kCOPy - kHCog) * aero_drag;
    tire_loads_.rr_ += (1 - kCOPx) * aero_lift / 2 + (kCOPy - kHCog) * aero_drag;


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

    if(vx_ < 0.1){
        tire_slip_.lambda_fl_ = kTireDynRadius * wheel_speed_.fl_ / (vx_fl + eps);
        tire_slip_.lambda_fr_ = kTireDynRadius * wheel_speed_.fr_ / (vx_fr + eps);
        tire_slip_.lambda_rl_ = kTireDynRadius * wheel_speed_.rl_ / (vx_rl + eps);
        tire_slip_.lambda_rr_ = kTireDynRadius * wheel_speed_.rr_ / (vx_rr + eps);
    }

}

VehicleDynamics::Tire_force VehicleDynamics::calculate_tire_forces(double slip_angle, double slip_ratio, double tire_load){

    // Combined slip
    double alpha_star = slip_angle / pac_param_.kAlphaP;
    double lambda_star = slip_ratio / pac_param_.kLambdaP;

    double s_star = std::max(std::sqrt(alpha_star*alpha_star + lambda_star*lambda_star), 0.01);

    slip_angle = s_star * pac_param_.kAlphaP;
    slip_ratio = s_star * pac_param_.kLambdaP;

    double fy_pure = tire_load * pac_param_.Dlat * std::sin(pac_param_.Clat * std::atan(pac_param_.Blat * slip_angle));
    double fx_pure = tire_load * pac_param_.Dlon * std::sin(pac_param_.Clon * std::atan(pac_param_.Blon * slip_ratio));

    Tire_force tire_force;
    tire_force.fy = fy_pure * alpha_star / s_star;
    tire_force.fx = fx_pure * lambda_star / s_star;

    return tire_force;
}

void VehicleDynamics::kinematic_correction(){
    double lambda = 1 - std::clamp((vx_ - 2.0)/(5.0 - 2.0),0.0,1.0);
    double r_kinematic = std::tan(delta_) * vx_ / kWheelBase;
    double vy_kinematic = r_kinematic * kLr;

    r_ = lambda * r_kinematic + (1 - lambda) * r_;
    vy_ = lambda * vy_kinematic + (1 - lambda) * vy_;
}

void VehicleDynamics::set_torque_vectoring(bool value)
{
    kTorqueVectoring = value;
}

void VehicleDynamics::update_torque_cmd(){
    double total_fx_cmd = input_acc_ * kMass;

    if(kTorqueVectoring){
        double target_r = vx_ * std::tan(delta_) / kWheelBase;

        double target_mz = kTVKp * (target_r - r_);

        double fz_total = kMass * kG + 0.5 * kAirDensity * kCLA * vx_*vx_;
        double fz_front_mean = 0.5*(tire_loads_.fl_ + tire_loads_.fr_);
        double fz_rear_mean = 0.5*(tire_loads_.rl_ + tire_loads_.rr_);

        torque_cmd_.fl_ = kTireDynRadius / fz_total * (fz_front_mean * total_fx_cmd - tire_loads_.fl_ * 2*target_mz/kTrackWidth);
        torque_cmd_.fr_ = kTireDynRadius / fz_total * (fz_front_mean * total_fx_cmd + tire_loads_.fr_ * 2*target_mz/kTrackWidth);
        torque_cmd_.rl_ = kTireDynRadius / fz_total * (fz_rear_mean * total_fx_cmd - tire_loads_.rl_ * 2*target_mz/kTrackWidth);
        torque_cmd_.rr_ = kTireDynRadius / fz_total * (fz_rear_mean * total_fx_cmd + tire_loads_.rr_ * 2*target_mz/kTrackWidth);
    }else{
        torque_cmd_.fl_ = 0.2 * total_fx_cmd * kTireDynRadius;
        torque_cmd_.fr_ = 0.2 * total_fx_cmd * kTireDynRadius;
        torque_cmd_.rl_ = 0.3 * total_fx_cmd * kTireDynRadius;
        torque_cmd_.rr_ = 0.3 * total_fx_cmd * kTireDynRadius;
    }

    torque_cmd_.fl_ = std::clamp(torque_cmd_.fl_, kTorqueMin, kTorqueMax);
    torque_cmd_.fr_ = std::clamp(torque_cmd_.fr_, kTorqueMin, kTorqueMax);
    torque_cmd_.rl_ = std::clamp(torque_cmd_.rl_, kTorqueMin, kTorqueMax);
    torque_cmd_.rr_ = std::clamp(torque_cmd_.rr_, kTorqueMin, kTorqueMax);

}

void VehicleDynamics::write_csv_row(){
    if(!csv_generator_vehicle_dynamics_){
        RCLCPP_ERROR(rclcpp::get_logger("VehicleDynamics"), "CSVGenerator not initialized.");
        return;
    }
    std::vector<std::string> row_values;
    // Collect all member variables
    row_values.push_back(std::to_string(x_));
    row_values.push_back(std::to_string(y_));
    row_values.push_back(std::to_string(yaw_));
    row_values.push_back(std::to_string(vx_));
    row_values.push_back(std::to_string(vy_));
    row_values.push_back(std::to_string(r_));
    row_values.push_back(std::to_string(ax_));
    row_values.push_back(std::to_string(ay_));
    row_values.push_back(std::to_string(delta_));
    row_values.push_back(std::to_string(delta_v_));
    // Wheel speeds
    row_values.push_back(std::to_string(wheel_speed_.fl_));
    row_values.push_back(std::to_string(wheel_speed_.fr_));
    row_values.push_back(std::to_string(wheel_speed_.rl_));
    row_values.push_back(std::to_string(wheel_speed_.rr_));
    // Torque commands
    row_values.push_back(std::to_string(torque_cmd_.fl_));
    row_values.push_back(std::to_string(torque_cmd_.fr_));
    row_values.push_back(std::to_string(torque_cmd_.rl_));
    row_values.push_back(std::to_string(torque_cmd_.rr_));
    // Inputs
    row_values.push_back(std::to_string(input_delta_));
    row_values.push_back(std::to_string(input_acc_));
    row_values.push_back(std::to_string(dt_));
    // Tire slip
    row_values.push_back(std::to_string(tire_slip_.alpha_fl_));
    row_values.push_back(std::to_string(tire_slip_.alpha_fr_));
    row_values.push_back(std::to_string(tire_slip_.alpha_rl_));
    row_values.push_back(std::to_string(tire_slip_.alpha_rr_));
    row_values.push_back(std::to_string(tire_slip_.lambda_fl_));
    row_values.push_back(std::to_string(tire_slip_.lambda_fr_));
    row_values.push_back(std::to_string(tire_slip_.lambda_rl_));
    row_values.push_back(std::to_string(tire_slip_.lambda_rr_));
    // Tire loads
    row_values.push_back(std::to_string(tire_loads_.fl_));
    row_values.push_back(std::to_string(tire_loads_.fr_));
    row_values.push_back(std::to_string(tire_loads_.rl_));
    row_values.push_back(std::to_string(tire_loads_.rr_));
    // Forces
    row_values.push_back(std::to_string(force.fy));
    row_values.push_back(std::to_string(force.fx));
    // Write header and row
    csv_generator_vehicle_dynamics_->write_row("x,y,yaw,vx,vy,r,ax,ay,delta,delta_v,fl_wheel_speed,fr_wheel_speed,rl_wheel_speed,rr_wheel_speed,fl_torque,fr_torque,rl_torque,rr_torque,input_delta,input_acc,dt,alpha_fl,alpha_fr,alpha_rl,alpha_rr,lambda_fl,lambda_fr,lambda_rl,lambda_rr,fl_load,fr_load,rl_load,rr_load,force_fy,force_fx", 
                                                row_values);
}
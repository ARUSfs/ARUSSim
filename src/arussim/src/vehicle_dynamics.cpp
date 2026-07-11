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
    delta_fl_ = 0;
    delta_fr_ = 0;
    delta_v_ = 0;
    dt_ = 0.001;

    struct {
        double fl_ = 0;
        double fr_ = 0;
        double rl_ = 0;
        double rr_ = 0;
    } wheel_speed;
    
    struct {
        double fl_ = 0;
        double fr_ = 0;
        double rl_ = 0;
        double rr_ = 0;
    } torque_cmd;

    struct {
        double alpha_fl_ = 0;
        double alpha_fr_ = 0;
        double alpha_rl_ = 0;
        double alpha_rr_ = 0;

        double lambda_fl_ = 0;
        double lambda_fr_ = 0;
        double lambda_rl_ = 0;
        double lambda_rr_ = 0;
    } tire_slip;
    
    struct {
        double fl_ = 0;
        double fr_ = 0;
        double rl_ = 0;
        double rr_ = 0;
    } tire_loads;

    csv_generator_vehicle_dynamics_ = nullptr;
}

void VehicleDynamics::set_parameters(std::map<std::string, double>& params) {

    kG = params["g"];
    kSMass = params["sm"];
    kNsMassF = params["nsm_f"];
    kNsMassR = params["nsm_r"];
    kIzz = params["Iz"];
    kWheelBase = params["wheelbase"];
    kHCog = params["h_cdg"];
    kMassDistributionRear = params["r_cdg"];
    kSpringStiffnessF = params["k_F"];
    kSpringStiffnessR = params["k_R"];
    kMotionRatioF = params["MR_F"];
    kMotionRatioR = params["MR_R"];
    kTrackWidth = params["trackwidthF"];
    kTireDynRadius = params["rdyn"];
    kTireInertia_F = params["I_wheel_F"];
    kTireInertia_R = params["I_wheel_R"];
    kAirDensity = params["rho"];
    kCDA = params["CDA"];
    kCLA = params["CLA"];
    kCOPx = params["r_cdp"];
    kCOPy = params["h_cdp"];
    kGearRatio = params["gear_ratio"];


    kSMassF = kSMass * (1-kMassDistributionRear);
    kSMassR = kSMass * kMassDistributionRear;
    kMass = kSMass + kNsMassF + kNsMassR;
    kLf = kWheelBase*kMassDistributionRear;
    kLr = kWheelBase*(1-kMassDistributionRear); 

    kHRollCenterF = 0.033;  // TODO: añadir roll centers al csv
    kHRollCenterR = 0.097;
    kHRollAxis = kHRollCenterF + (kHRollCenterR - kHRollCenterF) * kLf / kWheelBase;

    kWheelRateF = kSpringStiffnessF / std::pow(kMotionRatioF,2);
    kWheelRateR = kSpringStiffnessR / std::pow(kMotionRatioR,2);
    kRollStiffnessF = 0.5 * std::pow(kTrackWidth,2) * 0.01745 * kWheelRateF;
    kRollStiffnessR = 0.5 * std::pow(kTrackWidth,2) * 0.01745 * kWheelRateR;
    kRollStiffness = kRollStiffnessF + kRollStiffnessR;

    kHCogNsF = kTireDynRadius;
    kHCogNsR = kTireDynRadius;

    kAckermann1 = 0.1175;
    kAckermann2 = 0.9724;

    kRollingResistance = 100;

    kStaticLoadFront = (1 - kMassDistributionRear) * kMass * kG / 2;
    kStaticLoadRear = kMassDistributionRear * kMass * kG / 2;

    kCoefDelta = 306.3; 
    kCoefV = 25.69;
    kCoefInput = 307;
    kSteeringAMax = 3.0;
    kSteeringVMax = 2.3;


    pac_param_.Fz0 = params["Fz0"];

    pac_param_.D1_x = params["D1_x"];
    pac_param_.D2_x = params["D2_x"];
    pac_param_.Cx   = params["Cx"];
    pac_param_.Bx   = params["Bx"];
    pac_param_.Ex   = params["Ex"];

    pac_param_.D1_y = params["D1_y"];
    pac_param_.D2_y = params["D2_y"];
    pac_param_.Cy   = params["Cy"];
    pac_param_.By   = params["By"];
    pac_param_.Ey   = params["Ey"];
    pac_param_.Lambda_mu_y = params["Lambda_mu_y"];

    pac_param_.SH = params["SH"];
    pac_param_.SV = params["SV"];

    pac_param_.rB1_x = params["rB1_x"];
    pac_param_.rB2_x = params["rB2_x"];
    pac_param_.rC1_x = params["rC1_x"];
    pac_param_.rE1_x = params["rE1_x"];

    pac_param_.rB1_y = params["rB1_y"];
    pac_param_.rB2_y = params["rB2_y"];
    pac_param_.rC1_y = params["rC1_y"];
    pac_param_.rSh   = params["rSh"];

    pac_param_.rGx1 = params["rGx1"];
    pac_param_.rBx  = params["rBx"];
    pac_param_.rAx  = params["rAx"];
    pac_param_.rCx  = params["rCx"];

    pac_param_.rGy1 = params["rGy1"];
    pac_param_.rBy  = params["rBy"];

    pac_param_.comb_model = params["comb_model"];
}


void VehicleDynamics::update_simulation(double input_delta, 
                                        std::vector<double> input_torque, 
                                        double dt){
    input_delta_ = input_delta;
    torque_cmd_.fl_ = input_torque[0];
    torque_cmd_.fr_ = input_torque[1];
    torque_cmd_.rl_ = input_torque[2];
    torque_cmd_.rr_ = input_torque[3];
    dt_ = dt;  

    calculate_dynamics();
    integrate_dynamics();

    }

void VehicleDynamics::calculate_dynamics(){
    
    calculate_ackermann();
    calculate_tire_slip();
    calculate_tire_loads();

    Tire_force force_fl = calculate_tire_forces(tire_slip_.alpha_fl_, tire_slip_.lambda_fl_, tire_loads_.fl_);
    Tire_force force_fr = calculate_tire_forces(tire_slip_.alpha_fr_, tire_slip_.lambda_fr_, tire_loads_.fr_);
    Tire_force force_rl = calculate_tire_forces(tire_slip_.alpha_rl_, tire_slip_.lambda_rl_, tire_loads_.rl_);
    Tire_force force_rr = calculate_tire_forces(tire_slip_.alpha_rr_, tire_slip_.lambda_rr_, tire_loads_.rr_);

    double fy_front = force_fl.fy + force_fr.fy;
    double fy_rear = force_rl.fy + force_rr.fy;

    // TODO: calculate forces with individual wheels (because of ackermann)

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
    double alpha_w = std::min(std::max(0.01,0.01*vx_),1.0);
    w_fl_dot_ = alpha_w*(torque_cmd_.fl_ - force_fl.fx * kTireDynRadius) / kTireInertia_F + (1-alpha_w)*w_fl_dot_;
    w_fr_dot_ = alpha_w*(torque_cmd_.fr_ - force_fr.fx * kTireDynRadius) / kTireInertia_F + (1-alpha_w)*w_fr_dot_;
    w_rl_dot_ = alpha_w*(torque_cmd_.rl_ - force_rl.fx * kTireDynRadius) / kTireInertia_R + (1-alpha_w)*w_rl_dot_;
    w_rr_dot_ = alpha_w*(torque_cmd_.rr_ - force_rr.fx * kTireDynRadius) / kTireInertia_R + (1-alpha_w)*w_rr_dot_;

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
    delta_ = std::clamp(delta_, -kSteeringMax, kSteeringMax);
    delta_v_ += delta_v_dot_ * dt_;

    if(vx_ < 0){
        vx_ = 0;
    }
}

double VehicleDynamics::calculate_fx(Tire_force force_fl, Tire_force force_fr, Tire_force force_rl, Tire_force force_rr){

    double longitudinal_tire_force = (force_fl.fx + force_fr.fx) * std::cos(delta_) + force_rl.fx + force_rr.fx;
    longitudinal_tire_force -= (force_fl.fy + force_fr.fy) * std::sin(delta_);

    double aero_drag = 0.5 * kAirDensity * kCDA * vx_*vx_;

    double longitudinal_force = longitudinal_tire_force - aero_drag;
    if(abs(longitudinal_force) >= kRollingResistance) {
        int sign = longitudinal_force/abs(longitudinal_force);
        longitudinal_force -= sign*kRollingResistance;
    }

    return longitudinal_force;
}

void VehicleDynamics::calculate_tire_loads(){

    // Lateral load transfer
    // Nonsuspended
    double lateral_ns_f = kNsMassF * ay_ * kHCogNsF / kTrackWidth;
    double lateral_ns_r = kNsMassR * ay_ * kHCogNsR / kTrackWidth;
    
    // Suspended geometric
    double lateral_s_g_f = kSMassF * ay_ * kHRollCenterF / kTrackWidth;
    double lateral_s_g_r = kSMassR * ay_ * kHRollCenterR / kTrackWidth;

    // Suspended elastic
    double lateral_s_e_f = kSMass * ay_ * (kHCog - kHRollAxis) * kRollStiffnessF / kRollStiffness / kTrackWidth;
    double lateral_s_e_r = kSMass * ay_ * (kHCog - kHRollAxis) * kRollStiffnessR / kRollStiffness / kTrackWidth;

    // Longitudinal load transfer 
    double longitudinal_ns = (kNsMassF * kHCogNsF + kNsMassR * kHCogNsR) * ax_ / kWheelBase;
    double longitudinal_s = kSMass * kHCog * ax_ / kWheelBase;

    double lateral_load_transfer_front = lateral_ns_f + lateral_s_e_f + lateral_s_g_f;
    double lateral_load_transfer_rear = lateral_ns_r + lateral_s_e_r + lateral_s_g_r;
    double longitudinal_load_transfer = longitudinal_ns + longitudinal_s;

    tire_loads_.fl_ = kStaticLoadFront - lateral_load_transfer_front - longitudinal_load_transfer/2;
    tire_loads_.fr_ = kStaticLoadFront + lateral_load_transfer_front - longitudinal_load_transfer/2;
    tire_loads_.rl_ = kStaticLoadRear - lateral_load_transfer_rear + longitudinal_load_transfer/2;
    tire_loads_.rr_ = kStaticLoadRear + lateral_load_transfer_rear + longitudinal_load_transfer/2;

    double aero_lift = 0.5 * kAirDensity * kCLA * vx_*vx_;
    double aero_drag = 0.5 * kAirDensity * kCDA * vx_*vx_;

    tire_loads_.fl_ += (1 - kCOPx) * aero_lift / 2 - kCOPy / kWheelBase * aero_drag / 2;
    tire_loads_.fr_ += (1 - kCOPx) * aero_lift / 2 - kCOPy / kWheelBase * aero_drag / 2;
    tire_loads_.rl_ += kCOPx * aero_lift / 2 + kCOPy / kWheelBase * aero_drag / 2;
    tire_loads_.rr_ += kCOPx * aero_lift / 2 + kCOPy / kWheelBase * aero_drag / 2;

    if(tire_loads_.fl_ < 0){tire_loads_.fl_ = 0;}
    if(tire_loads_.fr_ < 0){tire_loads_.fr_ = 0;}
    if(tire_loads_.rl_ < 0){tire_loads_.rl_ = 0;}
    if(tire_loads_.rr_ < 0){tire_loads_.rr_ = 0;}

}

void VehicleDynamics::calculate_ackermann(){
    double delta_in_ackermann = std::atan( kWheelBase * std::tan(delta_) / (kWheelBase - std::abs(std::tan(delta_)) * kTrackWidth));

    if(delta_ <= 0){
        delta_fl_ = kAckermann1 * (delta_in_ackermann - delta_) + delta_;
        delta_fr_ = kAckermann2 * delta_;
    } else {
        delta_fl_ = kAckermann2 * delta_;
        delta_fr_ = kAckermann1 * (delta_in_ackermann - delta_) + delta_;
    }
}

void VehicleDynamics::calculate_tire_slip(){
    double vy_front = vy_ + kLf * r_;
    double vy_rear = vy_ - kLr * r_;
    double vx_left = vx_ - r_ * kTrackWidth / 2;
    double vx_right = vx_ + r_ * kTrackWidth / 2;

    if(vx_ > 1){
        tire_slip_.alpha_fl_ = std::atan(vy_front / vx_left) - delta_fl_;
        tire_slip_.alpha_fr_ = std::atan(vy_front / vx_right) - delta_fr_;
        tire_slip_.alpha_rl_ = std::atan(vy_rear / vx_left);
        tire_slip_.alpha_rr_ = std::atan(vy_rear / vx_right);
    }

    double vx_fl = std::sqrt(vy_front*vy_front + vx_left*vx_left) * std::cos(tire_slip_.alpha_fl_);
    double vx_fr = std::sqrt(vy_front*vy_front + vx_right*vx_right) * std::cos(tire_slip_.alpha_fr_);
    double vx_rl = std::sqrt(vy_rear*vy_rear + vx_left*vx_left) * std::cos(tire_slip_.alpha_rl_);
    double vx_rr = std::sqrt(vy_rear*vy_rear + vx_right*vx_right) * std::cos(tire_slip_.alpha_rr_);

    double eps = 0.001;

    tire_slip_.lambda_fl_ = kTireDynRadius * wheel_speed_.fl_ / (vx_fl + eps) - 1;
    tire_slip_.lambda_fr_ = kTireDynRadius * wheel_speed_.fr_ / (vx_fr + eps) - 1;
    tire_slip_.lambda_rl_ = kTireDynRadius * wheel_speed_.rl_ / (vx_rl + eps) - 1;
    tire_slip_.lambda_rr_ = kTireDynRadius * wheel_speed_.rr_ / (vx_rr + eps) - 1;

    if(vx_ < 0.1){
        tire_slip_.lambda_fl_ = kTireDynRadius * wheel_speed_.fl_ - vx_fl;
        tire_slip_.lambda_fr_ = kTireDynRadius * wheel_speed_.fr_ - vx_fr;
        tire_slip_.lambda_rl_ = kTireDynRadius * wheel_speed_.rl_ - vx_rl;
        tire_slip_.lambda_rr_ = kTireDynRadius * wheel_speed_.rr_ - vx_rr;
    }

}

VehicleDynamics::Tire_force VehicleDynamics::calculate_tire_forces(
    double slip_angle, double slip_ratio, double tire_load)
{
    Tire_force tf;

    double SA = slip_angle;
    double SR = slip_ratio;
    double Fz = tire_load;

    double D_lon, arg_x, fx_pure, fy_pure;
    double Bxa, Cxa, Exa, arg_gxa, Gxa;
    double Byk, Gyk, byk_term;
    double Gx_alpha, Gy_kappa;

    // Longitudinal puro
    D_lon = pac_param_.D1_x + pac_param_.D2_x * (Fz / pac_param_.Fz0);
    arg_x = pac_param_.Bx * SR;
    fx_pure = Fz * D_lon * sin(pac_param_.Cx * atan(arg_x - pac_param_.Ex * (arg_x - atan(arg_x))));

    // Lateral puro
    double Dy       = pac_param_.Lambda_mu_y * (pac_param_.D1_y + pac_param_.D2_y * (Fz / pac_param_.Fz0));  
    double alpha_eq = SA + pac_param_.SH;                           
    double aux_fy   = pac_param_.Ey * (pac_param_.By * alpha_eq - atan(pac_param_.By * alpha_eq));
    fy_pure = Fz * Dy * sin(pac_param_.Cy * atan(pac_param_.By * alpha_eq - aux_fy)) + pac_param_.SV;

    if(pac_param_.comb_model == 1)
    {
        // MODELO CHECHU
        // Long combined
        Bxa = pac_param_.rB1_x * cos(atan(pac_param_.rB2_x * SR));
        Cxa = pac_param_.rC1_x;
        Exa = pac_param_.rE1_x;

        arg_gxa = Bxa * SA - Exa * (Bxa * SA - atan(Bxa * SA));
        Gxa = cos(Cxa * atan(arg_gxa));

        // Lat combined
        Byk = pac_param_.rB1_y * cos(atan(pac_param_.rB2_y * SA));
        byk_term = Byk * (SR + pac_param_.rSh);
        Gyk = pac_param_.rC1_y * exp(-(byk_term * byk_term));

        tf.fx = fx_pure * Gxa;
        tf.fy = fy_pure * Gyk;
    }
    else if(pac_param_.comb_model == 2)
    {
        // MODELO CALERO
        double SA_tan = tan(SA);

        Gx_alpha = (1.0 - pac_param_.rBx) * exp(-pac_param_.rGx1 *
            exp(-pow(fabs(pac_param_.rAx * SR), pac_param_.rCx)) * SA_tan * SA_tan) + pac_param_.rBx;

        Gy_kappa = pac_param_.rBy + (1.0 - pac_param_.rBy) * exp(-pac_param_.rGy1 * SR * SR);

        tf.fx = fx_pure * Gx_alpha;
        tf.fy = fy_pure * Gy_kappa;
    }
    else
    {
        tf.fx = fx_pure;
        tf.fy = fy_pure;
    }

    return tf;
}

void VehicleDynamics::kinematic_correction(){
    double lambda = 1 - std::clamp((vx_ - 2.0)/(5.0 - 2.0),0.0,1.0);
    double r_kinematic = std::tan(delta_) * vx_ / kWheelBase;
    double vy_kinematic = r_kinematic * kLr;

    r_ = lambda * r_kinematic + (1 - lambda) * r_;
    vy_ = lambda * vy_kinematic + (1 - lambda) * vy_;
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
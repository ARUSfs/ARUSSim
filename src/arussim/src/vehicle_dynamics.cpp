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

    kHCoGNsMassF = params["h_cdg_nsm_f"];
    kHCoGNsMassR = params["h_cdg_nsm_r"];
    kHCoGsMass = params["h_cdg_sm"];

    kSMassF = kSMass * (1-kMassDistributionRear);
    kSMassR = kSMass * kMassDistributionRear;
    kMass = kSMass + kNsMassF + kNsMassR;
    kLf = kWheelBase*kMassDistributionRear;
    kLr = kWheelBase*(1-kMassDistributionRear); 

    kHRollCenterF = params["h_RC_f"];
    kHRollCenterR = params["h_RC_r"];
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


// Parámetros generales y escalas básicas
    pac_param_.LONGVL          = params["LONGVL"];
    pac_param_.NOMPRES         = params["NOMPRES"];
    pac_param_.FNOMIN          = params["FNOMIN"];
    pac_param_.UNLOADED_RADIUS = params["UNLOADED_RADIUS"];

    // Factores de escala (Lámbitas)
    pac_param_.LFZO   = params["LFZO"];
    pac_param_.LMUX   = params["LMUX"];
    pac_param_.LMUY   = params["LMUY"];
    pac_param_.LKX    = params["LKX"];
    pac_param_.LKY    = params["LKY"];
    pac_param_.LKYC   = params["LKYC"];
    pac_param_.LHY    = params["LHY"];
    pac_param_.LTR    = params["LTR"];
    pac_param_.LRES   = params["LRES"];
    pac_param_.LKZC   = params["LKZC"];
    pac_param_.LXAL   = params["LXAL"];
    pac_param_.LYKA   = params["LYKA"];
    pac_param_.LVYKA  = params["LVYKA"];
    pac_param_.LS     = params["LS"];

    // Fuerza longitudinal pura (Fx0)
    pac_param_.LCX  = params["LCX"];
    pac_param_.LEX  = params["LEX"];
    pac_param_.LHX  = params["LHX"];
    pac_param_.LVX  = params["LVX"];
    pac_param_.PCX1 = params["PCX1"];
    pac_param_.PDX1 = params["PDX1"];
    pac_param_.PDX2 = params["PDX2"];
    pac_param_.PDX3 = params["PDX3"];
    pac_param_.PEX1 = params["PEX1"];
    pac_param_.PEX2 = params["PEX2"];
    pac_param_.PEX3 = params["PEX3"];
    pac_param_.PEX4 = params["PEX4"];
    pac_param_.PKX1 = params["PKX1"];
    pac_param_.PKX2 = params["PKX2"];
    pac_param_.PKX3 = params["PKX3"];
    pac_param_.PHX1 = params["PHX1"];
    pac_param_.PHX2 = params["PHX2"];
    pac_param_.PVX1 = params["PVX1"];
    pac_param_.PVX2 = params["PVX2"];
    pac_param_.PPX1 = params["PPX1"];
    pac_param_.PPX2 = params["PPX2"];
    pac_param_.PPX3 = params["PPX3"];
    pac_param_.PPX4 = params["PPX4"];

    // Fuerza lateral pura (Fy0)
    pac_param_.LCY  = params["LCY"];
    pac_param_.LEY  = params["LEY"];
    pac_param_.LVY  = params["LVY"];
    pac_param_.PCY1 = params["PCY1"];
    pac_param_.PDY1 = params["PDY1"];
    pac_param_.PDY2 = params["PDY2"];
    pac_param_.PDY3 = params["PDY3"];
    pac_param_.PEY1 = params["PEY1"];
    pac_param_.PEY2 = params["PEY2"];
    pac_param_.PEY3 = params["PEY3"];
    pac_param_.PEY4 = params["PEY4"];
    pac_param_.PEY5 = params["PEY5"];
    pac_param_.PKY1 = params["PKY1"];
    pac_param_.PKY2 = params["PKY2"];
    pac_param_.PKY3 = params["PKY3"];
    pac_param_.PKY4 = params["PKY4"];
    pac_param_.PKY5 = params["PKY5"];
    pac_param_.PHY1 = params["PHY1"];
    pac_param_.PHY2 = params["PHY2"];
    pac_param_.PHY3 = params["PHY3"];
    pac_param_.PVY1 = params["PVY1"];
    pac_param_.PVY2 = params["PVY2"];
    pac_param_.PVY3 = params["PVY3"];
    pac_param_.PVY4 = params["PVY4"];
    pac_param_.PPY1 = params["PPY1"];
    pac_param_.PPY2 = params["PPY2"];
    pac_param_.PPY3 = params["PPY3"];
    pac_param_.PPY4 = params["PPY4"];

    // Momento de autoalineamiento puro (Mz0)
    pac_param_.QBZ1  = params["QBZ1"];
    pac_param_.QBZ2  = params["QBZ2"];
    pac_param_.QBZ3  = params["QBZ3"];
    pac_param_.QBZ4  = params["QBZ4"];
    pac_param_.QBZ5  = params["QBZ5"];
    pac_param_.QBZ9  = params["QBZ9"];
    pac_param_.QBZ10 = params["QBZ10"];
    pac_param_.QCZ1  = params["QCZ1"];
    pac_param_.QDZ1  = params["QDZ1"];
    pac_param_.QDZ2  = params["QDZ2"];
    pac_param_.QDZ3  = params["QDZ3"];
    pac_param_.QDZ4  = params["QDZ4"];
    pac_param_.QDZ6  = params["QDZ6"];
    pac_param_.QDZ7  = params["QDZ7"];
    pac_param_.QDZ8  = params["QDZ8"];
    pac_param_.QDZ9  = params["QDZ9"];
    pac_param_.QDZ10 = params["QDZ10"];
    pac_param_.QDZ11 = params["QDZ11"];
    pac_param_.QEZ1  = params["QEZ1"];
    pac_param_.QEZ2  = params["QEZ2"];
    pac_param_.QEZ3  = params["QEZ3"];
    pac_param_.QEZ4  = params["QEZ4"];
    pac_param_.QEZ5  = params["QEZ5"];
    pac_param_.QHZ1  = params["QHZ1"];
    pac_param_.QHZ2  = params["QHZ2"];
    pac_param_.QHZ3  = params["QHZ3"];
    pac_param_.QHZ4  = params["QHZ4"];
    pac_param_.PPZ1  = params["PPZ1"];
    pac_param_.PPZ2  = params["PPZ2"];
    pac_param_.SSZ1  = params["SSZ1"];
    pac_param_.SSZ2  = params["SSZ2"];
    pac_param_.SSZ3  = params["SSZ3"];
    pac_param_.SSZ4  = params["SSZ4"];

    // Coeficientes de adherencia combinada longitudinal (Gxa)
    pac_param_.RBX1 = params["RBX1"];
    pac_param_.RBX2 = params["RBX2"];
    pac_param_.RBX3 = params["RBX3"];
    pac_param_.RCX1 = params["RCX1"];
    pac_param_.REX1 = params["REX1"];
    pac_param_.REX2 = params["REX2"];
    pac_param_.RHX1 = params["RHX1"];

    // Coeficientes de adherencia combinada lateral (Gyk)
    pac_param_.RBY1 = params["RBY1"];
    pac_param_.RBY2 = params["RBY2"];
    pac_param_.RBY3 = params["RBY3"];
    pac_param_.RBY4 = params["RBY4"];
    pac_param_.RCY1 = params["RCY1"];
    pac_param_.REY1 = params["REY1"];
    pac_param_.REY2 = params["REY2"];
    pac_param_.RHY1 = params["RHY1"];
    pac_param_.RHY2 = params["RHY2"];
    pac_param_.RVY1 = params["RVY1"];
    pac_param_.RVY2 = params["RVY2"];
    pac_param_.RVY3 = params["RVY3"];
    pac_param_.RVY4 = params["RVY4"];
    pac_param_.RVY5 = params["RVY5"];
    pac_param_.RVY6 = params["RVY6"];
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

    Tire_force force_fl = calculate_tire_forces(tire_slip_.alpha_fl_, tire_slip_.lambda_fl_, tire_loads_.fl_, 0, pac_param_.NOMPRES);
    Tire_force force_fr = calculate_tire_forces(tire_slip_.alpha_fr_, tire_slip_.lambda_fr_, tire_loads_.fr_, 0, pac_param_.NOMPRES);
    Tire_force force_rl = calculate_tire_forces(tire_slip_.alpha_rl_, tire_slip_.lambda_rl_, tire_loads_.rl_, 0, pac_param_.NOMPRES);
    Tire_force force_rr = calculate_tire_forces(tire_slip_.alpha_rr_, tire_slip_.lambda_rr_, tire_loads_.rr_, 0, pac_param_.NOMPRES);

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
    //total_mz += force_fl.mz + force_fr.mz + force_rl.mz + force_rr.mz;

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
    double v = sqrt(vx_ * vx_ + vy_ * vy_);
    double F_L = 0.5 * kAirDensity * kCLA * pow(v, 2);
    double F_D = 0.5 * kAirDensity * kCDA * pow(v, 2);

    //LATERAL LOAD TRANSFER
    //nonsuspended weight transfer
    double y_WT_ns_f = kNsMassF * ay_ * kHCoGNsMassF / kTrackWidth;
    double y_WT_ns_r = kNsMassR * ay_ * kHCoGNsMassR / kTrackWidth;

    //susoended geometric weight transfer
    double y_WT_s_g_f = kSMassF * ay_ * kHRollCenterF / kTrackWidth;
    double y_WT_s_g_r = kSMassR * ay_ * kHRollCenterR / kTrackWidth;

    // susp. elastic WT
    double y_WT_s_e_f = kSMass * ay_ * (kHCoGsMass - kHRollAxis) * kRollStiffnessF/kRollStiffness/kTrackWidth;
    double y_WT_s_e_r = kSMass * ay_ * (kHCoGsMass - kHRollAxis) * kRollStiffnessR/kRollStiffness/kTrackWidth;

    //LONG. LOAD TRANSFER
    //long. nonsuspended wt
    double x_WT_ns = (kNsMassF * ax_ * kHCoGNsMassF + kNsMassR * ax_ * kHCoGNsMassR) / kWheelBase;

    //long suspended wt
    double x_WT_s = kSMass * ax_ * kHCoGsMass / kWheelBase;

    //long. suspended elastic weight transfer
    double x_WT_s_e_f = -x_WT_s * (ax_ < 0) - x_WT_s * (ax_ > 0); 
    double x_WT_s_e_r = x_WT_s * (ax_ < 0) + x_WT_s * (ax_ > 0);

    //TODO continuar a partir de wheel loads y mirar si hay que cambiar mas codigo (tema de mz que se calcula en forces)
    tire_loads_.fl_ = 0.5 * kMass * kG * (1 - kMassDistributionRear) + 0.5 * (1 - kCOPx) * F_L - 0.5 * kCOPy/kWheelBase * F_D - y_WT_ns_f - y_WT_s_g_f - y_WT_s_e_f - 0.5 * (x_WT_ns + x_WT_s);
    tire_loads_.fr_ = 0.5 * kMass * kG * (1 - kMassDistributionRear) + 0.5 * kCOPx * F_L - 0.5 * kCOPy/kWheelBase * F_D + y_WT_ns_f + y_WT_s_g_f + y_WT_s_e_f - 0.5 * (x_WT_ns + x_WT_s);
    tire_loads_.rl_ = 0.5 * kMass * kG * kMassDistributionRear + 0.5 * kCOPx * F_L + 0.5 * kCOPy/kWheelBase * F_D - y_WT_ns_r - y_WT_s_g_r - y_WT_s_e_r + 0.5 * (x_WT_ns + x_WT_s);
    tire_loads_.rr_ = 0.5 * kMass * kG * kMassDistributionRear + 0.5 * kCOPx * F_L + 0.5 * kCOPy/kWheelBase * F_D + y_WT_ns_r + y_WT_s_g_r + y_WT_s_e_r + 0.5 * (x_WT_ns + x_WT_s);

    //Negative tire load recalculation
    double tire_load [4] = {tire_loads_.fl_, tire_loads_.fr_, tire_loads_.rl_, tire_loads_.rr_};
    double tire_loadneg [4] = {0.0, 0.0, 0.0, 0.0};
    for (int i=0; i<4; i++) {
        if (tire_load[i] < 0) {
            tire_loadneg[i] = tire_load[i];
            tire_load[i] = 0.0;
        }
    }

    double tire_loadty[4] = {0.0, 0.0, 0.0, 0.0};
    double tire_loadtx[4] = {0.0, 0.0, 0.0, 0.0};
    double eps = 1e-9;

    if((tire_loadneg[0] < 0 && tire_loadneg[1] < 0) || (tire_loadneg[2] < 0 && tire_loadneg[3] < 0)) {
        tire_loadty[0] =  0.5 * (tire_loadneg[2] - tire_loadneg[3]);
        tire_loadty[1] = -0.5 * (tire_loadneg[2] - tire_loadneg[3]);
        tire_loadty[2] =  0.5 * (tire_loadneg[0] - tire_loadneg[1]);
        tire_loadty[3] = -0.5 * (tire_loadneg[0] - tire_loadneg[1]);    

        tire_loadtx[0] =  0.5 * (tire_loadneg[2] + tire_loadneg[3]);
        tire_loadtx[1] =  0.5 * (tire_loadneg[2] + tire_loadneg[3]);
        tire_loadtx[2] =  0.5 * (tire_loadneg[0] + tire_loadneg[1]);
        tire_loadtx[3] =  0.5 * (tire_loadneg[0] + tire_loadneg[1]);

        for(int i = 0; i < 4; ++i) {
            tire_load[i] += tire_loadty[i] + tire_loadtx[i];
        }
    }
    else if ((tire_loadneg[0] < 0 && tire_loadneg[2] < 0) || (tire_loadneg[1] < 0 && tire_loadneg[3] < 0)) {
        tire_loadty[0] = 0.5 * (tire_loadneg[1] + tire_loadneg[3]);
        tire_loadty[1] = 0.5 * (tire_loadneg[0] + tire_loadneg[2]);
        tire_loadty[2] = 0.5 * (tire_loadneg[1] + tire_loadneg[3]);
        tire_loadty[3] = 0.5 * (tire_loadneg[0] + tire_loadneg[2]);

        tire_loadtx[0] =  0.5 * (tire_loadneg[1] - tire_loadneg[3]);
        tire_loadtx[1] =  0.5 * (tire_loadneg[0] - tire_loadneg[2]);
        tire_loadtx[2] = -0.5 * (tire_loadneg[1] - tire_loadneg[3]);
        tire_loadtx[3] = -0.5 * (tire_loadneg[0] - tire_loadneg[2]);

        for(int i = 0; i < 4; ++i) {
            tire_load[i] += tire_loadty[i] + tire_loadtx[i];
        }
    }
    else if (tire_loadneg[0] < 0 || tire_loadneg[1] < 0) 
    {
        double WT_x = 0.5 * (x_WT_ns + x_WT_s);
        double WT_y = (y_WT_ns_f + y_WT_s_g_f + y_WT_s_e_f);
        double sign_ay = (ay_ > 0) ? 1.0 : ((ay_ < 0) ? -1.0 : 0.0);

        double factor_x = WT_x / (WT_x + std::abs(WT_y) + eps);
        double factor_y = WT_y / (WT_x + std::abs(WT_y) + eps) * sign_ay;

        for(int i = 0; i < 4; ++i) {
            tire_loadtx[i] = factor_x * tire_loadneg[i];
            tire_loadty[i] = factor_y * tire_loadneg[i];
        }

        tire_load[0] += tire_loadtx[1];
        tire_load[1] += tire_loadtx[0];
        tire_load[2] += tire_loadty[0];
        tire_load[3] += tire_loadty[1];

        // Recalcular si la transferencia provocó que el eje trasero ahora sea negativo
        double tire_loadneg2[4] = {0.0, 0.0, 0.0, 0.0};
        for(int i = 0; i < 4; ++i) {
            if (tire_load[i] < 0) {
                tire_loadneg2[i] = tire_load[i];
                tire_load[i] = 0.0;
            }
        }

        if (tire_loadneg2[2] < 0 || tire_loadneg2[3] < 0) {
            tire_load[0] += tire_loadneg2[3];
            tire_load[1] += tire_loadneg2[2];
        }
        if (tire_loadneg2[0] < 0 || tire_loadneg2[1] < 0) {
            double sum_front = 0.5 * (tire_loadneg2[0] + tire_loadneg2[1]);
            tire_load[2] += sum_front;
            tire_load[3] += sum_front;
        }
    }
    else if (tire_loadneg[2] < 0 || tire_loadneg[3] < 0) 
    {
        double WT_x = 0.5 * (x_WT_ns + x_WT_s);
        double WT_y = (y_WT_ns_f + y_WT_s_g_f + y_WT_s_e_f);
        double sign_ay = (ay_ > 0) ? 1.0 : ((ay_ < 0) ? -1.0 : 0.0);

        double factor_x = -WT_x / (-WT_x + std::abs(WT_y) + eps);
        double factor_y = WT_y / (-WT_x + std::abs(WT_y) + eps) * sign_ay;

        for(int i = 0; i < 4; ++i) {
            tire_loadtx[i] = factor_x * tire_loadneg[i];
            tire_loadty[i] = factor_y * tire_loadneg[i];
        }

        tire_load[0] += tire_loadty[2];
        tire_load[1] += tire_loadty[3];
        tire_load[2] += tire_loadtx[3];
        tire_load[3] += tire_loadtx[2];

        double tire_loadneg2[4] = {0.0, 0.0, 0.0, 0.0};
        for(int i = 0; i < 4; ++i) {
            if (tire_load[i] < 0) {
                tire_loadneg2[i] = tire_load[i];
                tire_load[i] = 0.0;
            }
        }

        if (tire_loadneg2[0] < 0 || tire_loadneg2[1] < 0) {
            tire_load[2] += tire_loadneg2[1];
            tire_load[3] += tire_loadneg2[0];
        }
        if (tire_loadneg2[2] < 0 || tire_loadneg2[3] < 0) {
            double sum_rear = 0.5 * (tire_loadneg2[2] + tire_loadneg2[3]);
            tire_load[0] += sum_rear;
            tire_load[1] += sum_rear;
        }
    }
    tire_loads_.fl_ = tire_load[0];
    tire_loads_.fr_ = tire_load[1];
    tire_loads_.rl_ = tire_load[2];
    tire_loads_.rr_ = tire_load[3];
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
    double slip_angle, double slip_ratio, double tire_load, double camber, double pressure)
{
    Tire_force tf;

    double alpha = slip_angle;
    double kappa = slip_ratio;
    double Fz = tire_load;
    double gamma = camber;
    double p = pressure;

    double epsilon = 1e-6;

    // Calculate basic variables
    double V0 = pac_param_.LONGVL;
    double pi0 = pac_param_.NOMPRES;
    double Fz0 = pac_param_.FNOMIN;
    double LFZO = pac_param_.LFZO;
    double LMUX = pac_param_.LMUX;
    double LMUY = pac_param_.LMUY;

    double Fz0_prime = LFZO * Fz0;
    double dfz = (Fz - Fz0_prime) / Fz0_prime;
    double dpi = (p - pi0) / pi0;

    double alpha_star = tan(alpha);
    double gamma_star = sin(gamma);

    // Same correction as dynamic model
    Fz = 0.5*(Fz + sqrt(pow(Fz, 2) + 1e-3));

    // Calculate FX0
    double LCX  = pac_param_.LCX;
    double LEX  = pac_param_.LEX;
    double LKX  = pac_param_.LKX;
    double LHX  = pac_param_.LHX;
    double LVX  = pac_param_.LVX;

    double PCX1 = pac_param_.PCX1;
    double PDX1 = pac_param_.PDX1;
    double PDX2 = pac_param_.PDX2;
    double PDX3 = pac_param_.PDX3;
    double PEX1 = pac_param_.PEX1;
    double PEX2 = pac_param_.PEX2;
    double PEX3 = pac_param_.PEX3;
    double PEX4 = pac_param_.PEX4;
    double PKX1 = pac_param_.PKX1;
    double PKX2 = pac_param_.PKX2;
    double PKX3 = pac_param_.PKX3;
    double PHX1 = pac_param_.PHX1;
    double PHX2 = pac_param_.PHX2;
    double PVX1 = pac_param_.PVX1;
    double PVX2 = pac_param_.PVX2;
    double PPX1 = pac_param_.PPX1;
    double PPX2 = pac_param_.PPX2;
    double PPX3 = pac_param_.PPX3;
    double PPX4 = pac_param_.PPX4;
    
    double Cx = PCX1 * LCX;
    double mux = (PDX1 + PDX2 * dfz) * (1 + PPX3 * dpi + PPX4 * pow(dpi, 2)) * (1 - PDX3 * pow(gamma, 2)) * LMUX;
    double Dx = mux * Fz;
    double Kxk = Fz * (PKX1 + PKX2 * dfz) * exp(PKX3 * dfz) * (1 + PPX1 * dpi + PPX2 * pow(dpi, 2) * LKX);
    double Bx = Kxk / (Cx * Dx + epsilon*tanh(1e4*Dx));
    double SHx = (PHX1 + PHX2 * dfz) * LHX;
    double SVx = Fz * (PVX1 + PVX2 * dfz) * LVX * LMUX;
    double kappax = kappa + SHx;
    double Ex = (PEX1 + PEX2 * dfz + PEX3 * pow(dfz, 2)) * (1 - PEX4 * tanh(1e4*kappax)) * LEX;
    Ex = 0.5 * (Ex + 1 - sqrt(pow(Ex-1, 2) + 1e-3));
    double Fx0 = Dx * sin(Cx * atan(Bx * kappax - Ex * (Bx * kappax - atan(Bx * kappax)))) + SVx;

    // Calculate Fy0
    double epsilony_local = epsilon;
    double LCY   = pac_param_.LCY;
    double LEY   = pac_param_.LEY;
    double LKY   = pac_param_.LKY;
    double LHY   = pac_param_.LHY;
    double LVY   = pac_param_.LVY;
    double LKYC  = pac_param_.LKYC;

    double PCY1  = pac_param_.PCY1;
    double PDY1  = pac_param_.PDY1;
    double PDY2  = pac_param_.PDY2;
    double PDY3  = pac_param_.PDY3;
    double PEY1  = pac_param_.PEY1;
    double PEY2  = pac_param_.PEY2;
    double PEY3  = pac_param_.PEY3;
    double PEY4  = pac_param_.PEY4;
    double PEY5  = pac_param_.PEY5;
    double PKY1  = pac_param_.PKY1;
    double PKY2  = pac_param_.PKY2;
    double PKY3  = pac_param_.PKY3;
    double PKY4  = pac_param_.PKY4;
    double PKY5  = pac_param_.PKY5;
    double PHY1  = pac_param_.PHY1;
    double PHY2  = pac_param_.PHY2;
    double PHY3  = pac_param_.PHY3;
    double PVY1  = pac_param_.PVY1;
    double PVY2  = pac_param_.PVY2;
    double PVY3  = pac_param_.PVY3;
    double PVY4  = pac_param_.PVY4;
    double PPY1  = pac_param_.PPY1;
    double PPY2  = pac_param_.PPY2;
    double PPY3  = pac_param_.PPY3;
    double PPY4  = pac_param_.PPY4;

    double Kya = PKY1 * Fz0_prime * (1 + PPY1 * dpi) * (1 - PKY3 * sqrt(pow(gamma_star, 2) + 1e-4)) * sin(PKY4 * atan(Fz / Fz0_prime) / ((PKY2 + PKY5 * (pow(gamma_star, 2)) * (1 + PPY2 * dpi)))) * LKY;
    double SVyg = Fz * (PVY3 + PVY4 * dfz) *  gamma_star * LKYC * LMUY;

    double SHy = (PHY1 + PHY2 * dfz) * LHY + PHY3 * gamma_star * LKYC;
    double SVy = Fz * (PVY1 + PVY2 * dfz) * LVY * LMUY + SVyg;
    double alphay = alpha_star + SHy;
    double Cy = PCY1 * LCY;
    double muy = (PDY1 + PDY2 * dfz) * (1 + PPY3 * dpi + PPY4 * pow(dpi, 2)) * (1 - PDY3 * pow(gamma_star, 2)) * LMUY;
    double Dy = muy * Fz;

    double Ey = (PEY1 + PEY2 * dfz) * (1 + PEY5 * pow(gamma_star, 2) - (PEY3 + PEY4 * gamma_star) * tanh(1e4*alphay)) * LEY;
    Ey = 0.5*(Ey + 1 - sqrt(pow(Ey - 1, 2) + 1e-3));
    double By = Kya / (Cy * Dy + epsilony_local * tanh(1e4*Dy));
    double Fy0 = Dy * sin(Cy * atan(By * alphay - Ey *(By * alphay -atan(By * alphay)))) + SVy;

    // Calculate Mz0 and related variables
    double epsilonk_local = epsilon;
    double R0    = pac_param_.UNLOADED_RADIUS;
    double LTR   = pac_param_.LTR;
    double LRES  = pac_param_.LRES;
    double LKZC  = pac_param_.LKZC;

    double QBZ1  = pac_param_.QBZ1;
    double QBZ2  = pac_param_.QBZ2;
    double QBZ3  = pac_param_.QBZ3;
    double QBZ4  = pac_param_.QBZ4;
    double QBZ5  = pac_param_.QBZ5;
    double QBZ9  = pac_param_.QBZ9;
    double QBZ10 = pac_param_.QBZ10;
    double QCZ1  = pac_param_.QCZ1;
    double QDZ1  = pac_param_.QDZ1;
    double QDZ2  = pac_param_.QDZ2;
    double QDZ3  = pac_param_.QDZ3;
    double QDZ4  = pac_param_.QDZ4;
    double QDZ6  = pac_param_.QDZ6;
    double QDZ7  = pac_param_.QDZ7;
    double QDZ8  = pac_param_.QDZ8;
    double QDZ9  = pac_param_.QDZ9;
    double QDZ10 = pac_param_.QDZ10;
    double QDZ11 = pac_param_.QDZ11;
    double QEZ1  = pac_param_.QEZ1;
    double QEZ2  = pac_param_.QEZ2;
    double QEZ3  = pac_param_.QEZ3;
    double QEZ4  = pac_param_.QEZ4;
    double QEZ5  = pac_param_.QEZ5;
    double QHZ1  = pac_param_.QHZ1;
    double QHZ2  = pac_param_.QHZ2;
    double QHZ3  = pac_param_.QHZ3;
    double QHZ4  = pac_param_.QHZ4;
    double PPZ1  = pac_param_.PPZ1;
    double PPZ2  = pac_param_.PPZ2;

    double SHt = QHZ1 + QHZ2 * dfz + (QHZ3 + QHZ4 * dfz) * gamma_star;
    double Kya_prime = Kya + epsilonk_local * tanh(1e4*Kya);
    double SHf = SHy + SVy / Kya_prime;
    double alphat = alpha_star + SHt;

    double Dt = (QDZ1 + QDZ2 * dfz) * (1 - PPZ1 * dpi) * (1 + QDZ3 * gamma + QDZ4 * pow(gamma, 2)) * Fz * (R0 / Fz0_prime) * LTR;
    double Bt = (QBZ1 + QBZ2 * dfz + QBZ3 * pow(dfz, 2)) * (1 + QBZ4 * gamma + QBZ5 * sqrt(pow(gamma, 2) + 1e-4)) * LKY / LMUY;
    double Ct = QCZ1;
    double Et =  (QEZ1 + QEZ2 * dfz + QEZ3 * pow(dfz, 2)) * (1 + (QEZ4 + QEZ5 * gamma_star) * (2/M_PI) * atan(Bt * Ct * alphat));
    Et = 0.5 * (Et + 1 - sqrt (pow(Et-1, 2) + 1e-3));

    double Dr = Fz * R0 * ((QDZ6 + QDZ7 * dfz) * LRES + ((QDZ8 + QDZ9 * dfz) * (1 + PPZ2 * dpi) + (QDZ10 + QDZ11 * dfz) * sqrt(pow(gamma_star, 2) + 1e-4)) * gamma_star * LKZC) * LMUY * cos(alpha_star);
    double Br = (QBZ9 * LKY / LMUY + QBZ10 * By * Cy);

    // CALCULATE COMBINED FX
    double LXAL = pac_param_.LXAL;
    double RBX1 = pac_param_.RBX1;
    double RBX2 = pac_param_.RBX2;
    double RBX3 = pac_param_.RBX3;
    double RCX1 = pac_param_.RCX1;
    double REX1 = pac_param_.REX1;
    double REX2 = pac_param_.REX2;
    double RHX1 = pac_param_.RHX1;
    double Cxa = RCX1;
    double Exa =  REX1 + REX2 * dfz;
    Exa = 0.5 * (Exa + 1 - sqrt(pow(Exa-1, 2) + 1e-3));
    double SHxa = RHX1;
    double Bxa = (RBX1 + RBX3 * pow(gamma_star, 2)) * cos(atan(RBX2 * kappa)) * LXAL;
    double alphas = alpha_star + SHxa;
    double Gxa0 = cos(Cxa * atan(Bxa * SHxa - Exa * (Bxa * SHxa - atan(Bxa *  SHxa))));
    double Gxa = cos(Cxa * atan(Bxa * alphas - Exa * (Bxa * alphas - atan(Bxa * alphas)))) / Gxa0;
    double force_fx = Gxa * Fx0;

    //CALCUALTE COMBINED FY
    double LYKA  = pac_param_.LYKA;
    double LVYKA = pac_param_.LVYKA;
    double RBY1  = pac_param_.RBY1;
    double RBY2  = pac_param_.RBY2;
    double RBY3  = pac_param_.RBY3;
    double RBY4  = pac_param_.RBY4;
    double RCY1  = pac_param_.RCY1;
    double REY1  = pac_param_.REY1;
    double REY2  = pac_param_.REY2;
    double RHY1  = pac_param_.RHY1;
    double RHY2  = pac_param_.RHY2;
    double RVY1  = pac_param_.RVY1;
    double RVY2  = pac_param_.RVY2;
    double RVY3  = pac_param_.RVY3;
    double RVY4  = pac_param_.RVY4;
    double RVY5  = pac_param_.RVY5;
    double RVY6  = pac_param_.RVY6;

    double DVyk = muy * Fz * (RVY1 + RVY2 * dfz + RVY3 * gamma_star) * cos(atan(RVY4 * alpha_star));
    double SVyk = DVyk * sin(RVY5 * atan(RVY6 * kappa)) * LVYKA;
    double SHyk = RHY1 + RHY2 * dfz;
    double Eyk = REY1 + REY2 * dfz;
    Eyk = 0.5* (Eyk + 1 - sqrt(pow(Eyk - 1, 2) + 1e-3));
    double Cyk = RCY1;
    double Byk = (RBY1 + RBY4 * pow(gamma_star, 2)) * cos(atan(RBY2 * (alpha_star - RBY3))) * LYKA;
    double kappas = kappa + SHyk;
    double Gyk0 = cos(Cyk * atan(Byk * SHyk - Eyk * (Byk * SHyk - atan(Byk * SHyk))));
    double Gyk = cos(Cyk * atan(Byk * kappas - Eyk * (Byk * kappas - atan(Byk * kappas)))) / Gyk0;
    double force_fy = Gyk * Fy0 + SVyk;

    // CALCULATE COMBINED MZ
    double LS   = pac_param_.LS;
    double SSZ1 = pac_param_.SSZ1;
    double SSZ2 = pac_param_.SSZ2;
    double SSZ3 = pac_param_.SSZ3;
    double SSZ4 = pac_param_.SSZ4;

    double alpha_eq = atan(sqrt(pow(tan(alphat), 2) + pow(Kxk / Kya_prime, 2) * pow(kappa, 2))) * tanh(1e4*alphat);
    double s_val = R0 * (SSZ1 + SSZ2 * (force_fy / Fz0) + (SSZ3 + SSZ4 * dfz) * gamma) * LS;
    double Mzr = Dr * cos(atan(Br * (alpha_star + SHf)));
    double t_val = Dt * cos(Ct * atan(Bt * alpha_eq - Et * (Bt * alpha_eq - atan(Bt * alpha_eq))));
    t_val = t_val * LFZO;
    double Mz = -t_val * (force_fy - SVyk) + Mzr + s_val * force_fx;

    tf.fx = force_fx;
    tf.fy = force_fy;
    tf.mz = Mz;

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
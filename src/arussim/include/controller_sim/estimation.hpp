#include "controller_sim.hpp"


void ControllerSim::initialize_speed_estimator(){   
    est_.B_(0,0) = 1.;
    est_.B_(1,1) = est_.cf_ / kMass;
    est_.B_(2,1) = est_.cf_ * kLf / kIzz;
    
    est_.Q_(0,0) = 0.025;
    est_.Q_(1,1) = 0.01;
    est_.Q_(2,2) = 0.01;

    est_.H_TME_(2,0) = 1.;
    est_.H_TME_(2,2) = -0.5*kTrackWidth;
    est_.H_TME_(3,0) = 1.;
    est_.H_TME_(3,2) = 0.5*kTrackWidth;
    est_.H_TME_(6,1) = 1.;
    est_.H_TME_(6,2) = -kLr;
    est_.H_TME_(7,1) = 1.;
    est_.H_TME_(7,2) = -kLr;
    est_.H_TME_(8,2) = 1.;

    est_.R_TME_(0,0) = 0.015;
    est_.R_TME_(1,1) = 0.015;
    est_.R_TME_(2,2) = 0.015;
    est_.R_TME_(3,3) = 0.015;
    est_.R_TME_(4,4) = 0.05;
    est_.R_TME_(5,5) = 0.05;
    est_.R_TME_(6,6) = 0.025;
    est_.R_TME_(7,7) = 0.025;
    est_.R_TME_(8,8) = 0.002;

    est_.R_GSS_(0,0) = 0.0128;
    est_.R_GSS_(1,1) = 0.0128;
    est_.R_GSS_(2,2) = 0.002;
    
    est_.v_filter_.set_estimation_period(dt_);
    if(est_.use_GSS_){
        est_.v_filter_.set_problem_size(est_.n_, est_.m_, est_.p_GSS_);
        est_.v_filter_.set_measurement_matrices(est_.H_GSS_, est_.R_GSS_);

    } else {
        est_.v_filter_.set_problem_size(est_.n_, est_.m_, est_.p_TME_);
        est_.v_filter_.set_measurement_matrices(est_.H_TME_, est_.R_TME_);
    }
    est_.v_filter_.set_initial_data(est_.x_, est_.P_, est_.u_);
    est_.v_filter_.set_process_matrices(est_.M_, est_.B_, est_.Q_);
};

void ControllerSim::get_estimation(){
    // Update model matrix        
    double vx_prev = est_.x_(0);
    if(vx_prev < est_.vx_threshold_){
        est_.M_(1,1) = 0.;
        est_.M_(1,2) = 0.;
        est_.M_(2,1) = 0.;
        est_.M_(2,2) = 0.;
    } else {
        est_.M_(1,1) = -(est_.cf_ + est_.cr_) / (kMass*vx_prev);
        est_.M_(1,2) = (kLr*est_.cr_ - kLf*est_.cf_) / (kMass*vx_prev) - vx_prev;
        est_.M_(2,1) = (kLr*est_.cr_ - kLf*est_.cf_) / (kIzz*vx_prev);
        est_.M_(2,2) = -(kLf*kLf*est_.cf_ + kLr*kLr*est_.cr_) / (kIzz*vx_prev);
    }

    est_.v_filter_.update_model_matrix(est_.M_);

    // Update control vector
    est_.u_(0) = ax_;
    est_.u_(1) = delta_;
    
    // TMeasy inverse model
    calculate_tire_loads(*this);

    VectorXd F_wz(4);
    F_wz(0) = tire_loads_.fl_;
    F_wz(1) = tire_loads_.fr_;
    F_wz(2) = tire_loads_.rl_;
    F_wz(3) = tire_loads_.rr_;

    double k_fz = 1/(F_wz.sum() + est_.eps_);
    double F_wx_i, F_wy_i, alpha_i;

    double dF_x0_i, F_x_M_i, F_x_S_i;
    double dF_y0_i, F_y_M_i, F_y_S_i;
    double sx_norm_i, sy_norm_i, phi_i;
    double s_prev_i, s_i, F_i, dF_0_i, F_M_i, F_S_i, s_M_i, s_S_i;
    double cos_phi_i, sin_phi_i;

    VectorXd sr_TME(4);
    VectorXd w(4), torque(4);
    w(0) = wheel_speed_.fl_;
    w(1) = wheel_speed_.fr_;
    w(2) = wheel_speed_.rl_;
    w(3) = wheel_speed_.rr_;

    torque(0) = torque_cmd_.fl_;
    torque(1) = torque_cmd_.fr_;
    torque(2) = torque_cmd_.rl_;
    torque(3) = torque_cmd_.rr_;


    for (int i = 0; i < 4; i++) {
        // Estimate tire forces
        alpha_i = (est_.w_prev_(i) - w(i))/dt_;
        est_.w_prev_(i) = w(i);
        
        F_wx_i = (kGearRatio*torque(i) - kTireInertia*alpha_i)/kTireDynRadius;
        F_wy_i = F_wz(i) * k_fz * (kMass*ay_);

        // Calculate combined parameters
        dF_x0_i = est_.dMu_x_0_ * F_wz(i);
        F_x_M_i  = est_.Mu_x_M_ * F_wz(i);
        F_x_S_i  = est_.Mu_x_S_ * F_wz(i);

        dF_y0_i = est_.dMu_y_0_ * F_wz(i);
        F_y_M_i  = est_.Mu_y_M_ * F_wz(i);
        F_y_S_i  = est_.Mu_y_S_ * F_wz(i);

        sx_norm_i = est_.s_x_M_/(est_.s_x_M_ + est_.s_y_M_) + (F_x_M_i/(dF_x0_i+est_.eps_))/(F_x_M_i/(dF_x0_i+est_.eps_) + F_y_M_i/(dF_y0_i+est_.eps_));
        sy_norm_i = est_.s_y_M_/(est_.s_x_M_ + est_.s_y_M_) + (F_y_M_i/(dF_y0_i+est_.eps_))/(F_x_M_i/(dF_x0_i+est_.eps_) + F_y_M_i/(dF_y0_i+est_.eps_));
        
        F_i = std::sqrt(F_wx_i*F_wx_i + F_wy_i*F_wy_i);
        phi_i  = std::atan2(F_wy_i, F_wx_i);
        cos_phi_i = std::cos(phi_i);
        sin_phi_i = std::sin(phi_i);

        dF_0_i  = std::sqrt((dF_x0_i*sx_norm_i *cos_phi_i)*(dF_x0_i*sx_norm_i *cos_phi_i) + (dF_y0_i*sy_norm_i *sin_phi_i)*(dF_y0_i*sy_norm_i *sin_phi_i));
        F_M_i  = std::sqrt((F_x_M_i*cos_phi_i)*(F_x_M_i*cos_phi_i) + (F_y_M_i*sin_phi_i)*(F_y_M_i*sin_phi_i));
        F_S_i  = std::sqrt((F_x_S_i*cos_phi_i)*(F_x_S_i*cos_phi_i) + (F_y_S_i*sin_phi_i)*(F_y_S_i*sin_phi_i));
        s_M_i  = std::sqrt((est_.s_x_M_/sx_norm_i *cos_phi_i)*(est_.s_x_M_/sx_norm_i *cos_phi_i) + (est_.s_y_M_/sy_norm_i *sin_phi_i)*(est_.s_y_M_/sy_norm_i *sin_phi_i));
        s_S_i  = std::sqrt((est_.s_x_S_/sx_norm_i *cos_phi_i)*(est_.s_x_S_/sx_norm_i *cos_phi_i) + (est_.s_y_S_/sy_norm_i *sin_phi_i)*(est_.s_y_S_/sy_norm_i *sin_phi_i));

        s_prev_i = std::sqrt((est_.sr_(i)/sx_norm_i *cos_phi_i)*(est_.sr_(i)/sx_norm_i *cos_phi_i) + (est_.sa_(i)/sy_norm_i *sin_phi_i)*(est_.sa_(i)/sy_norm_i *sin_phi_i));

        // Calculate combined slip from TMEasy inverse model
        double solutions[5] = {-1. ,-1., -1., -1., -1.};

        // First piece
        double b = dF_0_i*s_M_i*(1/(F_M_i + est_.eps_) - 1/(F_i + est_.eps_)) - 2;

        double sol;
        if(b*b -4 < 0){	
            solutions[0] = s_M_i;
        } else {
            sol = s_M_i *  0.5*(-b + std::sqrt(b*b - 4));
            if(0 <= sol && sol <= s_M_i){
                solutions[0] = sol;
            }

            sol = s_M_i *  0.5*(-b - std::sqrt(b*b - 4));
            if(0 <= sol && sol <= s_M_i){
                solutions[1] = sol;
            }
        }

        // Second piece
        double a = 2;
        b = -3;
        double d = (F_i - F_M_i) / (F_S_i - F_M_i + est_.eps_);

        double p = - b*b / (3*a*a);
        double q = (2*b*b*b + 27*a*a*d) / (27*a*a*a);
        double D = q*q/4 + p*p*p/27;

        if(D < 0){
            double r = std::sqrt(-p*p*p/27);
            double theta = std::acos(-q/(2*r));

            sol = (s_S_i - s_M_i) * (-b/(3*a) + 2*std::sqrt(-p/3)*std::cos(theta/3)) + s_M_i;
            if(s_M_i < sol && sol <= s_S_i){
                solutions[2] = sol;
            }

            sol = (s_S_i - s_M_i) * (-b/(3*a) + 2*std::sqrt(-p/3)*std::cos((theta+2*M_PI)/3)) + s_M_i;
            if(s_M_i < sol && sol <= s_S_i){
                solutions[3] = sol;
            }

            sol = (s_S_i - s_M_i) * (-b/(3*a) + 2*std::sqrt(-p/3)*std::cos((theta+4*M_PI)/3)) + s_M_i;
            if(s_M_i < sol && sol <= s_S_i){
                solutions[4] = sol;
            }
        } else {
            double alpha = std::cbrt(-q/2 + std::sqrt(D));
            double beta = std::cbrt(-q/2 - std::sqrt(D));

            sol = (s_S_i - s_M_i) * (alpha + beta) + s_M_i;
            if(s_M_i < sol && sol <= s_S_i){
                solutions[2] = sol;
            }
        }

        // Choose the solution
        if(solutions[0]>=0 || solutions[1]>=0 || solutions[2]>=0 || solutions[3]>=0 || solutions[4]>=0){
            if(F_i < F_S_i){
                if(solutions[0]>=0 && std::abs(s_prev_i - solutions[0]) < std::abs(s_prev_i - solutions[1])){
                    s_i = solutions[0];
                } else if(solutions[1]>=0.f) {
                    s_i = solutions[1];
                } else {
                    s_i = s_prev_i;
                }
            } else {
                double s_min_diff;
                double min_diff = 1000.;
                double diff;
                for(int i=0; i<5; i++){
                    diff = std::abs(s_prev_i - solutions[i]);
                    if(solutions[i]>=0 && diff < min_diff){
                        s_min_diff = solutions[i];
                        min_diff = diff;
                    }
                }
                s_i = s_min_diff;
            }

            sr_TME(i) = s_i*cos_phi_i*sx_norm_i;
            est_.sa_(i) = -s_i*sin_phi_i*sy_norm_i;
        } else {
            sr_TME(i) = est_.sr_(i);
        }
    }

    // Update observation matrix
    VectorXd delta_ack = calculate_steering(*this);
    est_.H_TME_(0,0) = std::cos(delta_ack(0));
    est_.H_TME_(0,1) = std::sin(delta_ack(0));
    est_.H_TME_(0,2) = -0.5*kTrackWidth*std::cos(delta_ack(0)) + kLf*std::sin(delta_ack(0));
    est_.H_TME_(1,0) = std::cos(delta_ack(1));
    est_.H_TME_(1,1) = std::sin(delta_ack(1));
    est_.H_TME_(1,2) = 0.5*kTrackWidth*std::cos(delta_ack(1)) + kLf*std::sin(delta_ack(1));
    est_.H_TME_(4,0) = -std::sin(delta_ack(0));
    est_.H_TME_(4,1) = std::cos(delta_ack(0));
    est_.H_TME_(4,2) = kLf*std::cos(delta_ack(0)) + 0.5*kTrackWidth*std::sin(delta_ack(0));
    est_.H_TME_(5,0) = -std::sin(delta_ack(1));
    est_.H_TME_(5,1) = std::cos(delta_ack(1));
    est_.H_TME_(5,2) = kLf*std::cos(delta_ack(1)) - 0.5*kTrackWidth*std::sin(delta_ack(1));

    if(est_.use_GSS_){
        // Form measurement vector
        VectorXd z(est_.p_GSS_);
        z(0) = vx_;
        z(1) = vy_;
        z(2) = r_;

        // Estimate state
        est_.v_filter_.estimate_state(est_.u_, z);
    } else {
        est_.v_filter_.update_observation_matrix(est_.H_TME_);
    
        // Calculate measurement vector
        VectorXd z(est_.p_TME_);
        for(int i=0; i<4; i++){
            z(i) = kTireDynRadius*w(i) / (sr_TME(i) + 1.);
            z(i+4) = std::tan(est_.sa_(i))*z(i);
        }
        z(8) = r_;

        // Estimate state
        est_.v_filter_.estimate_state(est_.u_, z);
    }
    
    est_.x_ = est_.v_filter_.get_estimated_state();

    vx_ = est_.x_(0);
    vy_ = est_.x_(1);
    r_ = est_.x_(2);

    // Correct sr estimation
    VectorXd z_est = est_.H_TME_ * est_.x_;
    double sr_def_i, k_i;

    for(int i=0; i<4; i++){
        sr_def_i = (kTireDynRadius*w(i)) / (z_est(i) + est_.eps_) - 1;    

        if(std::abs(est_.sr_(i)) <= 0.035){
            k_i = 0.;
        } else if(0.035 < std::abs(est_.sr_(i)) && std::abs(est_.sr_(i)) <= 0.045){
            if(est_.sr_(i) > 0){
                k_i = 50*(est_.sr_(i) - 0.035);
            } else {
                k_i = -50*(est_.sr_(i) + 0.035);
            }
        } else {
            k_i = 1.;
        }

        est_.sr_(i) = k_i*sr_def_i + (1-k_i)*sr_TME(i);
    }

    slip_ratio_.fl_ = est_.sr_(0);
    slip_ratio_.fr_ = est_.sr_(1);
    slip_ratio_.rl_ = est_.sr_(2);
    slip_ratio_.rr_ = est_.sr_(3);

    // PROVISIONAL: lo dejo por ahora por las diferencias entre este cálculo de sr y el de vdc
    calculate_slip_ratio(*this);
}

void calculate_tire_loads(ControllerSim& controller_sim){

    double load_transfer_ay = controller_sim.get_kMass() * controller_sim.get_kHCog() * controller_sim.ay_ / controller_sim.get_kTrackWidth();
    double load_transfer_ax = controller_sim.get_kMass() * controller_sim.get_kHCog() * controller_sim.ax_ / controller_sim.get_kWheelBase();

    controller_sim.tire_loads_.fl_ = controller_sim.get_kStaticLoadFront() - (1 - controller_sim.get_kMassDistributionRear()) * load_transfer_ay - load_transfer_ax/2;
    controller_sim.tire_loads_.fr_ = controller_sim.get_kStaticLoadFront() + (1 - controller_sim.get_kMassDistributionRear()) * load_transfer_ay - load_transfer_ax/2;
    controller_sim.tire_loads_.rl_ = controller_sim.get_kStaticLoadRear() - controller_sim.get_kMassDistributionRear() * load_transfer_ay + load_transfer_ax/2;
    controller_sim.tire_loads_.rr_ = controller_sim.get_kStaticLoadRear() + controller_sim.get_kMassDistributionRear() * load_transfer_ay + load_transfer_ax/2;

    double aero_lift = 0.5 * controller_sim.get_kAirDensity() * controller_sim.get_kCLA() * controller_sim.vx_*controller_sim.vx_;
    double aero_drag = 0.5 * controller_sim.get_kAirDensity() * controller_sim.get_kCDA() * controller_sim.vx_*controller_sim.vx_;

    controller_sim.tire_loads_.fl_ += controller_sim.get_kCOPx() * aero_lift / 2 - (controller_sim.get_kCOPy() - controller_sim.get_kHCog()) * aero_drag;
    controller_sim.tire_loads_.fr_ += controller_sim.get_kCOPx() * aero_lift / 2 - (controller_sim.get_kCOPy() - controller_sim.get_kHCog()) * aero_drag;
    controller_sim.tire_loads_.rl_ += (1 - controller_sim.get_kCOPx()) * aero_lift / 2 + (controller_sim.get_kCOPy() - controller_sim.get_kHCog()) * aero_drag;
    controller_sim.tire_loads_.rr_ += (1 - controller_sim.get_kCOPx()) * aero_lift / 2 + (controller_sim.get_kCOPy() - controller_sim.get_kHCog()) * aero_drag;


    if(controller_sim.tire_loads_.fl_ < 0){controller_sim.tire_loads_.fl_ = 0;}
    if(controller_sim.tire_loads_.fr_ < 0){controller_sim.tire_loads_.fr_ = 0;}
    if(controller_sim.tire_loads_.rl_ < 0){controller_sim.tire_loads_.rl_ = 0;}
    if(controller_sim.tire_loads_.rr_ < 0){controller_sim.tire_loads_.rr_ = 0;}

}

VectorXd calculate_steering(ControllerSim& controller_sim){
    VectorXd delta_ack(2);
    double delta_in_ackermann = std::atan(controller_sim.get_kWheelBase()*std::tan(controller_sim.delta_)/(controller_sim.get_kWheelBase() - std::abs(std::tan(controller_sim.delta_))*controller_sim.get_kTrackWidth()));

    if(controller_sim.delta_ > 0.0f){
        delta_ack(0) = controller_sim.get_kAckermann1()*(delta_in_ackermann - controller_sim.delta_) + controller_sim.delta_;
        delta_ack(1) = controller_sim.get_kAckermann2()*controller_sim.delta_;
    } else {
        delta_ack(0) = controller_sim.get_kAckermann2()*controller_sim.delta_;
        delta_ack(1) = controller_sim.get_kAckermann1()*(delta_in_ackermann - controller_sim.delta_) + controller_sim.delta_;
    }

    return delta_ack;
}

void calculate_slip_ratio(ControllerSim& controller_sim){
    double vy_front = controller_sim.vy_ + controller_sim.get_kLf() * controller_sim.r_;
    double vy_rear = controller_sim.vy_ - controller_sim.get_kLr() * controller_sim.r_;
    double vx_left = controller_sim.vx_ - controller_sim.r_ * controller_sim.get_kTrackWidth() / 2;
    double vx_right = controller_sim.vx_ + controller_sim.r_ * controller_sim.get_kTrackWidth() / 2;

    if(controller_sim.vx_ > 1){
        controller_sim.slip_angle_.fl_ = std::atan(vy_front / vx_left) - controller_sim.delta_;
        controller_sim.slip_angle_.fr_ = std::atan(vy_front / vx_right) - controller_sim.delta_;
        controller_sim.slip_angle_.rl_ = std::atan(vy_rear / vx_left);
        controller_sim.slip_angle_.rr_ = std::atan(vy_rear / vx_right);
    }

    double vx_fl = std::sqrt(vy_front*vy_front + vx_left*vx_left) * std::cos(controller_sim.slip_angle_.fl_);
    double vx_fr = std::sqrt(vy_front*vy_front + vx_right*vx_right) * std::cos(controller_sim.slip_angle_.fr_);
    double vx_rl = std::sqrt(vy_rear*vy_rear + vx_left*vx_left) * std::cos(controller_sim.slip_angle_.rl_);
    double vx_rr = std::sqrt(vy_rear*vy_rear + vx_right*vx_right) * std::cos(controller_sim.slip_angle_.rr_);

    double eps = 0.01;

    controller_sim.slip_ratio_.fl_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.fl_ / (vx_fl + eps) - 1;
    controller_sim.slip_ratio_.fr_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.fr_ / (vx_fr + eps) - 1;
    controller_sim.slip_ratio_.rl_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.rl_ / (vx_rl + eps) - 1;
    controller_sim.slip_ratio_.rr_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.rr_ / (vx_rr + eps) - 1;

    if(controller_sim.vx_ < 0.1){
        controller_sim.slip_ratio_.fl_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.fl_ / (vx_fl + eps);
        controller_sim.slip_ratio_.fr_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.fr_ / (vx_fr + eps);
        controller_sim.slip_ratio_.rl_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.rl_ / (vx_rl + eps);
        controller_sim.slip_ratio_.rr_ = controller_sim.get_kTireDynRadius() * controller_sim.wheel_speed_.rr_ / (vx_rr + eps);
    }

}

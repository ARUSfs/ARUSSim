#include "controller_sim.hpp"

double sr_e[4], sr_e_prev_[4], sr_e_int_[4];

void ControllerSim::get_traction_control()
{

    if(!get_kTractionControl()) return;

    double tc_in[4], t_obj[4], tc_calc[4], tc_out[4];
    
    tc_in[0] = torque_cmd_.fl_;
    tc_in[1] = torque_cmd_.fr_;
    tc_in[2] = torque_cmd_.rl_;
    tc_in[3] = torque_cmd_.rr_;

    sr_e[0] = get_kTargetSlipRatio() - std::abs(slip_ratio_.fl_);
    sr_e[1] = get_kTargetSlipRatio() - std::abs(slip_ratio_.fr_);
    sr_e[2] = get_kTargetSlipRatio() - std::abs(slip_ratio_.rl_);
    sr_e[3] = get_kTargetSlipRatio() - std::abs(slip_ratio_.rr_);

    double sr[4], sa[4], fz[4];
    sr[0] = slip_ratio_.fl_;
    sr[1] = slip_ratio_.fr_;
    sr[2] = slip_ratio_.rl_;
    sr[3] = slip_ratio_.rr_;

    sa[0] = slip_angle_.fl_;
    sa[1] = slip_angle_.fr_;
    sa[2] = slip_angle_.rl_;
    sa[3] = slip_angle_.rr_;

    fz[0] = tire_loads_.fl_;
    fz[1] = tire_loads_.fr_;
    fz[2] = tire_loads_.rl_;
    fz[3] = tire_loads_.rr_;

    for(int i=0; i<4; i++) sr_e_int_[i] += sr_e[i];

    for(int i=0; i<4; i++) {

        if(std::abs(tc_in[i]) < 0.1){
            tc_calc[i] = 0.;
            sr_e_int_[i] = 0.;
            sr_e_prev_[i] = 0.;
            continue;
        }

        double fx_obj = calculate_tire_force(sa[i], get_kTargetSlipRatio(), fz[i], *this);
        double inertia_term = ax_ * (1 + get_kTargetSlipRatio()) / get_kTireDynRadius() * get_kTireInertia() / get_kGearRatio();
        t_obj[i] = fx_obj * get_kTireDynRadius() / get_kGearRatio() + inertia_term;

        sr_e_int_[i] = sr_e_int_[i] + sr_e[i];
        tc_calc[i] = t_obj[i]
            + get_kTCKp() * sr_e[i]
            + get_kTCKi() * sr_e_int_[i]
            - get_kTCKd() * (sr_e[i] - sr_e_prev_[i]);

        tc_out[i] = std::clamp(tc_in[i], -tc_calc[i], tc_calc[i]);

        if(tc_in[i] >= 0.0f && tc_calc[i] < 0.0f){
            tc_out[i] = tc_in[i];
        }

        // Anti-windup
        if(std::abs(tc_calc[i]) > std::abs(tc_out[i])) sr_e_int_[i] = 0.;

        // Error memory
        sr_e_prev_[i] = sr_e[i];

    }

    torque_cmd_.fl_ = tc_out[0];
    torque_cmd_.fr_ = tc_out[1];
    torque_cmd_.rl_ = tc_out[2];
    torque_cmd_.rr_ = tc_out[3];

}

double calculate_tire_force(double slip_angle, double slip_ratio, double fz, ControllerSim& controller_sim){
  
    double Blon = controller_sim.get_pac_param().Blon;
    double Elon = controller_sim.get_pac_param().Elon;
    double Clon = controller_sim.get_pac_param().Clon;
    double Dlon = controller_sim.get_pac_param().Dlon;
    double Gx1 = controller_sim.get_pac_param().Gx1;
    double a = controller_sim.get_pac_param().a;
    double bx = controller_sim.get_pac_param().bx;
    double c = controller_sim.get_pac_param().c;
   
    slip_angle = std::tan(slip_angle);

    // Combined scaling factors
    double gx_alpha = (1 - bx) * std::exp(-Gx1 * std::exp(-std::pow(std::abs(a * slip_ratio), c))*std::pow(slip_angle,2)) + bx;

    // Pure force
    double aux_fx = Elon * (Blon * slip_ratio - std::atan(Blon * slip_ratio));
    double fx_pure = fz * Dlon * std::sin(Clon * std::atan(Blon * slip_ratio - aux_fx));

    return gx_alpha * fx_pure;

}

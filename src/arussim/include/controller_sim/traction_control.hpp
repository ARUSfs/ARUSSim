#include "controller_sim.hpp"

void ControllerSim::get_traction_control()
{

    double torque_tc_[4];

    calculate_torque_feedforward(torque_tc_, *this);

    calculate_pid_control(torque_tc_, *this);

    torque_cmd_.fl_ = std::clamp(torque_cmd_.fl_, -torque_tc_[0], torque_tc_[0]);
    torque_cmd_.fr_ = std::clamp(torque_cmd_.fr_, -torque_tc_[1], torque_tc_[1]);
    torque_cmd_.rl_ = std::clamp(torque_cmd_.rl_, -torque_tc_[2], torque_tc_[2]);
    torque_cmd_.rr_ = std::clamp(torque_cmd_.rr_, -torque_tc_[3], torque_tc_[3]);

}

void calculate_torque_feedforward(double torque_tc[], ControllerSim& controller_sim){
    double D = controller_sim.get_pac_param().Dlon;
    double B = controller_sim.get_pac_param().Blon;
    double C = controller_sim.get_pac_param().Clon;

    double mux_max = D * std::sin(C * std::atan(B * controller_sim.get_kTargetSlipRatio()));

    torque_tc[0] = controller_sim.tire_loads_.fl_ * mux_max * controller_sim.get_kTireDynRadius();
    torque_tc[1] = controller_sim.tire_loads_.fr_ * mux_max * controller_sim.get_kTireDynRadius();
    torque_tc[2] = controller_sim.tire_loads_.rl_ * mux_max * controller_sim.get_kTireDynRadius();
    torque_tc[3] = controller_sim.tire_loads_.rr_ * mux_max * controller_sim.get_kTireDynRadius();
}

void calculate_pid_control(double torque_tc[], ControllerSim& controller_sim){
    
    double Kp = controller_sim.get_kTCKp();
    if (controller_sim.vx_ > 2){
        torque_tc[0] += Kp * (controller_sim.get_kTargetSlipRatio() - std::abs(controller_sim.slip_ratio_.fl_));
        torque_tc[1] += Kp * (controller_sim.get_kTargetSlipRatio() - std::abs(controller_sim.slip_ratio_.fr_));
        torque_tc[2] += Kp * (controller_sim.get_kTargetSlipRatio() - std::abs(controller_sim.slip_ratio_.rl_));
        torque_tc[3] += Kp * (controller_sim.get_kTargetSlipRatio() - std::abs(controller_sim.slip_ratio_.rr_));
    }
}
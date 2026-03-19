#include "controller_sim.hpp"

void ControllerSim::get_power_limitation(float kMaxPower)
{

    double total_pw = estimate_power(*this);
    float P_max[] = {kMaxPower/4, kMaxPower/4, kMaxPower/4, kMaxPower/4};

    if (total_pw > kMaxPower) {
        limit_power(P_max, *this);
    }

}

double estimate_power( ControllerSim& controller_sim){
    double total_pw = controller_sim.torque_cmd_.fl_ * controller_sim.wheel_speed_.fl_ +
                      controller_sim.torque_cmd_.fr_ * controller_sim.wheel_speed_.fr_ +
                      controller_sim.torque_cmd_.rl_ * controller_sim.wheel_speed_.rl_ +
                      controller_sim.torque_cmd_.rr_ * controller_sim.wheel_speed_.rr_;
    return total_pw;
}

void limit_power(float P_max[], ControllerSim& controller_sim)
{
    controller_sim.torque_cmd_.fl_ = P_max[0] / controller_sim.wheel_speed_.fl_;
    controller_sim.torque_cmd_.fr_ = P_max[1] / controller_sim.wheel_speed_.fr_;
    controller_sim.torque_cmd_.rl_ = P_max[2] / controller_sim.wheel_speed_.rl_;
    controller_sim.torque_cmd_.rr_ = P_max[3] / controller_sim.wheel_speed_.rr_;
}

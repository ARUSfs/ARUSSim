#include "controller_sim.hpp"


void ControllerSim::get_torque_vectoring(double input_acc, double target_r)
{
    double total_fx_cmd = input_acc * kMass;

    double target_mz = get_target_mz(r_,target_r,*this);

    control_allocation(target_mz, total_fx_cmd,*this);
}

void control_allocation(double target_mz, double total_fx_cmd, ControllerSim& controller_sim){

    if(controller_sim.get_RWD()){
        if(controller_sim.get_kTorqueVectoring()){

            controller_sim.torque_cmd_.fl_ = 0.;
            controller_sim.torque_cmd_.fr_ = 0.;
            controller_sim.torque_cmd_.rl_ = controller_sim.get_kTireDynRadius() * (0.5 * total_fx_cmd - target_mz/controller_sim.get_kTrackWidth());
            controller_sim.torque_cmd_.rr_ = controller_sim.get_kTireDynRadius() * (0.5 * total_fx_cmd + target_mz/controller_sim.get_kTrackWidth());
        }else{
            controller_sim.torque_cmd_.fl_ = 0. * total_fx_cmd * controller_sim.get_kTireDynRadius();
            controller_sim.torque_cmd_.fr_ = 0. * total_fx_cmd * controller_sim.get_kTireDynRadius();
            controller_sim.torque_cmd_.rl_ = 0.5 * total_fx_cmd * controller_sim.get_kTireDynRadius();
            controller_sim.torque_cmd_.rr_ = 0.5 * total_fx_cmd * controller_sim.get_kTireDynRadius();
        }

        controller_sim.torque_cmd_.fl_ = 0.;
        controller_sim.torque_cmd_.fr_ = 0.;
        controller_sim.torque_cmd_.rl_ = std::min(std::max(controller_sim.torque_cmd_.rl_, controller_sim.get_kTorqueMin()), controller_sim.get_kTorqueMax());
        controller_sim.torque_cmd_.rr_ = std::min(std::max(controller_sim.torque_cmd_.rr_, controller_sim.get_kTorqueMin()), controller_sim.get_kTorqueMax());
    } else {
        if(controller_sim.get_kTorqueVectoring()){
            double fz_front_mean = 0.5*(controller_sim.get_tire_loads().fl_ + controller_sim.get_tire_loads().fr_);
            double fz_rear_mean = 0.5*(controller_sim.get_tire_loads().rl_ + controller_sim.get_tire_loads().rr_);
            double fz_total = 2 * fz_front_mean + 2 * fz_rear_mean;

            controller_sim.torque_cmd_.fl_ = controller_sim.get_kTireDynRadius() / fz_total * (fz_front_mean * total_fx_cmd - controller_sim.get_tire_loads().fl_ * 2*target_mz/controller_sim.get_kTrackWidth());
            controller_sim.torque_cmd_.fr_ = controller_sim.get_kTireDynRadius() / fz_total * (fz_front_mean * total_fx_cmd + controller_sim.get_tire_loads().fr_ * 2*target_mz/controller_sim.get_kTrackWidth());
            controller_sim.torque_cmd_.rl_ = controller_sim.get_kTireDynRadius() / fz_total * (fz_rear_mean * total_fx_cmd -  controller_sim.get_tire_loads().rl_ * 2*target_mz/controller_sim.get_kTrackWidth());
            controller_sim.torque_cmd_.rr_ = controller_sim.get_kTireDynRadius() / fz_total * (fz_rear_mean * total_fx_cmd +  controller_sim.get_tire_loads().rr_ * 2*target_mz/controller_sim.get_kTrackWidth());
        }else{
            controller_sim.torque_cmd_.fl_ = 0.2 * total_fx_cmd * controller_sim.get_kTireDynRadius();
            controller_sim.torque_cmd_.fr_ = 0.2 * total_fx_cmd * controller_sim.get_kTireDynRadius();
            controller_sim.torque_cmd_.rl_ = 0.3 * total_fx_cmd * controller_sim.get_kTireDynRadius();
            controller_sim.torque_cmd_.rr_ = 0.3 * total_fx_cmd * controller_sim.get_kTireDynRadius();
        }

        controller_sim.torque_cmd_.fl_ = std::min(std::max(controller_sim.torque_cmd_.fl_, controller_sim.get_kTorqueMin()), controller_sim.get_kTorqueMax());
        controller_sim.torque_cmd_.fr_ = std::min(std::max(controller_sim.torque_cmd_.fr_, controller_sim.get_kTorqueMin()), controller_sim.get_kTorqueMax());
        controller_sim.torque_cmd_.rl_ = std::min(std::max(controller_sim.torque_cmd_.rl_, controller_sim.get_kTorqueMin()), controller_sim.get_kTorqueMax());
        controller_sim.torque_cmd_.rr_ = std::min(std::max(controller_sim.torque_cmd_.rr_, controller_sim.get_kTorqueMin()), controller_sim.get_kTorqueMax());
    }
}

double get_target_yawrate(ControllerSim& controller_sim){
    return controller_sim.vx_ * std::tan(controller_sim.delta_) / controller_sim.get_kWheelBase();
}

double get_target_mz(double actual, double target,ControllerSim& controller_sim){
    // Ahora mismo es un control P simple
    double target_mz = controller_sim.get_kTVKp() * (target - actual);

    return target_mz;
}
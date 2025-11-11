/**
 * @file controller_sin.cpp
 * @author Carmen Menenendez (carmenmenendezda@gmail.com)
 * @brief contol_sim node for the ARUSSIM package. This node simulates the Control PCB of the vehicle.
 * @version 0.1
 * @date 2024-10-16
 * 
 */
#include "arussim/control_sim/control_sim.hpp"

/**
 * @class ControlSim
 * @brief ControlSim class for ARUSsim
 * This class simulates the Control PCB of the vehicle.
 * 
 */

ControlSim::ControlSim() : Node("control_sim") {


    clock_ = std::make_shared<rclcpp::Clock>(RCL_SYSTEM_TIME);
    // Timers
    timer_defaultTask_ = this->create_wall_timer(std::chrono::milliseconds(5),
        std::bind(&ControlSim::default_task, this));


    // CAN socket
    can_socket_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    std::strcpy(ifr_.ifr_name, "can0");
    ioctl(can_socket_, SIOCGIFINDEX, &ifr_);
    addr_.can_family = AF_CAN;
    addr_.can_ifindex = ifr_.ifr_ifindex;
    bind(can_socket_, (struct sockaddr *)&addr_, sizeof(addr_));


    std::thread thread_(&ControlSim::receive_can, this);
    thread_.detach();

    //Initialize state
    std::vector<float> state = {0.0f, 0.0f, 0.0f};
    pid.init = 0;
    Parameters_Init(&parameters);
	Estimation_Init();
	PowerControl_Init(&parameters);
	TractionControl_Init(&pid, &parameters);
	TorqueVectoring_Init(&pid);

        
}

 void ControlSim::receive_can()
{
    while (rclcpp::ok()) {
    read(can_socket_, &frame, sizeof(struct can_frame));

        if (frame.can_id == 0x222) { 
            acc_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
            yaw_scaled_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);

            dv.acc = acc_scaled_ / 100.0;
            dv.target_r = yaw_scaled_ / 1000.0;
            
        }

        else if (frame.can_id == 0x134){
            int16_t steering_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
            sensors.steering_angle = steering_scaled_* -0.000031688042484 + 0.476959989071;
        }

        else if (frame.can_id == 0x1A0) {
            vx_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
            vy_scaled_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);

            sensors.speed_x = vx_scaled_ * 0.2 / 3.6;
            sensors.speed_y = vy_scaled_ * 0.2 / 3.6;

        }
        else if (frame.can_id == 0x1A3) {
            int16_t acc_x_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
            int16_t acc_y_scaled_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
            int16_t acc_z_scaled_ = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]);
            sensors.acceleration_x = acc_x_scaled_ * 0.02;
            sensors.acceleration_y = acc_y_scaled_ * 0.02;
            sensors.acceleration_z = acc_z_scaled_ * 0.02;

        }

        else if (frame.can_id == 0x1A4) {
            int16_t pitch_scaled_ = static_cast<int16_t>((frame.data[1] << 8) | frame.data[0]);
            int16_t roll_scaled_ = static_cast<int16_t>((frame.data[3] << 8) | frame.data[2]);
            r_scaled_ = static_cast<int16_t>((frame.data[5] << 8) | frame.data[4]);
            
            sensors.angular_x = pitch_scaled_  * 0.02f * 0.0174532;
            sensors.angular_y = roll_scaled_ * 0.02f * 0.0174532;
            sensors.angular_z = r_scaled_ * 0.02f * 0.0174532;

        }
        else if (frame.can_id == 0x102){
            int32_t ws_scaled_0 = static_cast<int32_t>((frame.data[3] << 24) | (frame.data[2] << 16)| (frame.data[1] << 8) | frame.data[0]);
            sensors.motor_speed[0] = ws_scaled_0 * 0.00000018879763543;
        }
        else if (frame.can_id == 0x106){
            int32_t ws_scaled_1 = static_cast<int32_t>((frame.data[3] << 24) | (frame.data[2] << 16)| (frame.data[1] << 8) | frame.data[0]);
            sensors.motor_speed[1] = ws_scaled_1 * 0.00000018879763543;
        }
        else if (frame.can_id == 0x110){
            int32_t ws_scaled_2 = static_cast<int32_t>((frame.data[3] << 24) | (frame.data[2] << 16)| (frame.data[1] << 8) | frame.data[0]);
            sensors.motor_speed[2] = ws_scaled_2 * 0.00000018879763543;
        }
        else if (frame.can_id == 0x114){
            int32_t ws_scaled_3 = static_cast<int32_t>((frame.data[3] << 24) | (frame.data[2] << 16)| (frame.data[1] << 8) | frame.data[0]);
            sensors.motor_speed[3] = ws_scaled_3 * 0.00000018879763543;
        }
        
    }
    
}

void ControlSim::Parameters_Init(Parameters *p) {
    p->gear_ratio = DEFAULT_GEAR_RATIO;
    p->maximum_power = DEFAULT_MAX_POWER;
    p->minimum_power = DEFAULT_MIN_POWER;
    p->V_max = DEFAULT_MAX_VOLTAGE;
    p->V_min = DEFAULT_MIN_VOLTAGE;
    p->wheelbase = DEFAULT_WB;
    p->rdyn = DEFAULT_RDYN;
    p->trackwidthF = DEFAULT_TW;
    p->trackwidthR = DEFAULT_TW;
    p->wheel_inertia = DEFAULT_WHEEL_INERTIA;
    for (int i = 0; i < 4; i++) {
        p->torque_limit_positive[i] = DEFAULT_TL_POS;
        p->torque_limit_negative[i] = DEFAULT_TL_NEG;
    }
    p->mass = DEFAULT_MASS;
    p->g = GRAVITY;
    p->r_cdg =DEFAULT_R_CDG;
    p->lf = p->wheelbase * p->r_cdg;
    p->lr = p->wheelbase * (1 - p->r_cdg);
    p->nsm_f = DEFAULT_NSM;
    p->nsm_r = DEFAULT_NSM;
    p->sm= p->mass - p->nsm_f - p->nsm_r;
    p->sm_f = p->sm * p->r_cdg;
    p->sm_r = p->sm * (1- p->r_cdg);
    p->h_cdg = DEFAULT_H_CDG;
    p->h_cdg_nsm_f = DEFAULT_H_CDG_NSM;
    p->h_cdg_nsm_r= DEFAULT_H_CDG_NSM;
    p->h_cdg_sm = DEFAULT_H_CDG_SM;
    p->h_RC_f = DEFAULT_H_RC_F;
    p->h_RC_r = DEFAULT_H_RC_R;
    p->h_RA = p->h_RC_f + (p->h_RC_r - p->h_RC_f) * p->lf / p->wheelbase;

    float MR_ARB_f = MR_ARB_f_DIRK * 180 /pi * 1 / (r_ARB_f * 1000 * cos(psi_ARB_f));
    p->RS_f = 0.5f * p->trackwidthF * p->trackwidthF * tan(pi/180) * (K_s_f / (MR_s * MR_s) + K_ARB_f / (MR_ARB_f * MR_ARB_f));
    float MR_ARB_r = MR_ARB_f_DIRK * 180 /pi * 1 / (r_ARB_r * 1000 * cos(psi_ARB_r));
    p->RS_r = 0.5f * p->trackwidthF * p->trackwidthF * tan(pi/180) * (K_s_r / (MR_s * MR_s) + K_ARB_r / (MR_ARB_r * MR_ARB_r));
    p->RS = p->RS_f + p->RS_r;

	p->rho = DEFAULT_RHO;
	p->CDA = DEFAULT_CDA;
	p->CLA = DEFAULT_CLA;
	p->r_cdp = DEFAULT_R_CDP;
	p->h_cdp = DEFAULT_H_CDP;

	p->fz_params[0] = 0.5f * DEFAULT_RHO * DEFAULT_CLA;
	p->fz_params[1] = 0.5f * DEFAULT_RHO * DEFAULT_CDA;
	p->fz_params[2] = DEFAULT_NSM *DEFAULT_H_CDG_NSM/ DEFAULT_TW;
	p->fz_params[3] = p->sm_f * DEFAULT_H_RC_F / DEFAULT_TW;
	p->fz_params[4] = p->sm_r * DEFAULT_H_RC_R / DEFAULT_TW;
	p->fz_params[5] = p->sm * (DEFAULT_H_CDG_SM - p->h_RA) * p->RS_f / p->RS / DEFAULT_TW;
	p->fz_params[6] = p->sm * (DEFAULT_H_CDG_SM - p->h_RA) * p->RS_r / p->RS / DEFAULT_TW;
	p->fz_params[7] = (DEFAULT_NSM * DEFAULT_H_CDG_NSM * 2)/ DEFAULT_WB;
	p->fz_params[8] = p->sm * DEFAULT_H_CDG_SM / DEFAULT_WB;
	p->fz_params[9] = 0.5 * DEFAULT_MASS * GRAVITY * p->lf / DEFAULT_WB;
	p->fz_params[10] = 0.5 * DEFAULT_R_CDP;
	p->fz_params[11] = 0.5 * DEFAULT_H_CDP / DEFAULT_WB;
	p->fz_params[12] = 0.5 * DEFAULT_MASS* GRAVITY * p->lr / DEFAULT_WB;

}

void ControlSim::send_torque(float torque_out[4])
{
   std::array<uint32_t, 4> ids = {0x200, 0x203, 0x206, 0x209};

  for (size_t i = 0; i < ids.size(); ++i) {
        torque_i_ = static_cast<int16_t>(torque_out[i] * 1000 / 9.8); 

        frame.can_id = ids[i];
        frame.can_dlc = 8;

        frame.data[0] = 0x00;  
        frame.data[1] = 0x07;  

        frame.data[2] = torque_i_ & 0xFF;
        frame.data[3] = (torque_i_ >> 8) & 0xFF;

        int16_t torque_max = 1000;
        int16_t torque_min = -1000;
        frame.data[4] = torque_max & 0xFF;
        frame.data[5] = (torque_max >> 8) & 0xFF;
        frame.data[6] = torque_min & 0xFF;
        frame.data[7] = (torque_min >> 8) & 0xFF;

        write(can_socket_, &frame, sizeof(struct can_frame));
    }
       
}


void ControlSim::send_state(){
    
    frame.can_id = 0x122;
    frame.can_dlc = 6;

    vx_i_ = (int)(state[0]* 100.0f);
    frame.data[0] = vx_i_ & 0xFF;
    frame.data[1] = (vx_i_ >> 8) & 0xFF;

    vy_i_ = (int)(state[1] * 100.0f);
    frame.data[2] = vy_i_ & 0xFF;
    frame.data[3] = (vy_i_ >> 8) & 0xFF;

    r_i_ = (int)(state[2] * 1000.0f);
    frame.data[4] = r_i_ & 0xFF;
    frame.data[5] = (r_i_ >> 8) & 0xFF;
    write(can_socket_, &frame, sizeof(struct can_frame));

    
}

void ControlSim::default_task()
{
    rclcpp::Time current_time = clock_->now();
    if (pid.init == 0){
        pid.last_timestamp = current_time.seconds();
    }
    else{
        pid.TS = current_time.seconds() - pid.last_timestamp;
        pid.last_timestamp = current_time.seconds();
	}

    Estimation_Update(&sensors, &parameters, state);
    send_state();
    fx_request = pc_request(&dv, &parameters);

    Calculate_Tire_Loads(&sensors, &parameters, state, &tire);
    TorqueVectoring_Update(&sensors, &parameters, &pid, &tire, &dv, fx_request, state, TV_out);
    TractionControl_Update(&sensors, &parameters, &pid, &tire, TV_out, TC_out, SR, &dv);
    PowerControl_Update(&sensors, &parameters, TC_out);

    if (pid.init == 0){
        pid.init = 1;
    }

    send_torque(TC_out);
}

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ControlSim>());
    rclcpp::shutdown();
    return 0;
}
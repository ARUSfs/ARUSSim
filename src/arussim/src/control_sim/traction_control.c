/*
 * traction_control.c
 *
 *  Created on: Nov 16, 2025
 *      Author: Jose
 */

#include "arussim/control_sim/traction_control.h"
#include <math.h>


typedef struct {
    float int_SRe[4];
    float SR_e1[4];
} TC_State;

static TC_State tc_state;

void TractionControl_Init(PID *pid, Parameters *parameters) {
    // Configure PID gains
    pid->TC_K = TC_K_;
    pid->TC_Ti = TC_TI;
    pid->TC_Td = TC_TD;
    pid->init = 1;  

    parameters->v0 = DEFAULT_TC_V0;
    parameters->v_gain = DEFAULT_TC_V_GAIN;

    for (int i = 0; i < 4; i++) {
        tc_state.int_SRe[i] = 0.0f;
        tc_state.SR_e1[i] = 0.0f;
    }
}

void TractionControl_Update(SensorData *sensors, Parameters *parameters, PID *pid, TIRE *tire, float *Tin, float *TC, float *SR, DV *dv) {


//    //DEMAND VALIDATION
//    int has_demand = 0;
//    for (int i = 0; i < 4; i++) {
//        if (fabsf(Tin[i]) > 0.1f) {  // Demand threshold: 0.1 Nm
//            has_demand = 1;
//            break;
//        }
//    }
//
//    if (!has_demand) {
//        // No demand: reset all outputs and state
//        for (int i = 0; i < 4; i++) {
//            TC[i] = 0.0f;
//            SR[i] = 0.0f;
//            tc_state.int_SRe[i] = 0.0f;
//        }
//        return;
//    }


    //SYSTEM ACTIVATION CHECK
    if (TC_ACTIVE != 1 || pid->init != 1 || dv->inspection) {
        for (int i = 0; i < 4; i++) {
            TC[i] = Tin[i];

            // Apply hardware torque limits
            if (TC[i] > parameters->torque_limit_positive[i]) 
                TC[i] = parameters->torque_limit_positive[i];
            if (TC[i] < parameters->torque_limit_negative[i]) 
                TC[i] = parameters->torque_limit_negative[i];
        }
        return;
    }

    //Calculate all necessary variables
    float vx = sensors->speed_x;
    float vy = sensors->speed_y;
    float yaw_rate = sensors->angular_z;

    float vx_wheel[4];
    vx_wheel[0] = vx + yaw_rate * 0.5f * (-parameters->trackwidthF);  // FL
    vx_wheel[1] = vx + yaw_rate * 0.5f * parameters->trackwidthF;      // FR
    vx_wheel[2] = vx + yaw_rate * 0.5f * (-parameters->trackwidthR);   // RL
    vx_wheel[3] = vx + yaw_rate * 0.5f * parameters->trackwidthR;      // RR

    float vy_wheel[4];
    vy_wheel[0] = vy + yaw_rate * parameters->lf;     // FL
    vy_wheel[1] = vy + yaw_rate * parameters->lf;     // FR
    vy_wheel[2] = vy + yaw_rate * (-parameters->lr);  // RL
    vy_wheel[3] = vy + yaw_rate * (-parameters->lr);  // RR


    float vx_wheel_tire[4];
    float cos_steer = cosf(sensors->steering_angle);
    float sin_steer = sinf(sensors->steering_angle);

    vx_wheel_tire[0] = vx_wheel[0] * cos_steer + vy_wheel[0] * sin_steer;  // FL (steered)
    vx_wheel_tire[1] = vx_wheel[1] * cos_steer + vy_wheel[1] * sin_steer;  // FR (steered)
    vx_wheel_tire[2] = vx_wheel[2];                                         // RL (no steering)
    vx_wheel_tire[3] = vx_wheel[3];                                         // RR (no steering)


    //SLIP RATIO CALCULATION
    float min_vx_wheel_tire = vx_wheel_tire[0];
    for (int i = 1; i < 4; i++) {
        if (vx_wheel_tire[i] < min_vx_wheel_tire) 
            min_vx_wheel_tire = vx_wheel_tire[i];
    }

    float wr[4];
    wr[0] = sensors->motor_speed[0];
    wr[1] = sensors->motor_speed[1];
    wr[2] = sensors->motor_speed[2];
    wr[3] = sensors->motor_speed[3];
    if (min_vx_wheel_tire < 1.0f) {
        for (int i = 0; i < 4; i++) {
            SR[i] = parameters->rdyn * wr[i] - vx_wheel_tire[i];
        }
    } else {
        for (int i = 0; i < 4; i++) {
            SR[i] = parameters->rdyn * wr[i] / (vx_wheel_tire[i] + eps) - 1.0f;
        }
    }

    //FEEDFORWARD TORQUE CALCULATION
    float SR_t[4] = {0.1f, 0.1f, 0.1f, 0.1f};
    float slip_angle[4] = {0.0f, 0.0f, 0.0f, 0.0f};
    Calculate_Tire_Forces(tire, slip_angle, SR_t);

    // Feedforward torque = tire_force * tire_radius + wheel_inertia * acceleration
    float T_obj[4];
    float inertia_term = sensors->acceleration_x / parameters->rdyn * 
                         parameters->wheel_inertia / parameters->gear_ratio;

    for (int i = 0; i < 4; i++) {
        T_obj[i] = tire->force_fx[i] * parameters->rdyn / parameters->gear_ratio + inertia_term;
    }

    //PID FEEDBACK CONTROL
    float SR_e[4];
    float int_SRep[4];
    float TC_calc[4];

    for (int i = 0; i < 4; i++) {
        if (fabsf(Tin[i]) < 0.1f) {
            TC[i] = 0.0f;
            tc_state.int_SRe[i] = 0.0f;
            continue;
        }

        SR_e[i] = fabs(SR[i]) - SR_t[i];


        int_SRep[i] = tc_state.int_SRe[i] + SR_e[i];
        if (int_SRep[i] > 50.0f) int_SRep[i] = 50.0f;
        if (int_SRep[i] < -50.0f) int_SRep[i] = -50.0f;

        TC_calc[i] = T_obj[i] 
                   + pid->TC_K * SR_e[i]
                   + (pid->TS / pid->TC_Ti) * int_SRep[i]
                   - (pid->TC_Td / pid->TS) * (SR_e[i] - tc_state.SR_e1[i]);



        TC[i] = fminf(TC_calc[i], fmaxf(Tin[i], -TC_calc[i]));
        if(Tin[i] >= 0.0f && TC_calc[i] < 0.0f){
            TC[i] = Tin[i];
        }

        
    }

    //Memoria
    for (int i = 0; i < 4; i++) {
        if (TC_calc[i] > TC[i]) {
            int_SRep[i] = 0.0f;
        }
        tc_state.int_SRe[i] = int_SRep[i];
    }

//    if (vx < 4.0f) {
//        float T_limit = parameters->v0 + parameters->v_gain * vx;
//
//        if (TC[0] > T_limit) TC[0] = T_limit;
//        if (TC[1] > T_limit) TC[1] = T_limit;
//    }
//

    //Saturation
    for (int i = 0; i < 4; i++) {
        if (TC[i] > parameters->torque_limit_positive[i]) 
            TC[i] = parameters->torque_limit_positive[i];
        if (TC[i] < parameters->torque_limit_negative[i]) 
            TC[i] = parameters->torque_limit_negative[i];
    }

    for (int i = 0; i < 4; i++) {
        tc_state.SR_e1[i] = SR_e[i];
    }
}
/* End of traction_control.c */
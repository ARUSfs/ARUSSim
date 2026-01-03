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
    float SR_prev[4];
} TC_State;

static TC_State tc_state;

void TractionControl_Init(PID *pid, Parameters *parameters) {
    // Configure PID gains
    pid->TC_K = TC_K_;
    pid->TC_Ti = TC_TI;
    pid->TC_Td = TC_TD;
    pid->init = 0;  

    parameters->v0 = DEFAULT_TC_V0;
    parameters->v_gain = DEFAULT_TC_V_GAIN;

    for (int i = 0; i < 4; i++) {
        tc_state.int_SRe[i] = 0.0f;
        tc_state.SR_e1[i] = 0.0f;
        tc_state.SR_prev[i] = 0.0f;
    }
}

void TractionControl_Update(SensorData *sensors, Parameters *parameters, PID *pid, TIRE *tire, float *Tin, float *TC, float *SR, DV *dv, float *T_obj, float *state) {

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
    float vx = state[0];
    float vy = state[1];
    float yaw_rate = state[2];

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
    float vy_wheel_tire[4];
    float cos_steer = cosf(sensors->steering_angle);
    float sin_steer = sinf(sensors->steering_angle);

    vx_wheel_tire[0] = vx_wheel[0] * cos_steer + vy_wheel[0] * sin_steer;  // FL
    vx_wheel_tire[1] = vx_wheel[1] * cos_steer + vy_wheel[1] * sin_steer;  // FR
    vx_wheel_tire[2] = vx_wheel[2];                                         // RL
    vx_wheel_tire[3] = vx_wheel[3];                                         // RR

    vy_wheel_tire[0] = -vx_wheel[0] * sin_steer + vy_wheel[0] * cos_steer; // FL
    vy_wheel_tire[1] = -vx_wheel[1] * sin_steer + vy_wheel[1] * cos_steer; // FR
    vy_wheel_tire[2] = vy_wheel[2];                                        // RL
    vy_wheel_tire[3] = vy_wheel[3];                                        // RR

    float wr[4];
    wr[0] = sensors->motor_speed[0]/parameters->gear_ratio;
    wr[1] = sensors->motor_speed[1]/parameters->gear_ratio;
    wr[2] = sensors->motor_speed[2]/parameters->gear_ratio;
    wr[3] = sensors->motor_speed[3]/parameters->gear_ratio;
    
    if (vx < 1.0f) {
        for (int i = 0; i < 4; i++) {
            SR[i] = parameters->rdyn * wr[i] - vx_wheel_tire[i];
            SR[i] = 0.1*tc_state.SR_prev[i] + 0.9*SR[i];
            tc_state.SR_prev[i] = SR[i];
        }
    } else {
        for (int i = 0; i < 4; i++) {
            SR[i] = parameters->rdyn * wr[i] / (vx_wheel_tire[i] + eps) - 1.0f;
            SR[i] = 0.1*tc_state.SR_prev[i] + 0.9*SR[i];
            tc_state.SR_prev[i] = SR[i];
        }
    }



    //FEEDFORWARD TORQUE CALCULATION
    float SR_t[4] = {0.1f, 0.1f, 0.1f, 0.1f};
    float slip_angle[4];

    slip_angle[0] = atan2f(vy_wheel_tire[0], vx_wheel_tire[0]); // FL
    slip_angle[1] = atan2f(vy_wheel_tire[1], vx_wheel_tire[1]); // FR
    slip_angle[2] = atan2f(vy_wheel_tire[2], vx_wheel_tire[2]); // RL
    slip_angle[3] = atan2f(vy_wheel_tire[3], vx_wheel_tire[3]); // RR

    Calculate_Tire_Forces(tire, slip_angle, SR_t);


    float TC_calc[4];
    float inertia_term = (1 + SR_t[0]) * sensors->acceleration_x / parameters->rdyn *
                         parameters->wheel_inertia / parameters->gear_ratio;

    for (int i = 0; i < 4; i++) {
        T_obj[i] = tire->force_fx[i] * parameters->rdyn / parameters->gear_ratio + inertia_term;
    }

    //PID FEEDBACK CONTROL
    float SR_e[4];
    float int_SRep[4];
    float alpha = 0.2;

    for (int i = 0; i < 4; i++) {
        if (fabsf(Tin[i]) < 0.1f) {
            TC[i] = 0.0f;
            tc_state.int_SRe[i] = 0.0f;
            tc_state.SR_e1[i]   = 0.0f;
            TC_calc[i]          = 0.0f; //Comentar cuando se use suavizado
            continue;
        }

        SR_e[i] = SR_t[i] - fabsf(SR[i]);

        int_SRep[i] = tc_state.int_SRe[i] + SR_e[i];
        
        float pid_calc = pid->TC_K*SR_e[i]
                       + pid->TC_Ti*int_SRep[i];

                
        // TC_calc[i] = alpha * TC_calc[i] + (1-alpha) * (T_obj[i] + pid_calc); //Suavizado
        TC_calc[i] = T_obj[i] + pid_calc;


        TC[i] = fminf(TC_calc[i], fmaxf(Tin[i], -TC_calc[i]));
        if(Tin[i] >= 0.0f && TC_calc[i] < 0.0f){
            TC[i] = Tin[i];
        }

        //Anti-windup
        if (TC_calc[i] > TC[i]) {
            int_SRep[i] = 0.0f;
        }

        // Memoria
        tc_state.int_SRe[i] = int_SRep[i];
        tc_state.SR_e1[i]   = SR_e[i];
    }

   if (vx < 3.0f) {
       float T_limit = 10.0 + 2.0 * vx;

       if (TC[0] > T_limit) TC[0] = T_limit;
       if (TC[1] > T_limit) TC[1] = T_limit;
   }

    //Saturation
    for (int i = 0; i < 4; i++) {
        if (TC[i] > parameters->torque_limit_positive[i]) 
            TC[i] = parameters->torque_limit_positive[i];
        if (TC[i] < parameters->torque_limit_negative[i]) 
            TC[i] = parameters->torque_limit_negative[i];
    }
}
/* End of traction_control.c */
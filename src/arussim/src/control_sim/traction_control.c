/*
 * traction_control.c
 *
 *  Created on: Jan 29, 2025
 *      Author: carme
 */

#include "arussim/control_sim/traction_control.h"



float vx_wheel_tire[4];
//float SR[4];
float TC_calc[4];
float T_obj[4];
float int_SRe[4];
float int_SRep[4];
float SR_e[4];
float SR_e1[4];
float SR_e1p[4];
float slip_angle[4];


void TractionControl_Init(PID *pid, Parameters *parameters) {
	pid->TC_K = TC_K_;
	pid->TC_Ti = TC_TI;
	pid->TC_Td = TC_TD;
	pid->init = 0;
	for (int i = 0; i < 4; i ++){
		int_SRe[i] = 0.f;
		int_SRep [i] = 0.f;
		SR_e [i] = 0.f;
		SR_e1 [i] = 0.f;
		SR_e1p [i] = 0.f;
		slip_angle[i] = 0.f;

	}
	parameters->v0 = DEFAULT_TC_V0;
	parameters->v_gain = DEFAULT_TC_V_GAIN;
}
void TractionControl_Update(SensorData *sensors, Parameters *parameters, PID *pid, TIRE *tire, float *Tin, float *TC, float *SR, DV *dv) {
	if (TC_ACTIVE == 1 && pid->init == 1 && !dv->inspection) {
		float state[3] = {sensors->speed_x, sensors->speed_y, sensors->angular_z};
		int bool_min_speed = 0;

		//Calculate all necessary variables
		float vx_wheel[4] = {state[0] + state[2] * 0.5f * (- parameters->trackwidthF),
			state[0] + state[2] * 0.5f * parameters->trackwidthF,
			state[0] + state[2] * 0.5f * (- parameters->trackwidthR),
			state[0] + state[2] * 0.5f * parameters->trackwidthR,
		};

		float vy_wheel[4] = {
				state[1] + state[2] * parameters->lf,
				state[1] + state[2] * parameters->lf,
				state[1] + state[2] * (- parameters->lr),
				state[1] + state[2] * (- parameters->lr),
		};

		vx_wheel_tire[0] = vx_wheel[0] * cosf(sensors->steering_angle) + vy_wheel[0] * sinf(sensors->steering_angle);
		vx_wheel_tire[1] = vx_wheel[1] * cosf(sensors->steering_angle) + vy_wheel[1] * sinf(sensors->steering_angle);
		vx_wheel_tire[2] = vx_wheel[2];
		vx_wheel_tire[3] = vx_wheel[3];

		for (int i = 0; i < 4; i++) {
			   if (vx_wheel_tire[i] < 1.f){
				   bool_min_speed = 1;
				   break;
			   }
		}

		for (int i = 0; i < 4; ++i) {
			if (bool_min_speed == 1){
				SR[i] = parameters->rdyn * sensors->motor_speed[i] / parameters->gear_ratio - vx_wheel_tire[i];
			}
			else{
				SR[i] = parameters->rdyn * sensors->motor_speed[i] / parameters->gear_ratio / (vx_wheel_tire[i] + eps) -1;
			}

		}

		//Define SR target
		float SR_t[4] = { 0.1f, 0.1f, 0.1f, 0.1f };

		//Calculate feedfoward torque
		Calculate_Tire_Forces(tire, slip_angle, SR_t);

		for (int i = 0; i < 4; i++) {
			T_obj[i] = tire->force_fx[i] * parameters->rdyn / parameters->gear_ratio + sensors->acceleration_x / parameters->rdyn * parameters->wheel_inertia / parameters->gear_ratio;

			//Control the value
			SR_e[i] = SR_t[i] - fabsf(SR[i]);
			int_SRep[i] = int_SRe[i] + SR_e[i];
			SR_e1p [i] = SR_e[i];
			TC_calc[i] = T_obj[i] + pid->TC_K * SR_e[i] - (pid->TC_Td / pid->TS) * (SR_e[i] - SR_e1[i]);
			TC[i] = fminf(TC_calc[i], fmaxf(Tin[i], -TC_calc[i]));
		}


		//Traction control activation logic
		for (int i = 0; i < 4; i++) {
			if (TC_calc[i] > TC[i]) {
				int_SRep[i] = 0;
			}
		}

		//Low Speed limitation
		if (state[0] < 4.f) {
			float T_limit = parameters->v0 + parameters->v_gain * state[0];
		    if (Tin[0] > T_limit) TC[0] = T_limit;
		    if (Tin[1] > T_limit) TC[1] = T_limit;
		}

		//Saturation
		for (int i = 0; i < 4; i++) {
			TC[i] = (TC[i] > parameters->torque_limit_positive[i]) ?  parameters->torque_limit_positive[i] : TC[i];
			TC[i] = (TC[i] < parameters->torque_limit_negative[i]) ?  parameters->torque_limit_negative[i] : TC[i];
		}

		//memory
		for (int i = 0; i < 4; i++) {
			int_SRe[i] = int_SRep[i];
			SR_e1[i] = SR_e1p[i];
		}

//		// Ignore TC at low speed
//		if(bool_min_speed==1){
//			for(int i=0; i<4; i++){
//				TC[i] = Tin[i];
//			}
//		}

	} else {

		for (int i = 0; i < 4; i ++){
			TC[i] = Tin[i];

			//Saturation
			TC[i] = (TC[i] > parameters->torque_limit_positive[i]) ?  parameters->torque_limit_positive[i] : TC[i];
			TC[i] = (TC[i] < parameters->torque_limit_negative[i]) ?  parameters->torque_limit_negative[i] : TC[i];
		}

	}
}
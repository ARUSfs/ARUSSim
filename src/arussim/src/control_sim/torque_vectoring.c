/*
 * torque_vectoring.c
 *
 *  Created on: Apr 19, 2025
 *      Author: carme
 */
//TODO: Add gear ratio here and in vehicle dynamics
#include "arussim/control_sim/torque_vectoring.h"

float tire_load[4];
float target_r;


void TorqueVectoring_Init(PID *pid) {

	pid->TV_Kd= TV_KD;
	pid->TV_Ki= TV_KI;
	pid->TV_Kp= TV_KP;
	pid->TV_N= DEFAULT_TV_N;
	pid->max_Mz = DEFAULT_MAX_MZ;
}

void TorqueVectoring_Update(SensorData *sensors, Parameters *parameters, PID *pid, TIRE *tire, DV *dv, float fx_request, float *state, float *torque_out) {
	if (TV_ACTIVE == 1){
		float Mz_request = Target_Generation(sensors, parameters, pid, dv, state);

//		float fz_front_mean = 0.5f * (tire->tire_load[0] + tire->tire_load[1]);
//		float fz_rear_mean = 0.5f * (tire->tire_load[2] + tire->tire_load[3]);
//
//		float fx_dist[4] = { fx_request * fz_front_mean, fx_request * fz_front_mean, fx_request * fz_rear_mean, fx_request * fz_rear_mean };
//
//		float mz_dist[4] = { 2.0f * Mz_request * tire->tire_load[0] / (-parameters->trackwidthF),
//				2.0f * Mz_request * tire->tire_load[1] / (parameters->trackwidthF),
//				2.0f * Mz_request * tire->tire_load[2] / (-parameters->trackwidthR),
//				2.0f * Mz_request * tire->tire_load[3] / (parameters->trackwidthR) };
//
//		float sum_fz = tire->tire_load[0] + tire->tire_load[1] + tire->tire_load[2] + tire->tire_load[3];
//
//		for (int i = 0; i < 4; ++i) {
//			torque_out[i] = parameters->rdyn * (fx_dist[i] + mz_dist[i]) / sum_fz / parameters->gear_ratio;
//
//			if (torque_out[i] > parameters->torque_limit_positive[i]) {
//				torque_out[i] = parameters->torque_limit_positive[i];
//			} else if (torque_out[i] < parameters->torque_limit_negative[i]) {
//				torque_out[i] = parameters->torque_limit_negative[i];
//			}
//		}

		torque_out[0] = 0.;
		torque_out[1] = 0.;
		torque_out[2] = parameters->rdyn/parameters->gear_ratio*(0.5*fx_request - Mz_request/parameters->trackwidthR);
		torque_out[3] = parameters->rdyn/parameters->gear_ratio*(0.5*fx_request + Mz_request/parameters->trackwidthR);

		for(int i=0; i<4; i++){
			if (torque_out[i] > parameters->torque_limit_positive[i]) {
				torque_out[i] = parameters->torque_limit_positive[i];
			} else if (torque_out[i] < parameters->torque_limit_negative[i]) {
				torque_out[i] = parameters->torque_limit_negative[i];
			}
		}

	} else {
		torque_out[0]=0.;
		torque_out[1]=0.;
		for (int i = 2; i < 4; ++i) {
			torque_out[i] = parameters->rdyn * fx_request / parameters->gear_ratio / 2;

			if (torque_out[i] > parameters->torque_limit_positive[i]) {
				torque_out[i] = parameters->torque_limit_positive[i];
			} else if (torque_out[i] < parameters->torque_limit_negative[i]) {
				torque_out[i] = parameters->torque_limit_negative[i];
			}
		}
	}

	if (fx_request <= 0){
		for(int i = 0; i<4; ++i){
			if(torque_out[i] > 0.0){
				torque_out[i] = 0.0;
			}
		}
	}

}

float Target_Generation(SensorData *sensors, Parameters *parameters, PID *pid, DV *dv, float *state) {

	//Calculate Error
	if (dv->autonomous == 1){
		target_r = dv->target_r;
	}
	else{
		target_r = tan(sensors->steering_angle) * state[0] / parameters->wheelbase;
	}
	float error = target_r - state[2];

	//PID Init
	if (pid->init == 0){
		pid->err_prev = error;
        pid->integral = 0.f;
        pid->deriv_filt = 0.f;
        return 0.f;
	}

	//PID
	pid-> integral += error * pid->TS;
	float deriv = (error - pid->err_prev) / pid->TS;
	pid->deriv_filt = (pid->TV_N * deriv + pid->deriv_filt) / (1 + pid->TV_N * pid->TS);
	float Mz_request = pid->TV_Kp * error + pid->TV_Ki * pid->integral + pid->TV_Kd * pid->deriv_filt;
	pid->err_prev = error;

	//Saturation
	Mz_request = (Mz_request > pid->max_Mz) ? pid->max_Mz : Mz_request;
	Mz_request = (Mz_request < -pid->max_Mz) ? -pid->max_Mz : Mz_request;

	if (state[0] < 3.0) {Mz_request = 0.0;}
	return Mz_request;

}
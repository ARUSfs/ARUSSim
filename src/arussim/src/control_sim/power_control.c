/*
 * power_control.c
 *
 *  Created on: Jan 29, 2025
 *      Author: carme
 */

#include "arussim/control_sim/power_control.h"



void PowerControl_Init(Parameters *parameters)
{

}
void PowerControl_Update(SensorData *sensors, Parameters *parameters, float * torque_out) {

	float PW_total = torque_out[0] * sensors->motor_speed[0] + torque_out[1]*sensors->motor_speed[1] +
			torque_out[2]*sensors->motor_speed[2] + torque_out[3]* sensors->motor_speed[3];

	// float r_battery = 1.2f;
	// float PW_max_bms = sensors->V_soc*(sensors->V_soc - parameters->V_min) / r_battery;
	// float PW_min_bms = sensors->V_soc*(sensors->V_soc - parameters->V_max) / r_battery;

	// float PW_max = (PW_max_bms < parameters->maximum_power) ? PW_max_bms : parameters->maximum_power;
	// float PW_min = (PW_min_bms > parameters->minimum_power) ? PW_min_bms : parameters->minimum_power;
	float PW_max = parameters->maximum_power;
	float PW_min = parameters->minimum_power;
	
	if(PW_min > 0) PW_min = 0;

	if (PW_total > PW_max) {

		float PW_extra = (PW_total - PW_max)/PW_total;
		for (int i = 2; i < 4; i++) {
//			if (sensors->motor_speed[i] <= 0) {
//				torque_out[i] = 0;
//			} else {
				torque_out[i] = torque_out[i] * (1 - PW_extra);
//			}
		}

	} else if (PW_total < PW_min){
		if (sensors->V_soc > parameters->V_max){
			torque_out[2] = 0;
			torque_out[3] = 0;
		} else {

		float PW_extra = (PW_total - PW_min)/PW_total;
		for (int i = 2; i < 4; i++) {
//			if (sensors->motor_speed[i] <= 0) {
//				torque_out[i] = 0;
//			} else {
				torque_out[i] = torque_out[i] * (1 - PW_extra);
//			}
		}
		}

	}

}
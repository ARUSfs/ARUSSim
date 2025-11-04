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
			torque_out[2]*sensors->motor_speed[2]+ torque_out[3]* sensors->motor_speed[3];

	if (PW_total > parameters->maximum_power) {

		for (int i = 0; i < 4; i++) {
			if (sensors->motor_speed[i] <= 0) {
				torque_out[i] = 0;
			} else {
				torque_out[i] = parameters->maximum_power / (4 * sensors->motor_speed[i]);
			}
		}
	}
}
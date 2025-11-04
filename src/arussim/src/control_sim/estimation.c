/*
 * estimation.c
 *
 *  Created on: Jan 29, 2025
 *      Author: carme
 */


#include "arussim/control_sim/estimation.h"


void Estimation_Init()
{

}
void Estimation_Update(SensorData *sensors, Parameters *params, float estimation_out[3]){

	// Use wheel speeds' mean and kinematic bicycle model
	estimation_out[0] = sensors->speed_x;//0.25*(params->rdyn/params->gear_ratio)*(sensors->motor_speed[0]+sensors->motor_speed[1]+sensors->motor_speed[2]+sensors->motor_speed[3]);
	estimation_out[1] = sensors->speed_y;//params->lr/params->wheelbase*tanf(sensors->steering_angle)*estimation_out[0];
	estimation_out[2] = sensors->angular_z;
}
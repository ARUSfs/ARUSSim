/*
 * power_control.h
 *
 *  Created on: Jan 29, 2025
 *      Author: carme
 */

#ifndef SRC_POWER_CONTROL_H_
#define SRC_POWER_CONTROL_H_

#ifdef __cplusplus
extern "C" { 
#endif


#include "arussim/control_sim/SensorData.h"
#include "arussim/control_sim/Parameters.h"
#include <stdint.h>


void PowerControl_Init(Parameters *parameters);
void PowerControl_Update(SensorData *sensors, Parameters *parameters, float * torque_out);

#ifdef __cplusplus
}
#endif

#endif /* SRC_POWER_CONTROL_H_ */
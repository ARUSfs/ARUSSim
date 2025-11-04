/*
 * torque_vectoring.h
 *
 *  Created on: Apr 19, 2025
 *      Author: carme
 */

#ifndef INC_TORQUE_VECTORING_H_
#define INC_TORQUE_VECTORING_H_

#ifdef __cplusplus
extern "C" { 
#endif


#include "arussim/control_sim/SensorData.h"
#include "arussim/control_sim/Parameters.h"
#include "arussim/control_sim/aux_functions.h"
#include <stdint.h>

void TorqueVectoring_Init(PID *pid);
void TorqueVectoring_Update(SensorData *sensors, Parameters *parameters, PID *pid, TIRE *tire, DV *dv, float fx_request, float *state, float *torque_out) ;
float Target_Generation (SensorData *sensors, Parameters *parameters, PID *pid, DV *dv, float *state);

#ifdef __cplusplus
}
#endif
#endif /* INC_TORQUE_VECTORING_H_ */
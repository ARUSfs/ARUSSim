/*
 * traction_control.h
 *
 *  Created on: Jan 29, 2025
 *      Author: carme
 */

#ifndef SRC_TRACTION_CONTROL_H_
#define SRC_TRACTION_CONTROL_H_

#ifdef __cplusplus
extern "C" { 
#endif

#include "arussim/control_sim/SensorData.h"
#include "arussim/control_sim/Parameters.h"
#include "arussim/control_sim/aux_functions.h"
#include <stdint.h>



void TractionControl_Init(PID *pid, Parameters *parameters);
void TractionControl_Update(SensorData *sensors,  Parameters *parameters, PID *pid, TIRE *tire, float *Tin, float *TC,  float *SR, DV *dv);

#ifdef __cplusplus
}
#endif
#endif /* INC_TORQUE_VECTORING_H_ */
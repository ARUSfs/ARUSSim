/*
 * estimation.h
 *
 *  Created on: Jan 29, 2025
 *      Author: carme
 */


#ifndef ESTIMATION_H_
#define ESTIMATION_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "arussim/control_sim/Parameters.h"
#include <stdint.h>
#include "arussim/control_sim/SensorData.h"

void Estimation_Init(void);
void Estimation_Update(SensorData *sensors, Parameters *params, float estimation_out[3]);

#ifdef __cplusplus
}
#endif
#endif /* SRC_ESTIMATION_H_ */
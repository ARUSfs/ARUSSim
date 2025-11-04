/*
 * aux_functions.h
 *
 *  Created on: May 14, 2025
 *      Author: carme
 */

#ifndef INC_AUX_FUNCTIONS_H_
#define INC_AUX_FUNCTIONS_H_

#ifdef __cplusplus
extern "C" {
#endif

#include "arussim/control_sim/SensorData.h"
#include "arussim/control_sim/Parameters.h"
#include <math.h>
typedef struct {
    float kAlphaP;
    float kLambdaP;
    float Blat;
    float Blon;
    float Dlat;
    float Clat;
    float Dlon;
    float Clon;
} PAC;

typedef struct{
	float force_fy[4];
	float force_fx[4];
	float Mz;
	float force_fx_rolling;
	float tire_load[4];
} TIRE;

float driver_request(SensorData *sensors, Parameters *parameters);
void Calculate_Tire_Loads(SensorData *sensors, Parameters *parameters, float *state, TIRE *tire);
void Calculate_Tire_Forces(TIRE *tire, const float slip_angle[4], const float slip_ratio[4]);
float pc_request(DV *dv, Parameters *parameters);


#ifdef __cplusplus
}
#endif

#endif /* INC_AUX_FUNCTIONS_H_ */
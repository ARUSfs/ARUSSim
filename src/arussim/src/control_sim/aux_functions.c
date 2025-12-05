	/*
 * aux_functions.c
 *
 *  Created on: May 14, 2025
 *      Author: carme
 */


#include "arussim/control_sim/aux_functions.h"
float tire_loadtx[4];
float tire_loadty[4];
float tire_loadneg[4];
float tire_loadneg2[4];

float driver_request(SensorData *sensors, Parameters *parameters) {
	float fx_request = sensors->apps * 1.25f - 100*sensors->load_cell;
	if (fx_request <= -95) {
		fx_request = -99;
	} else if (fx_request >= 95) {
		fx_request = 99;
	}

	if(fx_request < 0){
		fx_request = -fx_request*2*parameters->torque_limit_negative[0]*parameters->gear_ratio/parameters->rdyn*0.01;
	} else {
		fx_request = fx_request*2*parameters->torque_limit_positive[0]*parameters->gear_ratio/parameters->rdyn*0.01;
	}

	return fx_request;
}
float pc_request (DV *dv, Parameters *parameters){
	return dv->acc * parameters->mass;
}
void Calculate_Tire_Loads(SensorData *sensors, Parameters *parameters, float *state, TIRE *tire) {

    float FL = parameters->fz_params[0] * state[0] * state[0];
    float FD = parameters->fz_params[1] * state[0] * state[0];

    // LATERAL LOAD TRANSFER

    //nonsuspended weight transfer (N)
    float y_WT_ns_f = parameters->fz_params[2] * sensors->acceleration_y;
    float y_WT_ns_r = parameters->fz_params[2] * sensors->acceleration_y;

    //suspended geometric weight transfer (N)
    float y_WT_s_g_f = parameters->fz_params[3] * sensors->acceleration_y;
    float y_WT_s_g_r = parameters->fz_params[4] * sensors->acceleration_y;

    //susp elastic WT (N)
    float y_WT_s_e_f = parameters->fz_params[5] * sensors->acceleration_y;
    float y_WT_s_e_r = parameters->fz_params[6] * sensors->acceleration_y;

    // LONG LOAD TRANSFER

    //long nonsuspended weight transfer (N)
    float x_WT_ns = parameters->fz_params[7] * sensors->acceleration_x;

    //long suspended weight transfer (N)
    float x_WT_s = parameters->fz_params[8] * sensors->acceleration_x;
    //long suspended elastic weight transfer (N)
//    float x_WT_s_e_f = -x_WT_s * ((sensors->acceleration_x < 0) ? 1 : 0) - x_WT_s * ((sensors->acceleration_x > 0) ? 1 : 0); //HA
//    float x_WT_s_e_r = x_WT_s * ((sensors->acceleration_x < 0) ? 1 : 0) + x_WT_s * ((sensors->acceleration_x > 0) ? 1 : 0);	 //VA


    //WHEEL LOADS
	tire->tire_load[0] = parameters->fz_params[9] + parameters->fz_params[10] * FL
			- parameters->fz_params[11] * FD - y_WT_ns_f - y_WT_s_g_f
			- y_WT_s_e_f - 0.5f * (x_WT_ns + x_WT_s);

	tire->tire_load[1] = parameters->fz_params[9] + parameters->fz_params[10] * FL
			- parameters->fz_params[11] * FD + y_WT_ns_f + y_WT_s_g_f
			+ y_WT_s_e_f - 0.5f * (x_WT_ns + x_WT_s);

	tire->tire_load[2] = parameters->fz_params[12] + (0.5f - parameters->fz_params[10]) * FL
			+ parameters->fz_params[11] * FD - y_WT_ns_r - y_WT_s_g_r
			- y_WT_s_e_r + 0.5f * (x_WT_ns + x_WT_s);

	tire->tire_load[3] = parameters->fz_params[12] + (0.5f - parameters->fz_params[10]) * FL
			+ parameters->fz_params[11] * FD + y_WT_ns_r
			+ y_WT_s_g_r + y_WT_s_e_r + 0.5f * (x_WT_ns + x_WT_s);


    // NEGATIVE TIRE LOAD RECALCULATION

    for (int i = 0; i < 4; i++) {
        tire_loadneg[i] = (tire->tire_load[i] < 0.f) ? tire->tire_load[i] : 0.f;
        tire->tire_load[i] = (tire->tire_load[i] > 0.f) ? tire->tire_load[i] : 0.f;
    }

    if ((tire_loadneg[0] < 0 && tire_loadneg[1] < 0) || (tire_loadneg[2] < 0 && tire_loadneg[3] < 0)) {

        tire_loadty[0] = 0.5f * (tire_loadneg[2] - tire_loadneg[3]);
		tire_loadty[1] = -0.5f * (tire_loadneg[2] - tire_loadneg[3]);
		tire_loadty[2] = 0.5f * (tire_loadneg[0] - tire_loadneg[1]);
		tire_loadty[3] = -0.5f * (tire_loadneg[0] - tire_loadneg[1]);

		tire_loadtx[0] = 0.5f * (tire_loadneg[2] + tire_loadneg[3]);
		tire_loadtx[1] = 0.5f * (tire_loadneg[2] + tire_loadneg[3]);
		tire_loadtx[2] = 0.5f * (tire_loadneg[0] + tire_loadneg[1]);
		tire_loadtx[3] = 0.5f * (tire_loadneg[0] + tire_loadneg[1]);

        for (int i = 0; i < 4; i++) {
            tire->tire_load[i] += tire_loadty[i] + tire_loadtx[i];
        }
    }

	else if ((tire_loadneg[0] < 0 && tire_loadneg[2] < 0) || (tire_loadneg[1] < 0 && tire_loadneg[3] < 0)) {
		tire_loadty[0] = 0.5f * (tire_loadneg[1] + tire_loadneg[3]);
		tire_loadty[1] = 0.5f * (tire_loadneg[0] + tire_loadneg[2]);
		tire_loadty[2] = 0.5f * (tire_loadneg[1] + tire_loadneg[3]);
		tire_loadty[3] = 0.5f * (tire_loadneg[0] + tire_loadneg[2]);

		tire_loadtx[0] = 0.5f * (tire_loadneg[1] - tire_loadneg[3]);
		tire_loadtx[1] = 0.5f * (tire_loadneg[0] - tire_loadneg[2]);
		tire_loadtx[2] = -0.5f * (tire_loadneg[1] - tire_loadneg[3]);
		tire_loadtx[3] = -0.5f * (tire_loadneg[0] - tire_loadneg[2]);

        for (int i = 0; i < 4; i++) {
            tire->tire_load[i] += tire_loadty[i] + tire_loadtx[i];
        }
    }


    else if (tire_loadneg[0] < 0 || tire_loadneg[1] < 0) {
        float WT_x = 0.5f * (x_WT_ns + x_WT_s);
        float WT_y = (y_WT_ns_f + y_WT_s_g_f + y_WT_s_e_f);

        tire_loadtx[0] = WT_x / (WT_x + fabs(WT_y) + eps) * tire_loadneg[0];
        tire_loadtx[1] = WT_x / (WT_x + fabs(WT_y) + eps) * tire_loadneg[1];
        tire_loadty[0] = WT_y / (WT_x + fabs(WT_y) + eps) * tire_loadneg[0] * ((sensors->acceleration_y > 0) - (sensors->acceleration_y < 0));
        tire_loadty[1] = WT_y / (WT_x + fabs(WT_y) + eps) * tire_loadneg[1] * ((sensors->acceleration_y > 0) - (sensors->acceleration_y < 0));

        tire->tire_load[0] += tire_loadtx[1];
        tire->tire_load[1] += tire_loadtx[0];
        tire->tire_load[2] += tire_loadty[0];
        tire->tire_load[3] += tire_loadty[1];

        for (int i = 0; i < 4; i++) {
            tire_loadneg2[i] = (tire->tire_load[i] < 0.f) ? tire->tire_load[i] : 0.f;
        }

        for (int i = 0; i < 4; i++) {
            tire->tire_load[i] = (tire->tire_load[i] > 0.f) ? tire->tire_load[i] : 0.f;
        }

        if (tire_loadneg2[2] < 0 || tire_loadneg2[3] < 0) {
            tire->tire_load[0] += tire_loadneg2[3];
            tire->tire_load[1] += tire_loadneg2[2];
        }
        if (tire_loadneg2[0] < 0 || tire_loadneg2[1] < 0) {
            tire->tire_load[0] += 0.5f * (tire_loadneg2[0] + tire_loadneg2[1]);
            tire->tire_load[1] += 0.5f * (tire_loadneg2[0] + tire_loadneg2[1]);
        }


    } else if (tire_loadneg[2] < 0 || tire_loadneg[3] < 0) {
        float WT_x = 0.5f * (x_WT_ns + x_WT_s);
        float WT_y = (y_WT_ns_f + y_WT_s_g_f + y_WT_s_e_f);


        tire_loadtx[2] = -WT_x / (-WT_x + fabs(WT_y)) * tire_loadneg[2];
        tire_loadtx[3] = -WT_x / (-WT_x + fabs(WT_y)) * tire_loadneg[3];
        tire_loadty[2] = WT_y / (-WT_x + fabs(WT_y)) * tire_loadneg[2] * ((sensors->acceleration_y > 0) - (sensors->acceleration_y < 0));
        tire_loadty[3] = WT_y / (-WT_x + fabs(WT_y)) * tire_loadneg[3] * ((sensors->acceleration_y > 0) - (sensors->acceleration_y < 0));

        tire->tire_load[0] += tire_loadty[2];
        tire->tire_load[1] += tire_loadty[3];
        tire->tire_load[2] += tire_loadtx[3];
        tire->tire_load[3] += tire_loadtx[2];

        for (int i = 0; i < 4; i++) {
            tire_loadneg2[i] = (tire->tire_load[i] < 0.f) ? tire->tire_load[i] : 0.f;
        }

        for (int i = 0; i < 4; i++) {
            tire->tire_load[i] = (tire->tire_load[i] > 0.f) ? tire->tire_load[i] : 0.f;
        }

        if (tire_loadneg2[0] < 0 || tire_loadneg2[1] < 0) {
            tire->tire_load[2] += tire_loadneg2[1];
            tire->tire_load[3] += tire_loadneg2[0];
        }
        if (tire_loadneg2[2] < 0 || tire_loadneg2[3] < 0) {
            tire->tire_load[0] += 0.5f * (tire_loadneg2[2] + tire_loadneg2[3]);
            tire->tire_load[1] += 0.5f * (tire_loadneg2[2] + tire_loadneg2[3]);
        }
    }
}

void Calculate_Tire_Forces(TIRE *tire, const float slip_angle[4], const float slip_ratio[4]) {
    PAC pac = {
        .kAlphaP = PAC_KALPHAP,
        .kLambdaP = PAC_KLAMBDA_P,
        .Blat = PAC_BLAT,
        .Blon = PAC_BLON,
        .Dlat = PAC_DLAT,
        .Clat = PAC_CLAT,
        .Dlon = PAC_DLON,
        .Clon = PAC_CLON
    };
    double Elat = 0.4035;
    double Elon = 0.999;
    double Gx1 = 25000;
    double bx = 0.2367;
    double a = 93733;
    double c = 0.1689;
    double Gy1 = 38.21;
    double by = 0.5365;

    float alpha_star[4];
    float lambda_star[4];
    float s_star[4];
    float slip_angle_local[4];
    float min_s_star = 0.01f;

    slip_angle_local[0] = tanf(slip_angle[0]);
    slip_angle_local[1] = tanf(slip_angle[1]);
    slip_angle_local[2] = tanf(slip_angle[2]);
    slip_angle_local[3] = tanf(slip_angle[3]);


    //Combined slip
    for (int i = 0; i < 4; i++) {
        alpha_star[i] = (1 - bx) * exp(-Gx1 * exp(-pow(abs(a * slip_ratio[i]), c)) * slip_angle_local[i] * slip_angle_local[i]) + pac.kAlphaP;
        lambda_star[i] = by + (1 - by) * exp(-Gy1 * slip_ratio[i] * slip_ratio[i]); 

        double aux_fy = Elat * (pac.Blat * slip_angle_local[i] - atanf(pac.Blat * slip_angle_local[i]));
        double aux_fx = Elon * (pac.Blon * slip_ratio[i] - atanf(pac.Blon * slip_ratio[i]));

        float fy_pure = tire->tire_load[i] * pac.Dlat * sinf(pac.Clat * atanf(pac.Blat * slip_angle_local[i] - aux_fy));
        float fx_pure = tire->tire_load[i] * pac.Dlon * sinf(pac.Clon * atanf(pac.Blon * slip_ratio[i] - aux_fx));
        tire->force_fy[i] = fy_pure * alpha_star[i];
        tire->force_fx[i] = fx_pure * lambda_star[i];
    }

}
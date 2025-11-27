/*
 * Parameters.h
 *
 *  Created on: Feb 22, 2025
 *      Author: carme
 */
#include <stdint.h>



#ifndef INC_PARAMETERS_H_
#define INC_PARAMETERS_H_
#ifdef __cplusplus
extern "C" { 
#endif

//VDC MODELS ACTIVATION
#define TV_ACTIVE			0
#define TC_ACTIVE			1
#define EST_ACTIVE			0


#define pi 					3.141592
#define eps					0.00001
#define GRAVITY 			9.81
#define DEFAULT_MAX_POWER   50000
#define DEFAULT_MIN_POWER 	-30000

#define DEFAULT_MAX_VOLTAGE 595
#define DEFAULT_MIN_VOLTAGE 430

//CAR DATA
//distance
#define DEFAULT_TW			1.22
#define DEFAULT_WB			1.535
#define DEFAULT_R_CDG		0.5


//Mass and inertia
#define DEFAULT_MASS		340.
#define DEFAULT_NSM			25
#define DEFAULT_H_CDG		0.273
#define DEFAULT_H_CDG_NSM 	0.225
#define DEFAULT_H_CDG_SM	0.3
#define DEFAULT_H_RC_F		0.033
#define DEFAULT_H_RC_R		0.097

//Stiffness
#define K_s_f				87563
#define K_s_r				87563
#define K_ARB_f				3786
#define K_ARB_r				11

//Motion Ratios
#define MR_s				1.1003
#define MR_ARB_f_DIRK		2.05
#define MR_ARB_r_DIRK		2.745
#define r_ARB_f				0.0628
#define psi_ARB_f			0.07079055 //4.056 * pi /180
#define r_ARB_r				0.07
#define psi_ARB_r			0.1282817 //7.35 * pi /180
#define DEFAULT_GEAR_RATIO  12.48
#define DEFAULT_RDYN		0.225
#define DEFAULT_WHEEL_INERTIA    0.4
#define DEFAULT_TL_POS		21.	
#define DEFAULT_TL_NEG		-21.

//AERO
#define DEFAULT_RHO			1.225
#define DEFAULT_CDA			1.971
#define DEFAULT_CLA			4.747
#define DEFAULT_R_CDP		0.4604
#define DEFAULT_H_CDP		0.517

//YAW RATE PID
#define TV_KP				400			//Pruebas mesa
#define TV_KI				0
#define TV_KD				0
#define DEFAULT_TV_N		0
#define DEFAULT_MAX_MZ		600


//TC PID
#define TC_K_				30 //10
#define TC_TI				0.1
#define TC_TD				0


//TC Paramtets
#define DEFAULT_TC_V0		5
#define DEFAULT_TC_V_GAIN	3

//PACEJKA TIRE MODEL
// PAC parameters
#define PAC_KALPHAP      0.1809f
#define PAC_KLAMBDA_P    0.1397f
#define PAC_BLAT         12.f
#define PAC_BLON         17.f
#define PAC_DLAT        -1.33f
#define PAC_CLAT         2.f
#define PAC_DLON         1.198f
#define PAC_CLON         1.3f

//TODO: eliminar de la estructura y del init lo que no se use
typedef struct {
	float g;
    float maximum_power;
    float minimum_power;

    float V_max;
    float V_min;

    //CAR DATA
    //Distances
    float trackwidthR, trackwidthF;
    float wheelbase;
    float r_cdg;
    float lf, lr;

    //Mass and inertia
    uint16_t mass;
    uint8_t nsm_f, nsm_r;
    uint8_t sm;
    float sm_f, sm_r;
    float h_cdg;
    float h_cdg_nsm_f, h_cdg_nsm_r;
    float h_cdg_sm;
    float h_RC_f, h_RC_r, h_RA;

    //Motion Ratios
    float RS_f, RS_r, RS;
    float gear_ratio;
    float rdyn, wheel_inertia;
    float torque_limit_negative [4];
    float torque_limit_positive [4];


    //AERO
    float rho, CDA, CLA;
    float r_cdp, h_cdp;

    float fz_params[13];

    //TC
    int v0;
    int v_gain;
} Parameters;

typedef struct{
	int autonomous, driving, inspection;
	float acc, target_r;
	float inspection_torque[4];
} DV;

typedef struct{
	//TV
	float TV_Kp, TV_Ki, TV_Kd, TV_N;
	float integral;
	float err_prev;
	float deriv_filt;
	uint16_t max_Mz;

	//TC
	float TC_K, TC_Ti, TC_Td;

	//general
	float TS;
	uint32_t last_timestamp;
	int init;

} PID;



#ifdef __cplusplus
}
#endif


#endif /* INC_PARAMETERS_H_ */
#ifndef CONTROL_SIM_SENSORDATA_H
#define CONTROL_SIM_SENSORDATA_H

#include <stdint.h>

#ifdef __cplusplus
extern "C" { 
#endif

typedef struct {
    float acceleration_x, acceleration_y, acceleration_z;
    float angular_x, angular_y, angular_z;
    float speed_x, speed_y;
    float steering_angle;
    float apps;
    float load_cell;
    float motor_speed[4];
    float current, vehicle_side_voltage, battery_voltage;
    float power;
    float V_soc;
} SensorData;

#ifdef __cplusplus
}
#endif

#endif // CONTROL_SIM_SENSORDATA_H

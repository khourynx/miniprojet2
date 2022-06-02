#ifndef MAIN_H
#define MAIN_H

#ifdef __cplusplus
extern "C" {
#endif

#include "msgbus/messagebus.h"
#include "parameter/parameter.h"

#define MOTOR_L					500	// steps/s = 5.55 cm/s
#define MOTOR_R					500	// steps/s = 5.55 cm/s
#define DISTANCE_LIMITE_VIRAGE  1
#define TURN_FACTOR				360
#define WHEEL_PERIMETER         13 // [cm]
#define PI                  	3.1415926536f
#define WHEEL_DISTANCE      	5.35f    //cm
#define EPUCK_DIAMETER			54
#define DISTANCE_PROX			200
#define SPEED_0					0
#define ANGLE_90				50

void SendUint8ToComputer(uint8_t* data, uint16_t size);

/** Robot wide IPC bus. */
extern messagebus_t bus;

extern parameter_namespace_t parameter_root;

#ifdef __cplusplus
}
#endif

#endif

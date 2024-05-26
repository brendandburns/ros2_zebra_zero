#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#ifndef MOCK_HARDWARE
#include "nmccom.h"
#include "picservo.h"
#else
#define SERVOMODTYPE 0
#define STEPMODTYPE 1

#define START_NOW 1
#define ENABLE_SERVO 2
#define LOAD_ACC 4
#define LOAD_VEL 8
#define LOAD_POS 16
#define LOAD_PWM 32
#define VEL_MODE 64
#define REVERSE 128

#define MOTOR_OFF 1
#define AMP_ENABLE 2
#define STOP_ABRUPT 4
#define STOP_SMOOTH 0
#define ADV_FEATURE 8

#define SEND_POS 1
#define SEND_VEL 2

#endif

#endif // __CONSTANTS_H
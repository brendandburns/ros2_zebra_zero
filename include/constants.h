#ifndef __CONSTANTS_H__
#define __CONSTANTS_H__

#ifdef OLDNMC
#include "nmccom.h"
#include "picservo.h"
#else
#define SERVOMODTYPE 0
#define STEPMODTYPE 3

#define START_NOW 0x80
#define ENABLE_SERVO 0x10
#define LOAD_ACC 0x04
#define LOAD_VEL 0x02
#define LOAD_POS 0x01
#define LOAD_PWM 0x08
#define VEL_MODE 0x20
#define REVERSE 0x40

#define MOTOR_OFF 0x04
#define AMP_ENABLE 0x01
#define STOP_ABRUPT 0x04
#define STOP_SMOOTH 0x08
#define ADV_FEATURE 0x20

#define SEND_POS 0x01
#define SEND_VEL 0x04

#define MOVE_DONE 0x01
#endif

#endif // __CONSTANTS_H
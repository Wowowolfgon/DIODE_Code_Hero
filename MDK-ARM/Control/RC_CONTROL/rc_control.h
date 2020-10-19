#ifndef __RC_CONTROL_H
#define __RC_CONTROL_H
#include "main.h"

extern uint8_t rc_flag;
void RC_PC(void);
void RC_Chassis(void);
void RC_Shoot(float fri_speed,float dial_speed);
void RC_Vision_aiming();
void RC_Singleshot(float fri_speed,uint8_t pattern);
void RC_Spin(void);
#endif


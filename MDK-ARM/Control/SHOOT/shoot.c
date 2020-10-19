/**
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 * @file       shoot.c
 * @brief      this file contains sd card basic operating function
 * @note       
 * @Version    V1.0.0
 * @Date       2019.12.6      
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 */
 
#include "shoot.h"

// ÉèÖÃÄ¦²ÁÂÖµç»ú
void Friction_SetSpeed(uint16_t speed_l,uint16_t speed_s)
{
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_1,1100);	// H10 D ÓÒ±ß
    __HAL_TIM_SET_COMPARE(&htim5, TIM_CHANNEL_2,1100);	// H11 C ×ó±ß
}


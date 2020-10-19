/**
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 * @file       beep.c
 * @brief      this file contains sd card basic operating function
 * @note       TIM12 CH1 (PH6)
 * @Version    V1.0.0
 * @Date       2019.12.6     
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 */
#include "bsp_beep.h"

/* ·äÃùÆ÷Ãù½Ð */
void Beep_On(void)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
}
/* ·äÃùÆ÷Ï¨»ð */
void Beep_Off(void)
{
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}
/* ·äÃùÆ÷Ãù½ÐÒ»¶ÎÊ±¼ä ms */
void Beep(uint16_t t)
{
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_3);
	HAL_Delay(t);
	HAL_TIM_PWM_Stop(&htim4, TIM_CHANNEL_3);
}

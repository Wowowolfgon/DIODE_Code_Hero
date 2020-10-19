/**
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 * @file       bsp_pwm.c
 * @brief      
 * @note       TIM1 (IN1) (IN2) (IN3) (IN4)
 * 		       TIM5 (IN1) (IN2)
 * @Version    V1.0.0
 * @Date       2019.12.6      
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 */
#include "bsp_tim.h"

uint16_t tim11_cnt=0;

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
	// 1msµÄÖÐ¶Ï
	if (htim->Instance == htim11.Instance)
    {
		tim11_cnt++;
		if(tim11_cnt>=tim11_cnt_max)
			tim11_cnt = tim11_cnt_max;
		
		
    }
}


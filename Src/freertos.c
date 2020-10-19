/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * <h2><center>&copy; Copyright (c) 2019 STMicroelectronics.
  * All rights reserved.</center></h2>
  *
  * This software component is licensed by ST under Ultimate Liberty license
  * SLA0044, the "License"; You may not use this file except in compliance with
  * the License. You may obtain a copy of the License at:
  *                             www.st.com/SLA0044
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */     
#include "bsp_can.h"
#include "pid.h"
#include "bsp_pwm.h"
//#include "bsp_imu.h"
#include "gimbal.h"

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern int 			setSpeed[4];
extern UART_HandleTypeDef huart1;
extern UART_HandleTypeDef huart2;
extern uint8_t RxCounter1,RxBuffer1[500],RxTemp1,F_Usart1;
extern VisionData datadata;
extern imu_t      imu;
float i=0;
/* USER CODE END Variables */
osThreadId defaultTaskHandle;
osThreadId ControlHandle;
osThreadId DebugHandle;

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void const * argument);
void StartControl(void const * argument);
void StartDebug(void const * argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/* GetIdleTaskMemory prototype (linked to static allocation support) */
void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize );

/* USER CODE BEGIN GET_IDLE_TASK_MEMORY */
static StaticTask_t xIdleTaskTCBBuffer;
static StackType_t xIdleStack[configMINIMAL_STACK_SIZE];

void vApplicationGetIdleTaskMemory( StaticTask_t **ppxIdleTaskTCBBuffer, StackType_t **ppxIdleTaskStackBuffer, uint32_t *pulIdleTaskStackSize )
{
    *ppxIdleTaskTCBBuffer = &xIdleTaskTCBBuffer;
    *ppxIdleTaskStackBuffer = &xIdleStack[0];
    *pulIdleTaskStackSize = configMINIMAL_STACK_SIZE;
    /* place for user code */
}
/* USER CODE END GET_IDLE_TASK_MEMORY */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
    /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
    /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
    /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
    /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* definition and creation of defaultTask */
  osThreadDef(defaultTask, StartDefaultTask, osPriorityNormal, 0, 128);
  defaultTaskHandle = osThreadCreate(osThread(defaultTask), NULL);

  /* definition and creation of Control */
  osThreadDef(Control, StartControl, osPriorityNormal, 0, 128);
  ControlHandle = osThreadCreate(osThread(Control), NULL);

  /* definition and creation of Debug */
  osThreadDef(Debug, StartDebug, osPriorityNormal, 0, 128);
  DebugHandle = osThreadCreate(osThread(Debug), NULL);

  /* USER CODE BEGIN RTOS_THREADS */
    /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void const * argument)
{
  /* USER CODE BEGIN StartDefaultTask */
tim11_cnt=0;
    /* Infinite loop */
    for(;;)
    {
//		IMU_Get();
//		HAL_GPIO_WritePin(LED_R_GPIO_Port, LED_R_Pin, 1);
//		HAL_Delay(300);
//		LED_R_Toggle();
//		if(flag1)
//		LED_R_On();
//		 CAN_GM6020[1].angle;
//		CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],7200, CAN_M2006[0].speed);
//	Vision_aiming();
//		CAN_Chassis_SendCurrent2();
//		CAN_GM6020_Gimbal_SendVoltage();
//		printf("%s","33   ");
//		HAL_UART_Transmit_DMA(&huart1, "1", 10);
//		HAL_UART_RxCpltCallback(&huart1);
		if(rc_flag)
		{			
//			Vision_aiming();
//			CAN_Chassis_SendCurrent2();
//			CAN_GM6020_Gimbal_SendVoltage();
//			RC_Shoot(6000,6000);  //连发模式
			RC_Singleshot(8000,6);//单发模式
//			RC_Shoot(10000,200);
			RC_Vision_aiming();   //自瞄模式
//			RC_Chassis();		  //移动模式
//			RC_PC();
//			RC_Spin();			  //陀螺模式
		}

    }
  /* USER CODE END StartDefaultTask */
}

/* USER CODE BEGIN Header_StartControl */
/**
* @brief Function implementing the Control thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartControl */
void StartControl(void const * argument)
{
  /* USER CODE BEGIN StartControl */
    /* Infinite loop */
    for(;;)
    {
//        RC_Shoot(3000,5000);
        if(rc_flag==1)
            Beep_Off();
        else {
//			Beep_On();
            CAN_M3508[0].set_current=0;
            CAN_M3508[1].set_current=0;
            CAN_M3508[2].set_current=0;
            CAN_M3508[3].set_current=0;
            CAN_M3508[4].set_current=0;
            CAN_M3508[5].set_current=0;
            CAN_GM6020[0].set_voltage=0;
            CAN_GM6020[1].set_voltage=0;
			CAN_M2006[0].set_current =0;
			CAN_M2006[1].set_current =0;
            CAN_M3508_Chassis_SendCurrent();
			CAN_Chassis_SendCurrent2();
            CAN_GM6020_Gimbal_SendVoltage();
        }
        rc_flag=0;
        osDelay(100);

//		printf("%d",rc.mouse_x);
//		printf("%f %f %f %f\n",datadata.pitch_angle.f,datadata.yaw_angle.f,datadata.z_angle.f,datadata.dis.f);
//		VisionUartTxSenddata();
//		OLED_Printf(0,1,"%f",-CAN_M2006[0].total_angle);
//		OLED_RefreshGram();
    }
  /* USER CODE END StartControl */
}

/* USER CODE BEGIN Header_StartDebug */
/**
* @brief Function implementing the Debug thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartDebug */
void StartDebug(void const * argument)
{
  /* USER CODE BEGIN StartDebug */

    /* Infinite loop */
    for(;;)
    {
//		HAL_UART_RxCpltCallback(&huart1);
		IMU_Get();
//		osDelay(5);
//        IMU_Get();
//		BT_Send6020Wave();
//		BT_SendWave();
//        BT_SendPowerHeatWave();
//		i=PID_Calculate(&PID_HEAT_PWM,50,imu.temp)*100;
//        __HAL_TIM_SET_COMPARE(&htim3,TIM_CHANNEL_2,PID_Calculate(&PID_HEAT_PWM,45,imu.temp)*100);
    }
  /* USER CODE END StartDebug */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

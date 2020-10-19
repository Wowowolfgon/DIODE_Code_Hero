/**
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 * @file       bluetooth.c
 * @brief      this file contains sd card basic operating function
 * @note       
 * @Version    V1.0.0
 * @Date       2019.12.6      
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 */
#include "bluetooth.h"
#include "usart.h"
extern CAN_M3508_TypeDef	CAN_M3508[];	// 引用M3508结构体
extern CAN_GM6020_TypeDef 	CAN_GM6020[];	// 引用GM6020结构体
extern CAN_M2006_TypeDef	CAN_M2006[2];	// 引用M2006结构体

extern PID_TypeDef PID_GM6020[];			// 引用GM6020 PID结构体
extern PID_TypeDef PID_M2006[];			// 引用M2006 PID结构体
extern PID_TypeDef PID_M3508[];			// 引用M3508 PID结构体

extern rc_info_t	rc;					//遥控器接收
extern imu_t 		imu;				// 引用陀螺仪
extern int 			setSpeed[4];
extern float2uchar ph01;
extern float i;
extern VisionData datadata;
void BT_SendChasisWave(void)
{
	uint8_t num = 4;
	int16_t temp[num];
	
	temp[0] = CAN_M3508[0].speed;
	temp[1] = CAN_M3508[0].current;
	temp[2] = CAN_M3508[0].set_current;
	temp[3] = 0;

	VCAN_SendWare(&huart6, &temp,2*num);
}
void BT_SendDialWave(void)
{
	uint8_t num = 4;
	int16_t temp[num];
	
	temp[0] = CAN_M2006[0].speed;
	temp[1] = CAN_M2006[0].current;
	temp[2] = CAN_M2006[0].set_current;
	temp[3] = 0;

	VCAN_SendWare(&huart6, &temp,2*num);
}
extern float spin_angle_set;
void BT_SendgimbalWave(void)
{
	uint8_t num = 4;
	int16_t temp[num];
	
	temp[0] = CAN_GM6020[0].speed;
	temp[1] = CAN_GM6020[0].current;
	temp[2] = spin_angle_set;
	temp[3] = CAN_GM6020[0].angle;

	VCAN_SendWare(&huart6, &temp,2*num);
}
void BT_Check(void)
{
	printf("\r\n OK \r\n");
	printf(" OK \r\n");
	printf(" OK \r\n");
}
void BT_ResetPID1(float p,float i,float d)
{
	PID_Reset(&PID_M3508[0],p,i,d);
}

void BT_SendWave(void)
{
	uint8_t num = 8;
	int16_t temp[num];
	
	temp[0] = CAN_GM6020[0].set_voltage;
	temp[1] = map(CAN_GM6020[0].angle,0,8191,0,360);
	temp[2] = CAN_GM6020[1].set_voltage;
	temp[3] = CAN_GM6020[1].angle/8191*360;
	temp[4] = spin_angle_set;
	temp[5] = pitch_angle_set;
	temp[6] = PID_GM6020[0].pos_out;
	temp[7] = PID_GM6020[1].pos_out;
	
	VCAN_SendWare(&huart6, &temp,2*num);
}
void BT_SendWave2(void)
{
	uint8_t num = 8;
	int16_t temp[num];
	
	temp[0] = PID_M3508_Follow.err[0];
	temp[1] = PID_M3508_Follow.feedback[0];
//	temp[2] = imu.yaw;
	temp[3] = CAN_M3508[1].speed;
	temp[4] = CAN_M3508[2].speed;
//	temp[5] = imu.temp;
	temp[6] = CAN_GM6020[0].angle;
	temp[7] = PID_M3508_Follow.pos_out;
	
	VCAN_SendWare(&huart6, &temp,2*num);
}
// 陀螺仪温度调试
void BT_SendWave3(void)
{
	uint8_t num = 8;
	int16_t temp[num];
	
	temp[0] = PID_HEAT_PWM.pos_out;
//	temp[1] = imu.temp;
	temp[2] = MUC_TEMPERATE;
//	temp[3] = (int16_t)imu.yaw;
	temp[4] = 0;
	temp[5] = 0;
	temp[6] = 0;
	temp[7] = 0;
	
	VCAN_SendWare(&huart6, &temp,2*num);
}
void BT_Sendgimbal_3508Wave(void)
{
	uint8_t num = 4;
	int16_t temp[num];
	
	temp[0] = CAN_M3508[4].speed;
	temp[1] = CAN_M3508[5].speed;
	temp[2] = CAN_M2006[0].speed;
	temp[3] = 0;

	VCAN_SendWare(&huart6, &temp,2*num);
}
void BT_SendPowerHeatWave(void)
{
	uint8_t num = 4;
	float temp[4];
	
	temp[0] = datadata.yaw_angle;//imu.yaw;
	temp[1] = datadata.pitch_angle;//imu.temp;
	temp[2] = CAN_GM6020[0].speed;
	temp[3] = CAN_GM6020[1].set_voltage;
	VCAN_SendWare(&huart6, &temp,4*num);
}
void BT_Send6020Wave(void)
{
	uint8_t num = 2;
	float temp[num];
	
	temp[0] = spin_angle_set;
	temp[1] = CAN_GM6020[1].total_angle/8191*360;
//	temp[0] = 344.0;
//	temp[1] = 0.89898;
	VCAN_SendWare(&huart6, &temp,4*num);
}

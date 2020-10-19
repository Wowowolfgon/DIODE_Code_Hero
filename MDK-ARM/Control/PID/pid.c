/**
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 * @file       pid.c
 * @brief      this file contains sd card basic operating function
 * @note
 * @Version    V1.0.0
 * @Date       2019.12.6
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 */
#include "pid.h"
#include "math.h"
#include "bsp_can.h"

float pi=3.1415926535898;
float angle0=0,yaw0=0;
extern imu_t imu;

PID_TypeDef PID_GM6020[2];	// 定义GM6020 PID结构体
PID_TypeDef PID_GM6020_speed[2];
PID_TypeDef PID_M2006[2];		// 定义M2006 PID结构体
PID_TypeDef PID_M2006_ANGLE[2];// 定义M2006 角度PID结构体
PID_TypeDef PID_M3508[6];	// 定义M3508 PID结构体 前四个为底盘电机 后两个为摩擦轮电机
PID_TypeDef PID_M3508_Follow;// 定义M3508底盘 PID结构体

PID_TypeDef PID_HEAT_PWM;// 定义陀螺仪加热 PID结构体
void abs_limit(float *a, float ABS_MAX) {
    if(*a > ABS_MAX)
        *a = ABS_MAX;
    if(*a < -ABS_MAX)
        *a = -ABS_MAX;
}

void PID_Reset(PID_TypeDef	*pid, float kp, float ki, float kd)
{
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;
}
/**
  * @brief  初始化PID结构体
  * @param  PID结构体指针
    @param  比例系数
		@param  积分系数
		@param  微分系数
		@param  积分最大值
		@param  总输出最大值
  * @retval None
  */
void PID_Init(
    PID_TypeDef*	pid,
    uint32_t 			mode,
    uint32_t 			maxout,
    uint32_t 			intergral_limit,
    float 				kp,
    float 				ki,
    float 				kd)
{
    pid->IntegralLimit = intergral_limit;
    pid->MaxOutput = maxout;
    pid->pid_mode = mode;

    pid->target[0]=0;
    pid->Kp = kp;
    pid->Ki = ki;
    pid->Kd = kd;

}

float PID_Calculate(PID_TypeDef *pid, float target, float feedback)
{
    pid->feedback[NOW] = feedback;
    pid->target[NOW] = target;
    pid->err[NOW] = target - feedback;

    if (pid->max_err != 0 && ABS(pid->err[NOW]) >  pid->max_err  )
        return 0;
    if (pid->deadband != 0 && ABS(pid->err[NOW]) < pid->deadband)
        return 0;

    if(pid->pid_mode == POSITION_PID)					 //位置式PID
    {
        pid->pout = pid->Kp * pid->err[NOW];
        pid->iout += pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - pid->err[LAST] );

        abs_limit(&(pid->iout), pid->IntegralLimit);				//限制积分输出
        pid->pos_out = pid->pout + pid->iout + pid->dout;		// 计算总输出
        abs_limit(&(pid->pos_out), pid->MaxOutput);					// 限制总输出
        pid->last_pos_out = pid->pos_out;										//更新上一次总输出
    }
    else if(pid->pid_mode == DELTA_PID)					//增量式PID
    {
        pid->pout = pid->Kp * (pid->err[NOW] - pid->err[LAST]);
        pid->iout = pid->Ki * pid->err[NOW];
        pid->dout = pid->Kd * (pid->err[NOW] - 2*pid->err[LAST] + pid->err[LLAST]);

        abs_limit(&(pid->iout), pid->IntegralLimit);
        pid->delta_u = pid->pout + pid->iout + pid->dout;
        pid->delta_out = pid->last_delta_out + pid->delta_u;
        abs_limit(&(pid->delta_out), pid->MaxOutput);
        pid->last_delta_out = pid->delta_out;	//update last time
    }

    pid->err[LLAST] = pid->err[LAST];
    pid->err[LAST] = pid->err[NOW];
    pid->feedback[LLAST] = pid->feedback[LAST];
    pid->feedback[LAST] = pid->feedback[NOW];
    pid->target[LLAST] = pid->target[LAST];
    pid->target[LAST] = pid->target[NOW];

    return pid->pid_mode==POSITION_PID ? (fabs(target)<=3? 0 : pid->pos_out) : pid->delta_out;
}

void M3508_follow(PID_TypeDef *pid_6020,float target)//底盘跟随
{
    float n;
	
//    if(CAN_GM6020[1].angle>4095)
//        angle=-8191+CAN_GM6020[1].angle;
//	else
	float angle=CAN_GM6020[1].total_angle/8191*360;
	PID_Calculate(pid_6020,target,angle);
	if(angle<-124&&angle>-132)
	{
		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],0,CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],0,CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],0,CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],0,CAN_M3508[3].speed);
	}
	else
	{
		n=-pid_6020->pos_out;
	//	OLED_Printf(0,1,"%f",PID_Calculate(pid_6020,0,angle));
	//	OLED_Printf(1,1,"%f",n);
	//	OLED_Printf(2,1,"%f",PID_Calculate(&PID_M3508[0],n,CAN_M3508[0].speed));
	//	OLED_RefreshGram();
		
		CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],n,CAN_M3508[0].speed);
		CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],n,CAN_M3508[1].speed);
		CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],n,CAN_M3508[2].speed);
		CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],n,CAN_M3508[3].speed);
	}
}
void Gyro_mobile_init()//小陀螺移动初始化
{
//    __HAL_TIM_SetCompare(&htim2, TIM_CHANNEL_1, 10000);
//	HAL_Delay(0);
    angle0=CAN_GM6020[1].total_angle/8191*2*pi/2;
    yaw0=imu.yaw;

}
void Gyro_mobile(float angle,float speed,float target)//小陀螺移动
{
    float angle1;
//	angle=angle/180*pi;
    angle1=	angle0;//+imu.yaw-yaw0;
    angle=-CAN_GM6020[1].total_angle/8191*2*pi/2+angle1+angle;
//	printf("%f",angle);
    CAN_M3508[0].set_current=PID_Calculate(&PID_M3508[0],cos(angle+pi/4)*speed+target,CAN_M3508[0].speed);
    CAN_M3508[1].set_current=PID_Calculate(&PID_M3508[1],-sin(angle+pi/4)*speed+target,CAN_M3508[1].speed);
    CAN_M3508[2].set_current=PID_Calculate(&PID_M3508[2],sin(angle+pi/4)*speed+target,CAN_M3508[2].speed);
    CAN_M3508[3].set_current=PID_Calculate(&PID_M3508[3],-cos(angle+pi/4)*speed+target,CAN_M3508[3].speed);

//    CAN_M3508_Chassis_SendCurrent();
}


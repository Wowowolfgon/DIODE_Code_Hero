/**
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 * @file       rc_control.c
 * @brief
 * @note
 * @Version    V1.0.0
 * @Date       2019.12.6
 ***************************************(C) COPYRIGHT 2019 DIODE***************************************
 */
#include "rc_control.h"

extern imu_t imu;
extern int 			setSpeed[4];
uint8_t rc_flag=0;	// 遥控器连接标志
uint8_t rc_f=0,rc_n=1;     //拨弹判断标志
float GM6020_1feedback,GM6020_1feedback1,GM6020_flag=0;
void RC_PC(void)
{
	pitch_angle_set += map(rc.mouse_y,-660,660,-10,10);
	if(pitch_angle_set>=pitch_angle_max)pitch_angle_set= pitch_angle_max;
	else if(pitch_angle_set<=pitch_angle_min)pitch_angle_set= pitch_angle_min;
    spin_angle_set += map(rc.mouse_x,-660,660,-10,10);
	CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020[1],spin_angle_set+100,imu.yaw+100);	
	CAN_GM6020[0].set_voltage = PID_Calculate(&PID_GM6020[0],pitch_angle_set,map((float)CAN_GM6020[0].angle,0,8191,0,360));
	int16_t x=(rc.key[0]==0?0:1000)+(rc.key[1]==0?0:-1000);
	int16_t y=(rc.key[2]==0?0:-1000)+(rc.key[3]==0?0:1000);
	
	if(!rc.key[6]){
		if(GM6020_flag==0){
			GM6020_flag=1;
			CAN_GM6020[1].total_angle=fmod(CAN_GM6020[1].total_angle,16382);
		}
		if(y!=0||x!=0)
		{
			velocity(y,x);
			for(int i=0; i<4; i++) 	//对M3508的操作
			{
				setSpeed[i] = setSpeed[i]+map(rc.mouse_x,-660,660,-10,10);//,-1320,1320,-10000,10000);
				CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
			}
		}
		else
			M3508_follow(&PID_M3508_Follow,-128);
	}
	else
	{
		GM6020_flag=0;
		if(rc.mouse_x==0)
			Gyro_mobile(-1.088736,0,y);
		else 
		{
			Gyro_mobile(-1.088736-map(rc.mouse_x,-660,660,-2*3.1415926,2*3.1415926),x,y);
		}
	}
	if(rc.key[5])
		Pluck_angle(125,1);
	else
		Pluck_angle(4,1);
    if(rc.mouse_press_l==1)
	{
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -10000, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], 10000, CAN_M3508[5].speed);
        if(rc_f==1){Pluck_angle(360/12*rc_n,0);rc_n++;rc_f=0;}
		else{Pluck_angle(360/12*rc_n,0);}
	}
    else
	{
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -10000, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], 10000, CAN_M3508[5].speed);
        if(rc_f==0){Pluck_angle(360/12*rc_n,0);rc_n++;rc_f=1;}
		else{Pluck_angle(360/12*rc_n,0);}
	}
	
	    CAN_GM6020_Gimbal_SendVoltage();
		CAN_Chassis_SendCurrent2();
		CAN_M3508_Chassis_SendCurrent();
}
void RC_Chassis(void)
{
	pitch_angle_set -= map(rc.ch1,-660,660,-0.8,0.8);
	if(pitch_angle_set>=pitch_angle_max)pitch_angle_set= pitch_angle_max;
	else if(pitch_angle_set<=pitch_angle_min)pitch_angle_set= pitch_angle_min;
    spin_angle_set += map(rc.ch0,-660,660,-1.5,1.5);
	
//    spin_angle_set += map(rc.ch0,-660,660,-1,1);//1
//	if(GM6020_flag)
//		GM6020_1feedback=(float)CAN_GM6020[1].angle,GM6020_flag=0;
//	else
//		GM6020_1feedback1=(float)CAN_GM6020[1].angle,GM6020_flag=1;
//	if(abs(GM6020_1feedback1-GM6020_1feedback)>6000)
//	{
//		if(spin_angle_set>360)
//			spin_angle_set = spin_angle_set-360;
//		else
//			spin_angle_set = spin_angle_set+360;
//	}
//	if(spin_angle_set==0)spin_angle_set=10;
	CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020[1],spin_angle_set+100,imu.yaw+100);
//    CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020[1],spin_angle_set,map((float)CAN_GM6020[1].angle,0,8191,0,360));	
	CAN_GM6020[0].set_voltage = PID_Calculate(&PID_GM6020[0],pitch_angle_set,map((float)CAN_GM6020[0].angle,0,8191,0,360));
	if(rc.sw2==3||rc.sw2==2){
		if(GM6020_flag==0){
			GM6020_flag=1;
			CAN_GM6020[1].total_angle=fmod(CAN_GM6020[1].total_angle,16382);
		}
		if(rc.ch2!=0||rc.ch3!=0) {
			velocity(rc.ch2,rc.ch3);
			for(int i=0; i<4; i++) 	//对M3508的操作
			{
				setSpeed[i] = map(setSpeed[i]+rc.ch0,-1320,1320,-10000,10000);
				CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
			}
		}
		else	
			M3508_follow(&PID_M3508_Follow,-128);
	//	{
	//	    CAN_M3508[0].set_current = 0;
	//	    CAN_M3508[1].set_current = 0;
	//	    CAN_M3508[2].set_current = 0;
	//	    CAN_M3508[3].set_current = 0;
	//	}
		if(rc.sw2==2) Pluck_angle(125,1);if(rc.sw2==3)Pluck_angle(4,1);
	}
	else 
	{
		GM6020_flag=0;
//		map(rc.ch2,-660,660,-10000,10000);
		if(rc.ch2==0)
			Gyro_mobile(-1.088736,map(rc.ch3,-660,660,-6000,6000),map(rc.ch4,-660,660,-6000,6000));
		else 
		{
			Gyro_mobile(-1.088736-map(rc.ch2,-660,660,-2*3.1415926,2*3.1415926),map(rc.ch3,-660,660,-6000,6000),map(rc.ch4,-660,660,-6000,6000));
		}
		Pluck_angle(4,1);
	}
	
	  CAN_GM6020_Gimbal_SendVoltage();
		CAN_Chassis_SendCurrent2();
		CAN_M3508_Chassis_SendCurrent();
}
/*
SW1 = 1	: 不发射
SW1 = 2 ：开启拨盘和摩擦轮
SW1 = 3 ：只开启摩擦轮
*/
void RC_Shoot(float fri_speed,float dial_speed)
{
    // 根据SW1设置发射机构电机电流
    switch(rc.sw1)
    {
//	if(CAN_M2006[0].speed<dial_speed/10)rc.sw1=1;
    case 1:
        CAN_M3508[4].set_current = 0;
        CAN_M3508[5].set_current = 0;
        CAN_M2006[0].set_current = 0;
        break;
    case 3:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0],0, CAN_M2006[0].speed);//0

        break;
    case 2:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        CAN_M2006[0].set_current = PID_Calculate(&PID_M2006[0], dial_speed, CAN_M2006[0].speed);

        break;
    }
    // 发送电流值给电机
//    CAN_Shoot_SendCurrent();
}
void RC_Singleshot(float fri_speed,uint8_t pattern)//pattern为6时为小弹丸，4时为大弹丸
{

	switch(rc.sw1)
    {
//	if(CAN_M2006[0].speed<dial_speed/10)rc.sw1=1;
    case 1:
        CAN_M3508[4].set_current = 0;
        CAN_M3508[5].set_current = 0;
        CAN_M2006[0].set_current = 0;
        break;
    case 3:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        if(rc_f==1){Pluck_angle(360/pattern*rc_n,0);rc_n++;rc_f=0;}
		else{Pluck_angle(360/pattern*rc_n,0);}
        break;
    case 2:
        CAN_M3508[4].set_current =  PID_Calculate(&PID_M3508[4], -fri_speed, CAN_M3508[4].speed);
        CAN_M3508[5].set_current =  PID_Calculate(&PID_M3508[5], fri_speed, CAN_M3508[5].speed);
        if(rc_f==0){Pluck_angle(360/pattern*rc_n,0);rc_n++;rc_f=1;}
		else{Pluck_angle(360/pattern*rc_n,0);}
        break;
    }
    // 发送电流值给电机
//    CAN_Shoot_SendCurrent();
}
void RC_Spin(void)
{
	pitch_angle_set -= map(rc.ch1,-660,660,-0.2,0.2);
	if(pitch_angle_set>=pitch_angle_max)pitch_angle_set= pitch_angle_max;
	else if(pitch_angle_set<=pitch_angle_min)pitch_angle_set= pitch_angle_min;
    spin_angle_set += map(rc.ch0,-660,660,-1,1);
	CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020[1],spin_angle_set,imu.yaw);
//    CAN_GM6020[1].set_voltage = PID_Calculate(&PID_GM6020[1],spin_angle_set,map((float)CAN_GM6020[1].angle,0,8191,0,360));	
	CAN_GM6020[0].set_voltage = PID_Calculate(&PID_GM6020[0],pitch_angle_set,map((float)CAN_GM6020[0].angle,0,8191,0,360));
//	Gyro_mobile_init();
	if(rc.sw1==1)
	Gyro_mobile(-1.088736,400,2000);
	else
	{
		if(rc.ch1!=0)
		{
			pitch_angle_set = -map(rc.ch1,-660,660,-4000,4000);
			for(int i=0; i<4; i++) 	//对M3508的操作
			{
				CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], pitch_angle_set, CAN_M3508[i].speed);
			}
		}
		else if(rc.ch2!=0||rc.ch3!=0) {
//        velocity(rc.ch2,rc.ch3);
		velocity(0,-rc.ch3);
        for(int i=0; i<4; i++) 	//对M3508的操作
        {
            setSpeed[i] = map(setSpeed[i],-660,660,-20000,20000);
            CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
        }
    }
	else{	
		if(rc.sw1==2)M3508_follow(&PID_M3508_Follow,0);
		else
		{
			CAN_M3508[0].set_current = 0;
			CAN_M3508[1].set_current = 0;
			CAN_M3508[2].set_current = 0;
			CAN_M3508[3].set_current = 0;
		}
	}
}
	CAN_GM6020_Gimbal_SendVoltage();
	CAN_Chassis_SendCurrent2();
	CAN_M3508_Chassis_SendCurrent();
//	M3508_follow(&PID_M3508_Follow,60);
}
void RC_Vision_aiming()
{
	Vision_aiming();
//	RC_Shoot(fri_speed,dial_speed);
	if(rc.ch2!=0||rc.ch3!=0) {
    velocity(rc.ch2,rc.ch3);
    for(int i=0; i<4; i++) 	//对M3508的操作
    {
        setSpeed[i] = map(setSpeed[i],-660,660,-4000,4000);
        CAN_M3508[i].set_current =  PID_Calculate(&PID_M3508[i], setSpeed[i], CAN_M3508[i].speed);
       }
    }
	else	
//		M3508_follow(&PID_M3508_Follow,0);
	{
	    CAN_M3508[0].set_current = 0;
	    CAN_M3508[1].set_current = 0;
	    CAN_M3508[2].set_current = 0;
	    CAN_M3508[3].set_current = 0;

	}
	  CAN_GM6020_Gimbal_SendVoltage();
		CAN_Chassis_SendCurrent2();
		CAN_M3508_Chassis_SendCurrent();
}


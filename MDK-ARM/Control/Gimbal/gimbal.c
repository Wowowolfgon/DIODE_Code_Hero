#include "gimbal.h"
#include "bsp_led.h"
#define num 10
float spin_angle_set = 0/*154*/,pitch_angle_set=300,gyro_angle_set=0;
uint8_t Vision_receiving = 0;
uint8_t Vision_buffer[num] = {0}; 
uint8_t Vision_buffercnt = 0;
uint8_t tmp_vision;
VisionData datadata;
uint8_t Tdata[num];
uint8_t aTxVisionMessages[22];
extern PID_TypeDef PID_GM6020[];
//extern float Gyro[3],Accel[3];

//void VisionUartRxCpltCallback()
//{
//	if(Vision_receiving){
//		Vision_buffer[Vision_buffercnt] = tmp_vision;
//		Vision_buffercnt++;
//		if(Vision_buffercnt >=22){
//			for(int i=0; i<4; i++) {
//                datadata.pitch_angle.c[i]=Vision_buffer[i+3];
//                datadata.yaw_angle.c[i]=Vision_buffer[i+7];
//                datadata.dis.c[i]=Vision_buffer[i+11];
//			}
//			Vision_buffercnt = 0,Vision_receiving=0;
//			datadata.isFindTarget=Vision_buffer[16];
//		}
//	}
//	else{
//		if(tmp_vision == 0xA5){
//			Vision_receiving = 1;
//			Vision_buffercnt = 0;
//			Vision_buffer[0] = tmp_vision;
//			Vision_buffercnt++;
//		}
//	}

//	HAL_UART_Receive_DMA(&huart1, &tmp_vision, 1);
////	HAL_UART_Receive_DMA(&huart1, "121212", 10);
////	if(HAL_UART_Receive_DMA(&huart6, &tmp_vision, 1) != HAL_OK){
////		Error_Handler();
////	}
//}
void VisionUartRxCpltCallback_2()
{
//	printf("%s","11");
	if(Vision_receiving){
		Vision_buffer[Vision_buffercnt] = tmp_vision;
		Vision_buffercnt++;
		if(tmp_vision == 0x65){
			if(Vision_buffer[1]>>7)
				datadata.pitch_angle=	-(100-(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599);
			else
				datadata.pitch_angle=	(((Vision_buffer[1]<<8)| Vision_buffer[2])&0x7fff)*0.003051850947599;
			if(Vision_buffer[3]>>7)
				datadata.yaw_angle=  	-(100-(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599);
			else
				datadata.yaw_angle=		(((Vision_buffer[3]<<8)| Vision_buffer[4])&0x7fff)*0.003051850947599;
			if(Vision_buffer[5]>>7)
			datadata.dis=	   			-(100-(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599);
			else
				datadata.dis=	   		(((Vision_buffer[5]<<8)| Vision_buffer[6])&0x7fff)*0.003051850947599;
			Vision_buffercnt = 0,Vision_receiving=0;
			//HAL_UART_Receive_IT(&huart6, (uint8_t *)aRxBuffer, 1);
			
			printf("%f %f %f \n",datadata.pitch_angle,datadata.yaw_angle,datadata.dis);
			
		}
	}
	else{
		if(tmp_vision == 0x73){

			Vision_receiving = 1;
			Vision_buffercnt = 0;
			Vision_buffer[0] = tmp_vision;
			Vision_buffercnt++;
		}
	}

	HAL_UART_Receive_DMA(&huart1, &tmp_vision, 1);
//	if(HAL_UART_Receive_DMA(&huart6, &tmp_vision, 1) != HAL_OK){
//		Error_Handler();
//	}
}
//void VisionUartRxCpltCallback_2()
//{
//static int CNT=0;if (CNT >= 10){CNT = 0;LED_R_Toggle();}CNT++;
//	if(Vision_receiving){
//		Vision_buffer[Vision_buffercnt] = tmp_vision;
//		Vision_buffercnt++;
//		if(Vision_buffercnt >=num){
//			datadata.pitch_angle.f=((Vision_buffer[1]<<8)| Vision_buffer[2]);
////			datadata.yaw_angle.f=  ((Vision_buffer[3]<<8)| Vision_buffer[4])*10/(32768-1);
////			datadata.z_angle.f=	   ((Vision_buffer[5]<<8)| Vision_buffer[6])*10/(32768-1);
////			datadata.dis.f=		   ((Vision_buffer[7]<<8)| Vision_buffer[8])*10/(32768-1);
////			datadata.pitch_angle.c[0]=Vision_buffer[1]<<8;
////			datadata.pitch_angle.c[1]=Vision_buffer[2];
//			Vision_buffercnt = 0,Vision_receiving=0;
////			LED_R_On();
//			
//		}
//	}
//	else{
//		if(tmp_vision == 0x73){
//			Vision_receiving = 1;
//			Vision_buffercnt = 0;
//			Vision_buffer[0] = tmp_vision;
//			Vision_buffercnt++;
//			}
//		}

//	

//	HAL_UART_Receive_DMA(&huart1, &tmp_vision, 1);
////	HAL_UART_Receive_DMA(&huart1, "121212", 10);
////	if(HAL_UART_Receive_DMA(&huart6, &tmp_vision, 1) != HAL_OK){
////		Error_Handler();
////	}
//}
//void VisionUartTxSenddata()
//{
////	datadata.pitch_angle.f=10.22;
////	datadata.yaw_angle.f=7.98;
//	
//	aTxVisionMessages[0] = 0xA5;
////    aTxVisionMessages[1] = CmdID1;
////    crcToUse.Append_CRC8_Check_Sum(aTxVisionMessages, 3);
//     
//    aTxVisionMessages[3] = datadata.pitch_angle.c[0];
//    aTxVisionMessages[4] = datadata.pitch_angle.c[1];
//    aTxVisionMessages[5] = datadata.pitch_angle.c[2];
//    aTxVisionMessages[6] = datadata.pitch_angle.c[3];

//    aTxVisionMessages[7] = datadata.yaw_angle.c[0];
//    aTxVisionMessages[8] = datadata.yaw_angle.c[1];
//    aTxVisionMessages[9] = datadata.yaw_angle.c[2];
//    aTxVisionMessages[10] = datadata.yaw_angle.c[3];

//    aTxVisionMessages[11] = datadata.dis.c[0];
//    aTxVisionMessages[12] = datadata.dis.c[1];
//    aTxVisionMessages[13] = datadata.dis.c[2];
//    aTxVisionMessages[14] = datadata.dis.c[3];

//    aTxVisionMessages[15] = datadata.ismiddle;
//    aTxVisionMessages[16] = datadata.isFindTarget;

//    aTxVisionMessages[17] = datadata.isfindDafu;
//    aTxVisionMessages[18] = 0x00;
//    aTxVisionMessages[19] = datadata.nearFace;
////	for(int i=0;i<sizeof(aTxVisionMessages);i++)
////	{
////		aTxVisionMessages[i]=0;
////	}
//	HAL_UART_Transmit_DMA(&huart1 ,(uint8_t*)aTxVisionMessages,sizeof(aTxVisionMessages));
////	HAL_UART_Transmit_DMA(&huart1 ,Gyro,sizeof(Gyro));
////    crcToUse.Append_CRC16_Check_Sum(Tdata, 22);
////    write(fd, Tdata, 22);
//}
void  Vision_aiming()
{
//	if(datadata.dis.f>0){
//		if(datadata.yaw_angle.f<15&&datadata.yaw_angle.f>-15&&datadata.pitch_angle.f<20&&datadata.pitch_angle.f>-20){
	
	
		if(datadata.yaw_angle!=0&&datadata.yaw_angle!=0){
			pitch_angle_set=map((float)CAN_GM6020[0].angle,0,8191,0,360);
			spin_angle_set=map((float)CAN_GM6020[1].angle,0,8191,0,360);
			CAN_GM6020[0].set_voltage = -PID_Calculate(&PID_GM6020[0],pitch_angle_set+datadata.yaw_angle/8,pitch_angle_set);
			CAN_GM6020[1].set_voltage = -PID_Calculate(&PID_GM6020[1],spin_angle_set-datadata.pitch_angle/4,spin_angle_set);
		}
		else
		{
			
		}
}
void Pluck_angle(float angle,int m)
{
	float n;
    n=PID_Calculate(&PID_M2006_ANGLE[0],angle,CAN_M2006[m].total_angle);
	CAN_M2006[m].set_current = PID_Calculate(&PID_M2006[0],n,CAN_M2006[m].speed);
//	CAN_M2006.total_angle=0;
}

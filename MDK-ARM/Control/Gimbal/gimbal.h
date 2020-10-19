#ifndef __GIMBAL_H
#define __GIMBAL_H
#include "main.h"

#define angle8191_to_angle2pi(a) ((a)/8191.0*360)
#define yaw_to_2pi(a)	((a)+180)

#define pitch_angle_min	276 //5984
#define pitch_angle_max	322//319 //7264


typedef struct
{
	float yaw_angle;//ƫ����
	float pitch_angle;//������
	float shoot_delay;
	float dis;//Ŀ�����
    int ismiddle;//����1��ʾĿ������˿��Կ���ķ�Χ������0���ʾĿ����δ����ɿ���ķ�Χ��Ŀǰ�ݲ�ʹ�ã�Ĭ����0
	int isFindTarget;//��ʶ���ͼƬ��Χ����Ŀ���ҵ�ط������źŲ�Ϊ0x00���ر��Ӿ�����Ϊ1��������0
    int isfindDafu;
    int nearFace;
} VisionData;
typedef union
{
    float f;
    unsigned char c[2];
} float2uchar;

////���ڱ���Ŀ����ؽǶȺ;�����Ϣ����׼���
//typedef struct
//{
//	float2uchar yaw_angle;//ƫ����
//	float2uchar pitch_angle;//������
//	float2uchar z_angle;
//	float2uchar dis;//Ŀ�����
//    int ismiddle;//����1��ʾĿ������˿��Կ���ķ�Χ������0���ʾĿ����δ����ɿ���ķ�Χ��Ŀǰ�ݲ�ʹ�ã�Ĭ����0
//	int isFindTarget;//��ʶ���ͼƬ��Χ����Ŀ���ҵ�ط������źŲ�Ϊ0x00���ر��Ӿ�����Ϊ1��������0
//    int isfindDafu;
//    int nearFace;
//} VisionData;

extern float spin_angle_set;
extern float pitch_angle_set;
void VisionUartTxSenddata();
void VisionUartRxCpltCallback_2();
void VisionUartRxCpltCallback();
void  Vision_aiming();
void Pluck_angle(float angle,int m);
#endif


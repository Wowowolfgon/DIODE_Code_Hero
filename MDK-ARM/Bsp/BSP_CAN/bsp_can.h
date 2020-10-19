#ifndef __BSP_CAN
#define __BSP_CAN
#include "main.h"
#include "can.h"

/* �������ID ����*/
#define	CAN1_M3508_ID1 0x201	// ���̵��
#define	CAN1_M3508_ID2 0x202	// ���̵��
#define	CAN1_M3508_ID3 0x203	// ���̵��
#define	CAN1_M3508_ID4 0x204	// ���̵��
#define	CAN1_M3508_ID5 0x206	// Ħ���ֵ�� ��
#define	CAN1_M3508_ID6 0x207	// Ħ���ֵ�� ��
#define CAN1_M2006_ID7 0x207	// ���̵��
#define CAN1_M2006_ID8 0x208	// ���̵��

#define CAN2_GM6020_ID1 0x205	// ��̨�ײ����
#define CAN2_GM6020_ID2 0x206	// ��̨������

#define FWAngle2Angle(x) ((x)/8191.0*360)
#define Angle2FWAngle(x) ((x)/360.0*8191)

/* ����3508״̬�ṹ�壬�洢��ǰ3508�ķ���ֵ*/
typedef struct {
    int16_t  	set_current;	// д��ĵ���
    uint16_t 	angle;			// �Ƕ�
	int16_t  	last_angle;
	float 		total_angle;
    int16_t 	speed;			// �ٶ�
    int16_t		current;		// ����
    uint8_t 	temperature;	// �¶�
} CAN_M3508_TypeDef;


/* ����6020״̬�ṹ�壬�洢��ǰ6020�ķ���ֵ*/
typedef struct
{
    int16_t  set_voltage;	// д��ĵ�ѹ
    int16_t angle;			// �Ƕ�
	int16_t  	last_angle;
	float 		total_angle;
    int16_t  speed;         // �ٶ�
    int16_t  current;       // ����
    int8_t  temperature;   // �¶�
	
} CAN_GM6020_TypeDef;

/* ����2006״̬�ṹ�壬�洢��ǰ6020�ķ���ֵ*/
typedef struct
{
    int16_t  set_current;	// д��ĵ���
    uint16_t angle;			// �Ƕ�
	int16_t  	last_angle;
	float 		total_angle;
    int16_t  speed;         // �ٶ�
    int16_t  current;       // ����
    uint8_t  temperature;   // �¶�
} CAN_M2006_TypeDef;

extern CAN_GM6020_TypeDef 	CAN_GM6020[2];	// ��̨���
extern CAN_M2006_TypeDef 	CAN_M2006[2]	;	// ���̵��
extern CAN_M3508_TypeDef 	CAN_M3508[6];	// ǰ�ĸ��ǵ��̵�� ��������Ħ���ֵ��

void CAN_FilterInit(CAN_HandleTypeDef* hcan);
uint8_t CAN_SendMsg(CAN_HandleTypeDef* hcan,uint8_t ide,uint32_t id,uint8_t len,uint8_t *data);

void CAN_M3508_SetCurrent(int16_t i1,int16_t i2,int16_t i3,int16_t i4);
void get_total_angle_2006(CAN_M2006_TypeDef *p);
void get_total_angle_6020(CAN_GM6020_TypeDef *p);

void CAN_M3508_Chassis_SendCurrent(void);
void CAN_GM6020_Gimbal_SendVoltage(void);
void CAN_Chassis_SendCurrent2(void);
#endif



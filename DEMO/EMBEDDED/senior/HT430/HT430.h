#ifndef __HT430_H
#define __HT430_H
#include "public.h"


/**********************************HT430**********************************/
typedef enum
{
    OFF_STATE=0,
    OPEN_LOOP=1,
    SPEED_MODE=3,
    ANGLE_MODE=5,
} Operating_State_t;
typedef struct{
	uint16_t Angle;//��Ȧ����ֵ�Ƕ�
	int32_t Total_Angle;//��Ȧ����ֵ�Ƕ�
	int16_t V;//���ת��
	Operating_State_t Operating_State;//����״̬
	uint8_t Voltage;//��Դ��ѹ
	uint8_t Currents;//����
	uint8_t Temperature;//�¶�
	uint8_t DTC;//������
}HT430_J10_t;

#ifndef STRUCT_MOTOR
#define STRUCT_MOTOR

#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	double ecd_angle;											//�Ƕ�
	u32 temperature;
	int16_t rate_rpm;
	
}Encoder;




#endif




void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v);


void HT_430_Encoder_Calibration(CAN_TypeDef *CANx,int ID);
void HT_430_Encoder_Origin(CAN_TypeDef *CANx,int ID);
void Motor_Information_Request(CAN_TypeDef *CANx,int ID);
void HT_430_Fault_Clear(CAN_TypeDef *CANx,int ID);
void HT_430_Tuen_Off(CAN_TypeDef *CANx,int ID);
void HT_430_Origin_Total(CAN_TypeDef *CANx,int ID);
void HT_430_Back(CAN_TypeDef *CANx,int ID);
void HT_430_Power_Open_Loop(CAN_TypeDef *CANx,int ID,int16_t Pow);
void HT_430_V_Clossed_Loop(CAN_TypeDef *CANx,int ID,int16_t V);
void HT_430_Absolute_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle);
void HT_430_Relative_Position_closed_Loop(CAN_TypeDef *CANx,int ID,uint32_t Angle);
void HT_430_Position_closed_Loop_T_R_OR_W(CAN_TypeDef *CANx,int ID,int16_t V,int Flag_RW);

















#endif

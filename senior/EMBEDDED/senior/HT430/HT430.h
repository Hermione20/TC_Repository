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


















#endif

#ifndef _CAN_BUS_TASK_H_
#define _CAN_BUS_TASK_H_

#include "main.h"

/* CAN Bus 1 */  


#define CAN_BUS1_POKE_FEEDBACK_MSG_ID             0x207//pitch
#define CAN_BUS1_MOTOR1_FEEDBACK_MSG_ID           0x201
#define CAN_BUS1_MOTOR2_FEEDBACK_MSG_ID           0x202


/* CAN Bus 2 */  
#define CAN_BUS2_MOTOR1_FEEDBACK_MSG_ID           0x201//��ǰ ��ʱ��
#define CAN_BUS2_MOTOR2_FEEDBACK_MSG_ID           0x202 
#define CAN_BUS2_MOTOR3_FEEDBACK_MSG_ID           0x203
#define CAN_BUS2_MOTOR4_FEEDBACK_MSG_ID           0x204
#define CAN_BUS2_MOTOR5_FEEDBACK_MSG_ID           0x209
#define CAN_BUS2_MOTOR6_FEEDBACK_MSG_ID           0x20A

#define CAN_BUS2_MOTOR7_FEEDBACK_MSG_ID           0x205
#define CAN_BUS2_MOTOR8_FEEDBACK_MSG_ID           0x206
#define CAN_BUS2_MOTOR9_FEEDBACK_MSG_ID           0x207
#define CAN_BUS2_MOTOR10_FEEDBACK_MSG_ID           0x208

#define CAN_BUS1_POWER_FEEDBACK_MSG_ID            0x407
#define CAN_BUS2_DISTANCE_FEEDBACK_MSG_ID3         0x406
#define CAN_BUS2_DISTANCE_FEEDBACK_MSG_ID2         0x408


#define CAN_BUS2_DISTANCE_FEEDBACK_MSG_ID         0x409
#define VOLTAGE_BUF_SIZE  50
typedef struct
{
	uint8_t  charge_power;                       //��繦��/W
	char     charge_current;                     //������/A
	uint16_t  cap_voltage_filte;                        //���ݵ�ѹ/V
	uint8_t  boost_voltage;                      //boost��ѹ/V
	uint8_t  battery_voltage;                    //��ص�ѹ/V
	uint8_t  output_voltage;                     //�����ѹ/V
  uint8_t  cap_voltage_buff[VOLTAGE_BUF_SIZE];
	uint8_t  raw_cap_voltage;              //δ���˲��ĵ��ݵ�ѹ
	uint8_t  buf_count;

}capacitance_message1_t;

typedef struct
{
	uint8_t  max_charge_power;                   //����繦��/W      0-100
	char     max_charge_current;                 //��������/A       0-10
	uint8_t  boost_voltage_ref;                  //boost��ѹ����ֵ/V     20-30
	uint8_t  output_mode_set;                    //���õ����ģʽ  0��������;1��������ѹ���
	uint8_t  output_mode_get;                    //��ǰ���ģʽ    0��������;1��������ѹ���
	
	
	union
	{
		u8 fault;
		uint8_t  charge_over_current_flag:1;           //������
		uint8_t  cap_over_voltage_flag:1;              //���ݹ�ѹ
		uint8_t  battery_over_under_voltage_flag:1;    //���Ƿѹ���ѹ
		uint8_t  battery_off_flag:1;                   //��ص���,��⵽��ضϵ�
		uint8_t  two_leg_over_current_flag:1;          //���ű��¹ܹ���,�ù���һ��Ϊmos�𻵵����ű�ֱͨ
		uint8_t  output_change_switch_flag:1;          //�����Դ�л�����ֱͨ
		uint8_t  boost_over_voltage_flag:1;            //boost��ѹ��ѹ
		uint8_t  boost_over_current_flag:1;            //boost��ѹ����
	}
	fault_union;	//0���޹���  1������

	
	uint8_t  system_mode;                        //ϵͳ����ģʽ  0��ϵͳ��ʼ�� 1���ȴ�ģʽ 2���������� 3��ϵͳ���� 4��ϵͳ�ػ� 5��ϵͳ��λ

}capacitance_message2_t;

typedef struct
{

	uint16_t mode;
	uint16_t mode_sure;
	
	uint16_t in_power;
	uint16_t in_v;
	uint16_t in_i;
	
	uint16_t out_power;
	uint16_t out_v;
	uint16_t out_i;

	uint16_t tempureture;
	uint16_t time;
	uint16_t this_time;
	
}capacitance_message3_t;


typedef struct
{
	double front_left_distance;
	double front_right_distance;
	double right_distance;
	int16_t  front_left_right_bias;
	

	
	
}distance_message_t;

extern volatile Encoder CM1Encoder;//3508
extern volatile Encoder CM2Encoder;
extern volatile Encoder CM3Encoder;
extern volatile Encoder CM4Encoder;
extern volatile Encoder GMYawEncoder;
extern volatile Encoder GMPitchEncoder;
extern volatile Encoder GM1Encoder1;//6020
extern volatile Encoder GM2Encoder1;
extern volatile Encoder GM3Encoder1;
extern volatile Encoder GM4Encoder1;
extern volatile Encoder BUS1_CM1Encoder;
extern volatile Encoder BUS1_CM2Encoder;
extern volatile Encoder PokeEncoder;
extern float ZGyroModuleAngle;
extern u8 Fault;
extern u8 Sys_Mode;
extern u8 Out_Mode;
extern u8 Charge_State;
extern u8  not_receive_time;
extern float I_Icharge;
extern float Vin;
extern float V_Cap;
extern float speed_yaw;

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void PitchEncoderProcess(volatile Encoder *v, CanRxMsg * msg);

void CapacitanceProcess1(volatile capacitance_message1_t *v,CanRxMsg * msg);
void CapacitanceProcess2(volatile capacitance_message2_t *v,CanRxMsg * msg);

void capacitance_message(CAN_TypeDef *CANx, int16_t cap_voltage_filte);

void POWER_Control1(uint16_t Power,uint16_t StdId);
void POWER_Control1l(uint16_t StdId);

void Can2ReceiveMsgProcess(CanRxMsg * msg);
void Can1ReceiveMsgProcess(CanRxMsg * msg);
void Set_Gimbal_Current(CAN_TypeDef *CANx, int16_t gimbal_pitch_iq, int16_t gimbal_yaw_iq);
void Set_Gimbal_Current1(CAN_TypeDef *CANx, int16_t ch_1_iq, int16_t ch_2_iq, int16_t ch_3_iq, int16_t ch_4_iq);
void Set_CM_Speed(CAN_TypeDef *CANx, int16_t cm1_iq, int16_t cm2_iq, int16_t cm3_iq, int16_t cm4_iq);

#endif


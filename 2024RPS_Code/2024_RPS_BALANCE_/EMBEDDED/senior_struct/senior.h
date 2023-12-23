#ifndef __SENIOR_H
#define __SENIOR_H
#include "public.h"


#define  GMPitchEncoder_Offset 6165
//yaw������ʼλ��
#define  GMYawEncoder_Offset   6013
//���̺���������ʼλ��
#define  GM1Encoder_Offset   1437
#define  GM2Encoder_Offset   8042
#define  GM3Encoder_Offset   4141
#define  GM4Encoder_Offset   6732
//ƽ�����̵����ʼλ��
#define  JM1Encoder_Offset   62945
#define  JM2Encoder_Offset   28467
#define  JM3Encoder_Offset   8821
#define  JM4Encoder_Offset   35481

#define  TM1Encoder_Offset   0
#define  TM2Encoder_Offset   0

#ifndef STRUCT_MOTOR
#define STRUCT_MOTOR

#define RATE_BUF_SIZE 6
typedef struct 
{
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
	int32_t can_cnt;					//��¼������ʹ�ô������ڵ����ʼ��ɲ�������
}Encoder_cal;


typedef struct{
	Encoder_cal cal_data;


	int32_t filter_rate;											//�ٶ�
	double ecd_angle;											//�Ƕ�
	int16_t rate_rpm;

	double angle;
	double gyro;

	float Torque;
	u32 temperature;
	
	
}Encoder;
	





#endif

#ifndef GENERAL_GYRO_T
/**************************general gyro*********************************/
#define GENERAL_GYRO_T
typedef struct 
{
	float pitch_Angle;
	float yaw_Angle;
	float roll_Angle;
	float pitch_Gyro;
	float yaw_Gyro;
	float roll_Gyro;
	float x_Acc;
	float y_Acc;
	float z_Acc;
}general_gyro_t;



#endif

//���µ��̽ṹ������鷽λ��ʾΪ���ޱ�ʾ

/********************general chassis encoder********************************/

//��������
//�����м�ṹ��
typedef struct 
{
	volatile Encoder Heading_Encoder[4];

	volatile Encoder Driving_Encoder[4];
	
}steering_wheel_t;

typedef struct
{
	volatile Encoder Driving_Encoder[4];
}Mecanum_wheel_t;

typedef struct
{
	volatile Encoder joint_Encoder[4];

	volatile Encoder Driving_Encoder[2];//��0��1
}balance_t;

/***************************general friction encoder********************************************/
typedef struct 
{
	volatile Encoder right_up_motor;
	volatile Encoder left_up_motor;
	volatile Encoder left_down_motor;
	volatile Encoder right_down_motor;

	volatile Encoder left_motor;
	volatile Encoder right_motor;
}friction_t;

/************************************general poke encoder******************************************************/

typedef struct 
{
	//�ڱ�
	volatile Encoder right_poke;
	volatile Encoder left_poke;
	//Ӣ��
	volatile Encoder up_poke;
	volatile Encoder down_poke;
	//����
	volatile Encoder poke;
}poke_t;

/****************************************hero small gimbal encoder*****************************************************************/
typedef struct 
{
	volatile Encoder scope_encoder;
	volatile Encoder small_gimbal_encoder;
}hero_small_gimbal_t;







/**************general_gyro define**********************/
extern general_gyro_t gimbal_gyro;
extern general_gyro_t chassis_gyro;
extern steering_wheel_t steering_wheel_chassis;
extern Mecanum_wheel_t Mecanum_chassis;
extern balance_t balance_chassis;
extern volatile Encoder Pitch_Encoder;
extern volatile Encoder yaw_Encoder;
extern hero_small_gimbal_t hero_small_gimbal;
extern friction_t general_friction;
extern poke_t general_poke;



#endif


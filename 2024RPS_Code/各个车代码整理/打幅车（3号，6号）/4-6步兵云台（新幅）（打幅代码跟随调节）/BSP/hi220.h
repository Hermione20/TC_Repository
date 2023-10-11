#ifndef __HI220_H
#define __HI220_H
#include "STM32F4xx.h"

#define LENGTH_USER_ID_0x90 			1
#define LENGTH_ACC_0xa0 					6
#define LENGTH_LINEAR_ACC_0xa5 		6
#define LENGTH_ANG_VEL_0xb0 			6
#define LENGTH_MAG_0xc0 					6
#define LENGTH_EULER_ANG_s16_0xd0 6
#define LENGTH_EULER_ANG_f_0xd9 	12
#define LENGTH_QUATERNION_0xd1 		16
#define LENGTH_AIR_PRESS_0xf0 		4
#define USART6_RX_BUF_LENGTH 100

typedef struct {
					u8 User_ID;
					s16 Acc_X;
					s16 Acc_Y;
					s16 Acc_Z;
					s16 Linear_Acc_X;
					s16 Linear_Acc_Y;
					s16 Linear_Acc_Z;
					s16 Ang_Velocity_X;
					s16 Ang_Velocity_Y;
					s16 Ang_Velocity_Z;
					s16 Mag_X;
					s16 Mag_Y;
					s16 Mag_Z;
					s16 Euler_Angle_Pitch_s16;
					s16 Euler_Angle_Roll_s16;
					s16 Euler_Angle_Yaw_s16;
					
					union
					{
						float Euler_Angle_Pitch_f;
						u8 Euler_Angle_Pitch_u8[4];
					}Euler_Angle_Pitch;
					
					union
					{
						float Euler_Angle_Roll_f;
						u8 Euler_Angle_Roll_u8[4];
					}Euler_Angle_Roll;

					union
					{
						float Euler_Angle_Yaw_f;
						u8 Euler_Angle_Yaw_u8[4];
					}Euler_Angle_Yaw;
					
					union
					{
						float Quaternion_W_f;
						u8 Quaternion_W_u8[4];
					}Quaternion_W;		

					union
					{
						float Quaternion_X_f;
						u8 Quaternion_X_u8[4];
					}Quaternion_X;	
						union
					{
						float Quaternion_Y_f;
						u8 Quaternion_Y_u8[4];
					}Quaternion_Y;	
						union
					{
						float Quaternion_Z_f;
						u8 Quaternion_Z_u8[4];
					}Quaternion_Z;	
					float Euler_Angle_Pitch_s16_2_f;
					float Euler_Angle_Roll_s16_2_f;
					float Euler_Angle_Yaw_s16_2_f;	
} HI220_Stucture;
typedef struct
{
	union
	{
		u8 Flag_Configured;
		struct
		{
			u8 Eout_Configured 	: 1;
			u8 ODR_Configured 	: 1;
			u8 BAUD_Configured 	: 1;
			u8 SETPTL_Configured: 1;
			u8 MODE_Configured 	: 1;
			u8 MCAL_Configured 	: 1;
			u8 Reserve 					: 2;
			
		}Bits;
	}Hi220_Flag_Configured;
	union
	{
		u8 Flag_Reconfig;
		struct
		{
			u8 Eout_Reconfig 	: 1;
			u8 ODR_Reconfig 	: 1;
			u8 BAUD_Reconfig 	: 1;
			u8 SETPTL_Reconfig: 1;
			u8 MODE_Reconfig 	: 1;
			u8 MCAL_Reconfig 	: 1;
			u8 Reserve 				: 2;
			
		}Bits;
	}Hi220_Flag_Reconfig;
}Hi220_Flags_t;

extern HI220_Stucture HI220_Data_From_Usart;
	
void Hi220_Init();
void Hi220_Reset();
void USART6_Configuration_For_Hi220();
void Hi220_getYawPitchRoll();
#endif



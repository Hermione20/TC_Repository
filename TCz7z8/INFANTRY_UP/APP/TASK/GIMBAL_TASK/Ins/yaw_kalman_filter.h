#ifndef __YAW_KALMAN_FILTER_H
#define __YAW_KALMAN_FILTER_H
#include "public.h"

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32
#define mat_copy    arm_copy_f32
#define dt  0.01
typedef struct
{
	float yaw_angle;  //滤波后yaw轴角度
	float yaw_speed;	//滤波后yaw轴角速度
  float pitch_angle;//滤波后pitch轴角度
	float pitch_speed;//滤波后pitch轴角速度
	
	mat Xhat,//X的预测
			A,//状态预测
			G,//输入矩阵
			Z,//观测矩阵
			Q,//系统误差协方差
			R,//测量噪声协方差
			H,//转移矩阵
			P,//协方差矩阵
			K,//卡尔曼增益矩阵
			I,//单位矩阵
			AT,//A阵转置
			HT;//H阵转置
	
	float Xhat_data[4],
				A_data[16],
				G_data[4],
				Z_data[4],
				Q_data[16],
				R_data[16],
				H_data[16],
				P_data[16],
				K_data[16],
				I_data[16],
				AT_data[16],
				HT_data[16];
}yaw_kalman_filter_t;
extern yaw_kalman_filter_t yaw_kalman_filter;
void yaw_kalman_filter_init(yaw_kalman_filter_t *Y);
void yaw_kalman_filter_calc(yaw_kalman_filter_t *Y,float yaw_angle0,float pitch_angle0,float yaw_speed0,float pitch_speed0,float yaw_angle1,float pitch_angle1,float yaw_speed1,float pitch_speed1);


#endif

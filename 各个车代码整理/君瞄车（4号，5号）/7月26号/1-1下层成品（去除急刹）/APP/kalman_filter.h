/*
*********************************************************************************************************
* Copyright: 
* File namde:
* Author: LQ Version: V1.0 Date: 20190503
* Description:
* Others:
* Function List:
* History:
*********************************************************************************************************
*/
#ifndef __kalman_filter_H__
#define __kalman_filter_H__
#include "stm32f4xx.h"
#include "arm_math.h"

#define mat         arm_matrix_instance_f32 
#define mat_init    arm_mat_init_f32
#define mat_add     arm_mat_add_f32
#define mat_sub     arm_mat_sub_f32
#define mat_mult    arm_mat_mult_f32
#define mat_trans   arm_mat_trans_f32
#define mat_inv     arm_mat_inverse_f32


#define Ts_Filter 2e-3


typedef struct
{
  float raw_value;
  float filtered_value[4];	//�˲�������ֵ���ֱ��Ӧx,y,vx,vy
  mat xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_t;

typedef struct
{
  float xhat_data[4];				//�м���
	float	xhatminus_data[4];	//�м���
	float	z_data[4];					//������
	float	Pminus_data[16];		//�м���
	float	K_data[16];					//�м���
  float P_data[16];					//�м���
  float AT_data[16], HT_data[16];	//������
  float A_data[16];					//������
  float H_data[16];					//������
  float Q_data[16];					//������
  float R_data[16];					//������ 
} kalman_filter_init_t;



float *kalman_filter_calc(kalman_filter_t *F, float x_in, float y_in, float vx_in, float vy_in);
void kalman_filter_init(kalman_filter_t *F, kalman_filter_init_t *I);
void kalman_filter_reset(kalman_filter_t *F, kalman_filter_init_t *I);

extern kalman_filter_t kalman_filter_F;
extern kalman_filter_init_t kalman_filter_I;


typedef struct
{
  float raw_value;
  float filtered_value;
  float xhat, xhatminus, z, A, H, AT, HT, Q, R, P, Pminus, K;
} kalman_filter_1t;

float kalman_filter_calc_1(kalman_filter_1t *F, float signal1);


#endif
/*
*********************************************************************************************************
*                                                  No More
*********************************************************************************************************
*/

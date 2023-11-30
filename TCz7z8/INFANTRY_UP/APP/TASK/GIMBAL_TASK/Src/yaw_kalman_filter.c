#include "yaw_kalman_filter.h"


float gTEMP41_data[4] = {0};float gTEMP2_data[16] = {0};float gTEMP24_data[16] = {0};float gTEMP_22_data[16] = {0};float gTEMP__2_data[4] = {0};
float gTEMP_data[16] = {0};float gTEMP42_data[16] =  {0};float gTEMP22_data[16] = {0};float gTEMP_2_data[4] = {0};float gTEMP_41_data[4] = {0};


yaw_kalman_filter_t yaw_kalman_filter=
{
	.Q_data={ 20, 0, 0, 0,\
				     0,20, 0, 0,\
						 0, 0,70, 0,\
						 0, 0, 0,70},
  .R_data={0.1,  0, 0,0,\
						 0,0.1, 0,0,\
						 0, 0,0.1,0,\
						 0, 0, 0,0.1},
	.A_data={  1, 0,dt, 0,\
						 0, 1, 0,dt,\
						 0, 0, 1, 0,\
						 0, 0, 0, 1},	
	.H_data={  1, 0, 0, 0,\
						 0, 1, 0, 0,\
						 0, 0, 1, 0,\
						 0, 0, 0, 1},
	.I_data={  1, 0, 0, 0,\
						 0, 1, 0, 0,\
						 0, 0, 1, 0,\
						 0, 0, 0, 1},
};


void yaw_kalman_filter_init(yaw_kalman_filter_t *Y)
{
	mat_init(&Y->Xhat,4,1,(float *)Y->Xhat_data);
	mat_init(&Y->Z,4,1, (float *)Y->Z_data);
	mat_init(&Y->A,4,4, (float *)Y->A_data);
	mat_init(&Y->AT,4,4,(float *)Y->AT_data);
	mat_trans(&Y->A,&Y->AT);
	mat_init(&Y->Q,4,4,(float *)Y->Q_data);
	mat_init(&Y->H,4,4,(float *)Y->H_data);
	mat_init(&Y->HT,4,4,(float *)Y->HT_data);
	mat_trans(&Y->H,&Y->HT);		
	mat_init(&Y->R,4,4,(float *)Y->R_data);
	mat_init(&Y->P,4,4,(float *)Y->P_data);
	mat_init(&Y->K,4,4,(float *)Y->K_data);
	mat_init(&Y->I,4,4,(float *)Y->I_data);
}

void yaw_kalman_filter_calc(yaw_kalman_filter_t *Y,float yaw_angle0,float pitch_angle0,float yaw_speed0,float pitch_speed0,float yaw_angle1,float pitch_angle1,float yaw_speed1,float pitch_speed1)
{
	static u8 state_kalman_init = 0;
	if(state_kalman_init==0)
	{
		state_kalman_init=1;
		yaw_kalman_filter_init(&yaw_kalman_filter);
	}
	mat gTEMP;//进行中间矩阵的初始化
	mat_init(&gTEMP,4,4,(float *)gTEMP_data);
	
	mat gTEMP2;
	mat_init(&gTEMP2,4,4,(float *)gTEMP2_data);
	
	mat gTEMP42;
	mat_init(&gTEMP42,4,4,(float *)gTEMP42_data);
	
	mat gTEMP24;
	mat_init(&gTEMP24,4,4,(float *)gTEMP24_data);
	
	mat gTEMP22;
	mat_init(&gTEMP22,4,4,(float *)gTEMP22_data);
	
	mat gTEMP_22;
	mat_init(&gTEMP_22,4,4,(float *)gTEMP_22_data);
	
	mat gTEMP_2;
	mat_init(&gTEMP_2,4,1,(float *)gTEMP_2_data);
	
	mat gTEMP__2;
	mat_init(&gTEMP__2,4,1,(float *)gTEMP__2_data);
	
	mat gTEMP41;
	mat_init(&gTEMP41,4,1,(float *)gTEMP41_data);
	
	mat gTEMP_41;
	mat_init(&gTEMP_41,4,1,(float *)gTEMP_41_data);
	
	Y->Z.pData[0] = yaw_angle1;//观测值
	Y->Z.pData[1] = pitch_angle1;
	Y->Z.pData[2] = yaw_speed1;
	Y->Z.pData[3] = pitch_speed1;

	//1. xhat'(k)= A xhat(k-1) //后面考虑加入输入
	gTEMP41_data[0]=yaw_angle0;
	gTEMP41_data[1]=pitch_angle0;
	gTEMP41_data[2]=yaw_speed0;
	gTEMP41_data[3]=pitch_speed0;
	mat_copy(gTEMP41_data, Y->Xhat_data,4);
//	if(*buff_kf_flag)
	{
	//2. P'(k) = A P(k-1) AT + Q
	mat_mult(&Y->A, &Y->P,&gTEMP);
	mat_mult(&gTEMP,&Y->AT,&gTEMP2);
	mat_add(&gTEMP2,&Y->Q,&Y->P);
	
	//3. K(k) = P'(k) HT / (H P'(k) HT + R)
	mat_mult(&Y->P,&Y->HT,&gTEMP42);
	
	mat_mult(&Y->H,&Y->P,&gTEMP24);
	mat_mult(&gTEMP24,&Y->HT,&gTEMP22);
	mat_add(&gTEMP22,&Y->R,&gTEMP_22);
	mat_inv(&gTEMP_22,&gTEMP22);
	
	mat_mult(&gTEMP42,&gTEMP22,&Y->K);
	
	//4. xhat(k) = xhat'(k) + K(k) (z(k) - H xhat'(k))
	mat_mult(&Y->H,&Y->Xhat,&gTEMP_2);
	mat_sub(&Y->Z,&gTEMP_2,&gTEMP__2);
	mat_mult(&Y->K,&gTEMP__2,&gTEMP41);
	mat_add(&Y->Xhat,&gTEMP41,&gTEMP_41);
	mat_copy(gTEMP_41_data,Y->Xhat_data,4);
	//5. P(k) = (1-K(k)H)P'(k)
	mat_mult(&Y->K,&Y->H,&gTEMP);
	mat_sub(&Y->I,&gTEMP,&gTEMP2);
	mat_mult(&gTEMP2,&Y->P,&gTEMP);
	mat_copy(gTEMP_data,Y->P_data,16);
	
//	*buff_kf_flag = 0;
 }
	
	Y->yaw_angle = Y->Xhat.pData[0];
	Y->pitch_angle = Y->Xhat.pData[1];
	Y->yaw_speed = Y->Xhat.pData[2];
	Y->pitch_speed = Y->Xhat.pData[3];
}

	


#ifndef __fuzzypid_H__
#define __fuzzypid_H__

#include "stm32f4xx.h"
#include "public.h"


typedef struct fuzzypid_t
{
		float set;			//设定目标值
	
		float get;  //实际值

    float output_deadband;
		
		float LastError;		//前次误差
		float PreError;			//当前误差
		float SumError;			//积分误差
	
		float integral_limit;					//积分限制
		
		float pout;					//比例输出
		float iout;					//积分输出
		float dout;					//微分输出
	  float dout_last;    //上一次微分输出
		float max_out;       //限幅
	  float out;          //总输出
		float out_last;     //上一次输出
		
		float I_U;          //变速积分上限
		float I_L;          //变速积分下限
		
		float RC_DM;        //微分先行滤波系数
		float RC_DF;        //不完全微分滤波系数
	
	  float kp0;          //PID初值
	  float ki0;
  	float kd0;
	
	  float d_kp;          //PID变化量
	  float d_ki;
  	float d_kd;
	
    float stair ;	      //动态调整梯度   //0.25f
	  float Kp_stair;                      //0.015f
	  float Ki_stair;                      //0.0005f
	  float Kd_stair;                      //0.001f
	  
		void (*f_param_init)(struct fuzzypid_t*   fuzzypid,
                       float      max_output,
                       float      inte_limit,
                       float         			 p,
                       float         			 i,
                       float        	     d,
											 float           stair,
											 float        Kp_stair,
											 float 				Ki_stair,
											 float 				Kd_stair);
										 											 
		
}fuzzypid_t;

void FUZZYPID_struct_init(
    fuzzypid_t*   fuzzypid,
    float maxout,
    float intergral_limit,

    float kp0,
    float ki0,
    float kd0,
		float stair,
		float Kp_stair,
		float Ki_stair,
		float Kd_stair);

float FuzzyPID_Calc(fuzzypid_t *fuzzypid);

#endif
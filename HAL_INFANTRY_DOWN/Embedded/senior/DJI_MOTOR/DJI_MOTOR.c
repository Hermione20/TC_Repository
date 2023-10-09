#include "DJI_MOTOR.h"

uint8_t encoder_Data[8];
/**
  ******************************************************************************
  * @file    DJI_MOTOR.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写了DJI各型号电机的解算，包含c610电调
						 c620电调与GM6020电机的解算，入口参数包含通用
						 编码器结构体，can总线的计数，can结构体，（GM
						 6020还包含编码器初始值设定）
						 
@verbatim
 ===============================================================================
 **/
 
/*****************************dji encoder*************************************/

void GetEncoderBias(volatile Encoder *v, uint8_t *RxData)
{

            v->ecd_bias = (RxData[0]<<8)|RxData[1];  //保存初始编码器值作为偏差  
            v->ecd_value = v->ecd_bias;
            v->last_raw_value = v->ecd_bias;
            v->temp_count++;
}

void EncoderProcess(volatile Encoder *v,uint8_t *RxData)
{
	int i=0;
	int32_t temp_sum = 0;    
	v->last_raw_value = v->raw_value;
	v->raw_value = (RxData[0]<<8)|RxData[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	//计算得到连续的编码器输出值
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.04394531f + v->round_cnt * 360;
	v->rate_buf[v->buf_count++] = v->ecd_raw_rate;
	if(v->buf_count == RATE_BUF_SIZE)
	{
		v->buf_count = 0;
	}
	//计算速度平均值
	for(i = 0;i < RATE_BUF_SIZE; i++)
	{
		temp_sum += v->rate_buf[i];
	}
	v->filter_rate = (int32_t)(temp_sum/RATE_BUF_SIZE);		
	v->rate_rpm = (RxData[2]<<8)|RxData[3];
}

void GM6020EncoderProcess(volatile Encoder *v,uint8_t *RxData)
{
	v->last_raw_value = v->raw_value;
	v->raw_value = (RxData[0]<<8)|RxData[1];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -4096)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 8192;
	}
	else if(v->diff>4096)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 8192;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 8192;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0439453125f  + v->round_cnt * 360;
	v->filter_rate = (RxData[2]<<8)|RxData[3];
	if(v->filter_rate>=1000)
		v->filter_rate = v->filter_rate - 65535;
	v->temperature = RxData[6];
}


void M3508orM2006EncoderTask(uint32_t can_count,volatile Encoder *v,uint8_t *RxData)
{
	(can_count<=50)?GetEncoderBias(v,RxData):EncoderProcess(v,RxData);
}


void GM6020EncoderTask(uint32_t can_count,volatile Encoder *v,uint8_t *RxData,int offset)
{
	if(can_count <10)
	v->ecd_bias=offset ;
	
	GM6020EncoderProcess(v, RxData);
	// 码盘中间值设定也需要修改
	if (can_count <= 100)
	{
		if ((v->ecd_bias - v->ecd_value) < -4000)
		{
				v->ecd_bias = offset + 8192;
		}
		else if ((v->ecd_bias - v->ecd_value) > 4000)
		{
				v->ecd_bias = offset - 8192;
		}
	}
}
uint8_t Set_GM6020_IQ1_Data[8];
void Set_GM6020_IQ1(CAN_HandleTypeDef *hcanx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq)
{
    Set_GM6020_IQ1_Data[0] = (uint8_t)(motor1_iq >> 8);
    Set_GM6020_IQ1_Data[1] = (uint8_t)motor1_iq;
    Set_GM6020_IQ1_Data[2] = (uint8_t)(motor2_iq >> 8);
    Set_GM6020_IQ1_Data[3] = (uint8_t)motor2_iq;
    Set_GM6020_IQ1_Data[4] = (uint8_t)(motor3_iq >> 8);
    Set_GM6020_IQ1_Data[5] = (uint8_t)motor3_iq;
    Set_GM6020_IQ1_Data[6] = (uint8_t)(motor4_iq >> 8);
    Set_GM6020_IQ1_Data[7] = (uint8_t)motor4_iq;
    USER_CAN_transmit(hcanx,Set_GM6020_IQ1_Data,8,0,0x1FF,0);
//	USER_CAN1_transmit(Set_GM6020_IQ1_Data,8,2,0x1FF,0);
}

void Set_GM6020_IQ2(CAN_HandleTypeDef *hcanx,int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq)
{	
		uint8_t Set_GM6020_IQ2_Data[8]={0};
    Set_GM6020_IQ2_Data[0] = (uint8_t)(motor5_iq >> 8);
    Set_GM6020_IQ2_Data[1] = (uint8_t)motor5_iq;
    Set_GM6020_IQ2_Data[2] = (uint8_t)(motor6_iq >> 8);
    Set_GM6020_IQ2_Data[3] = (uint8_t)motor6_iq;
    Set_GM6020_IQ2_Data[4] = (uint8_t)(motor7_iq >> 8);
    Set_GM6020_IQ2_Data[5] = (uint8_t)motor7_iq;
    Set_GM6020_IQ2_Data[6] = (uint8_t)(motor8_iq >> 8);
    Set_GM6020_IQ2_Data[7] = (uint8_t)motor8_iq;
		USER_CAN_transmit(hcanx,Set_GM6020_IQ2_Data,8,2,0x2FF,0);
}

void Set_C620andC610_IQ1(CAN_HandleTypeDef *hcanx, int16_t motor1_iq, int16_t motor2_iq, int16_t motor3_iq, int16_t motor4_iq)
{
		uint8_t Set_C620andC610_IQ1_Data[8];

    Set_C620andC610_IQ1_Data[0] = (uint8_t)(motor1_iq >> 8);
    Set_C620andC610_IQ1_Data[1] = (uint8_t)motor1_iq;
    Set_C620andC610_IQ1_Data[2] = (uint8_t)(motor2_iq >> 8);
    Set_C620andC610_IQ1_Data[3] = (uint8_t)motor2_iq;
    Set_C620andC610_IQ1_Data[4] = (uint8_t)(motor3_iq >> 8);
    Set_C620andC610_IQ1_Data[5] = (uint8_t)motor3_iq;
    Set_C620andC610_IQ1_Data[6] = (uint8_t)(motor4_iq >> 8);
    Set_C620andC610_IQ1_Data[7] = (uint8_t)motor4_iq;
    USER_CAN_transmit(hcanx,Set_C620andC610_IQ1_Data,8,2,0x200,0);
}

void Set_C620andC610_IQ2(CAN_HandleTypeDef *hcanx, int16_t motor5_iq, int16_t motor6_iq, int16_t motor7_iq, int16_t motor8_iq)
{
		uint8_t Set_C620andC610_IQ2_Data[8];
   
    Set_C620andC610_IQ2_Data[0] = (uint8_t)(motor5_iq >> 8);
    Set_C620andC610_IQ2_Data[1] = (uint8_t)motor5_iq;
    Set_C620andC610_IQ2_Data[2] = (uint8_t)(motor6_iq >> 8);
    Set_C620andC610_IQ2_Data[3] = (uint8_t)motor6_iq;
    Set_C620andC610_IQ2_Data[4] = (uint8_t)(motor7_iq >> 8);
    Set_C620andC610_IQ2_Data[5] = (uint8_t)motor7_iq;
    Set_C620andC610_IQ2_Data[6] = (uint8_t)(motor8_iq >> 8);
    Set_C620andC610_IQ2_Data[7] = (uint8_t)motor8_iq;
    USER_CAN_transmit(hcanx,Set_C620andC610_IQ2_Data,8,2,0x1FF,0);
}

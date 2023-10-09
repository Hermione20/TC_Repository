#include "LK_TECH.h"


/**
  ******************************************************************************
  * @file    LK_TECH.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   此文件编写LK_TECH各型号电机的解算,入口参数包含通用
						 编码器结构体，can总线的计数，can结构体，编码器初始值
						 设定.发送任务函数见senior文件
						 
@verbatim
 ===============================================================================
 **/







/*******************************LK_Tech电机***********************************/

void MF_EncoderProcess(volatile Encoder *v,uint8_t *RxData)//云台yaw，pitch共用
{
	int i=0;
	int32_t temp_sum = 0;
	v->last_raw_value = v->raw_value;
	v->raw_value = (RxData[7]<<8)|RxData[6];
	v->diff = v->raw_value - v->last_raw_value;
	if(v->diff < -32768)    //两次编码器的反馈值差别太大，表示圈数发生了改变
	{
		v->round_cnt++;
		v->ecd_raw_rate = v->diff + 65536;
	}
	else if(v->diff>32768)
	{
		v->round_cnt--;
		v->ecd_raw_rate = v->diff- 65536;
	}		
	else
	{
		v->ecd_raw_rate = v->diff;
	}
	v->ecd_value = v->raw_value + v->round_cnt * 65536;
	//计算得到角度值，范围正负无穷大
	v->ecd_angle = (float)(v->raw_value - v->ecd_bias)*0.0054931641f  + v->round_cnt * 360;
	//从电机编码器读取的速度
	v->filter_rate = (RxData[5]<<8)|RxData[4];
	v->temperature = RxData[1];
}

void MF_EncoderTask(uint32_t can_count,volatile Encoder *v, uint8_t *RxData,int offset)
{

	MF_EncoderProcess(v, RxData);
	// 码盘中间值设定也需要修改
	if (can_count <= 100)
	{
		if ((v->ecd_bias - v->ecd_value) < -32700)
		{
				v->ecd_bias = offset + 65536;
		}
		else if ((v->ecd_bias - v->ecd_value) > 32700)
		{
				v->ecd_bias = offset - 65536;
		}
	}
}

/********************FM9025命令发送函数*********************/
void CAN_9015Command(CAN_HandleTypeDef *hcanx ,uint8_t command,uint8_t id)
{
	uint8_t CAN_9015Command_Data[8];

	CAN_9015Command_Data[0] = command;
	CAN_9015Command_Data[1] = 0;
	CAN_9015Command_Data[2] = 0;
	CAN_9015Command_Data[3] = 0;
	CAN_9015Command_Data[4] = 0;
	CAN_9015Command_Data[5] = 0;
	CAN_9015Command_Data[6] = 0;
	CAN_9015Command_Data[7] = 0;

	USER_CAN_transmit(hcanx,CAN_9015Command_Data,8,2,id,0);
	
}

void CAN_9015setpidCommand(CAN_HandleTypeDef *hcanx, float akp,
                           float aki,
                           float skp,
                           float ski,
                           float iqkp,
                           float iqki, uint8_t id)
{
		int8_t CAN_9015setpidCommand_Data[8];
    CAN_9015setpidCommand_Data[0] = 0x32;
    CAN_9015setpidCommand_Data[1] = 0x00;
    CAN_9015setpidCommand_Data[2] = akp;
    CAN_9015setpidCommand_Data[3] = aki;
    CAN_9015setpidCommand_Data[4] = skp;
    CAN_9015setpidCommand_Data[5] = ski;
    CAN_9015setpidCommand_Data[6] = iqkp;
    CAN_9015setpidCommand_Data[7] = iqki;

    USER_CAN_transmit(hcanx,CAN_9015setpidCommand_Data,8,2,id,0);
}

void CAN_9015angleControl(CAN_HandleTypeDef *hcanx ,int16_t maxSpeed ,uint32_t angleControl,uint8_t id)
{
	uint8_t CAN_9015angleControl_Data[8];

	CAN_9015angleControl_Data[0] = 0xA4;
	CAN_9015angleControl_Data[1] = 0x00;
	CAN_9015angleControl_Data[2] = (uint8_t)maxSpeed;
	CAN_9015angleControl_Data[3] = (uint8_t)(maxSpeed >> 8);
	CAN_9015angleControl_Data[4] = (uint8_t)angleControl;
	CAN_9015angleControl_Data[5] = (uint8_t)(angleControl >> 8);
	CAN_9015angleControl_Data[6] = (uint8_t)(angleControl >> 16);
	CAN_9015angleControl_Data[7] = (uint8_t)(angleControl >> 24);
	
		USER_CAN_transmit(hcanx,CAN_9015angleControl_Data,8,2,id,0);
}

void CAN_9015speedControl(CAN_HandleTypeDef *hcanx ,uint32_t speedControl,uint8_t id)
{
	uint8_t CAN_9015speedControl_Data[8];

	CAN_9015speedControl_Data[0] = 0xA2;
	CAN_9015speedControl_Data[1] = 0x00;
	CAN_9015speedControl_Data[2] = 0x00;
	CAN_9015speedControl_Data[3] = 0x00;
	CAN_9015speedControl_Data[4] = (uint8_t)speedControl;
	CAN_9015speedControl_Data[5] = (uint8_t)(speedControl >> 8);
	CAN_9015speedControl_Data[6] = (uint8_t)(speedControl >> 16);
	CAN_9015speedControl_Data[7] = (uint8_t)(speedControl >> 24);
	
		USER_CAN_transmit(hcanx,CAN_9015speedControl_Data,8,2,id,0);
}

void CAN_9015torsionControl(CAN_HandleTypeDef *hcanx ,int16_t iqcontrol,uint8_t id)
{
	uint8_t CAN_9015torsionControl_Data[8];
	CAN_9015torsionControl_Data[0] = 0xA1;
	CAN_9015torsionControl_Data[1] = 0x00;
	CAN_9015torsionControl_Data[2] = 0x00;
	CAN_9015torsionControl_Data[3] = 0x00;
	CAN_9015torsionControl_Data[4] = (uint8_t)iqcontrol;
	CAN_9015torsionControl_Data[5] = (uint8_t)(iqcontrol >> 8);
	CAN_9015torsionControl_Data[6] = 0x00;
	CAN_9015torsionControl_Data[7] = 0x00;
	
	USER_CAN_transmit(hcanx,CAN_9015torsionControl_Data,8,2,id,0);

	
}

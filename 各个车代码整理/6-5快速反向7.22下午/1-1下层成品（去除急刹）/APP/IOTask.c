#include "main.h"
#include "protocal.h"
//�������̽��ܣ�
/*
1����λ�����͹���У׼����
2������XXXCaliProcess,�����յ���У׼���ݱ��浽GyroCaliData��GimbalCaliData��MagCaliData����
3��У׼��ɺ󣬵���SetCaliData(*v)����У׼���ݱ��浽gAppParamStructFalsh��,��д��Flash��
*/
AppParam_t gAppParamStruct;	//������Ϣ,���ﱣ�������µ�У׼ֵ��������Flash�е�����ͬ��
static GyroCaliStruct_t GyroCaliData;        //����У׼������ƫ��ֵ
static GimbalCaliStruct_t  GimbalCaliData;   //������̨������ƫ��ֵ
static MagCaliStruct_t  MagCaliData;         //���������У׼ֵ
PIDParamStruct_t PIDCaliData;  //����pitch��positionУ׼ֵ
GimbalCaliStruct_t GimbalSavedCaliData;    	    //gimbal pitch yaw encoder offset
GyroCaliStruct_t GyroSavedCaliData;     	    //gyro offset data
AccCaliStruct_t AccSavedCaliData;    	    	//ACC offset data
MagCaliStruct_t MagSavedCaliData;			    //Mag offset data
PIDParamStruct_t PitchPositionSavedPID;        	//PID offset data
PIDParamStruct_t PitchSpeedSavedPID;        	//PID offset data
PIDParamStruct_t YawPositionSavedPID;        	//PID offset data
PIDParamStruct_t YawSpeedSavedPID;        	    //PID offset data


//4.130986




uint8_t app_param_calied_flag = 0;
uint8_t Is_AppParam_Calied(void)
{
	return app_param_calied_flag;    //paramδ��ʼ��
}

static uint8_t AppParamSave(void)
{
    uint8_t retval = 1;   
    retval = BSP_FLASH_Write(PARAM_SAVED_START_ADDRESS, (uint8_t *)&gAppParamStruct, sizeof(AppParam_t));    
    if(retval == 0)
    {
			
    }
    return retval;   
}

//�ֶ������������
void AppParamInit(void)
{
	
	GMPitchEncoder.ecd_bias = GMPitchEncoder_Offset;
	GMYawEncoder.ecd_bias = GMYawEncoder_Offset;
  GM1Encoder1.ecd_bias = GM1Encoder_Offset;   
  GM2Encoder1.ecd_bias = GM2Encoder_Offset;  
  GM3Encoder1.ecd_bias = GM3Encoder_Offset;   
  GM4Encoder1.ecd_bias = GM4Encoder_Offset;  


	
	GyroSavedCaliData.GyroXOffset = 0;
	GyroSavedCaliData.GyroYOffset = 0;
	GyroSavedCaliData.GyroZOffset = 0;
}

void SetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.GimbalCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}


void SetGyroCaliData(GyroCaliStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.GyroCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
	}
}  

void SetAccCaliData(AccCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(&gAppParamStruct.AccCaliData, cali_data, sizeof(*cali_data));
		AppParamSave();
    }
}

void SetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
		memcpy(&gAppParamStruct.MagCaliData, cali_data, sizeof(*cali_data));   //step1: copy data to struct
		AppParamSave();	
    }
																														 //step2:write data to the flash
}
void SetPIDCaliData(PIDParamStruct_t *cali_data)
{
	if(cali_data != NULL)
    {
		if(cali_data->pid_type == PID_TYPE_POSITION && cali_data->motor_type == MOTOR_TYPE_PITCH)
		{
			cali_data->kp_offset += gAppParamStruct.PitchPositionPID.kp_offset;
			cali_data->ki_offset += gAppParamStruct.PitchPositionPID.ki_offset;
			cali_data->kd_offset += gAppParamStruct.PitchPositionPID.kd_offset;			
			memcpy(&gAppParamStruct.PitchPositionPID, cali_data, sizeof(*cali_data));
		}
		else if(cali_data->pid_type == PID_TYPE_SPEED && cali_data->motor_type == MOTOR_TYPE_PITCH)
		{
			cali_data->kp_offset += gAppParamStruct.PitchSpeedPID.kp_offset;
			cali_data->ki_offset += gAppParamStruct.PitchSpeedPID.ki_offset;
			cali_data->kd_offset += gAppParamStruct.PitchSpeedPID.kd_offset;	
			memcpy(&gAppParamStruct.PitchSpeedPID, cali_data, sizeof(*cali_data));
		}
		else if(cali_data->pid_type == PID_TYPE_POSITION && cali_data->motor_type == MOTOR_TYPE_YAW)
		{
			cali_data->kp_offset += gAppParamStruct.YawPositionPID.kp_offset;
			cali_data->ki_offset += gAppParamStruct.YawPositionPID.ki_offset;
			cali_data->kd_offset += gAppParamStruct.YawPositionPID.kd_offset;	
			memcpy(&gAppParamStruct.YawPositionPID, cali_data, sizeof(*cali_data));
		}
		else if(cali_data->pid_type == PID_TYPE_SPEED && cali_data->motor_type == MOTOR_TYPE_YAW)
		{
			cali_data->kp_offset += gAppParamStruct.YawSpeedPID.kp_offset;
			cali_data->ki_offset += gAppParamStruct.YawSpeedPID.ki_offset;
			cali_data->kd_offset += gAppParamStruct.YawSpeedPID.kd_offset;	
			memcpy(&gAppParamStruct.YawSpeedPID, cali_data, sizeof(*cali_data));
		}
		 AppParamSave();	
	}
}

void GetGimbalCaliData(GimbalCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.GimbalCaliData, sizeof(GimbalCaliStruct_t));
    }
}

void GetGyroCaliData(GyroCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.GyroCaliData, sizeof(GyroCaliStruct_t));
    }
}

void GetAccCaliData(AccCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.AccCaliData, sizeof(AccCaliStruct_t));
    }
}

void GetMagCaliData(MagCaliStruct_t *cali_data)
{
    if(cali_data != NULL)
    {
        memcpy(cali_data, &gAppParamStruct.MagCaliData, sizeof(MagCaliStruct_t));
    }
}

uint8_t IsGimbalCalied(void)
{
    return (gAppParamStruct.GimbalCaliData.GimbalCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsGyroCalied(void)
{
    return (gAppParamStruct.GyroCaliData.GyroCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsAccCalied(void)
{
    return (gAppParamStruct.AccCaliData.AccCaliFlag == PARAM_CALI_DONE);
}

uint8_t IsMagCalied(void)
{
    return (gAppParamStruct.MagCaliData.MagCaliFlag == PARAM_CALI_DONE);
}

/*********************************************************************
 * @fn      CalibrateLoop
 *
 * @brief   do the calibration according to the corresponding cali flag
 *
 * @param   *flag_grp - the pointer to the cali flag group
 *
 * @return  none
 */


static uint32_t CaliCmdFlagGrp = 0;     //cali cmd flag group every bit represents a cali cmd received from the PC

void SetCaliCmdFlag(uint32_t flag)  //����У׼��־λ
{
	CaliCmdFlagGrp |= flag;
}

void ResetCaliCmdFlag(uint32_t flag)
{
	CaliCmdFlagGrp &= ~flag;
}

uint32_t GetCaliCmdFlagGrp()
{
	return CaliCmdFlagGrp;
}

//to check whether a specfic flag if set
uint8_t IsCaliCmdFlagSet(uint32_t flag)
{
	if(flag & CaliCmdFlagGrp)
	{
		return 1;
	}
	else
	{
		return 0;	
	}
}

void CalibrateLoop(void)
{
    CALI_STATE_e cali_result;    
    if(IsCaliCmdFlagSet(CALI_START_FLAG_GYRO))  
	{
		ResetCaliCmdFlag(CALI_START_FLAG_GYRO);
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_GYRO))   //calibrate the 
    {
        cali_result = GyroCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetGyroCaliData(&GyroCaliData);   //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
			Sensor_Offset_Param_Init(&gAppParamStruct);   //update the parameter
			ResetCaliCmdFlag(CALI_END_FLAG_GYRO);		//reset the cali cmd
		}
    }
	else if(IsCaliCmdFlagSet(CALI_START_FLAG_GIMBAL))   //calibrate the 
	{
		ResetCaliCmdFlag(CALI_START_FLAG_GIMBAL);
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_GIMBAL))
	{
	    cali_result = GimbalCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetGimbalCaliData(&GimbalCaliData);           //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
			Sensor_Offset_Param_Init(&gAppParamStruct);   //update the parameter
			ResetCaliCmdFlag(CALI_END_FLAG_GIMBAL);
		}	
	}
	else if(IsCaliCmdFlagSet(CALI_START_FLAG_MAG))
	{
		cali_result = MagStartCaliProcess();   //reset the max min data of the magenemter
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{			
			ResetCaliCmdFlag(CALI_START_FLAG_MAG);
		}	
	}
	else if(IsCaliCmdFlagSet(CALI_END_FLAG_MAG))
	{
		cali_result = MagEndCaliProcess();  
		if(cali_result == CALI_STATE_ERR)
		{
			
		}
		else if(cali_result == CALI_STATE_IN)
		{
			
		}
		else if(cali_result == CALI_STATE_DONE)
		{
			SetMagCaliData(&MagCaliData);                 //set the apparamStruct using the GyroCaliData, and save apparamStruct to the flash 
			Sensor_Offset_Param_Init(&gAppParamStruct);   //update the parameter
			ResetCaliCmdFlag(CALI_END_FLAG_MAG);
		}		
	}
	else if(IsCaliCmdFlagSet(CALI_FLAG_PID))
	{
			SetPIDCaliData(&PIDCaliData);                 //�����յ���PIDCaliData���ݱ��浽apparamStruct��
			Sensor_Offset_Param_Init(&gAppParamStruct);   //update the parameter
			ResetCaliCmdFlag(CALI_FLAG_PID);
	}
}



CALI_STATE_e  GimbalCaliProcess()     //����У׼״̬   ERROR DONE
{
static uint32_t loopCount = 0;
static uint32_t loopTime = 10;
static int32_t pitchSum = 0;
static int32_t yawSum = 0;

	if(Is_Lost_Error_Set(LOST_ERROR_MOTOR5) || Is_Lost_Error_Set(LOST_ERROR_MOTOR6))
	{
		return CALI_STATE_ERR;
	}
	else if(loopCount++<loopTime)   //in cali state
	{
		pitchSum += GMPitchEncoder.raw_value;
		yawSum += GMYawEncoder.raw_value;
		return CALI_STATE_IN;
	}
	else
	{		
		GimbalCaliData.GimbalPitchOffset = pitchSum/loopTime;   //��ȡpitch����������Ϊƫ��
	    GimbalCaliData.GimbalYawOffset = yawSum/loopTime;		//��ȡyaw����������Ϊƫ��
		GimbalCaliData.GimbalCaliFlag = PARAM_CALI_DONE;
		pitchSum = 0;
		yawSum = 0;
		loopCount = 0;
		return CALI_STATE_DONE;
	}	
}

CALI_STATE_e  GyroCaliProcess()     
{
	int16_t temp[6] = {0};
	static uint16_t loopCount = 0;
	static uint16_t loopTime = 20;
	static int32_t gyroXSum = 0;
	static int32_t gyroYSum = 0;
	static int32_t gyroZSum = 0;
	//��gyroֵ����,��˵õ��Ĳ���ԭʼֵ
	GyroSavedCaliData.GyroXOffset = 0;
	GyroSavedCaliData.GyroYOffset = 0;
	GyroSavedCaliData.GyroZOffset = 0;	
	//process of the cali if error return error, elseif in processing return in , and if done return done
	if(Is_Lost_Error_Set(LOST_ERROR_IMU))   
	{
		return CALI_STATE_ERR;
	}
	else if(loopCount++<loopTime)   //in cali state
	{
//		MPU6050_getMotion6(&temp[0],&temp[1],&temp[2],&temp[3],&temp[4],&temp[5]);
		gyroXSum += temp[3];
		gyroYSum += temp[4];
		gyroZSum += temp[5];
		return CALI_STATE_IN;
	}
	else
	{					
		GyroCaliData.GyroXOffset = gyroXSum/loopTime;   //��ȡpitch����������Ϊƫ��
	    GyroCaliData.GyroYOffset = gyroYSum/loopTime;		//��ȡyaw����������Ϊƫ��
		GyroCaliData.GyroZOffset = gyroZSum/loopTime;		//��ȡyaw����������Ϊƫ��
		GyroCaliData.GyroCaliFlag = PARAM_CALI_DONE;
		gyroXSum = 0;
		gyroYSum = 0;
		gyroZSum = 0;
		loopCount = 0;
		return CALI_STATE_DONE;
	}
}

CALI_STATE_e  MagStartCaliProcess()
{	
//	MagMaxMinData.MaxMagX = -4096;	//��ԭ���ı궨ֵ���
//	MagMaxMinData.MaxMagY = -4096;
//	MagMaxMinData.MaxMagZ = -4096;
//	MagMaxMinData.MinMagX = 4096;
//	MagMaxMinData.MinMagY = 4096;
//	MagMaxMinData.MinMagZ = 4096;
	return CALI_STATE_DONE;	
}
CALI_STATE_e  MagEndCaliProcess()
{
	if(Is_Lost_Error_Set(LOST_ERROR_IMU))    
	{
		return CALI_STATE_ERR;
	}
	else
	{
//		MagCaliData.MagXOffset = (MagMaxMinData.MaxMagX + MagMaxMinData.MinMagX)/2;
//		MagCaliData.MagYOffset = (MagMaxMinData.MaxMagY + MagMaxMinData.MinMagY)/2;
//		MagCaliData.MagZOffset = (MagMaxMinData.MaxMagZ + MagMaxMinData.MinMagZ)/2;
		MagCaliData.MagXScale = 1.0;
		MagCaliData.MagYScale = 1.0;
		MagCaliData.MagZScale = 1.0;	
		MagCaliData.MagCaliFlag = PARAM_CALI_DONE;
		return CALI_STATE_DONE;		
	}	
}

//copy src pid offset data received from the PC to the static PitchPostionCaliData/PitchSpeedCaliData
CALI_STATE_e PIDCaliProcess(PIDParamStruct_t *cali_data)
{
	if(cali_data!=NULL)
	{
		memcpy(&PIDCaliData, cali_data, sizeof(*cali_data));
		return CALI_STATE_DONE;
	}	
    return CALI_STATE_DONE;
}

void Sensor_Offset_Param_Init(AppParam_t *appParam)
{

}


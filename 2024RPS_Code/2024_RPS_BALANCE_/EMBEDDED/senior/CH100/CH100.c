#include "CH100.h"


/**
  ******************************************************************************
  * @file    CH100.c
  * @author  Lee_ZEKAI
  * @version V1.1.0
  * @date    03-October-2023
  * @brief   ���ļ���д����CH100�����ǵ����ݽ�������㣬
							������ڲ���Ϊ����dma���յ�ַ��ͨ��������
							�ṹ��,����Ϊ������У׼��ƫ����
                            ���ò�����������Ư����
						 
@verbatim
 ===============================================================================
 **/
 
 float offset = 0;

void CH100_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO,float pitch_offset,
                                                            float roll_offset,
                                                            float pitch_gyro_offset,
                                                            float yaw_gyro_offset,
                                                            float roll_gyro_offset,
                                                            float acc_x_offset,
                                                            float acc_y_offset,
                                                            float acc_z_offset)
{
    static __align(4) id0x91_t dat; /* struct must be 4 byte aligned */
    memcpy(&dat, &DataAddress[6], sizeof(id0x91_t));

    volatile static float Last_yaw_temp1, Yaw_temp1;
	volatile static int Yaw_count1;

    GYRO->pitch_Angle = -dat.eul[0];

    Last_yaw_temp1 = Yaw_temp1;
    Yaw_temp1 = dat.eul[2];
    if(Yaw_temp1 - Last_yaw_temp1 >= 324)
    {
        Yaw_count1--;
    }else if (Yaw_temp1 - Last_yaw_temp1 <= -324)
    {
        Yaw_count1++;
    }
    GYRO->yaw_Angle = -(Yaw_temp1 + Yaw_count1*360);

    GYRO->roll_Angle = dat.eul[1];

    GYRO->pitch_Gyro = -dat.gyr[1];
    GYRO->yaw_Gyro = -dat.gyr[2];
    GYRO->roll_Gyro = dat.gyr[0];

    GYRO->x_Acc = (dat.acc[0]+sinf(dat.eul[0]*PI/180.0))*9.81f;
    GYRO->y_Acc = (dat.acc[2]-cosf(dat.eul[1]*PI/180.0)*cosf(dat.eul[0]*PI/180.0))*9.81f;
    GYRO->z_Acc = -(dat.acc[1]-sinf(dat.eul[1]*PI/180.0)*cosf(dat.eul[0]*PI/180.0))*9.81f;

    GYRO->pitch_Angle -= pitch_offset;  
    GYRO->roll_Angle -= roll_offset;

    GYRO->pitch_Gyro -= pitch_gyro_offset;
    GYRO->yaw_Gyro -= yaw_gyro_offset;
    GYRO->roll_Gyro -= roll_gyro_offset;

    GYRO->x_Acc -= acc_x_offset;
    GYRO->y_Acc -= acc_y_offset;
    GYRO->z_Acc -= acc_z_offset;
}

void Gyroscope_calibration(float value)
{
    static int i = 0;
    static float buff;
    if (i<200)
    {
        buff+=value;
        i++;
    }else
    {
        i = 0;
        offset = buff/200.0f;
        buff = 0;
    }
    
    
}
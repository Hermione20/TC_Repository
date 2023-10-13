#include "main.h"

volatile MPU6050_RAW_DATA    MPU6050_Raw_Data;    //ԭʼ����
volatile MPU6050_REAL_DATA   MPU6050_Real_Data;
AHRS ahrs;
uint8_t mpu_buf[20]={0};       //save the data of acc gyro & mag using iic

int16_t MPU6050_FIFO[6][11] = {0};//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
int16_t HMC5883_FIFO[3][11] = {0};//[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ ע���Ŵ������Ĳ���Ƶ���������Ե����г�
MagMaxMinData_t MagMaxMinData;


float HMC5883_lastx,HMC5883_lasty,HMC5883_lastz;
//MPU6050 ��ʼ�����ɹ�����0  ʧ�ܷ��� 0xff
int MPU6050_Init(void)
{
    unsigned char temp_data = 0x00;

    IIC_GPIO_Init();  //��ʼ��IIC�ӿ�
    HEAT_Configuration();
    
    if(IIC_ReadData(MPU6050_DEVICE_ADDRESS,WHO_AM_I,&temp_data,1)==0) //ȷ��IIC�����Ϲҽӵ��Ƿ���MPU6050
    {
        if(temp_data != MPU6050_ID)
        {
            printf("error 1A\r\n");
            return 0xff; //У��ʧ�ܣ�����0xff
        }
    }
    else
    {
        printf("error 1B\r\n");
        return 0xff; //��ȡʧ�� ����0xff
    }
    
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,PWR_MGMT_1,0x01) == 0xff)    //�������״̬
    {
        printf("error 1C\r\n");
        return 0xff;
    }
		

    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,CONFIG,0x01) == 0xff)         //Digital Low-Pass Filter:DLPF_CFG is 3, Fs is 1khz 
    {                                                                     //acc bandwidth 44Hz,gyro 42Hz
        printf("error 1E\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,GYRO_CONFIG,0x10) == 0xff)    //FS_SEL 3 : gyroscope full scale range is +-1000degs/s 
    {
        printf("error 1F\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,ACCEL_CONFIG,0x00) == 0xff)   //AFS_SEL 1: accelerometer full scale range is +-2g
    {
        printf("error 1G\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_PIN_CFG,0x02) == 0xff)    //logic level for the INT pin is active high
                                                                          //the INT pin emits a 50us long pulse, not latched    bypass mode enabled
    {
        printf("error 1H\r\n");
        return 0xff;
    }
    if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
		
		//����mpu6050 IIC masters mode  disabled ����mpu6050����aux IIC�ӿ�
		if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,MPU6050_RA_USER_CTRL,0x00) == 0xff)      //disable data ready interrupt
    {
        printf("error 1I\r\n");
        return 0xff;
    }
		
		//����IIC masters mode Ϊ bypass mode enabled����INT_PIN_CFG������
		
		
    Delay_ms(500);
    //MPU6050_GyroCalibration();
    return 0;
}

int MPU6050_EnableInt(void)
{
	  if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,SMPLRT_DIV,0x01)==0xff)      //Sample Rate: Gyro output rate / (1 + 1) = 500Hz
	  {
        printf("Cannot enable interrupt successfully.\r\n");
        return 0xff;
	  }
    printf("MPU6050 set sample rate done.\n");
	  Delay_ms(10);
	  if(IIC_WriteData(MPU6050_DEVICE_ADDRESS,INT_ENABLE,0x01) == 0xff)     //enable data ready interrupt 
    {
      printf("error 1I\r\n");
      return 0xff;
    } 
#if 0
    printf("MPU6050 enable interrupt done.\n"); 
//	unsigned char  buf[14] ={0};    
//    Delay_ms(2);
//	IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14);    //dummy read to clear the data registers
//	Delay_ms(10);
#endif
	return 0;
}

void MPU6050_Initialize(void)
{
    while(MPU6050_Init() == 0xff) 
    {                       
       Delay_ms(200);      
       //printf("fdaf\n");			
    }
}

//MPU6050  ���ݶ�ȡ���ɹ�����0  ʧ�ܷ��� 0xff



int MPU6050_ReadData(uint8_t Slave_Addr, uint8_t Reg_Addr, uint8_t * Data, uint8_t Num)
{    
	//IIC_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START,buf,14)
    if(IIC_ReadData(Slave_Addr,Reg_Addr,Data,Num) == 0xff)
    {
        printf("error 1J\r\n");
        return 0xff;
    }
   
    return 0;
}

/**********************************************************************************/
/*��MPU6050_ax,MPU6050_ay, MPU6050_az,MPU6050_gx, MPU6050_gy, MPU6050_gz�����洢*/
/**********************************************************************************/
void MPU6050_DataSave(int16_t ax,int16_t ay,int16_t az,int16_t gx,int16_t gy,int16_t gz) //[0]-[9]Ϊ���10������ [10]Ϊ10�����ݵ�ƽ��ֵ
{
	uint8_t i = 0;
	int32_t sum=0;
	
	for(i=1;i<10;i++)
	{
		MPU6050_FIFO[0][i-1]=MPU6050_FIFO[0][i];
		MPU6050_FIFO[1][i-1]=MPU6050_FIFO[1][i];
		MPU6050_FIFO[2][i-1]=MPU6050_FIFO[2][i];
		MPU6050_FIFO[3][i-1]=MPU6050_FIFO[3][i];
		MPU6050_FIFO[4][i-1]=MPU6050_FIFO[4][i];
		MPU6050_FIFO[5][i-1]=MPU6050_FIFO[5][i];
	}
	
	MPU6050_FIFO[0][9]=ax;//���µ����ݷ��õ� ���ݵ������
	MPU6050_FIFO[1][9]=ay;
	MPU6050_FIFO[2][9]=az;
	MPU6050_FIFO[3][9]=gx;
	MPU6050_FIFO[4][9]=gy;
	MPU6050_FIFO[5][9]=gz;
	
	for(i=0;i<10;i++)//��ǰ����ĺϣ���ȡƽ��ֵ
	{	
		 sum+=MPU6050_FIFO[0][i];
	}
	MPU6050_FIFO[0][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[1][i];
	}
	MPU6050_FIFO[1][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[2][i];
	}
	MPU6050_FIFO[2][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[3][i];
	}
	MPU6050_FIFO[3][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[4][i];
	}
	MPU6050_FIFO[4][10]=sum/10;

	sum=0;
	for(i=0;i<10;i++){
		 sum+=MPU6050_FIFO[5][i];
	}
	MPU6050_FIFO[5][10]=sum/10;
	
}


int16_t MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz
				,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz;
/**************************ʵ�ֺ���********************************************
*����ԭ��:		void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
*��������:	    ��ȡ MPU6050�ĵ�ǰ����ֵ
*******************************************************************************/
void MPU6050_getMotion6(int16_t* ax, int16_t* ay, int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz) {
  
	if(isMPU6050_is_DRY)
	{
		isMPU6050_is_DRY = 0;
		MPU6050_ReadData(MPU6050_DEVICE_ADDRESS,MPU6050_DATA_START+8,mpu_buf,6);  //ֻ��ȡ����������
		//HMC58X3_ReadData(&(mpu_buf[14]));  //14-19Ϊ����������
//		MPU6050_Lastax=(((int16_t)mpu_buf[0]) << 8) | mpu_buf[1];
//		MPU6050_Lastay=(((int16_t)mpu_buf[2]) << 8) | mpu_buf[3];
//		MPU6050_Lastaz=(((int16_t)mpu_buf[4]) << 8) | mpu_buf[5];
//�����¶�ADC
		MPU6050_Lastgx=((((int16_t)mpu_buf[0]) << 8) | mpu_buf[1])-GyroSavedCaliData.GyroXOffset;
		MPU6050_Lastgy=((((int16_t)mpu_buf[2]) << 8) | mpu_buf[3])-GyroSavedCaliData.GyroYOffset;
		MPU6050_Lastgz=((((int16_t)mpu_buf[4]) << 8) | mpu_buf[5])-GyroSavedCaliData.GyroZOffset;
			
//		MPU6050_DataSave(MPU6050_Lastax,MPU6050_Lastay,MPU6050_Lastaz,MPU6050_Lastgx,MPU6050_Lastgy,MPU6050_Lastgz);  		
//		*ax  =MPU6050_FIFO[0][10];
//		*ay  =MPU6050_FIFO[1][10];
//		*az = MPU6050_FIFO[2][10];
//		*gx  =MPU6050_FIFO[3][10] - GyroSavedCaliData.GyroXOffset;
//		*gy = MPU6050_FIFO[4][10] - GyroSavedCaliData.GyroYOffset;
//		*gz = MPU6050_FIFO[5][10] - GyroSavedCaliData.GyroZOffset;

	} 		
	*gx = MPU6050_Lastgx;
	*gy = MPU6050_Lastgy;
	*gz = MPU6050_Lastgz;
}

void MPU6050_getlastMotion6(int16_t* ax, int16_t* ay, 
		int16_t* az, int16_t* gx, int16_t* gy, int16_t* gz)
{
	*ax  =MPU6050_FIFO[0][10];
	*ay  =MPU6050_FIFO[1][10];
	*az = MPU6050_FIFO[2][10];
	*gx  =MPU6050_FIFO[3][10]-GyroSavedCaliData.GyroXOffset;
	*gy = MPU6050_FIFO[4][10]-GyroSavedCaliData.GyroYOffset;
	*gz = MPU6050_FIFO[5][10]-GyroSavedCaliData.GyroZOffset;
}











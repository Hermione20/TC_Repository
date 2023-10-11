  #ifndef __SYS_H_��_
#define __SYS_H__

//-----------�����޸Ĳ���-----------//
//==========================================================
// <o> STANDARD  - ���Ų���
// <3=> NUM_3
// <4=> NUM_4
// <5=> NUM_5
#define STANDARD 			3 //ѡ�񲽱�Ϊ3�� ����4��

//==========================================================
//<h> ����������
#define HI219 		0
#define ICM20948 	1
// <o> IMU  �������ͺ�
// <0=> HI219
// <1=> ICM20948
//#define IMU 		ICM20948			//IMUѡ��ΪHI219 ����ICM20948
#define IMU  HI219
// <q> GYRO_CALI  - �Ƿ�У׼������
#define GYRO_CALI 			1                   //1ΪУ׼0Ϊ��У׼
// <q> HI219_FIRST_USED  - �Ƿ��ǵ�һ��ʹ��HI219
#define HI219_FIRST_USED 	0	 				//�Ƿ��ǵ�һ��ʹ��HI219
//</h>
//==========================================================
// <o> REMOTE_SHOOT  - ң�����󲦸˵Ĺ���ѡ��
// <0=> С����
// <1=> ����
#define REMOTE_SHOOT        1                 //1:ң�����󲦸�Ϊ����  0:С����

//==========================================================
// <q> NEW_CAP  - �Ƿ�Ϊ�µ��ݿ��ư�
#define NEW_CAP             1                   //1:�µ��ݿ��ư�   0:�ɵ��ݿ��ư�

//==========================================================
//<h> ��������
// <q> NEW_CAP  - �Ƿ�ʹ���ٶ�Ԥ��
#define ARMY_SPEED_PREDICTION 1
// <q> ENABLE_KALMAN_FILTER  - �Ƿ�ʹ�ܿ������˲�
#define ENABLE_KALMAN_FILTER  1
//</h>
#define  POWER_LIMT                   1  //1ʱʹ���Ͻ��㷨��0ʱʹ��ֱ���޵�����

//==========================================================
#if STANDARD == 3                                          //
//��ʶ3���벽��
//pitch������ʼλ��
#define  GMPitchEncoder_Offset 6930
//yaw������ʼλ��
#define GMYawEncoder_Offset   2709//4053

#define  Vehicle_Num         3
#define  GM1Encoder_Offset   1411+1024+2048       //5335
#define  GM2Encoder_Offset   5433+1024-50   //3595   //7910
#define  GM3Encoder_Offset   1269+1024+2048
#define  GM4Encoder_Offset   8037-1024-2048  //3676



//������X��Ĭ��У׼ֵ
#define GYRO_REAL_X_OFFSET 0.00559780467f
//������Y��Ĭ��У׼ֵ
#define GYRO_REAL_Y_OFFSET -0.0156064359
//������Z��Ĭ��У׼ֵ
#define GYRO_REAL_Z_OFFSET -0.0091483118

//ͼ��X������λ��
#define IMAGE_X_OFFET 0 //���������ʵ�����ƫ�ң�����Ϊ��
//ͼ��Y������λ��
#define IMAGE_Y_OFFET 0
//����ͷ��ǹ�����ĵİ�װƫ���-YAW����  ��������
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-1.2
//����ͷ��ǹ�����ĵİ�װƫ���
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA 		1.8F
//����ͷ��ǹ�����ĵľ���
#define HEIGHT_BETWEEN_GUN_CAMERA 			78.5F
//�������mm
#define FOCAL_LENGTH                6.0F
//���泤mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//�����mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
//���سߴ�mm
#define IMAGE_LENGTH                3.45e-3f

//Զ����ʱǹ�ܽǶȲ���  //no used
#define ANGLE_COMPENSATION_LONG_DISTANCE 	2
//ͼ�����̨�����ӳ�ʱ�� - �� /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   0e-3f//   0//s 
#define PIT_IMAGE_GIMBAL_DELAY_TIME       	0
#define DISTANCE_OFFSET           50


#elif STANDARD == 4
////��ʶ4���벽��
#define  GMPitchEncoder_Offset 6758
//yaw������ʼλ��
#define  GMYawEncoder_Offset   2750

#define  GM1Encoder_Offset   1350+1024+2048//5335
#define  GM2Encoder_Offset   2731+1024  //3595   //7910
#define  GM3Encoder_Offset   1304+1024+2048
#define  GM4Encoder_Offset   5400+1024  //3676

#define  Vehicle_Num         4

#define GYRO_REAL_X_OFFSET 0.00559780467
#define GYRO_REAL_Y_OFFSET -0.0156064359
#define GYRO_REAL_Z_OFFSET -0.0091483118

//ͼ��X������λ��
#define IMAGE_X_OFFET 0 //���������ʵ�����ƫ�ң�����Ϊ��
//ͼ��Y������λ��
#define IMAGE_Y_OFFET 0
//����ͷ��ǹ�����ĵİ�װƫ���-YAW����
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-0.8f
//����ͷ��ǹ�����ĵİ�װƫ���
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA   3.5f
//����ͷ��ǹ�����ĵľ���
#define HEIGHT_BETWEEN_GUN_CAMERA 	78.5f
//�������mm
#define FOCAL_LENGTH                6.0F
//���泤mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//�����mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
#define IMAGE_LENGTH                3.45e-3f
//ͼ�����̨�����ӳ�ʱ�� - �� /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   50e-3f         // ��λs
#define PIT_IMAGE_GIMBAL_DELAY_TIME    0
#define DISTANCE_ENABLE  0        //�����Ƿ��ܹ�ʹ�þ�����Ϣ
#define MAX_ATTACK_DISTANCE 800  //��λcm



#elif STANDARD == 5                                    //
////��ʶ5���벽��
#define  GMPitchEncoder_Offset 0
//yaw������ʼλ��
#define  GMYawEncoder_Offset   4758


/*˳ʱ�����ת������

*/
#define  GM1Encoder_Offset   5471        //5335
#define  GM2Encoder_Offset   2703  //3595   //7910
#define  GM3Encoder_Offset   5510
#define  GM4Encoder_Offset   1326   //3676


#define  Vehicle_Num         5
#define GYRO_REAL_X_OFFSET 0.00559780467
#define GYRO_REAL_Y_OFFSET -0.0156064359
#define GYRO_REAL_Z_OFFSET -0.0091483118

//ͼ��X������λ��
#define IMAGE_X_OFFET 0 //���������ʵ�����ƫ�ң�����Ϊ��
//ͼ��Y������λ��
#define IMAGE_Y_OFFET 0
//����ͷ��ǹ�����ĵİ�װƫ���-YAW����
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-0.5f
//����ͷ��ǹ�����ĵİ�װƫ���
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA   1.2f
//����ͷ��ǹ�����ĵľ���
#define HEIGHT_BETWEEN_GUN_CAMERA 	4.89f
//�������mm
#define FOCAL_LENGTH                4.0F
//���泤mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//�����mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
#define IMAGE_LENGTH                4.8e-3f
//ͼ�����̨�����ӳ�ʱ�� - �� /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   50e-3f         // ��λs
#define PIT_IMAGE_GIMBAL_DELAY_TIME    0
#define DISTANCE_ENABLE  0        //�����Ƿ��ܹ�ʹ�þ�����Ϣ
#define MAX_ATTACK_DISTANCE 800  //��λcm

#elif STANDARD == 6                                 //���ó�
////��ʶ4���벽��
#define  GMPitchEncoder_Offset 6800
//yaw������ʼλ��
#define  GMYawEncoder_Offset   4000
#define GYRO_REAL_X_OFFSET 0.00559780467
#define GYRO_REAL_Y_OFFSET -0.0156064359
#define GYRO_REAL_Z_OFFSET -0.0091483118

#define Vehicle_Num 6

#define  GM1Encoder_Offset   5205-1024       //5335
#define  GM2Encoder_Offset   4333+1024  //3595   //7910
#define  GM3Encoder_Offset   1110+1024+2048
#define  GM4Encoder_Offset   4339+1024  //3676


//ͼ��X������λ��
#define IMAGE_X_OFFET 0 //���������ʵ�����ƫ�ң�����Ϊ��
//ͼ��Y������λ��
#define IMAGE_Y_OFFET 0
//����ͷ��ǹ�����ĵİ�װƫ���-YAW����
#define YAW_ANGLE_BETWEEN_GUN_CAMERA 		-0.5f
//����ͷ��ǹ�����ĵİ�װƫ���
#define PITCH_ANGLE_BETWEEN_GUN_CAMERA   1.2f
//����ͷ��ǹ�����ĵľ���
#define HEIGHT_BETWEEN_GUN_CAMERA 	4.89f
//�������mm
#define FOCAL_LENGTH                6.0F
//���泤mm
#define TARGET_SURFACE_LENGTH      3.45e-3F
//�����mm
#define TARGET_SURFACE_WIDTH        3.45e-3F
#define IMAGE_LENGTH                4.8e-3f

//ͼ�����̨�����ӳ�ʱ�� - �� /
#define YAW_IMAGE_GIMBAL_DELAY_TIME   50e-3f         // ��λs
#define PIT_IMAGE_GIMBAL_DELAY_TIME    0
#define DISTANCE_ENABLE  0        //�����Ƿ��ܹ�ʹ�þ�����Ϣ
#define MAX_ATTACK_DISTANCE 800  //��λcm

#else
"ERROR,please define STANDARD as 3 or 4 or 5 or 6 "

#endif

//==========================================================
//<<< end of configuration section >>>\n
#endif

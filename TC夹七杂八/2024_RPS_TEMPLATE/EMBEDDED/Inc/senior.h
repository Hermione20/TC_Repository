#ifndef __SENIOR_H
#define __SENIOR_H
#include "main.h"
#include "send.pb-c.h"
#include "Recieve.pb-c.h"

#define  GMPitchEncoder_Offset 0
//yaw������ʼλ��
#define  GMYawEncoder_Offset   4758
//���̺���������ʼλ��
#define  GM1Encoder_Offset   1437
#define  GM2Encoder_Offset   8042
#define  GM3Encoder_Offset   4141
#define  GM4Encoder_Offset   6732

/********************DJI Encoder******************************/
#define RATE_BUF_SIZE 6
typedef struct{
	int32_t raw_value;   									//���������������ԭʼֵ
	int32_t last_raw_value;								//��һ�εı�����ԭʼֵ
	int32_t ecd_value;                       //��������������ı�����ֵ
	int32_t diff;													//���α�����֮��Ĳ�ֵ
	int32_t temp_count;                   //������
	uint8_t buf_count;								//�˲�����buf��
	int32_t ecd_bias;											//��ʼ������ֵ	
	int32_t ecd_raw_rate;									//ͨ������������õ����ٶ�ԭʼֵ
	int32_t rate_buf[RATE_BUF_SIZE];	//buf��for filter
	int32_t round_cnt;										//Ȧ��
	int32_t filter_rate;											//�ٶ�
	double ecd_angle;											//�Ƕ�
	u32 temperature;
	int16_t rate_rpm;
	
}Encoder;

typedef struct{
	
	uint8_t anglekp;
	uint8_t angleki;
	uint8_t speedkp;
	uint8_t speedki;
	uint8_t torquekp;
	uint8_t torqueki;
	
}PID9015Typedefine;

/***************************CH100********************************/

__packed typedef struct
{
uint8_t tag; /* ????:0x91 */
uint8_t id; /* ??ID */
uint8_t rev[2];
float prs; /* ?? */
uint32_t ts; /* ??? */
float acc[3]; /* ??? */
float gyr[3]; /* ??? */
float mag[3]; /* ?? */
float eul[3]; /* ???:
Roll,Pitch,Yaw */
float quat[4]; /* ??? */
}id0x91_t;



/****************************hi226********************************/
#define LENGTH_USER_ID_0x90 			1
#define LENGTH_ACC_0xa0 					6
#define LENGTH_LINEAR_ACC_0xa5 		6
#define LENGTH_ANG_VEL_0xb0 			6
#define LENGTH_MAG_0xc0 					6
#define LENGTH_EULER_ANG_s16_0xd0 6
#define LENGTH_EULER_ANG_f_0xd9 	12
#define LENGTH_QUATERNION_0xd1 		16
#define LENGTH_AIR_PRESS_0xf0 		4
#define USART6_RX_BUF_LENGTH 100

typedef struct {
					u8 User_ID;
					s16 Acc_X;
					s16 Acc_Y;
					s16 Acc_Z;
					s16 Linear_Acc_X;
					s16 Linear_Acc_Y;
					s16 Linear_Acc_Z;
					s16 Ang_Velocity_X;
					s16 Ang_Velocity_Y;
					s16 Ang_Velocity_Z;
					s16 Mag_X;
					s16 Mag_Y;
					s16 Mag_Z;
					s16 Euler_Angle_Pitch_s16;
					s16 Euler_Angle_Roll_s16;
					s16 Euler_Angle_Yaw_s16;
					
					union
					{
						float Euler_Angle_Pitch_f;
						u8 Euler_Angle_Pitch_u8[4];
					}Euler_Angle_Pitch;
					
					union
					{
						float Euler_Angle_Roll_f;
						u8 Euler_Angle_Roll_u8[4];
					}Euler_Angle_Roll;

					union
					{
						float Euler_Angle_Yaw_f;
						u8 Euler_Angle_Yaw_u8[4];
					}Euler_Angle_Yaw;
					
					union
					{
						float Quaternion_W_f;
						u8 Quaternion_W_u8[4];
					}Quaternion_W;		

					union
					{
						float Quaternion_X_f;
						u8 Quaternion_X_u8[4];
					}Quaternion_X;	
						union
					{
						float Quaternion_Y_f;
						u8 Quaternion_Y_u8[4];
					}Quaternion_Y;	
						union
					{
						float Quaternion_Z_f;
						u8 Quaternion_Z_u8[4];
					}Quaternion_Z;	
					float Euler_Angle_Pitch_s16_2_f;
					float Euler_Angle_Roll_s16_2_f;
					float Euler_Angle_Yaw_s16_2_f;	
} HI220_Stucture;

typedef struct
{
	union
	{
		u8 Flag_Configured;
		struct
		{
			u8 Eout_Configured 	: 1;
			u8 ODR_Configured 	: 1;
			u8 BAUD_Configured 	: 1;
			u8 SETPTL_Configured: 1;
			u8 MODE_Configured 	: 1;
			u8 MCAL_Configured 	: 1;
			u8 Reserve 					: 2;
			
		}Bits;
	}Hi220_Flag_Configured;
	union
	{
		u8 Flag_Reconfig;
		struct
		{
			u8 Eout_Reconfig 	: 1;
			u8 ODR_Reconfig 	: 1;
			u8 BAUD_Reconfig 	: 1;
			u8 SETPTL_Reconfig: 1;
			u8 MODE_Reconfig 	: 1;
			u8 MCAL_Reconfig 	: 1;
			u8 Reserve 				: 2;
			
		}Bits;
	}Hi220_Flag_Reconfig;
}Hi220_Flags_t;

/**************************general gyro*********************************/
typedef struct 
{
	float pitch_Angle;
	float yaw_Angle;
	float roll_Angle;
	float pitch_Gyro;
	float yaw_Gyro;
	float roll_Gyro;
	float x_Acc;
	float y_Acc;
	float z_Acc;
}general_gyro_t;

/****************************ddt motor**********************************************/

typedef struct
{
	uint16_t ID;						//ID
	uint16_t Mode;					//ģʽ
	int16_t current;				//ת�ص���
	int16_t rate_rpm;				//ת��
	int16_t ecd_value;			//������λ��
	uint16_t error_gate;		//������
	uint16_t crc_check;			//CRCУ��

	int16_t ecd_bias;			//��ʼֵ
	int rount_count;		//Ȧ��
	int16_t last_value;	//��һ�εı�����λ��
	float angle;					//�Ƕ�(�ۼ�)
	
}ddtEncoder_t;

/********************general chassis encoder********************************/

typedef struct 
{
	volatile Encoder right_front_GM6020;
	volatile Encoder left_front_GM6020;
	volatile Encoder left_behind_GM6020;
	volatile Encoder right_behind_GM6020;

	volatile Encoder right_front_motor;
	volatile Encoder left_front_motor;
	volatile Encoder left_behind_motor;
	volatile Encoder right_behind_motor;
}steering_wheel_t;

typedef struct
{
	volatile Encoder right_front_motor;
	volatile Encoder left_front_motor;
	volatile Encoder left_behind_motor;
	volatile Encoder right_behind_motor;
}Mecanum_wheel_t;

/***************************general friction encoder********************************************/
typedef struct 
{
	volatile Encoder right_motor1;
	volatile Encoder left_motor1;
	volatile Encoder left_motor2;
	volatile Encoder right_motor2;
}friction_t;

/************************************general poke encoder******************************************************/

typedef struct 
{
	volatile Encoder right_poke;
	volatile Encoder left_poke;
}poke_t;

/****************************************hero small gimbal encoder*****************************************************************/
typedef struct 
{
	volatile Encoder scope_encoder;
	volatile Encoder small_gimbal_encoder;
}hero_small_gimbal_t;
/*******************�������ݿ���ģ��*********************************/
typedef struct
{

	uint16_t mode;
	uint16_t mode_sure;
	
	uint16_t in_power;
	uint16_t in_v;
	uint16_t in_i;
	
	uint16_t out_power;
	uint16_t out_v;
	uint16_t out_i;

	uint16_t tempureture;
	uint16_t time;
	uint16_t this_time;
	
	uint16_t  cap_voltage_filte;
}capacitance_message_t;
/**********************************HT430**********************************/
typedef enum
{
    OFF_STATE=0,
    OPEN_LOOP=1,
    SPEED_MODE=3,
    ANGLE_MODE=5,
} Operating_State_t;
typedef struct{
	uint16_t Angle;//��Ȧ����ֵ�Ƕ�
	int32_t Total_Angle;//��Ȧ����ֵ�Ƕ�
	int16_t V;//���ת��
	Operating_State_t Operating_State;//����״̬
	uint8_t Voltage;//��Դ��ѹ
	uint8_t Currents;//����
	uint8_t Temperature;//�¶�
	uint8_t DTC;//������
}HT430_J10_t;

/****************************Judge***********************************/

#define myCRC8_INIT											0xff
#define UART5_TX_BUF_LENGTH             150
#define UART_RX_DMA_SIZE                1024
#define BSP_UART5_DMA_RX_BUF_LEN        512  
#define BSP_UART5_RX_BUF_SIZE_IN_FRAMES (BSP_UART5_DMA_RX_BUF_LEN / RC_FRAME_LENGTH)
#define RC_FRAME_LENGTH                 18u
#define JUDGE_FRAME_BUFLEN              200
#define ADRESS                          6
#define JudgeBufferLength               150
#define JudgeFrameLength_1              17  //����������״̬���ݳ���
#define JudgeFrameLength_2              15  //ʵʱ�����Ϣ
#define JudgeFrameLength_3              29  //ʵʱ����������Ϣ
#define JudgeFrameHeader                0xA5        //֡ͷ 
#define UP_REG_ID                       0xA0  //up layer regional id
#define DN_REG_ID                       0xA5  //down layer regional id
#define HEADER_LEN                      sizeof(frame_header_t)
#define CMD_LEN                         2    //cmdid bytes
#define CRC_LEN                         2    //crc16 bytes
#define ChassisLimitCurrent             2750            //���̵������Ƽ���
#define CHASSISMAXPOWER                 80.0F       //���������
#define CHASSISMAXPOWERRATE             0.82F       //�������Ƽ��޹��ʣ�80W������(�����ֵΪ0.9����ʵ�����ƹ���Ϊ0.9*80=72W��


#ifdef  __DRIVER_GLOBALS
#define __DRIVER_EXT
#else
#define __DRIVER_EXT extern
#endif
 /** 
  * @brief  judgement data command id
  */
typedef enum
{
	GAME_STATE_ID                      =0x0001,// ����״̬���ݣ�0x0001������Ƶ�ʣ�1Hz 
	GAME_RESULT_ID                     =0x0002,//����������ݣ�0x0002������Ƶ�ʣ������������� 
	GAME_ROBOT_SURVIVORS_ID            =0x0003,//�����˴�����ݣ�0x0003������Ƶ�ʣ�1Hz 
	GAME_ROBOT_HP_ID                   =0x0003,//�����˴�����ݣ�0x0003������Ƶ�ʣ�1Hz 
	EVENT_DADA_ID                      =0x0101,//�����¼����ݣ�0x0101������Ƶ�ʣ��¼��ı���� 
	SUPPLY_PROJECTILE_ACTION_ID        =0x0102,//����վ������ʶ��0x0102������Ƶ�ʣ������ı���� 
	SUPPLY_PROJECTILE_BOOKING_ID       =0x0103,//���󲹸�վ�����ӵ���cmd_id (0x0103)������Ƶ�ʣ����� 10Hz
	REFEREE_WARNING_ID                 =0x0104,//���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢������ 
	GAME_ROBOT_STATE_ID                =0x0201,// ����������״̬��0x0201������Ƶ�ʣ�10Hz 
	POWER_HEAT_DATA_ID                 =0x0202,//ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz 
	GAME_ROBOT_POS_ID                  =0x0203,//������λ�ã�0x0203������Ƶ�ʣ�10Hz 
	BUFF_MUSK_ID                       =0x0204,// ���������棺0x0204������Ƶ�ʣ�״̬�ı���� 
	AERIAL_ROBOT_ENERGY_ID             =0x0205,// ���л���������״̬��0x0205������Ƶ�ʣ�10Hz 
	ROBOT_HURT_ID                      =0x0206,//�˺�״̬��0x0206������Ƶ�ʣ��˺��������� 
	SHOOT_DATA_ID                      =0x0207,//ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������� 
	BULLET_REMAINING_ID                =0x0208,//�ӵ�ʣ�෢������0x0208������Ƶ�ʣ�1Hz ���ڷ��ͣ����л������Լ��ڱ����������ط��� 
	STUDENT_INTERACTIVE_HEADER_DATA_ID =0x0301,//�������ݽ�����Ϣ��0x0301������Ƶ�ʣ����� 10Hz 
	CLIENT_CUSTOM_DATA_ID              =0x0301,//�ͻ��� �ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180������Ƶ�ʣ����� 10Hz 
	ROBOT_INTERACTIVE_DATA_ID          =0x0301,//�������� �����˼�ͨ�ţ�0x0301������Ƶ�ʣ�����10Hz 
	CLIENT_GRAPHIC_DRAW_ID             =0x0301,//�ͻ����Զ���ͼ�� �����˼�ͨ�ţ�0x0301������Ƶ�ʣ����� 10Hz 
	ROBOT_COMMAND_ID                   =0x0303,
} judge_data_id_e;
/** 
  * @brief  2021���ڽӿ�Э��˵�� 
  */
typedef __packed struct
{
	 uint8_t game_type : 4;
	 uint8_t game_progress : 4;
	 uint16_t stage_remain_time;
	 int64_t SyncTimeStamp;
} ext_game_state_t;
	
	typedef __packed struct //����������ݣ�0x0002������Ƶ�ʣ������������� 
	{ 
		uint8_t winner;
	} ext_game_result_t; 
	
typedef __packed struct
{
 uint16_t red_1_robot_HP;
 uint16_t red_2_robot_HP; 
 uint16_t red_3_robot_HP; 
 uint16_t red_4_robot_HP; 
 uint16_t red_5_robot_HP; 
 uint16_t red_7_robot_HP; 
 uint16_t red_outpost_HP;
 uint16_t red_base_HP; 
 uint16_t blue_1_robot_HP; 
 uint16_t blue_2_robot_HP; 
 uint16_t blue_3_robot_HP; 
 uint16_t blue_4_robot_HP; 
 uint16_t blue_5_robot_HP; 
 uint16_t blue_7_robot_HP; 
 uint16_t blue_outpost_HP;
 uint16_t blue_base_HP;
} ext_game_robot_HP_t;

	
	
	typedef __packed struct //�����¼����ݣ�0x0101������Ƶ�ʣ��¼��ı���� 
	{  
	     uint32_t event_type; 
	} ext_event_data_t; 
	
	typedef __packed struct //����վ������ʶ��0x0102������Ƶ�ʣ������ı���� 
	{  
		uint8_t supply_projectile_id;   
		uint8_t supply_robot_id;    
		uint8_t supply_projectile_step; 
		uint8_t supply_projectile_num;
	} ext_supply_projectile_action_t; 
	

	typedef __packed struct //���󲹸�վ�����ӵ���cmd_id (0x0103)������Ƶ�ʣ����� 10Hz��RM �Կ�����δ���� 
	{
		uint8_t supply_projectile_id;  
		uint8_t supply_robot_id; 
		uint8_t supply_num;  
	} ext_supply_projectile_booking_t; 
	typedef __packed struct //���о�����Ϣ��cmd_id (0x0104)������Ƶ�ʣ����淢������ 
	{   
		uint8_t level; 
		uint8_t foul_robot_id; 
	} ext_referee_warning_t; 

	typedef __packed struct
{
	 uint8_t robot_id;
	 uint8_t robot_level;
	 uint16_t remain_HP;
	 uint16_t max_HP;
	 uint16_t shooter_id1_17mm_cooling_rate;
	 uint16_t shooter_id1_17mm_cooling_limit;
	 uint16_t shooter_id1_17mm_speed_limit;
	 uint16_t shooter_id2_17mm_cooling_rate;
	 uint16_t shooter_id2_17mm_cooling_limit;
	 uint16_t shooter_id2_17mm_speed_limit;
	 uint16_t shooter_id1_42mm_cooling_rate;
	 uint16_t shooter_id1_42mm_cooling_limit;
	 uint16_t shooter_id1_42mm_speed_limit;
	 uint16_t chassis_power_limit;
	 uint8_t mains_power_gimbal_output : 1;
	 uint8_t mains_power_chassis_output : 1;
	 uint8_t mains_power_shooter_output : 1;
}ext_game_robot_state_t;

	typedef __packed struct//ʵʱ�����������ݣ�0x0202������Ƶ�ʣ�50Hz 
	{   
		 uint16_t chassis_volt; 
		 uint16_t chassis_current; 
     float chassis_power; 
     uint16_t chassis_power_buffer; 
     uint16_t shooter_id1_17mm_cooling_heat;
     uint16_t shooter_id2_17mm_cooling_heat;
     uint16_t shooter_id1_42mm_cooling_heat;
	} ext_power_heat_data_t; 
 
	typedef __packed struct //������λ�ã�0x0203������Ƶ�ʣ�10Hz 
	{ 
		float x;  
		float y; 
		float z;  
		float yaw; 
	} ext_game_robot_pos_t; 

	typedef __packed struct// ���������棺0x0204������Ƶ�ʣ�״̬�ı���� 
	{   
		uint8_t power_rune_buff; 
	}ext_buff_musk_t; 

	typedef __packed struct//���л���������״̬��0x0205������Ƶ�ʣ�10Hz 
	{  
		uint8_t attack_time;
	} aerial_robot_energy_t; 

	typedef __packed struct //�˺�״̬��0x0206������Ƶ�ʣ��˺��������� 
	{   
		uint8_t armor_id : 4; 
		uint8_t hurt_type : 4; 
	} ext_robot_hurt_t; 
 
	typedef __packed struct// ʵʱ�����Ϣ��0x0207������Ƶ�ʣ�������� 
	{   
		 uint8_t bullet_type;
     uint8_t shooter_id;
		 uint8_t bullet_freq;
     float bullet_speed;
	} ext_shoot_data_t; 
	
	typedef __packed struct
 {
     uint16_t bullet_remaining_num_17mm;
     uint16_t bullet_remaining_num_42mm;
     uint16_t coin_remaining_num;
 } ext_bullet_remaining_t;
	


	
	typedef __packed struct//�ͻ����Զ������ݣ�cmd_id:0x0301������ ID:0xD180
	{ 
		float data1;
		float data2;
		float data3; 
		uint8_t masks; 
	} client_custom_data_t ;
	
/*****************************�����˼佻������**************************************/	
	
	typedef __packed struct//�������ݽ�����Ϣ��0x0301������Ƶ�ʣ����� 10Hz 
	{  
		uint16_t data_cmd_id;  
		uint16_t send_ID;  
		uint16_t receiver_ID;
	}ext_student_interactive_header_data_t; 
	
	typedef __packed struct //ѧ�������˼�ͨ�� cmd_id 0x0301������ ID:0x0200~0x02FF 
	{ 
		uint8_t data[6];
	} robot_interactive_data_t ;
	
	
	typedef __packed struct  //�ͻ���ɾ��ͼ��
{
	uint8_t operate_tpye; 
	uint8_t layer; 
} ext_client_custom_graphic_delete_t;


typedef __packed struct //ͼ������  �ͻ����Զ���ͼ�� �����˼�ͨ�ţ�0x0301������Ƶ�ʣ����� 10Hz 
{ 
uint8_t graphic_name[3]; 
uint32_t operate_type:3; 
uint32_t graphic_type:3; 
uint32_t layer:4; 
uint32_t color:4; 
uint32_t start_angle:9;
uint32_t end_angle:9;
uint32_t width:10; 
uint32_t start_x:11; 
uint32_t start_y:11; 
uint32_t radius:10; 
uint32_t end_x:11; 
uint32_t end_y:11; 
}graphic_data_struct_t;
	

typedef __packed struct  //�ͻ��˻���һ��ͼ��
{
 graphic_data_struct_t grapic_data_struct;
} ext_client_custom_graphic_single_t;


typedef __packed struct  //�ͻ��˻�������ͼ��
{
graphic_data_struct_t grapic_data_struct[2];
} ext_client_custom_graphic_double_t;

typedef __packed struct  ////�ͻ��˻������ͼ��
{
  graphic_data_struct_t grapic_data_struct[5];
}ext_client_custom_graphic_five_t;

typedef __packed struct  //�ͻ��˻����ַ�
{
	graphic_data_struct_t grapic_data_struct;
	uint8_t data[30];
} ext_client_custom_character_t;


typedef __packed struct  //�ͻ��˻����߸�ͼ��
{
graphic_data_struct_t grapic_data_struct[7];
} ext_client_custom_graphic_seven_t;

typedef __packed struct
{
float target_position_x;
float target_position_y;
float target_position_z;
uint8_t commd_keyboard;
uint16_t target_robot_ID;
} ext_robot_command_t;

typedef __packed struct
{
uint16_t target_robot_ID;
float target_position_x;
float target_position_y;
} ext_client_map_command_t;

/*********************�����˼佻������end***************************/



/** 
  * @brief  student custom data
  */
typedef __packed struct
{
  float data1;
  float data2;
  float data3;
	uint8_t mask;
} client_show_data_t;//client_show_data

typedef enum
{
  unkown = 0,
  blue = 1,
  red  = 2,
} robot_color_e;

/** 
  * @brief  the data structure receive from judgement
  */
typedef struct
{ 
		ext_game_state_t                      game_state;//����״̬����
		ext_game_result_t                     game_result;//�����������
		ext_game_robot_HP_t                   game_robot_HP;//�����˴������
		ext_event_data_t                      event_data;//�����¼�����
		ext_supply_projectile_action_t        supply_projectile_action;//����վ������ʶ
	  ext_referee_warning_t                 referee_warning;//���о�����Ϣ
		ext_supply_projectile_booking_t       supply_projectile_booking;//���󲹸�վ�����ӵ�
		ext_game_robot_state_t                game_robot_state;//����������״̬
		ext_power_heat_data_t                 power_heat_data;//ʵʱ������������
		ext_game_robot_pos_t                  game_robot_pos;//������λ��
		ext_buff_musk_t                       buff_musk;//����������
		aerial_robot_energy_t                 aerial_robot_energy;//���л���������״̬
		ext_robot_hurt_t                      robot_hurt;//�˺�״̬
		ext_shoot_data_t                      shoot_data;// ʵʱ�����Ϣ
	  ext_bullet_remaining_t                ext_bullet_remaining;

		ext_student_interactive_header_data_t student_interactive_header_data;//�������ݽ�����Ϣ
		client_custom_data_t                  client_custom_data;//�ͻ��� �ͻ����Զ�������
	    robot_interactive_data_t              robot_interactive_data;//�������� �����˼�ͨ��
	    graphic_data_struct_t             graphic_data_struct;//�ͻ����Զ���ͼ�� 
	   ext_robot_command_t                 ext_robot_command;
		 ext_client_map_command_t             ext_client_map_command;

		 robot_color_e                         robot_color;
//		game_robot_state_t game_information;
//		robot_hurt_data_t  blood_changed_data;
//		real_shoot_t       real_shoot_data;
//		PowerHeatData_t    PowerHeatData_data;
//		rfid_detect_t      rfid_data;
//		game_result_t      game_result_data;
//		get_buff_t         get_buff_data;
//		GameRobotPos_t     GameRobotPos_data;
//		server_to_user_t   student_download_data;
} receive_judge_t;

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;
/***********************************autoshoot****************************************/

typedef struct
{
  float x;
  float y;
  int16_t x1;
  int16_t y1;
  int16_t dis;
  uint8_t flag;
  uint8_t xy_0_flag;
  uint8_t color;
  int16_t receNewDataFlag;
  int16_t id;
  uint8_t crc;
  float yaw_speed;
  float pitch_speed;
} location;

/***********************************ң����*********************************************/

typedef __packed struct
{
	int16_t ch0;
	int16_t ch1;
	int16_t ch2;
	int16_t ch3;
	int16_t ch4;
	int8_t s1;
	int8_t s2;
}Remote;

typedef __packed struct
{
	int16_t x;
	int16_t y;
	int16_t z;
	uint8_t last_press_l;
	uint8_t last_press_r;
	uint8_t press_l;
	uint8_t press_r;
}Mouse;	

typedef	__packed struct
{
	uint16_t v;
	uint16_t last_v;
}Key;

typedef enum
{
  KEY_R_UP=0,
  KEY_R_DOWN=1,
 
} key_state_t;

typedef __packed struct
{
	Remote rc;
	Mouse mouse;
	Key key;
}RC_Ctl_t;

/***************************senior function*************************************/
void CH100_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO);

static void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes);
void HI220_getDATA(uint8_t *DataAddress,general_gyro_t *GYRO,uint8_t length);

void GetEncoderBias(volatile Encoder *v, CanRxMsg * msg);
void EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void GM6020EncoderProcess(volatile Encoder *v, CanRxMsg * msg);
void M3508orM2006EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg);
void GM6020EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset);
void MF_EncoderProcess(volatile Encoder *v, CanRxMsg * msg);//��̨yaw��pitch����
void MF_EncoderTask(uint32_t can_count,volatile Encoder *v, CanRxMsg * msg,int offset);

void PM01_message_Process(volatile capacitance_message_t *v,CanRxMsg * msg);

void HT_430_Information_Receive(CanRxMsg * msg,HT430_J10_t *HT430_J10_t,volatile Encoder *v);

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
unsigned char get_crc8(unsigned char* data, unsigned int length);

void judgement_data_handle(uint8_t *p_frame,u16 rec_len);

void vision_process_general_message(unsigned char* address, unsigned int length);
void send_protocol(float x, float y, float r, int id, float ammo_speed, int gimbal_mode, u8 *data);

void RemoteDataPrcess(uint8_t *pData);
/**************general_gyro define**********************/
extern general_gyro_t gimbal_gyro;
extern general_gyro_t chassis_gyro;
extern steering_wheel_t steering_wheel_chassis;
extern Mecanum_wheel_t Mecanum_chassis;
extern volatile Encoder Pitch_Encoder;
extern volatile Encoder yaw_Encoder;
extern hero_small_gimbal_t hero_small_gimbal;
extern friction_t general_friction;
extern poke_t general_poke;
extern volatile capacitance_message_t capacitance_message;
extern receive_judge_t judge_rece_mesg;
extern location new_location;
extern RC_Ctl_t RC_CtrlData;
#endif


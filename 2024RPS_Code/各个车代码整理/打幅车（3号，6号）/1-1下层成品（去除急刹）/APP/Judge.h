#ifndef __JUDGE_H
#define __JUDGE_H
#include "main.h"

#define UART5_RX_BUF_LENGTH   100
#define UART5_TX_BUF_LENGTH   100
#define myCRC8_INIT											0xff
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

		ext_student_interactive_header_data_t student_interactive_header_data;//�������ݽ�����Ϣ
		client_custom_data_t                  client_custom_data;//�ͻ��� �ͻ����Զ�������
	    robot_interactive_data_t              robot_interactive_data;//�������� �����˼�ͨ��
	    graphic_data_struct_t             graphic_data_struct;//�ͻ����Զ���ͼ�� 
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

/* data send (forward) */
/* data receive */


//��ʽת��������
typedef union
{
    uint8_t U[4];
    float F;
    int I;
}FormatTrans;



//С��״̬ö��
//typedef enum
//{
//    BUFF_TYPE_NONE, //��Ч
//    BUFF_TYPE_ARMOR = 0x01, //������
//    BUFF_TYPE_SUPPLY = 0x04, //��Ѫ��
//    BUFF_TYPE_BULLFTS= 0x08, //�ӵ���
//}LBuffType_Enum;


//λ��״̬�ṹ��
typedef __packed struct
{
    uint8_t flag; //0 ��Ч�� 1 ��Ч
    uint32_t x;
    uint32_t y;
    uint32_t z;
    uint32_t compass;
}GpsData_Struct;


//����������Ϣ�ṹ��
typedef __packed struct
{
    uint32_t remainTime;
    uint16_t remainLifeValue;
    float realChassisOutV;
    float realChassisOutA;
    uint8_t runeStatus[4];
    uint8_t bigRune0Status;
    uint8_t bigRune1status;
    uint8_t conveyorBelts0:2;
    uint8_t conveyorBelts1:2;
    uint8_t parkingApron0:1;
    uint8_t parkingApron1:1;
    uint8_t parkingApron2:1;
    uint8_t parkingApron3:1;
    GpsData_Struct gpsData;
}GameInfo_Struct;

//����ϵͳ�ṹ��
typedef struct
{
    float RealVoltage;                  //ʵʱ��ѹ
    float RealCurrent;                  //ʵʱ����
    int16_t LastBlood;                  //ʣ��Ѫ��
    uint8_t LastHartID;                 //�ϴ��յ��˺���װ�װ�ID��
    uint32_t LastHartTick;          //�ϴ����˺�ʱ�� 
    float LastShotSpeed;                //�ϴ�����ٶ�
    uint32_t LastShotTick;          //�ϴ����ʱ��
		float LastPower;                //ʣ������
#if INFANTRY == 7
    uint16_t ShootNum;                  //�ѷ����ӵ���
    uint8_t BulletUseUp;                //1 �����ӵ�����          0 �����ӵ�δ����
    uint16_t ShootFail;                 //����ʧ��ʱ�� 
#endif
}InfantryJudge_Struct;


/* data send (forward) */
/* data receive */
typedef enum
{
  STEP_HEADER_SOF  = 0,
  STEP_LENGTH_LOW  = 1,
  STEP_LENGTH_HIGH = 2,
  STEP_FRAME_SEQ   = 3,
  STEP_HEADER_CRC8 = 4,
  STEP_DATA_CRC16  = 5,
} unpack_step_e;

typedef enum
{
  UART_IDLE_IT     = 0,
  UART_DMA_HALF_IT = 1,
  UART_DMA_FULL_IT = 2,
} uart_it_type_e;

typedef struct
{
  //UART_HandleTypeDef *huart;
  FIFO_S_t           *data_fifo;
  uint16_t           buff_size;
  uint8_t            *buff[2];
  uint16_t           read_index;
  uint16_t           write_index;
} uart_dma_rxdata_t;

typedef __packed struct
{
  uint8_t  sof;
  uint16_t data_length;
  uint8_t  seq;
  uint8_t  crc8;
} frame_header_t;

typedef struct
{
  FIFO_S_t       *data_fifo;
  frame_header_t *p_header;
  uint16_t       data_len;
  uint8_t        protocol_packet[PROTOCAL_FRAME_MAX_SIZE];
  unpack_step_e  unpack_step;
  uint16_t       index;
} unpack_data_t;


/*FIFO*/
#define ASSERT(x) do {while(!(x));} while(0)
 
#define MUTEX_WAIT() \
do {\
  osMutexWait(pfifo->mutex, osWaitForever);\
} while(0)\

#define MUTEX_RELEASE() \
do {\
  osMutexRelease(pfifo->mutex);\
} while(0)\


//! FIFO Memory Model (Single Byte Mode)
typedef struct
{
  uint8_t   *start_addr;                   //Start Address
  uint8_t   *end_addr;                     //End Address
  uint32_t  free;                         //The capacity of FIFO
  uint32_t  buf_size;                     //Buffer size
  uint32_t  used;                         //The number of elements in FIFO
  uint8_t   read_index;                   //Read Index Pointer
  uint8_t   write_index;                  //Write Index Pointer
  //osMutexId mutex;
} fifo_s_t;


//fifo_s_t* fifo_s_create(uint32_t unit_cnt, osMutexId mutex);
void     fifo_s_destory(fifo_s_t* pfifo);

//int32_t fifo_s_init(fifo_s_t* pfifo, void* base_addr, uint32_t unit_cnt, osMutexId mutex);

int32_t fifo_s_put(fifo_s_t* pfifo, uint8_t element);
int32_t fifo_s_puts(fifo_s_t *pfifo, uint8_t *psource, uint32_t number);
int32_t fifo_s_puts_no_mutex(fifo_s_t *pfifo, uint8_t *psource, uint32_t number);

uint8_t  fifo_s_get(fifo_s_t* pfifo);
uint8_t  fifo_s_get_no_mutex(fifo_s_t* pfifo);

uint16_t fifo_s_gets(fifo_s_t* pfifo, uint8_t* source, uint8_t len);
uint16_t fifo_s_gets_no_mutex(fifo_s_t* pfifo, uint8_t* source, uint8_t len);

uint8_t  fifo_s_pre_read(fifo_s_t* pfifo, uint8_t offset);

uint8_t  fifo_is_empty(fifo_s_t* pfifo);
uint8_t  fifo_is_full(fifo_s_t* pfifo);
uint32_t fifo_used_count(fifo_s_t* pfifo);
uint32_t fifo_free_count(fifo_s_t* pfifo);
uint8_t  fifo_flush(fifo_s_t* pfifo);
//����ϵͳ���ݻ���
extern uint8_t JudgeDataBuffer[JudgeBufferLength];
//ʵʱ��ѹ
extern InfantryJudge_Struct InfantryJudge;
//֡�ʼ�����
extern float JudgeFrameCounter;
//֡��
extern float JudgeFrameRate;
//��������ܵ�������
extern float ChassisMaxSumCurrent;
extern receive_judge_t judge_rece_mesg;
extern judge_data_id_e id;
extern client_show_data_t client_show;
extern uint8_t  tx_buf[66];
extern uint8_t  pdata[32];
extern uint8_t  ddata[66];
extern FIFO_S_t  _UART5_RX_FIFO;
extern u8 shoot_num;


void get_distance3(void);

void UART5_Configuration(void);

void BSP_UART6_InitConfig(void);
void UART6_IRQHandler(void);
static void UART5_FIFO_Init(void);

unsigned char Get_CRC8_Check_Sum(unsigned char *pchMessage,unsigned int dwLength,unsigned char ucCRC8);
unsigned int Verify_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
void Append_CRC8_Check_Sum(unsigned char *pchMessage, unsigned int dwLength);
uint16_t Get_CRC16_Check_Sum(uint8_t *pchMessage,uint32_t dwLength,uint16_t wCRC);
uint32_t Verify_CRC16_Check_Sum(uint8_t *pchMessage, uint32_t dwLength);
void Append_CRC16_Check_Sum(uint8_t * pchMessage,uint32_t dwLength);
void BSP_UART5_InitConfig(void);
void get_dma_memory_msg(DMA_Stream_TypeDef *dma_stream, uint8_t *mem_id, uint16_t *remain_cnt);
extern Shoot_Limit_Mode_e Shoot_Limit_Mode; 
extern receive_judge_t judge_rece_mesg;
extern u8 have_heat0_flag ;
extern u32 bullet_num;
extern uint8_t UART5_DMA_TX_BUF[UART5_TX_BUF_LENGTH];
void data_upload_handle(uint16_t cmd_id, uint8_t *p_data, uint16_t len, uint8_t sof, uint8_t *tx_buf);
void UART5_PrintBlock(uint8_t* pdata, uint8_t len);
void judgement_data_handle(uint8_t *p_frame,u16 rec_len);
void Usart5SendBytesInfoProc(u8* pSendInfo, u16 nSendCount);
unsigned char get_crc8(unsigned char* data, unsigned int length);
#endif

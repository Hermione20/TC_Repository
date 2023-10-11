#include "main.h"

/* *** *** �Զ������ض��� ��ʼ *** *** */
uint8_t dataFromMF[7];		        //���ݻ���
uint8_t dataFromMFReadyFlag = 0; 		//�������ݽ�����ɱ�־λ

location new_location;            //������������ֵ

int sucflag = 0;    //�ɹ����յ�һ������
int recflag = 0;    //���ܵ�������ʶ��װ��
int receive = 0;    //���յ����ݸ���
/* *** *** ********************** *** *** */
int num_command=0;
uint8_t auto_shoot_mode_set;
robot_color_e robot_color = 1 ;   //0-9���±�ʶ�Լ����Ǻ췽��������������
/********* �Զ������ض��� ���� *********/
int wExpected = 0;
float auto_clck;
Signal *Uart4_Protobuf_Receive_Message;
int16_t Uart4_Content_Size;
u8 *CRC_Content_buf;
HostToDevice__Frame *Uart4_Protobuf_Receive_Gimbal_Angle;
//extern unsigned char get_crc8(unsigned char* data, unsigned int length);
float flagg_pitch;
float flagg_yaw;
float xy_0_flag;
float xy_o_time;
float fcount;
float usart_time;
void targetOffsetDataDeal ( uint8_t len, u8 *buf )
{
  receive++;
  usart_time=fcount;
	fcount=0;
//	CRC_Content_buf=(u8*)(buf+5);
//
//	Uart4_Content_Size=(int16_t)buf[4]<<8|(int16_t)buf[3];
//
//	Uart4_Protobuf_Receive_Message=signal__unpack(NULL,Uart4_Content_Size,CRC_Content_buf);
//
//  if (buf[0]==0xBE &&
//			get_crc8( CRC_Content_buf, Uart4_Content_Size) == buf[5+Uart4_Content_Size] &&
//			buf[5+Uart4_Content_Size+2 -1 ]==0xED )
//    {
//      new_location.receNewDataFlag = 1;    //��ⶪ֡��־λ,���������װ�װ�����ݣ�����û��⵽����Ϊ��֡
////      if ( buf[1] != 0 )
////        {
////          new_location.x =  -( ( ( int16_t ) buf[3] << 8 | ( int16_t ) buf[2]  ) - GIMBAL_YAW_MID );  //��������ͼ��������������Ͻ�ԭ��
////          new_location.y =  ( ( ( int16_t ) buf[5] << 8 | ( int16_t ) buf[4] ) - GIMBAL_PIT_MID );
////          new_location.flag =  buf[1];
////          new_location.dis =  ( ( int16_t ) buf[7] << 8 | ( int16_t ) buf[6] );
////        }
////      else
////        {

////          new_location.x = 0;
////          new_location.y = 0;
////          new_location.flag =  0 ;      //0�ҵ�װ�װ壬1δ�ҵ�װ�װ�
////          new_location.crc = buf[9] ;
////        }
////			new_location.x=Uart4_Protobuf_Receive_Message->yaw;
////			new_location.x=Uart4_Protobuf_Receive_Message->pitch;
//    }
//		signal__free_unpacked(Uart4_Protobuf_Receive_Message,NULL);
//			parse_signal(buf,len);
  process_general_message(buf,len);
}


void parse_signal(unsigned char* content_address, unsigned int content_length)
{
  //Todo: parse and process signal.
  Uart4_Protobuf_Receive_Message=signal__unpack(NULL,content_length,content_address);
  if(strcmp(Uart4_Protobuf_Receive_Message->name,"t_c_d"))
    {
      chassis.ctrl_mode=MANUAL_SEPARATE_GIMBAL;
    }
  else if(strcmp(Uart4_Protobuf_Receive_Message->name,"t_c_a"))
    {
      gim.ctrl_mode=GIMBAL_INIT;
    }
  signal__free_unpacked(Uart4_Protobuf_Receive_Message,NULL);
}

float yaw_buff=0;

void parse_turret_command(unsigned char* content_address, unsigned int content_length)
{
//	// Protobuf ��ȷ�÷����½�һ������ȥ�����ݣ����㲻�½�����ô���ʼ��һ�°�ɵX����
//	Uart4_Protobuf_Receive_Gimbal_Angle->current_pitch_ = 0.0f;
//	Uart4_Protobuf_Receive_Gimbal_Angle->current_yaw_ = 0.0f;
////	Uart4_Protobuf_Receive_Gimbal_Angle->command = 0;
//  Uart4_Protobuf_Receive_Gimbal_Angle=device_to_host__frame__unpack(NULL,content_length,content_address);
//  flagg_pitch=Uart4_Protobuf_Receive_Gimbal_Angle->current_pitch_;
//  flagg_yaw=Uart4_Protobuf_Receive_Gimbal_Angle->current_yaw_;
//      /*���������⣬�����Ƿ���*/
//	new_location.receNewDataFlag  =  1;
////	if(Uart4_Protobuf_Receive_Gimbal_Angle->command == 1)
//	{	
//			new_location.x=  Uart4_Protobuf_Receive_Gimbal_Angle->current_yaw_-yaw_buff;
//			new_location.y=  Uart4_Protobuf_Receive_Gimbal_Angle->current_pitch_;
////		  new_location.dis = Uart4_Protobuf_Receive_Gimbal_Angle->distance;
//			new_location.flag = 1;	
////		  LASER_ON();
//	}
//	
//  device_to_host__frame__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle,NULL);
}

typedef void(*Parser)(unsigned char* content_address, unsigned int content_length);

Parser parsers[] =
{
  NULL,
  &parse_signal,
  NULL,
  &parse_turret_command
};
unsigned int parsers_count = sizeof(parsers) / sizeof(Parser);

#ifndef DEBUG_MODE
#define DEBUG_MODE
#endif

#ifdef DEBUG_MODE
  // Counter for received packages.
  int counter_receive = 0;
  // Counter for packages that passed the head and tail check.
  int counter_complete = 0;
  // Contuner for pacakages that passed the CRC8 check.
  int counter_crc_passed = 0;
#endif

unsigned short package_id ;
unsigned short content_size;
unsigned short content_size1;
unsigned char getaddress[100];
float ab1,ab2;
float flag_x,flag_y;
float flagg_x,flagg_y;
u8 first_len[4];
u8 i;
float xy_0_flag;
float xy_o_time;
float this_yaw_angle,this_pit_angle;
float last_this_yaw_angle,last_this_pit_angle;
void process_general_message(unsigned char* address, unsigned int length)
{
	last_this_yaw_angle=this_yaw_angle;//10ms֮ǰ�ĽǶ�
	last_this_pit_angle=pitch_Angle;
	
	this_yaw_angle=yaw_Angle ;
	this_pit_angle=pitch_Angle ;
	
if (address[0] != 0xBE) return;
	i=0;
	for(u8 k=0;k<length;k++)
	{if(address[k]==0xED)
		{
		first_len[i]=k;
			i++;
		}	
	}
	content_size =first_len[0]-3;
  content_size1 = address[1];

#ifdef DEBUG_MODE
  if (length > 0) ++counter_receive;
#endif


for(int k=0;k<content_size;k++)
	{getaddress[k]=address[k+2];}	

		  if (address[0] != 0xBE) return;
  if (address[content_size+3] != 0xED) return;
//if(address[length-1-2] == 0xED)
//{memset (UART4_DMA_RX_BUF,0,sizeof (UART4_DMA_RX_BUF))}
#ifdef DEBUG_MODE
  ++counter_complete;
#endif

  unsigned char* content_address;
		content_address = getaddress;

  unsigned char crc8 = address[2 + content_size];
ab1=crc8;
ab2=get_crc8(content_address, content_size);

  if (crc8 != get_crc8(content_address, content_size)) 
		return;

#ifdef DEBUG_MODE
  ++counter_crc_passed;
#endif
Uart4_Protobuf_Receive_Gimbal_Angle=host_to_device__frame__unpack(NULL,content_size,content_address);
auto_clck=0;
flag_x=Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_;
flag_y=Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_;

	if(flag_y==Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_&&flag_x==Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_)
	{
	if(flag_x!=0&&flag_y!=0)
	{ 
			new_location.x=  Uart4_Protobuf_Receive_Gimbal_Angle->target_yaw_;
			new_location.y=  Uart4_Protobuf_Receive_Gimbal_Angle->target_pitch_;
		  new_location.pitch_speed=Uart4_Protobuf_Receive_Gimbal_Angle->pitch_speed;
		  new_location.yaw_speed =Uart4_Protobuf_Receive_Gimbal_Angle->yaw_speed;
		  new_location.flag=1;
		  autoshoot_kf_flag = 1;
	}
	else
  {
	  new_location.flag=0;
		new_location.yaw_speed=0;
		new_location.pitch_speed=0;
	}
}
	
flagg_x=Uart4_Protobuf_Receive_Gimbal_Angle->x_;
flagg_y=Uart4_Protobuf_Receive_Gimbal_Angle->y_;

//if(flagg_x==(-640)&&flagg_y==(-512))
//{
//	new_location.x1=new_location.x1;
//	new_location.y1=new_location.y1;
//}
//else
//{
//	if(flagg_x==Uart4_Protobuf_Receive_Gimbal_Angle->x_&&flagg_y==Uart4_Protobuf_Receive_Gimbal_Angle->y_&&flagg_x!=(-640)&&flagg_y!=(-512))
		
  	if(flagg_x!=0&&flagg_y!=0)
		{   
			  xy_0_flag=0;
			  buff_kf_flag=1;
				new_location.x1=  Uart4_Protobuf_Receive_Gimbal_Angle->x_;
				new_location.y1=  Uart4_Protobuf_Receive_Gimbal_Angle->y_;
		}
		else{xy_0_flag =1;}    //û���յ���һ������

	host_to_device__frame__free_unpacked(Uart4_Protobuf_Receive_Gimbal_Angle,NULL);
//}

}

	DeviceToHost__Frame msg;
  u8 DateLength;

void send_protocol(float x,float y,int id)
{
  device_to_host__frame__init(&msg);


	if(gim.ctrl_mode==GIMBAL_AUTO_SMALL_BUFF)
	{msg.mode_= 2;}
	else if(gim.ctrl_mode==GIMBAL_AUTO_BIG_BUFF)
	{msg.mode_= 1;}
	else{msg.mode_= 0;}
//	msg.mode_= 2;

	msg.current_pitch_=y;
	msg.current_yaw_=x;
	msg.current_color_=id;
	msg.bullet_speed_= (float)13.0f;//FRICTION_SPEED_plan_ref;
	msg.current_roll_=roll_Angle;
	msg.pitch_speed_=pitch_Gyro;
	msg.yaw_speed_=yaw_Gyro;


  device_to_host__frame__pack(&msg,UART4_DMA_TX_BUF+2);
  DateLength=device_to_host__frame__get_packed_size(&msg);
  UART4_DMA_TX_BUF[0]=0xBE;
	UART4_DMA_TX_BUF[1]=DateLength;
  Append_CRC8_Check_Sum(&UART4_DMA_TX_BUF[2],DateLength+1);
  UART4_DMA_TX_BUF[DateLength+3]=0xED;
	Uart4SendBytesInfoProc(UART4_DMA_TX_BUF,DateLength+4);

}


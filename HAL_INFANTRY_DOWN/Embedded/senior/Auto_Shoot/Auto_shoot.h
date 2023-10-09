#ifndef __AUTO_SHOOT_H
#define __AUTO_SHOOT_H
#include "public.h"
#include "send.pb-c.h"
#include "Recieve.pb-c.h"

/***********************************autoshoot****************************************/

typedef struct
{
  float x;                 //������yaw����
  float y;                 //������pitch����
  int16_t x1;              //yaw���
  int16_t y1;              //pitch���
  int16_t control_flag;    //�Ƿ��������ģʽ�ı�־λ���ڱ��ã�
  uint8_t flag;            //�����Ƿ�ʶ��
  uint8_t xy_0_flag;       //����Ƿ�ʶ��
  float xy_o_time;         //�����־λ
  int16_t lost_cnt;
  float yaw_speed;
  float pitch_speed;
  float last_x;
  float last_y;
	u8 buff_kf_flag;
} location;

void vision_process_general_message(unsigned char* address, unsigned int length);
void send_protocol(float x, float y, float r, int id, float ammo_speed, int gimbal_mode, u8 *data);


extern location new_location;


#endif


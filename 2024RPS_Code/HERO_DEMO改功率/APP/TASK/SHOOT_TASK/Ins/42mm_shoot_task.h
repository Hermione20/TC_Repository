#ifndef __42MM_SHOOT_TASK_H
#define __42MM_SHOOT_TASK_H
#include "public.h"



#define FRICTION_SPEED_10 2000
#define FRICTION_SPEED_12 2100
#define FRICTION_SPEED_14 2575
#define FRICTION_SPEED_16 3200

#define POKE_SPEED -200			//英雄下拨盘转速
#define POKE_MAX_OUT 6000		//英雄下拨盘力度限制
#define ONE_POKE_ANGLE_42 90.0f	//42mm单个弹丸角度
#define ONE_POKE_ANGLE_17 -30.0f	//17mm单个弹丸角度

typedef enum
{
    Stop = 0,
    START = 1,
    NORMAL= 2,
    BACK = 3,
    LOCK = 4
} friction_state_t; // 1正常 2堵转

typedef struct 
{
   float up_poke_angle_ref;
   float up_poke_angle_fdb;
   float up_poke_speed_ref;
   float up_poke_speed_fdb;

   float down_poke_speed_ref;
   float down_poke_speed_fdb;

   float left_friction_speed_ref;
   float left_friction_speed_fdb;

   float right_friction_speed_ref;
   float right_friction_speed_fdb;

   int16_t up_poke_motor_input;
   int16_t down_poke_motor_input;
   int16_t left_friction_motor_input;
   int16_t right_friction_motor_input;
}shoot_ref_and_fdb_t;


typedef struct
{
    shoot_ref_and_fdb_t shoot_ref_and_fdb;
    friction_state_t friction_state;
    u8 shoot_flag;

    pid_t pid_uppoke_angle;
    pid_t pid_uppoke_speed;

    pid_t pid_downpoke_speed;
    
    pid_t pid_left_friction_speed;
    pid_t pid_right_friction_speed;
} _42mm_shoot_t;




void shoot_task(void);
static void Shoot_42mm_speed_Select(uint16_t test_frictionSpeed_42);
void heat_limit_42mm(u8 ifignore);
void shoot_friction_handle_42(void);
















#endif


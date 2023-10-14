#ifndef __17MM_SHOOT_TASK
#define __17MM_SHOOT_TASK
#include "public.h"

#if  STANDARD == 3

#define FRICTION_SPEED_15  (-540)      //µØÀŸ
#define FRICTION_SPEED_18  (-607)
#define FRICTION_SPEED_30  (-935)

#define residual_heat_normal_else_1 (40)
#define residual_heat_normal_else_2 (28)
#define residual_heat_normal_else_3 (28)
#define residual_heat_normal_else_4 (25)


#define residual_heat_normal_coling_1 (32)
#define residual_heat_normal_coling_2 (25)
#define residual_heat_normal_coling_3 (27)

#define residual_heat_normal_speed_1 (27)
#define residual_heat_normal_speed_2 (27)
#define residual_heat_normal_speed_3 (25)

#define residual_heat_normal_outburst_1 (30)
#define residual_heat_normal_outburst_2 (23)
#define residual_heat_normal_outburst_3 (40)

#define residual_heat_middle_coling_1 (100)
#define residual_heat_middle_coling_2 (35)
#define residual_heat_middle_coling_3 (34)


#define residual_heat_middle_speed_1 (100)
#define residual_heat_middle_speed_2 (45)
#define residual_heat_middle_speed_3 (43)


#define residual_heat_middle_outburst_1 (100)
#define residual_heat_middle_outburst_2 (35)
#define residual_heat_middle_outburst_3 (80)

//…‰∆µ
#define PID_SHOOT_MOTOR_SPEED_1    (-300)//-400
#define PID_SHOOT_MOTOR_SPEED_2    (-400)//-250
#define PID_SHOOT_MOTOR_SPEED_3    (-450)
#define PID_SHOOT_MOTOR_SPEED_4    (-500)
#define PID_SHOOT_MOTOR_SPEED_5    (-600)
#endif
typedef enum
{
  OUTBURST  = 0,
  COOLING   = 1,
  ZOOM     = 2,

} shoot_mode_selection_e;

typedef enum
{
  SHOT_DISABLE       = 0,
  REMOTE_CTRL_SHOT   = 1,
  KEYBOARD_CTRL_SHOT = 2,
  SEMIAUTO_CTRL_SHOT = 3,
  AUTO_CTRL_SHOT     = 4,

} shoot_mode_e;

typedef enum
{
  TRIG_INIT       = 0,
  TRIG_PRESS_DOWN = 1,
  TRIG_BOUNCE_UP  = 2,
  TRIG_ONE_DONE   = 3,
} trig_state_e;

typedef struct
{
	int16_t speed_ref[2];
	int16_t speed_fdb[2];
	int16_t angle_ref[2];
	int16_t angle_fdb[2];
	
}shoot_pid_friction_t;

typedef struct
{
	int16_t speed_ref;
	int16_t speed_fdb;
	int16_t angle_ref;
	int16_t angle_fdb;
	
}shoot_pid_poke_t;

typedef struct
{
  /* shoot task relevant param */
	shoot_mode_selection_e  shoot_mode_selection;
  shoot_mode_e 						ctrl_mode;
	shoot_pid_poke_t        poke_pid;
	shoot_pid_friction_t		friction_pid;
  uint8_t      shoot_cmd;
  uint32_t     c_shoot_time;   //continuous
  uint8_t      c_shoot_cmd;
  uint8_t      fric_wheel_run; //run or not
  uint16_t     fric_wheel_spd;
  uint16_t     ref_shot_bullets;
  uint16_t     shot_bullets;
  uint16_t     remain_bullets;
  float        total_speed;
  float        limit_heart0;
  uint16_t     max_heart0;
  uint16_t     handle_timescouter;
  uint16_t     cooling_ratio;
  uint16_t     ShootMotorSpeed;
  uint16_t     NoLimitHeat;
  uint8_t			 Speed_Gear;
} shoot_t;

void shoot_task(void);
void performance_select(void);
void shoot_mode_switch(void);
void speed_switch(void);
void heat_switch(void);
void shoot_bullet_handle(void);

extern int bulletspead_level;




















#endif
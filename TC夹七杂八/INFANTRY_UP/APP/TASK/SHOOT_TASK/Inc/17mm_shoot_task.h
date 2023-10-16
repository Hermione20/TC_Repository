#ifndef __17MM_SHOOT_TASK
#define __17MM_SHOOT_TASK
#include "public.h"

#define SHOOT_TYPE 3 //3²½±ø 6ÎÞÈË»ú  7ÉÚ±ø

#define SHOOT_MOTOR_SPEED 550.0f //ÉÚ±ø²¦ÅÌ
#define FRICTION_SPEED    950// 950

#if  STANDARD == 3

#define FRICTION_SPEED_15  (-540)      //µ¯ËÙ
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

//ÉäÆµ
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
	int16_t speed_ref[4];
	int16_t speed_fdb[4];
	int16_t angle_ref[4];
	int16_t angle_fdb[4];
	
}shoot_pid_friction_t;

typedef struct
{
	int16_t speed_ref[2];
	int16_t speed_fdb[2];
	int16_t angle_ref[2];
	int16_t angle_fdb[2];
	
}shoot_pid_poke_t;

typedef struct
{
  /* shoot task relevant param */
	shoot_mode_selection_e  shoot_mode_selection;
  shoot_mode_e 						ctrl_mode;
	shoot_pid_poke_t        poke_pid;
	shoot_pid_friction_t		friction_pid;
	float        poke_current[2];
	float        fric_current[4];
	uint8_t      poke_run;
	uint8_t      bulletspead_level;
  uint8_t      fric_wheel_run; //run or not
  uint16_t     fric_wheel_spd;
  uint16_t     will_time_shoot;
  uint16_t     remain_bullets;
	uint8_t        single_angle;
	float        shoot_frequency;
  float        total_speed;
  float        limit_heart0;
	float        limit_heart1;
  uint16_t     max_heart0;
  uint16_t     cooling_ratio;
} shoot_t;


void heat0_limit(void);
void shoot_task(void);
void performance_select(void);
void shoot_mode_switch(void);
void speed_switch(void);
void heat_switch(void);
void shoot_bullet_handle(void);
void shoot_friction_handle(void);
void shoot_state_mode_switch(void);
extern shoot_t shoot;
extern u8 press_l_state_switch;



















#endif
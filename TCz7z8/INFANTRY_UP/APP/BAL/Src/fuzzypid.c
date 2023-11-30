#include "fuzzypid.h"
#include "main.h"
//float erro_max=10; //用于进行e/ec的映射，调整其有效范围,默认[-3,3]
//float erro_min=-10;
//float erro_of_change_max  =3;
//float erro_of_change_min  =-3;
//float kp_max=15;   //用于进行kp ki kd的反映射(标定变化范围)，默认[-3,3]
//float kp_min=-15;
//float ki_max=3;
//float ki_min=-3;
//float kd_max=6;
//float kd_min=-6;(已写入结构体)

/*
	默认e、ec在[-3,3]区间，
 如需改变e、ec范围，需引入量化因子(Ke、Kec=N/emax)、缩放因子(Ku=umax/N)
*/

/*

	确定论域为[-3,3]论域平均分为 6 等份，即 [-3, -2], [-2, -1], [-1, 0], [0, 1], [1, 2], [2, 3]
	把每个端点进行等级划分，
	依次为: -3—>NB(负大), -2—>NM(负中), -1---->NS(负小), 0—>ZO(零), 1---->PS(正小), 2---->PM(正中), 3---->PB(正大)

*/

/*
	区间映射
	先要确定erro和（d（erro）/dt）的范围 再决定映射因子
*/

/*
	确定隶属度函数，值域[-1,1]
*/

/*
	模糊推理规则表
*/

float fuzzy_pid_calc(fuzzy_pid_t *fuzzy_pid,float get,float set)
{

	fuzzy_pid->get=get;
	fuzzy_pid->set=set;
	fuzzy_pid->erro[2] = set-get;      																		// 误差erro =  实际值-目标值
	fuzzy_pid->erro_of_change  = fuzzy_pid->erro[2] - fuzzy_pid->erro[1]; // 误差变化率(误差erro的微分)  系统死区设置量 不一定为零

	fuzzy_pid->sum_of_erro = fuzzy_pid->erro[1]+fuzzy_pid->erro[2];				//累加偏差

	fuzzy_pid->erro[0]=fuzzy_pid->erro[1];
	fuzzy_pid->erro[1]=fuzzy_pid->erro[2];

	fuzzy_process(fuzzy_pid,fuzzy_pid->erro[2],fuzzy_pid->erro_of_change);//模糊处理--模糊控制调整  kp，ki，kd
	
	fuzzy_pid->fuzzy_iout = (fuzzy_pid->fuzzy_ki+fuzzy_pid->Ki_Init)*fuzzy_pid->sum_of_erro;
	fuzzy_pid->fuzzy_pout = (fuzzy_pid->fuzzy_kp+fuzzy_pid->Kp_Init)*fuzzy_pid->erro[2];
	fuzzy_pid->fuzzy_dout = (fuzzy_pid->fuzzy_kd+fuzzy_pid->Kd_Init)*fuzzy_pid->erro_of_change;

	if(fuzzy_pid->fuzzy_iout > fuzzy_pid->integral_limit)
	 fuzzy_pid->fuzzy_iout =  fuzzy_pid->integral_limit;
	else if(fuzzy_pid->fuzzy_iout < -fuzzy_pid->integral_limit)
	 fuzzy_pid->fuzzy_iout = -fuzzy_pid->integral_limit;
	
	fuzzy_pid->fuzzy_out=fuzzy_pid->fuzzy_pout+fuzzy_pid->fuzzy_iout+fuzzy_pid->fuzzy_dout; //最终输出值
  
  if(fuzzy_pid->fuzzy_out > fuzzy_pid->max_out)
	return fuzzy_pid->max_out;   //最终输出值
	else
	return fuzzy_pid->fuzzy_out; //最终输出值
}




void fuzzy_process(fuzzy_pid_t *fuzzy_pid,float erro,float erro_of_change)
{
		//模糊化

		fuzzy_pid->erro_mapped=Mapping(fuzzy_pid->erro_max,fuzzy_pid->erro_min,erro);

		fuzzy_pid->erro_middle = fuzzy_pid->erro_mapped >  3.0 ? 0.0 : \
														(fuzzy_pid->erro_mapped < -3.0 ? 0.0 : \
														(fuzzy_pid->erro_mapped >= 0.0 ? (fuzzy_pid->erro_mapped >= 2.0 ? 2.5: (fuzzy_pid->erro_mapped >= 1.0 ? 1.5 : 0.5)) : \
														(fuzzy_pid->erro_mapped >=-1.0 ? -0.5 : (fuzzy_pid->erro_mapped >= -2.0 ? -1.5 : (fuzzy_pid->erro_mapped >= -3.0 ? -2.5 : 0.0) ))));

//		fuzzy_pid->erro_left_index  = (int)erro;         //映射前误差的左区间边界值
//		fuzzy_pid->erro_right_index = fuzzy_pid->erro_left_index;
		fuzzy_pid->erro_left_index  = (int)((fuzzy_pid->erro_middle-0.5)+3);
		fuzzy_pid->erro_right_index = (int)((fuzzy_pid->erro_middle+0.5)+3);

		fuzzy_pid->erro_left_grade_of_membership  = (fuzzy_pid->erro_middle == 0.0 ? 0.0:(fuzzy_pid->erro_mapped - (fuzzy_pid->erro_middle-0.5)));
		fuzzy_pid->erro_right_grade_of_membership = (fuzzy_pid->erro_middle == 0.0 ? 0.0:((fuzzy_pid->erro_middle+0.5)-fuzzy_pid->erro_mapped));

		fuzzy_pid->erro_of_change_mapped=Mapping(fuzzy_pid->erro_of_change_max,fuzzy_pid->erro_of_change_min,erro_of_change);

		fuzzy_pid->erro_of_change_middle = fuzzy_pid->erro_of_change_mapped > 3.0 ? 0.0 : (fuzzy_pid->erro_of_change_mapped < - 3.0 ? 0.0 : (fuzzy_pid->erro_of_change_mapped >= 0.0 ? (fuzzy_pid->erro_of_change_mapped >= 2.0 ? 2.5: (fuzzy_pid->erro_of_change_mapped >= 1.0 ? 1.5 : 0.5)) : (fuzzy_pid->erro_of_change_mapped >= -1.0 ? -0.5 : (fuzzy_pid->erro_of_change_mapped >= -2.0 ? -1.5 : (erro_of_change >= -3.0 ? -2.5 : 0.0) ))));

		fuzzy_pid->erro_of_change_left_index  = (int)((fuzzy_pid->erro_of_change_middle-0.5)+3);
		fuzzy_pid->erro_of_change_right_index = (int)((fuzzy_pid->erro_of_change_middle+0.5)+3);

		fuzzy_pid->erro_of_change_left_grade_of_membership  = (fuzzy_pid->erro_of_change_middle == 0.0 ? 0.0 :(fuzzy_pid->erro_of_change_mapped-(fuzzy_pid->erro_of_change_middle-0.5)));
		fuzzy_pid->erro_of_change_right_grade_of_membership  = (fuzzy_pid->erro_of_change_middle == 0.0 ? 0.0 :((fuzzy_pid->erro_of_change_middle+0.5)-fuzzy_pid->erro_of_change_mapped));


		//模糊推理，计算出的kp/ki/kd都是在论域[-3,3]以内的增量

		fuzzy_pid->fuzzy_kp =
						(fuzzy_pid->erro_left_grade_of_membership * fuzzy_pid->erro_of_change_left_grade_of_membership *  fuzzyRuleKp[fuzzy_pid->erro_of_change_left_index][fuzzy_pid->erro_left_index]
					+  fuzzy_pid->erro_left_grade_of_membership * fuzzy_pid->erro_of_change_right_grade_of_membership * fuzzyRuleKp[fuzzy_pid->erro_of_change_right_index][fuzzy_pid->erro_left_index]
					+ fuzzy_pid->erro_right_grade_of_membership * fuzzy_pid->erro_of_change_left_grade_of_membership * fuzzyRuleKp[fuzzy_pid->erro_of_change_left_index][fuzzy_pid->erro_right_index]
					+ fuzzy_pid->erro_right_grade_of_membership * fuzzy_pid->erro_of_change_right_grade_of_membership * fuzzyRuleKp[fuzzy_pid->erro_of_change_right_index][fuzzy_pid->erro_right_index]);

		fuzzy_pid->fuzzy_ki =
					 (fuzzy_pid->erro_left_grade_of_membership * fuzzy_pid->erro_of_change_left_grade_of_membership * fuzzyRuleKi[fuzzy_pid->erro_of_change_left_index][fuzzy_pid->erro_left_index]
					+ fuzzy_pid->erro_left_grade_of_membership * fuzzy_pid->erro_of_change_right_grade_of_membership * fuzzyRuleKi[fuzzy_pid->erro_of_change_right_index][fuzzy_pid->erro_left_index]
					+ fuzzy_pid->erro_right_grade_of_membership * fuzzy_pid->erro_of_change_left_grade_of_membership * fuzzyRuleKi[fuzzy_pid->erro_of_change_left_index][fuzzy_pid->erro_right_index]
					+ fuzzy_pid->erro_right_grade_of_membership * fuzzy_pid->erro_of_change_right_grade_of_membership * fuzzyRuleKi[fuzzy_pid->erro_of_change_right_index][fuzzy_pid->erro_right_index]);

		fuzzy_pid->fuzzy_kd =
						(fuzzy_pid->erro_left_grade_of_membership * fuzzy_pid->erro_of_change_left_grade_of_membership *    fuzzyRuleKd[fuzzy_pid->erro_of_change_left_index][fuzzy_pid->erro_left_index]
					+  fuzzy_pid->erro_left_grade_of_membership * fuzzy_pid->erro_of_change_right_grade_of_membership * fuzzyRuleKd[fuzzy_pid->erro_of_change_right_index][fuzzy_pid->erro_left_index]
					+ fuzzy_pid->erro_right_grade_of_membership * fuzzy_pid->erro_of_change_left_grade_of_membership * fuzzyRuleKd[fuzzy_pid->erro_of_change_left_index][fuzzy_pid->erro_right_index]
					+ fuzzy_pid->erro_right_grade_of_membership * fuzzy_pid->erro_of_change_right_grade_of_membership * fuzzyRuleKd[fuzzy_pid->erro_of_change_right_index][fuzzy_pid->erro_right_index]);

		//反模糊 同时根据最值确定输出范围
		fuzzy_pid->fuzzy_kp=Inverse_mapping(fuzzy_pid->kp_max,fuzzy_pid->kp_min,fuzzy_pid->fuzzy_kp);
		fuzzy_pid->fuzzy_ki=Inverse_mapping(fuzzy_pid->ki_max,fuzzy_pid->ki_min,fuzzy_pid->fuzzy_ki);
		fuzzy_pid->fuzzy_kd=Inverse_mapping(fuzzy_pid->kd_max,fuzzy_pid->kd_min,fuzzy_pid->fuzzy_kd);
}



///区间映射函数///
float Mapping(float maximum,float minimum,float x)
{
    float map= 6.0 *(x-minimum)/(maximum - minimum)-3;

    return map;

    //qvalues[1] = 3.0 * ecerro / (maximum - minimum);
}


//反区间映射函数
float Inverse_mapping(float maximum, float minimum, float map)
{
    float x = (maximum - minimum) *(map + 3)/6 + minimum;
    return x;
}




static void fuzzy_pid_param_init(
    fuzzy_pid_t*             pid,
    uint32_t                mode,
    uint32_t              maxout,
    uint32_t     intergral_limit,
    float    kp,
    float    ki,
    float    kd,
		float kp_max,
		float kp_min,
		float ki_max,
		float ki_min,
		float kd_max,
		float kd_min,
		float erro_max,
		float erro_min,
		float erro_of_change_max,
		float erro_of_change_min
		)
{

  pid->integral_limit = intergral_limit;
  pid->max_out        = maxout;
  pid->pid_mode       = mode;

  pid->Kp_Init = kp;
  pid->Ki_Init = ki;
  pid->Kd_Init = kd;

	pid-> kp_max=kp_max;
	pid-> kp_min=kp_min;
	pid-> ki_max=ki_max;
	pid-> ki_min=ki_min;
	pid-> kd_max=kd_max;
	pid-> kd_min=kd_min;
	pid->erro_max=erro_max;
	pid->erro_min=erro_min;
	pid->erro_of_change_max=erro_of_change_max;
	pid->erro_of_change_min=erro_of_change_min;

}

static void fuzzy_pid_reset(fuzzy_pid_t *pid, float kp, float ki, float kd,float kp_max,float kp_min,float ki_max,float ki_min,float kd_max,float kd_min,float erro_max,float erro_min,float erro_of_change_max,float erro_of_change_min)
{
  pid->Kp_Init = kp;
  pid->Ki_Init = ki;
  pid->Kd_Init = kd;

  pid->fuzzy_pout = 0;
  pid->fuzzy_iout = 0;
  pid->fuzzy_dout = 0;
  pid->fuzzy_out  = 0;

}

void fuzzy_pid_struct_init(
    fuzzy_pid_t*   pid,
    uint32_t mode,
    uint32_t maxout,
    uint32_t intergral_limit,

    float kp,
    float ki,
    float kd,

		float kp_max,
		float kp_min,
		float ki_max,
		float ki_min,
		float kd_max,
		float kd_min,
		float erro_max,
		float erro_min,
		float erro_of_change_max,
		float erro_of_change_min
		)
{
  pid->fuzzy_param_init = fuzzy_pid_param_init;
  pid->fuzzy_pid_reset  = fuzzy_pid_reset;

  pid->fuzzy_param_init(pid, mode, maxout, intergral_limit, kp, ki, kd,kp_max,kp_min,ki_max,ki_min,kd_max,kd_min,erro_max,erro_min,erro_of_change_max,erro_of_change_min);
  pid->fuzzy_pid_reset(pid, kp, ki, kd,kp_max,kp_min,ki_max,ki_min,kd_max,kd_min,erro_max,erro_min,erro_of_change_max,erro_of_change_min);
}

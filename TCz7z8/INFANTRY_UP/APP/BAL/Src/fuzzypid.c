#include "fuzzypid.h"
#include "main.h"
//float erro_max=10; //���ڽ���e/ec��ӳ�䣬��������Ч��Χ,Ĭ��[-3,3]
//float erro_min=-10;
//float erro_of_change_max  =3;
//float erro_of_change_min  =-3;
//float kp_max=15;   //���ڽ���kp ki kd�ķ�ӳ��(�궨�仯��Χ)��Ĭ��[-3,3]
//float kp_min=-15;
//float ki_max=3;
//float ki_min=-3;
//float kd_max=6;
//float kd_min=-6;(��д��ṹ��)

/*
	Ĭ��e��ec��[-3,3]���䣬
 ����ı�e��ec��Χ����������������(Ke��Kec=N/emax)����������(Ku=umax/N)
*/

/*

	ȷ������Ϊ[-3,3]����ƽ����Ϊ 6 �ȷݣ��� [-3, -2], [-2, -1], [-1, 0], [0, 1], [1, 2], [2, 3]
	��ÿ���˵���еȼ����֣�
	����Ϊ: -3��>NB(����), -2��>NM(����), -1---->NS(��С), 0��>ZO(��), 1---->PS(��С), 2---->PM(����), 3---->PB(����)

*/

/*
	����ӳ��
	��Ҫȷ��erro�ͣ�d��erro��/dt���ķ�Χ �پ���ӳ������
*/

/*
	ȷ�������Ⱥ�����ֵ��[-1,1]
*/

/*
	ģ����������
*/

float fuzzy_pid_calc(fuzzy_pid_t *fuzzy_pid,float get,float set)
{

	fuzzy_pid->get=get;
	fuzzy_pid->set=set;
	fuzzy_pid->erro[2] = set-get;      																		// ���erro =  ʵ��ֵ-Ŀ��ֵ
	fuzzy_pid->erro_of_change  = fuzzy_pid->erro[2] - fuzzy_pid->erro[1]; // ���仯��(���erro��΢��)  ϵͳ���������� ��һ��Ϊ��

	fuzzy_pid->sum_of_erro = fuzzy_pid->erro[1]+fuzzy_pid->erro[2];				//�ۼ�ƫ��

	fuzzy_pid->erro[0]=fuzzy_pid->erro[1];
	fuzzy_pid->erro[1]=fuzzy_pid->erro[2];

	fuzzy_process(fuzzy_pid,fuzzy_pid->erro[2],fuzzy_pid->erro_of_change);//ģ������--ģ�����Ƶ���  kp��ki��kd
	
	fuzzy_pid->fuzzy_iout = (fuzzy_pid->fuzzy_ki+fuzzy_pid->Ki_Init)*fuzzy_pid->sum_of_erro;
	fuzzy_pid->fuzzy_pout = (fuzzy_pid->fuzzy_kp+fuzzy_pid->Kp_Init)*fuzzy_pid->erro[2];
	fuzzy_pid->fuzzy_dout = (fuzzy_pid->fuzzy_kd+fuzzy_pid->Kd_Init)*fuzzy_pid->erro_of_change;

	if(fuzzy_pid->fuzzy_iout > fuzzy_pid->integral_limit)
	 fuzzy_pid->fuzzy_iout =  fuzzy_pid->integral_limit;
	else if(fuzzy_pid->fuzzy_iout < -fuzzy_pid->integral_limit)
	 fuzzy_pid->fuzzy_iout = -fuzzy_pid->integral_limit;
	
	fuzzy_pid->fuzzy_out=fuzzy_pid->fuzzy_pout+fuzzy_pid->fuzzy_iout+fuzzy_pid->fuzzy_dout; //�������ֵ
  
  if(fuzzy_pid->fuzzy_out > fuzzy_pid->max_out)
	return fuzzy_pid->max_out;   //�������ֵ
	else
	return fuzzy_pid->fuzzy_out; //�������ֵ
}




void fuzzy_process(fuzzy_pid_t *fuzzy_pid,float erro,float erro_of_change)
{
		//ģ����

		fuzzy_pid->erro_mapped=Mapping(fuzzy_pid->erro_max,fuzzy_pid->erro_min,erro);

		fuzzy_pid->erro_middle = fuzzy_pid->erro_mapped >  3.0 ? 0.0 : \
														(fuzzy_pid->erro_mapped < -3.0 ? 0.0 : \
														(fuzzy_pid->erro_mapped >= 0.0 ? (fuzzy_pid->erro_mapped >= 2.0 ? 2.5: (fuzzy_pid->erro_mapped >= 1.0 ? 1.5 : 0.5)) : \
														(fuzzy_pid->erro_mapped >=-1.0 ? -0.5 : (fuzzy_pid->erro_mapped >= -2.0 ? -1.5 : (fuzzy_pid->erro_mapped >= -3.0 ? -2.5 : 0.0) ))));

//		fuzzy_pid->erro_left_index  = (int)erro;         //ӳ��ǰ����������߽�ֵ
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


		//ģ�������������kp/ki/kd����������[-3,3]���ڵ�����

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

		//��ģ�� ͬʱ������ֵȷ�������Χ
		fuzzy_pid->fuzzy_kp=Inverse_mapping(fuzzy_pid->kp_max,fuzzy_pid->kp_min,fuzzy_pid->fuzzy_kp);
		fuzzy_pid->fuzzy_ki=Inverse_mapping(fuzzy_pid->ki_max,fuzzy_pid->ki_min,fuzzy_pid->fuzzy_ki);
		fuzzy_pid->fuzzy_kd=Inverse_mapping(fuzzy_pid->kd_max,fuzzy_pid->kd_min,fuzzy_pid->fuzzy_kd);
}



///����ӳ�亯��///
float Mapping(float maximum,float minimum,float x)
{
    float map= 6.0 *(x-minimum)/(maximum - minimum)-3;

    return map;

    //qvalues[1] = 3.0 * ecerro / (maximum - minimum);
}


//������ӳ�亯��
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

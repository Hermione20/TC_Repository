#include "balance_task.h"



Balance_chassis_t b_chassis = { 0 };
int Init_cnt;

void balance_chassis_task(void)
{
    switch (b_chassis.ctrl_mode)
    {
    case CHASSIS_RELAX:
    {
			
        b_chassis.joint_T[0] = 0;
			  b_chassis.joint_T[1] = 0;
			  b_chassis.joint_T[2] = 0;
				b_chassis.joint_T[3] = 0;
				b_chassis.driving_T[0] = 0;
			  b_chassis.driving_T[1] = 0;
			Init_cnt = 0;
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
    }
    break;
    case CHASSIS_INIT:
    {
        chassis_Init_handle();

    }
    break;
		case CHASSIS_STAND_MODE:
		{
			chasis_standup_handle();
			balance_task();
		}break;
    case CHASSIS_SEPARATE:
    {
				chassis_seperate_handle();
        balance_task();
    }
    break;
    case MANUAL_FOLLOW_GIMBAL:
    {

        balance_task();
    }
    break;
    case CHASSIS_ROTATE:
    {
    }break;
    default:
        break;
    }
}

void chasis_standup_handle(void)
{
	b_chassis.chassis_ref.leglength = 0.15f;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	if(fabs(chassis_gyro.pitch_Angle)<3)
		b_chassis.ctrl_mode = CHASSIS_SEPARATE;
}

void chassis_Init_handle(void)
{
		Init_cnt++;
    b_chassis.chassis_ref.leglength = 0.14f;
    b_chassis.chassis_ref.vy = 0;
    b_chassis.chassis_ref.vx = 0;
    b_chassis.chassis_ref.vw = 0;
	VMC_data_get(&b_chassis.left_leg,-balance_chassis.joint_Encoder[1].angle,
                                        -balance_chassis.joint_Encoder[1].gyro,
                                        -balance_chassis.joint_Encoder[2].angle+PI,
                                        -balance_chassis.joint_Encoder[2].gyro);

        VMC_data_get(&b_chassis.right_leg,balance_chassis.joint_Encoder[0].angle,
                                        balance_chassis.joint_Encoder[0].gyro,
                                        balance_chassis.joint_Encoder[3].angle+PI,
                                        balance_chassis.joint_Encoder[3].gyro);
	float phi0 = ((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0)/2.0f) - 1.57f;
	if((fabs(phi0) >= 4*PI/180)||Init_cnt<1000)
	{
		float Init_Tp = pid_calc(&b_chassis.Init_Tp_pid,phi0,0);
		//˫��Э��pid
    float harmonize_output = pid_calc(&b_chassis.leg_harmonize_pid, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
		
		//�Ȳ���ֱ��F�ļ���
    b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength);
    b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength);
		
		leg_conv(b_chassis.left_leg.leg_F, Init_Tp-harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = 0 ;
		
		leg_conv(b_chassis.right_leg.leg_F, Init_Tp+ harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = 0;
		
		//�������޷�
    VAL_LIMIT(b_chassis.joint_T[1], -34 , 34);
    VAL_LIMIT(b_chassis.joint_T[2], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[0], -5, 5);

    VAL_LIMIT(b_chassis.joint_T[0], -34, 34);
    VAL_LIMIT(b_chassis.joint_T[3], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[1], -5, 5);
	}else
	{

		b_chassis.ctrl_mode = CHASSIS_STAND_MODE;
		Init_cnt = 0;
		
	}

    
    

}

void chassis_seperate_handle(void)
{
    b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
    b_chassis.chassis_ref.vy = b_chassis.chassis_dynemic_ref.vy;
    b_chassis.chassis_ref.vx = b_chassis.chassis_dynemic_ref.vx;
    b_chassis.chassis_ref.vw = b_chassis.chassis_dynemic_ref.vw;
	b_chassis.chassis_ref.y_position += b_chassis.chassis_ref.vy*0.001*TIME_STEP;
}

//��ʱ��һ��Ҫע�⻡���Ƶ�ת����
//�������Ҫ��ϸ���
//���ø����������ĵ�λ������
void balance_task(void)
{
    /********************�����������ĸ���************************************/
        VMC_data_get(&b_chassis.left_leg,-balance_chassis.joint_Encoder[1].angle,
                                        -balance_chassis.joint_Encoder[1].gyro,
                                        -balance_chassis.joint_Encoder[2].angle+PI,
                                        -balance_chassis.joint_Encoder[2].gyro);

        VMC_data_get(&b_chassis.right_leg,balance_chassis.joint_Encoder[0].angle,
                                        balance_chassis.joint_Encoder[0].gyro,
                                        balance_chassis.joint_Encoder[3].angle+PI,
                                        balance_chassis.joint_Encoder[3].gyro);
   
    //����״̬����    
    b_chassis.balance_loop.phi = chassis_gyro.pitch_Angle*PI/180.0f;
    b_chassis.balance_loop.dphi = chassis_gyro.pitch_Gyro*PI/180.0f;
    b_chassis.balance_loop.x = ((balance_chassis.Driving_Encoder[0].angle + (-balance_chassis.Driving_Encoder[1].angle))/2.0f) * WHEEL_R;
    b_chassis.balance_loop.dx = Mileage_kalman_filter.velocity;
    b_chassis.balance_loop.theta = ((((b_chassis.left_leg.phi0 + b_chassis.right_leg.phi0)/2.0f) - 1.57f) - chassis_gyro.pitch_Angle*PI/180.0f);
    b_chassis.balance_loop.dtheta = (((b_chassis.left_leg.dphi0 + b_chassis.right_leg.dphi0)/2.0f) - chassis_gyro.pitch_Gyro*PI/180.0f);
    b_chassis.balance_loop.ddz = chassis_gyro.z_Acc*cos(chassis_gyro.pitch_Angle*PI/180.0f);

    b_chassis.balance_loop.L0 = (b_chassis.left_leg.l0 + b_chassis.right_leg.l0)/2.0f;
    
    //����֧����
    FN_calculate(&b_chassis.left_leg,-balance_chassis.joint_Encoder[2].Torque,-balance_chassis.joint_Encoder[1].Torque);
    FN_calculate(&b_chassis.right_leg,balance_chassis.joint_Encoder[3].Torque,balance_chassis.joint_Encoder[0].Torque);
    /*****************************************************************/

    //lqr���������ȳ��ı仯��ȡ
    lqr_k(b_chassis.balance_loop.L0, b_chassis.balance_loop.K);
    for (int i = 0; i < 6; i++)
    {
        for (int j = 0; j < 2; j++)
        {
            
                b_chassis.balance_loop.k[j][i] = b_chassis.balance_loop.K[i * 2 + j];
            
            
        }
            
    }
   
   //������
    b_chassis.balance_loop.state_err[0] = -(b_chassis.balance_loop.theta - b_chassis.chassis_ref.theta);
	b_chassis.balance_loop.state_err[1] = -(b_chassis.balance_loop.dtheta);
	b_chassis.balance_loop.state_err[2] = -(b_chassis.balance_loop.x -b_chassis.chassis_ref.y_position);
	b_chassis.balance_loop.state_err[3] = -(b_chassis.balance_loop.dx - b_chassis.chassis_ref.vy);
	b_chassis.balance_loop.state_err[4] = -(b_chassis.balance_loop.phi);
	b_chassis.balance_loop.state_err[5] = -(b_chassis.balance_loop.dphi);
    //���ȱ仯���ٶȵ�����
    VAL_LIMIT(b_chassis.balance_loop.state_err[3], -3, 3);

    //lqrδ����������
    float V_T_gain = b_chassis.balance_loop.k[0][3] * b_chassis.balance_loop.state_err[3];
    float V_Tp_gain = b_chassis.balance_loop.k[1][3] * b_chassis.balance_loop.state_err[3];
    float balance_Tgain = b_chassis.balance_loop.k[0][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[0][1] * b_chassis.balance_loop.state_err[1] + b_chassis.balance_loop.k[0][2] * b_chassis.balance_loop.state_err[2] + b_chassis.balance_loop.k[0][4] * b_chassis.balance_loop.state_err[4] + b_chassis.balance_loop.k[0][5] * b_chassis.balance_loop.state_err[5];
    float balance_Tpgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1] + b_chassis.balance_loop.k[1][2] * b_chassis.balance_loop.state_err[2] + b_chassis.balance_loop.k[1][4] * b_chassis.balance_loop.state_err[4] + b_chassis.balance_loop.k[1][5] * b_chassis.balance_loop.state_err[5];
    //lqr����������
    float V_T_outlandgain = 0;
    float V_Tp_outlandgain = 0;
    float balance_Toutlandgain = 0;
    float balance_Tpoutlandgain = b_chassis.balance_loop.k[1][0] * b_chassis.balance_loop.state_err[0] + b_chassis.balance_loop.k[1][1] * b_chassis.balance_loop.state_err[1] ;
    
    //lqr���
    b_chassis.balance_loop.lqrOutT = balance_Tgain + V_T_gain;
    b_chassis.balance_loop.lqrOutTp = balance_Tpgain + V_Tp_gain;

    //˫��Э��pid
    float harmonize_output = pid_calc(&b_chassis.leg_harmonize_pid, (b_chassis.right_leg.phi0 - b_chassis.left_leg.phi0), 0);
    //ת��pid
    float vw_torque = pid_calc(&b_chassis.vw_pid, chassis_gyro.yaw_Gyro*PI/180.0f, b_chassis.chassis_ref.vw);
    //rollƽ��pid
    float roll_F_output = pid_calc(&b_chassis.roll_pid,chassis_gyro.roll_Angle*PI/180.0f,0);
    
    //���pitch�ǿ��Ʋ�ס���¶׻�ȡƽ��
//    if (fabs(b_chassis.balance_loop.phi*180/PI)>5)
//    {
//        
//        b_chassis.chassis_ref.leglength = 0.15;
//    }
//    else
//    {
//       
//       b_chassis.chassis_ref.leglength = b_chassis.chassis_dynemic_ref.leglength;
//    }
    //�Ȳ���ֱ��F�ļ���
    b_chassis.left_leg.leg_F = pid_calc(&b_chassis.left_leg.leglengthpid, b_chassis.left_leg.l0, b_chassis.chassis_ref.leglength)+ (BODY_MASS/2) * 9.8 + roll_F_output;
    b_chassis.right_leg.leg_F = pid_calc(&b_chassis.right_leg.leglengthpid, b_chassis.right_leg.l0, b_chassis.chassis_ref.leglength) + (BODY_MASS/2)*9.8 - roll_F_output;
    /*test*/
 
    /*float ddl = (b_chassis.left_leg.ddl0 + b_chassis.right_leg.ddl0) / 2;
    float dl = (b_chassis.left_leg.dl0 + b_chassis.right_leg.dl0) / 2;
    float ddXw = chassis_gyro.y_Acc - ddl * sinf(b_chassis.balance_loop.theta) - 2 * b_chassis.balance_loop.dtheta * cosf(b_chassis.balance_loop.theta) * dl - b_chassis.balance_loop.L0 * b_chassis.balance_loop.ddtheta * cosf(b_chassis.balance_loop.theta) + b_chassis.balance_loop.L0 * (b_chassis.balance_loop.dtheta) * (b_chassis.balance_loop.dtheta) * sinf((b_chassis.balance_loop.theta));
    float max_F = ((-b_chassis.balance_loop.lqrOutT * WHEEL_R + (-b_chassis.balance_loop.lqrOutTp) * b_chassis.balance_loop.L0 * cosf(b_chassis.balance_loop.theta)  ) - ddXw * WHEEL_MASS) / sinf(b_chassis.balance_loop.theta);
    */
   
    //�˴���T0Ϊphi1�����Ť�أ���һ����phi4��
    if (wheel_state_estimate(&b_chassis.left_leg)||(b_chassis.ctrl_mode==CHASSIS_INIT))
    {
        leg_conv(b_chassis.left_leg.leg_F, b_chassis.balance_loop.lqrOutTp-harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = b_chassis.balance_loop.lqrOutT / 2.0f + vw_torque;
    }
    else
    {
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
        leg_conv(b_chassis.left_leg.leg_F, balance_Tpoutlandgain + harmonize_output, b_chassis.left_leg.phi1, b_chassis.left_leg.phi4, b_chassis.left_leg.T);
        b_chassis.joint_T[1] = b_chassis.left_leg.T[1];
        b_chassis.joint_T[2] = b_chassis.left_leg.T[0];
        b_chassis.driving_T[0] = 0;
			
    }

    if (wheel_state_estimate(&b_chassis.right_leg)||(b_chassis.ctrl_mode==CHASSIS_INIT))
    {
        leg_conv(b_chassis.right_leg.leg_F, b_chassis.balance_loop.lqrOutTp+ harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = b_chassis.balance_loop.lqrOutT / 2.0f - vw_torque;
    }
    else
    {
			b_chassis.chassis_ref.y_position = b_chassis.balance_loop.x;
        leg_conv(b_chassis.right_leg.leg_F, balance_Tpoutlandgain - harmonize_output, b_chassis.right_leg.phi1, b_chassis.right_leg.phi4, b_chassis.right_leg.T);
        b_chassis.joint_T[0] = b_chassis.right_leg.T[1];
        b_chassis.joint_T[3] = b_chassis.right_leg.T[0];
        b_chassis.driving_T[1] = 0;
    }


    
    
    //�������޷�
    VAL_LIMIT(b_chassis.joint_T[1], -34 , 34);
    VAL_LIMIT(b_chassis.joint_T[2], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[0], -5, 5);

    VAL_LIMIT(b_chassis.joint_T[0], -34, 34);
    VAL_LIMIT(b_chassis.joint_T[3], -34, 34);
    VAL_LIMIT(b_chassis.driving_T[1], -5, 5);

    
}


void balance_param_init(void)
{
    memset(&b_chassis, 0, sizeof(Balance_chassis_t));
    b_chassis.chassis_dynemic_ref.leglength = 0.23;
    PID_struct_init(&b_chassis.left_leg.leglengthpid, POSITION_PID,2000,2000,800,0,40000);
    PID_struct_init(&b_chassis.right_leg.leglengthpid, POSITION_PID, 2000, 2000, 800, 0, 40000);
    PID_struct_init(&b_chassis.leg_harmonize_pid, POSITION_PID, 2000, 2000, 150, 0, 3000);
    PID_struct_init(&b_chassis.vw_pid, POSITION_PID,5,5,2,0,0);
    PID_struct_init(&b_chassis.roll_pid, POSITION_PID, 500, 200, 1000, 0, 0);
	
	PID_struct_init(&b_chassis.Init_Tp_pid, POSITION_PID, 500, 200, 40, 0, 60);
}


uint8_t wheel_state_estimate(leg_state_t *leg)
{
    if (leg->leg_FN < 10)
    {
        leg->wheel_state = 0;
        return 0;
    }
    else
    {
        leg->wheel_state = 1;
        return 1;
    }
}



void lqr_k(double L0, double K[12])
{
    double t2;
    double t3;
    /*     This function was generated by the Symbolic Math Toolbox version 23.2.
     */
     /*     2023-11-17 17:36:04 */
    t2 = L0 * L0;
    t3 = L0 * L0 * L0;
  K[0] = ((L0 * -100.894454139715 + t2 * 186.15407202778019) -
          t3 * 164.71714791853231) -
         9.0643185539432256;
  K[1] = ((L0 * -22.24445981993027 + t2 * 37.0647970786585) -
          t3 * 23.324012572880282) +
         6.6478307573260293;
  K[2] = ((L0 * -8.8338359698040048 - t2 * 5.3289547189472906) +
          t3 * 5.332543582893269) -
         0.677454180483319;
  K[3] = ((L0 * -0.84493166249022367 + t2 * 1.0561848214883061) -
          t3 * 0.41856935352784452) +
         0.59344724851797026;
  K[4] = ((L0 * -2.0360080085351981 + t2 * 5.823074783540295) -
          t3 * 5.8316266624952089) -
         0.698720843852313;
  K[5] = ((L0 * -2.857758657196158 + t2 * 6.4174455395863541) -
          t3 * 5.4555519509322528) +
         0.56842253383360752;
  K[6] = ((L0 * -9.5181215673992856 + t2 * 25.891984355714168) -
          t3 * 25.66832429493542) -
         4.0099365908433109;
  K[7] = ((L0 * -16.3636975946947 + t2 * 37.713064921031268) -
          t3 * 32.796043727186067) +
         3.17599239852152;
  K[8] = ((L0 * -150.6500155179067 + t2 * 355.72917401723151) -
          t3 * 315.3589239118059) +
         28.307393517205121;
  K[9] = ((L0 * 61.732820887336288 - t2 * 180.83468117506251) +
          t3 * 182.944871604467) +
         35.710799348588;
  K[10] = ((L0 * -13.22708180796258 + t2 * 29.627764194465591) -
           t3 * 25.6677747739944) +
          2.8516020824915231;
  K[11] = ((L0 * 5.0203680463653813 - t2 * 14.0592425863903) +
           t3 * 13.82247388589254) +
          2.5451089583353692;
}

/*
 * File trailer for lqr_k.c
 *
 * [EOF]
 */



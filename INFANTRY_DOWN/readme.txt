RPS 2024 ��������ģ��

��Ƕ��ʽ��ܷ��㷨�㣬Ƕ��㣬Ӳ���㣬�̼����Ĳ���

�����ͷ�ļ���main.h��Ͻ��
	1.����ͷ�ļ���������ÿ��.c�ļ��������һ��.h�ļ�����Щ.h�ļ���Ҫ��������ͷ�ļ�
		����ͷ�ļ����� 
			  �̼���
		      ���߿�
			  ��ѧ��
			  �����
			  ������
			  �㷨��
		����֮����ӵ�ģ���������ϲ��֣�����ӵ�����ͷ�ļ��⣩
	2.��������ͷ�ļ�
	     ��Ͻ�������������ͷ�ļ�
		 
�����ʼ����BSP.c�µ���

RTOS�µ��ļ������MAIN.H

�������۴������Ľ��㣬���ڴ�����.c�ļ���ͷ�ļ�

10.3 chassis_task�Ѹ�
//void get_remote_set()
//{
//		vx = can_chassis_data.x;//vx��x�Ǻ���
//		vy = can_chassis_data.y;//vy��y������
// Chassis_angle.Remote_speed = sqrt((vx*vx)+(vy*vy));
//if(Chassis_angle.Remote_speed >= 50)
//{
// if(vx > 0)
// {
//	 Chassis_angle.Remote_angle = atan(vy / vx);
//   if(Chassis_angle.Remote_angle < 0)
//   {
//		 Chassis_angle.Remote_angle += 2*PI;
//	 } 
// }
// else if(vx<0) 
// {
//	 Chassis_angle.Remote_angle = atan(vy / vx);
//	 Chassis_angle.Remote_angle += PI;
// }
// else
//  {
//		 if(vy < 0)
//		 {
//		 Chassis_angle.Remote_angle = 3 * PI /2;}
//		 else if(vy > 0)
//		 {
//		 Chassis_angle.Remote_angle = PI / 2;
//		 }
//  }
// } 
//}


10.3 chassis_task�Ѹ�
//	yaw_num_get = -yaw_Encoder.ecd_angle/360;
//	if(-yaw_Encoder .ecd_angle<0)
//	{yaw_num_get -=1;}
//	Chassis_angle.yaw_angle_0_2pi=(-yaw_Encoder.ecd_angle-yaw_num_get*360)*ANGLE_TO_RAD;
//��ʱ������
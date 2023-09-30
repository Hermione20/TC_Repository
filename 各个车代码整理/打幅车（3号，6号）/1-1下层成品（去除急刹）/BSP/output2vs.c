//#include "main.h"
#include "stm32f4xx_usart.h"
#include "main.h"
//***************************************����***************************************************
/*
************************************************************************************************
*������unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
*������Buf:У������ݣCCRC_CNT:У���λ��
*���أ�У����
*���ܣ���ϴ���ʾ����У������
************************************************************************************************
*/



unsigned short CRC_CHECK(unsigned char *Buf, unsigned char CRC_CNT)
{
  unsigned short CRC_Temp;
  unsigned char i,j;
  CRC_Temp = 0xffff;
  for (i=0;i<CRC_CNT; i++)
  {      
    CRC_Temp ^= Buf[i];
    for (j=0;j<8;j++) 
    {
      if (CRC_Temp & 0x01)
        CRC_Temp = (CRC_Temp >>1 ) ^ 0xa001;
      else
        CRC_Temp = CRC_Temp >> 1;
    }
  }
  return(CRC_Temp);
}
/*
************************************************************************************************
*������void OutPut_Data(float OutData[4])
*������OutData[4]:�������
*���أ�
*���ܣ� ��ϴ���ʾ���������Э�飬float �ֳ���Ϊ16λ������ע�⹤��
*     1 �������4·��Ϣ��ÿ·���ݳ�����16λ��
*     2 ÿ������ֽ�����10������������ֽ�ΪУ��λ
************************************************************************************************
*/
void OutPut_Data(int OutData[4])
{
  int temp[4] = {0};
  unsigned int temp1[4] = {0};
  unsigned char databuf[10] = {0};
  unsigned char i;
  unsigned short CRC16 = 0;
  for(i=0;i<4;i++)
  {
    temp[i]  = (int)OutData[i];
    temp1[i] = (unsigned int)temp[i]; 
  } 
  for(i=0;i<4;i++) 
  {
    databuf[i*2]   = (unsigned char)(temp1[i]%256);
    databuf[i*2+1] = (unsigned char)(temp1[i]/256);
  }
  
  CRC16 = CRC_CHECK(databuf,8);
  databuf[8] = CRC16%256;
  databuf[9] = CRC16/256;
  
  Usart3SendBytesInfoProc(databuf,10);
  
//  for(i=0;i<10;i++)
//	{
//	  USART_SendData(USART3,databuf[i]);
//	  while(USART_GetFlagStatus(USART3, USART_FLAG_TC) == RESET);//STM32��Ƭ����һ�������ϣ������ϲ�������
//	}
}
//****************************************END***************************************************

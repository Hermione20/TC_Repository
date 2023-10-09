#include "fuzzypid.h"
/*********ģ��pid����*/

 
#define NL   -3
#define NM	 -2
#define NS	 -1
#define ZE	 0
#define PS	 1
#define PM	 2
#define PL	 3
 
static const float fuzzyRuleKp[7][7]={
	PL,	PL,	PM,	PM,	PS,	ZE,	ZE,
	PL,	PL,	PM,	PS,	PS,	ZE,	NS,
	PM,	PM,	PM,	PS,	ZE,	NS,	NS,
	PM,	PM,	PS,	ZE,	NS,	NM,	NM,
	PS,	PS,	ZE,	NS,	NS,	NM,	NM,
	PS,	ZE,	NS,	NM,	NM,	NM,	NL,
	ZE,	ZE,	NM,	NM,	NM,	NL,	NL
};
 
static const float fuzzyRuleKi[7][7]={
	NL,	NL,	NM,	NM,	NS,	ZE,	ZE,
	NL,	NL,	NM,	NS,	NS,	ZE,	ZE,
	NL,	NM,	NS,	NS,	ZE,	PS,	PS,
	NM,	NM,	NS,	ZE,	PS,	PM,	PM,
	NS,	NS,	ZE,	PS,	PS,	PM,	PL,
	ZE,	ZE,	PS,	PS,	PM,	PL,	PL,
	ZE,	ZE,	PS,	PM,	PM,	PL,	PL
};
 
static const float fuzzyRuleKd[7][7]={
	PS,	NS,	NL,	NL,	NL,	NM,	PS,
	PS,	NS,	NL,	NM,	NM,	NS,	ZE,
	ZE,	NS,	NM,	NM,	NS,	NS,	ZE,
	ZE,	NS,	NS,	NS,	NS,	NS,	ZE,
	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,	ZE,
	PL,	NS,	PS,	PS,	PS,	PS,	PL,
	PL,	PM,	PM,	PM,	PS,	PS,	PL
};
 

 //�ؼ��㷨
void fuzzy( fuzzypid_t*  fuzzy_PID)
{
     float e = fuzzy_PID ->PreError/ fuzzy_PID->stair;
	   float ec = (fuzzy_PID ->out - fuzzy_PID ->out_last) / fuzzy_PID->stair;
     short etemp,ectemp;
     float eLefttemp,ecLefttemp;    //������
     float eRighttemp ,ecRighttemp; 
 
     short eLeftIndex,ecLeftIndex;  //��ǩ
     short eRightIndex,ecRightIndex;

	  //ģ����
     if(e>=PL)
			 etemp=PL;//������Χ
		 else if(e>=PM)
			 etemp=PM;
		 else if(e>=PS)
			 etemp=PS;
		 else if(e>=ZE)
			 etemp=ZE;
		 else if(e>=NS)
			 etemp=NS;
		 else if(e>=NM)
			 etemp=NM;
		 else if(e>=NL)
			 etemp=NL;
		 else 
			 etemp=2*NL;
 
		 if( etemp == PL)
		{
		 //����E������
				eRighttemp= 0 ;    //�����
				eLefttemp= 1 ;
			
     //�����ǩ
	   eLeftIndex = 6 ;      
	   eRightIndex= 6 ;
			
		}else if( etemp == 2*NL )
    {

			//����E������
				eRighttemp = 1;    //�����
				eLefttemp = 0;
	
     //�����ǩ
	   eLeftIndex = 0 ;       
	   eRightIndex = 0 ;
			
		}	else 
    {

			//����E������
				eRighttemp=(e-etemp);  //���Ժ�����Ϊ��������
				eLefttemp=(1- eRighttemp);
			
     //�����ǩ
	   eLeftIndex =(short) (etemp-NL);       //���� etemp=2.5��NL=-3����ô�õ������к�Ϊ5  ��0 1 2 3 4 5 6��
	   eRightIndex=(short) (eLeftIndex+1);
			
		}		
	   
		
		 if(ec>=PL)
			 ectemp=PL;
		 else if(ec>=PM)
			 ectemp=PM;
		 else if(ec>=PS)
			 ectemp=PS;
		 else if(ec>=ZE)
			 ectemp=ZE;
		 else if(ec>=NS)
			 ectemp=NS;
		 else if(ec>=NM)
			 ectemp=NM;
		 else if(ec>=NL)
			 ectemp=NL;
		 else 
			 ectemp=2*NL;
		 
	  
   if( ectemp == PL )
	 {
    //����EC������		 
		 ecRighttemp= 0 ;      //�����
		 ecLefttemp= 1 ;
			
		 ecLeftIndex = 6 ;  
	   ecRightIndex = 6 ;	 
	 
	 } else if( ectemp == 2*NL)
	 {
    //����EC������		 
		 ecRighttemp= 1 ;
		 ecLefttemp= 0 ;
			
		 ecLeftIndex = 0 ;  
	   ecRightIndex = 0 ;	 	 
	 }else
	 {
    //����EC������		 
		 ecRighttemp=(ec-ectemp);
		 ecLefttemp=(1- ecRighttemp);
			
		 ecLeftIndex =(short) (ectemp-NL);  
	   ecRightIndex= (short)(eLeftIndex+1);
	 }	

 
/*************************************��ģ��*************************************/
 
 
 
 
	fuzzy_PID->d_kp = fuzzy_PID->Kp_stair * (eLefttemp * ecLefttemp * fuzzyRuleKp[eLeftIndex][ecLeftIndex]                   
   + eLefttemp * ecRighttemp * fuzzyRuleKp[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKp[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKp[eRightIndex][ecRightIndex]);
 
	fuzzy_PID->d_ki = fuzzy_PID->Ki_stair * (eLefttemp * ecLefttemp * fuzzyRuleKi[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKi[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKi[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKi[eRightIndex][ecRightIndex]);

 
	fuzzy_PID->d_kd = fuzzy_PID->Kd_stair * (eLefttemp * ecLefttemp * fuzzyRuleKd[eLeftIndex][ecLeftIndex]
   + eLefttemp * ecRighttemp * fuzzyRuleKd[eLeftIndex][ecRightIndex]
   + eRighttemp * ecLefttemp * fuzzyRuleKd[eRightIndex][ecLeftIndex]
   + eRighttemp * ecRighttemp * fuzzyRuleKd[eRightIndex][ecRightIndex]);
 
}


float FuzzyPID_Calc(fuzzypid_t *P)
{
	
	  P->LastError = P->PreError;
	  
	  if((fabs(P->PreError)< P->output_deadband ))   //��������
		{
			P->PreError = 0.0f;			
		}
		else
		{
			P->PreError = P->set - P->get;
		}
		
		fuzzy(P);      //ģ������  kp,ki,kd   �β�1��ǰ���β�2ǰ�����Ĳ�ֵ
	
    float Kp = P->kp0 + P->d_kp , Ki = P->ki0 + P->d_ki , Kd = P->kd0 + P->d_kd ;   //PID��ģ��
//	float Kp = P->kp0 + P->d_kp , Ki = P->ki0  , Kd = P->kd0 + P->d_kd ;           //��PD��ģ��
//	float Kp = P->kp0 + P->d_kp , Ki = P->ki0  , Kd = P->kd0 ;                    //��P��ģ��

		
		
		      // ΢������
		float DM = Kd*(P->out - P->out_last);   //΢������	
         // ���ٻ��ַ���
    if(fabs(P->PreError) < P->I_L )			
		{
	       //���λ���
		P->SumError += (P->PreError+P->LastError)/2;    
		VAL_LIMIT(P->SumError,P->integral_limit,-P->integral_limit);
		}
		 else if( fabs(P->PreError) < P->I_U )
		{
	      //���λ��� 
		P->SumError += (P->PreError+P->LastError)/2*(P->PreError - P->I_L)/(P->I_U - P->I_L);    
		VAL_LIMIT(P->SumError,P->integral_limit,-(P->integral_limit));		
		}
			
		P->pout = Kp * P->PreError;
		
		P->iout = Ki * P->SumError;
		    
		    
		P->dout_last = P->dout;//����ȫ΢�� 
		P->dout = DM * P->RC_DF + P->dout_last * ( 1 - P->RC_DF ); //��DM����һ�׵�ͨ�˲�  
		
		
		P->out = P->pout+P->iout+P->dout;
		VAL_LIMIT(P->out,P->max_out ,-P->max_out);
		P->out_last  = P->out;
		
    return P->out;                             

}
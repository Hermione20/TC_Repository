#if(1)
#include "main.h"



u16 ADC_Buffer[CHANNAL_NUM];



void  Adc_Init(void)
{    
	GPIO_InitTypeDef  GPIO_InitStructure;
	ADC_CommonInitTypeDef ADC_CommonInitStructure;
	ADC_InitTypeDef       ADC_InitStructure;
	DMA_InitTypeDef     DMA_InitStructure;


	GPIO_StructInit(&GPIO_InitStructure);
	ADC_StructInit(&ADC_InitStructure);
	ADC_CommonStructInit(&ADC_CommonInitStructure);
	DMA_StructInit(&DMA_InitStructure);
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA, ENABLE);//ʹ��GPIOCʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1, ENABLE); //ʹ��ADC1ʱ��
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_DMA2, ENABLE);
	
	
	//CH0 1 2 3 5 6 7 8 14 15
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_2 | GPIO_Pin_3;													
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AN;//ģ������
	GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL ;//����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);//��ʼ��  	
	
	
	DMA_DeInit(DMA2_Stream4); 
	DMA_InitStructure.DMA_Channel           =   DMA_Channel_0;
	DMA_InitStructure.DMA_PeripheralBaseAddr=   (uint32_t)(&ADC1->DR);
	DMA_InitStructure.DMA_Memory0BaseAddr   =   (uint32_t)(&ADC_Buffer[0]);
	DMA_InitStructure.DMA_DIR               =   DMA_DIR_PeripheralToMemory;
	DMA_InitStructure.DMA_BufferSize        =   CHANNAL_NUM;
	DMA_InitStructure.DMA_PeripheralInc     =   DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc         =   DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_MemoryDataSize    =   DMA_MemoryDataSize_HalfWord;
	DMA_InitStructure.DMA_PeripheralDataSize=   DMA_PeripheralDataSize_HalfWord;
	DMA_InitStructure.DMA_Mode              =   DMA_Mode_Circular;
	DMA_InitStructure.DMA_Priority          =   DMA_Priority_High;
	DMA_InitStructure.DMA_FIFOMode          =   DMA_FIFOMode_Disable;
	DMA_InitStructure.DMA_FIFOThreshold     =   DMA_FIFOThreshold_Full;
	DMA_InitStructure.DMA_MemoryBurst       =   DMA_MemoryBurst_Single;
	DMA_InitStructure.DMA_PeripheralBurst   =   DMA_PeripheralBurst_Single;
	DMA_Init(DMA2_Stream4, &DMA_InitStructure);
	DMA_Cmd(DMA2_Stream4, ENABLE);
	

	
	ADC_DeInit();
	ADC_InitStructure.ADC_Resolution = ADC_Resolution_12b;//12λģʽ
	ADC_InitStructure.ADC_ScanConvMode = ENABLE;//ɨ��ģʽ	
	ADC_InitStructure.ADC_ContinuousConvMode = DISABLE;//�ر�����ת��
	ADC_InitStructure.ADC_ExternalTrigConvEdge = ADC_ExternalTrigConvEdge_None;//��ֹ������⣬ʹ���������
	ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;//�Ҷ���	
	ADC_InitStructure.ADC_NbrOfConversion = CHANNAL_NUM;//10��ת���ڹ��������� Ҳ����ֻת����������1 
	ADC_Init(ADC1, &ADC_InitStructure);//ADC��ʼ��
	
	
	ADC_CommonInitStructure.ADC_Mode = ADC_Mode_Independent;//����ģʽ
	ADC_CommonInitStructure.ADC_TwoSamplingDelay = ADC_TwoSamplingDelay_10Cycles;//���������׶�֮����ӳ�5��ʱ��
	ADC_CommonInitStructure.ADC_DMAAccessMode = ADC_DMAAccessMode_Disabled; //DMAʧ��
	ADC_CommonInitStructure.ADC_Prescaler = ADC_Prescaler_Div4;//Ԥ��Ƶ4��Ƶ��ADCCLK=PCLK2/4=84/4=21Mhz,ADCʱ����ò�Ҫ����36Mhz 
	ADC_CommonInit(&ADC_CommonInitStructure);//��ʼ��
	
	
	//CH0 2 3 5 6 8 9 10 11 12 13 14 15
  /* ADC1 regular channels configuration [????????]*/ 
	ADC_RegularChannelConfig(ADC1, ADC_Channel_2, 1, ADC_SampleTime_56Cycles);
	ADC_RegularChannelConfig(ADC1, ADC_Channel_3, 2, ADC_SampleTime_56Cycles);



	ADC_DMARequestAfterLastTransferCmd(ADC1, ENABLE); 
/* Enable ADC1 DMA [??ADC1 DMA]*/
	ADC_DMACmd(ADC1, ENABLE);

/* Enable ADC1 [??ADC1]*/
	ADC_Cmd(ADC1, ENABLE); 
/* Start ADC1 Software Conversion */
//	ADC_SoftwareStartConv(ADC1);
}

#else


	 
#endif
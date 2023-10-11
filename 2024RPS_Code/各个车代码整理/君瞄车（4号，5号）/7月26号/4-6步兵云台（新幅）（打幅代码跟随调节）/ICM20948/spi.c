#include "spi.h"

void spi1_init()
{
	SPI_InitTypeDef SPI_InitStruct;
	
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOA,ENABLE);  //
	RCC_AHB1PeriphClockCmd(RCC_AHB1Periph_GPIOC,ENABLE);
	GPIO_mode_af(GPIOA,GPIO_Pin_7);//MOSI
	GPIO_mode_af(GPIOA,GPIO_Pin_6);//MISO
	GPIO_mode_af(GPIOA,GPIO_Pin_5);//SCK
	
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource5,GPIO_AF_SPI1); //PA5����Ϊ SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource6,GPIO_AF_SPI1); //PA6����Ϊ SPI1
	GPIO_PinAFConfig(GPIOA,GPIO_PinSource7,GPIO_AF_SPI1); //PA7����Ϊ SPI1
	
	//CS   
	//gpio_mode_out(GPIOB,GPIO_Pin_12);	CS_AK = 1;
	GPIO_mode_out(GPIOC,GPIO_Pin_4);
  CS_H() ;
	//gpio_mode_out(GPIOA,GPIO_Pin_11);	CS_SPL = 1;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_SPI1,ENABLE);

	SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_32;//84/4 21Mhz
	SPI_InitStruct.SPI_CPHA = SPI_CPHA_2Edge;
	SPI_InitStruct.SPI_CPOL = SPI_CPOL_High;
	SPI_InitStruct.SPI_CRCPolynomial = 7;
	SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
	SPI_InitStruct.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
	SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
	SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
	SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
	SPI_Init(SPI1,&SPI_InitStruct);
	SPI_Cmd(SPI1, ENABLE);
 
}

void SPI1_SetSpeed(u8 SPI_BaudRatePrescaler)
{
	SPI_BaudRatePrescaler&=0X07;			//���Ʒ�Χ
	SPI1->CR1&=0XFFC7; 
	SPI1->CR1|=SPI_BaudRatePrescaler;	//����SPI1�ٶ�  
	SPI1->CR1|=1<<6; 		//SPI�豸ʹ��	  
} 

uint8_t spi1_read_write_byte(uint8_t txc)
{
	u16 retry = 0;
	while((SPI1->SR&SPI_SR_TXE)==0)
	{
		if(++retry > 100 )
			return 0;//�ӳ�һ��ʱ��󷵻�
	}
	SPI1->DR = txc;
	retry = 0;
	while((SPI1->SR&SPI_SR_RXNE)==0)
	{
			if(++retry > 100)
			return 0;//�ӳ�һ��ʱ��󷵻�
	}
	return SPI1->DR;	
}


uint8_t spi1_write_reg(uint8_t reg_addr,uint8_t reg_val)
{
	spi1_read_write_byte(reg_addr&0x7f);
	spi1_read_write_byte(reg_val);
	return 0;
}


uint8_t spi1_read_reg(uint8_t reg_addr)
{
	spi1_read_write_byte(reg_addr|0x80);
	return spi1_read_write_byte(0xff);
}

uint8_t spi1_read_reg_buffer(uint8_t reg_addr,void *buffer,uint16_t len)
{
	uint8_t *p = buffer;
	uint16_t i;
	spi1_read_write_byte(reg_addr|0x80);
	for(i=0;i<len;i++)
	{
		*p++= spi1_read_write_byte(0xff);
	}
	return 0;
}

void GPIO_mode_af(GPIO_TypeDef* GPIOx,uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_AF;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;//�������
	GPIO_InitStruct.GPIO_Pin = pin;
	GPIO_InitStruct.GPIO_PuPd = GPIO_PuPd_UP;//����
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_100MHz;
	GPIO_Init(GPIOx,&GPIO_InitStruct);
}

void GPIO_mode_out(GPIO_TypeDef* GPIOx,uint16_t pin)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	
	GPIO_InitStruct.GPIO_Mode = GPIO_Mode_OUT;
	GPIO_InitStruct.GPIO_Pin = pin;
	GPIO_InitStruct.GPIO_OType = GPIO_OType_PP;
	GPIO_InitStruct.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOx,&GPIO_InitStruct);
}

void GPIO_Set(GPIO_TypeDef* GPIOx,u32 BITx,u32 MODE,u32 OTYPE,u32 OSPEED,u32 PUPD)
{  
	u32 pinpos=0,pos=0,curpin=0;
	for(pinpos=0;pinpos<16;pinpos++)
	{
		pos=1<<pinpos;	//һ����λ��� 
		curpin=BITx&pos;//��������Ƿ�Ҫ����
		if(curpin==pos)	//��Ҫ����
		{
			GPIOx->MODER&=~(3<<(pinpos*2));	//�����ԭ��������
			GPIOx->MODER|=MODE<<(pinpos*2);	//�����µ�ģʽ 
			if((MODE==0X01)||(MODE==0X02))	//��������ģʽ/���ù���ģʽ
			{  
				GPIOx->OSPEEDR&=~(3<<(pinpos*2));	//���ԭ��������
				GPIOx->OSPEEDR|=(OSPEED<<(pinpos*2));//�����µ��ٶ�ֵ  
				GPIOx->OTYPER&=~(1<<pinpos) ;		//���ԭ��������
				GPIOx->OTYPER|=OTYPE<<pinpos;		//�����µ����ģʽ
			}  
			GPIOx->PUPDR&=~(3<<(pinpos*2));	//�����ԭ��������
			GPIOx->PUPDR|=PUPD<<(pinpos*2);	//�����µ�������
		}
	}
} 

void GPIO_AF_Set(GPIO_TypeDef* GPIOx,u8 BITx,u8 AFx)
{  
	GPIOx->AFR[BITx>>3]&=~(0X0F<<((BITx&0X07)*4));
	GPIOx->AFR[BITx>>3]|=(u32)AFx<<((BITx&0X07)*4);
}   

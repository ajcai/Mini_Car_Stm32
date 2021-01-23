#include "ioi2c.h"
#include "sys.h"
#include "delay.h"
/**************************************************************************
�������ܣ�IIC���ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void IIC_Init(void)
{			
	GPIO_InitTypeDef  GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB,ENABLE);//��ʹ������IO PORTCʱ�� 
		
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8;	 // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);					 //�����趨������ʼ��GPIO 
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;	 // �˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP; 		 //�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;		 //IO���ٶ�Ϊ50MHz
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	
	GPIO_SetBits(GPIOB,GPIO_Pin_8|GPIO_Pin_9);						 //PB8,PB9 �����	
}
/**************************************************************************
�������ܣ�ģ��IIC��ʼ�ź�
��ڲ�������
����  ֵ����
**************************************************************************/
void IIC_Start(void)
{
	SDA_OUT();    
	IIC_SDA=1;	  	  
	IIC_SCL=1;
	delay_us(4);
	//START:when CLK is high,DATA change form high to low 
 	IIC_SDA=0;
	delay_us(4);
	IIC_SCL=0;
}	  
/**************************************************************************
�������ܣ�ģ��IIC�����ź�
��ڲ�������
����  ֵ����
**************************************************************************/
void IIC_Stop(void)
{
	SDA_OUT();
	IIC_SCL=0;
	IIC_SDA=0;
 	delay_us(4);
	IIC_SCL=1; 
	IIC_SDA=1;
	delay_us(4);							   	
}
/**************************************************************************
�������ܣ�IIC����Ӧ���ź�
��ڲ�������
����  ֵ����
**************************************************************************/
unsigned char IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      
	IIC_SDA=1;delay_us(1);	   
	IIC_SCL=1;delay_us(1);	 
	while(READ_SDA)
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	}
	IIC_SCL=0;  
	return 0;  
} 
/**************************************************************************
�������ܣ�IICӦ��
��ڲ�������
����  ֵ����
**************************************************************************/
void IIC_Ack(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=0;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}
/**************************************************************************
�������ܣ�IIC��Ӧ��
��ڲ�������
����  ֵ����
**************************************************************************/ 
void IIC_NAck(void)
{
	IIC_SCL=0;
	SDA_OUT();
	IIC_SDA=1;
	delay_us(2);
	IIC_SCL=1;
	delay_us(2);
	IIC_SCL=0;
}					 				     
/**************************************************************************
�������ܣ�IIC����һ��λ
��ڲ�������
����  ֵ����
**************************************************************************/
void IIC_Send_Byte(u8 txd)
{                        
    u8 t;   
	SDA_OUT(); 	    
    IIC_SCL=0;
    for(t=0;t<8;t++)
    {              
        IIC_SDA=(txd&0x80)>>7;
        txd<<=1; 	  
		delay_us(2);   
		IIC_SCL=1;
		delay_us(2); 
		IIC_SCL=0;	
		delay_us(2);
    }	 
} 	    
/**************************************************************************
�������ܣ�II��ȡһ��λ
��ڲ�������
����  ֵ����
**************************************************************************/
unsigned char IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();
    for(i=0;i<8;i++ )
	{
        IIC_SCL=0; 
        delay_us(2);
		IIC_SCL=1;
        receive<<=1;
        if(READ_SDA)receive++;   
		delay_us(1); 
    }					 
    if (!ack)
        IIC_NAck();
    else
        IIC_Ack();
    return receive;
}


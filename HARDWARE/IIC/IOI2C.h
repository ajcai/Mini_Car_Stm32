#ifndef __IOI2C_H
#define __IOI2C_H
#include "stm32f10x.h"
#include "system.h"
 /**
 @ SCL  PB8
 @ SDA  PB9
**/ 	   		   
#define SDA_IN()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=8<<4;}
#define SDA_OUT()  {GPIOB->CRH&=0XFFFFFF0F;GPIOB->CRH|=3<<4;}

#define IIC_SCL    	PBout(8) 		//SCL
#define IIC_SDA    	PBout(9) //SDA
#define READ_SDA   PBin(9)  //ÊäÈëSDA 

void IIC_Init(void);                			 
void IIC_Start(void);				
void IIC_Stop(void);	  			
void IIC_Send_Byte(u8 txd);			
unsigned char IIC_Read_Byte(unsigned char ack);
unsigned char IIC_Wait_Ack(void); 				
void IIC_Ack(void);					
void IIC_NAck(void);				
#endif


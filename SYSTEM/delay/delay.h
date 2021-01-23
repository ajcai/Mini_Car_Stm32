#ifndef __DELAY_H
#define __DELAY_H 			   
#include "sys.h"  

u32 getSysTickCnt(void);	
void delay_init(void);
void delay_ms(u32 nms);
void delay_us(u32 nus);
void delay_xms(u32 nms);

#endif






























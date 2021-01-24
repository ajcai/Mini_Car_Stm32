#ifndef __TIMER_H
#define __TIMER_H
#include "system.h"
void TIM1_Cap_Init(u16 arr, u16 psc);


extern u32 L_Remoter_Ch1,L_Remoter_Ch2,L_Remoter_Ch3,L_Remoter_Ch4;	//航模遥控接收变量
extern u32 Remoter_Ch1,Remoter_Ch2,Remoter_Ch3,Remoter_Ch4;//航模遥控采集相关变量

#endif

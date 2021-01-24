#ifndef __LED_H
#define __LED_H
#include "sys.h"
#include "system.h"
//LED 端口定义

#define LED_TASK_PRIO		3     //任务优先级
#define LED_STK_SIZE 		128   //任务堆栈大小


/*--------Buzzer control pin--------*/

#define Buzzer_PORT GPIOB
#define Buzzer_PIN GPIO_Pin_10
#define Buzzer PBout(10)
/*----------------------------------*/

/*--------Buzzer control pin--------*/

#define LED_PORT GPIOE
#define LED_PIN GPIO_Pin_10
#define LED PEout(10) 
/*----------------------------------*/

void LED_Init(void);  
void Buzzer_Init(void); //init the buzzer
void Led_Flash(u16 time);
void led_task(void *pvParameters);
extern int Led_Count;
#endif

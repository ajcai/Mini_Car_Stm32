/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
版本V1.0
修改时间：2020-07-04
All rights reserved
***********************************************/
#include "system.h"

//任务优先级
#define START_TASK_PRIO			4
//任务堆栈大小	
#define START_STK_SIZE 			256  
//任务句柄
TaskHandle_t StartTask_Handler;
//任务函数
void start_task(void *pvParameters);

int main(void)
{ 
   systemInit();
	
	//创建开始任务
	
    xTaskCreate((TaskFunction_t )start_task,            //任务函数
                (const char*    )"start_task",          //任务名称
                (uint16_t       )START_STK_SIZE,        //任务堆栈大小
                (void*          )NULL,                  //传递给任务函数的参数
                (UBaseType_t    )START_TASK_PRIO,       //任务优先级
                (TaskHandle_t*  )&StartTask_Handler);   //任务句柄    
  						
    vTaskStartScheduler();          //开启任务调度
							
}

//开始任务任务函数
void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();           //进入临界区
    //创建任务
	  xTaskCreate(Balance_task, "Balance_task", BALANCE_STK_SIZE, NULL, BALANCE_TASK_PRIO, NULL);	//小车控制任务
	
	  xTaskCreate(MPU9250_task, "MPU9250_task", MPU9250_STK_SIZE, NULL, MPU9250_TASK_PRIO, NULL);	//陀螺仪数据读取任务
	
    xTaskCreate(show_task, "show_task", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO, NULL);  //显示屏显示任务
	
    xTaskCreate(led_task, "led_task", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL);	     //led灯任务

    xTaskCreate(pstwo_task, "PSTWO_task", PS2_STK_SIZE, NULL, PS2_TASK_PRIO, NULL);	 //手柄控制任务

    xTaskCreate(data_task, "DATA_task", DATA_STK_SIZE, NULL, DATA_TASK_PRIO, NULL);	 //串口3 发送/接收 数据任务
	
    vTaskDelete(StartTask_Handler); //删除开始任务

    taskEXIT_CRITICAL();            //退出临界区
	  
}




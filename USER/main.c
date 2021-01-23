/***********************************************
��˾����ݸ��΢�����ܿƼ����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
�汾V1.0
�޸�ʱ�䣺2020-07-04
All rights reserved
***********************************************/
#include "system.h"

//�������ȼ�
#define START_TASK_PRIO			4
//�����ջ��С	
#define START_STK_SIZE 			256  
//������
TaskHandle_t StartTask_Handler;
//������
void start_task(void *pvParameters);

int main(void)
{ 
   systemInit();
	
	//������ʼ����
	
    xTaskCreate((TaskFunction_t )start_task,            //������
                (const char*    )"start_task",          //��������
                (uint16_t       )START_STK_SIZE,        //�����ջ��С
                (void*          )NULL,                  //���ݸ��������Ĳ���
                (UBaseType_t    )START_TASK_PRIO,       //�������ȼ�
                (TaskHandle_t*  )&StartTask_Handler);   //������    
  						
    vTaskStartScheduler();          //�����������
							
}

//��ʼ����������
void start_task(void *pvParameters)
{

    taskENTER_CRITICAL();           //�����ٽ���
    //��������
	  xTaskCreate(Balance_task, "Balance_task", BALANCE_STK_SIZE, NULL, BALANCE_TASK_PRIO, NULL);	//С����������
	
	  xTaskCreate(MPU9250_task, "MPU9250_task", MPU9250_STK_SIZE, NULL, MPU9250_TASK_PRIO, NULL);	//���������ݶ�ȡ����
	
    xTaskCreate(show_task, "show_task", SHOW_STK_SIZE, NULL, SHOW_TASK_PRIO, NULL);  //��ʾ����ʾ����
	
    xTaskCreate(led_task, "led_task", LED_STK_SIZE, NULL, LED_TASK_PRIO, NULL);	     //led������

    xTaskCreate(pstwo_task, "PSTWO_task", PS2_STK_SIZE, NULL, PS2_TASK_PRIO, NULL);	 //�ֱ���������

    xTaskCreate(data_task, "DATA_task", DATA_STK_SIZE, NULL, DATA_TASK_PRIO, NULL);	 //����3 ����/���� ��������
	
    vTaskDelete(StartTask_Handler); //ɾ����ʼ����

    taskEXIT_CRITICAL();            //�˳��ٽ���
	  
}




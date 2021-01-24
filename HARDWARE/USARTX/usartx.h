#ifndef __USRATX_H
#define __USRATX_H

#include "stdio.h"
#include "sys.h"
#include "system.h"
#include "stm32f10x_conf.h"

#define DATA_STK_SIZE 512
#define DATA_TASK_PRIO 4

#define FRAME_HEADER 0X7B //发送数据的帧头
#define FRAME_TAIL 0X7D   //发送数据的帧头
#define SEND_DATA_SIZE 32
#define RECEIVE_DATA_SIZE 11
//#pragma pack(1)
/*****用于存放陀螺仪加速度计三轴数据结构体*****/
typedef struct __Mpu6050_Data_
{
  short X_data; //2个字节
  short Y_data; //2个字节
  short Z_data; //2个字节
} Mpu6050_Data;

/*****用于存放遥控数据结构体*****/
typedef struct __Control_Data_
{
  unsigned char Data[8]; //8个字节
} ControlData;

/*******串口发送数据的结构体************/
typedef struct _SEND_DATA_
{
  unsigned char buffer[SEND_DATA_SIZE];
  struct _Sensor_Str_
  {
    unsigned char Frame_Header; //1个字节
    // unsigned char Flag_Stop; //电机状态 1个字节
    short X_speed;       //2个字节
    short Y_speed;       //2个字节
    short Z_speed;       //2个字节

    Mpu6050_Data Accelerometer; //6个字节
    Mpu6050_Data Gyroscope;     //6个字节

    short Power_Voltage; //2个字节
    ControlData control_data; //8个字节
    // unsigned char check_sum; //校验位 1个字节
    unsigned char Frame_Tail; //1个字节
  } Sensor_Str;

} SEND_DATA;

typedef struct _RECEIVE_DATA_
{
  unsigned char buffer[RECEIVE_DATA_SIZE];
  struct _Control_Str_
  {
    unsigned char Frame_Header; // 1
    float X_speed;              //4个字节
    float Y_speed;              //4个字节
    float Z_speed;              //4个字节
    unsigned char Frame_Tail;   //1个字节
  } Control_Str;

} RECEIVE_DATA;
//#pragma pack(4)
void data_task(void *pvParameters);
void USART3_SEND(void);
void USART1_SEND(void);
void data_transition(void);
void usart2_send(u8 data);
void uart2_init(u32 bound);
int USART2_IRQHandler(void);
void usart3_send(u8 data);
void usart1_send(u8 data);
void uart3_init(u32 bound);
int USART3_IRQHandler(void);
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode);
void uart1_init(u32 bound);
int USART1_IRQHandler(void);

float XYZ_Target_Speed_transition(u8 High, u8 Low);
void CAN_SEND(void);

#endif

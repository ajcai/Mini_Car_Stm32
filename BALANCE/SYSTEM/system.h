#ifndef __SYSTEM_H
#define __SYSTEM_H

/* freertos 配置文件 */
#include "FreeRTOSConfig.h"
/*FreeRTOS相关头文件*/
#include "FreeRTOS.h"
#include "stm32f10x.h"
#include "task.h"
#include "queue.h"
#include "timers.h"
#include "semphr.h"
/*外设的相关头文件*/
#include "sys.h"
#include "delay.h"
#include "usart.h"
#include "balance.h"
#include "led.h"
#include "oled.h"
#include "usart.h"
#include "usartx.h"
#include "adc.h"
#include "can.h"
#include "motor.h"
#include "timer.h"
#include "encoder.h"
#include "ioi2c.h"
#include "mpu9250.h"
#include "show.h"
#include "pstwo.h"
#include "key.h"
#include "robot_select_init.h"

typedef enum
{
  Mec_Car = 0,
  Omni_Car,
  Akm_Car,
  Diff_Car,
  FourWheel_Car,
  Tank_Car
} CarMode;

typedef struct
{
  float Encoder;     //编码器数值
  float Motor_Pwm;   //电机PWM数值
  float Target;      //电机目标速度值
  float Velocity_KP; //速度控制PID参数
  float Velocity_KI; //速度控制PID参数
} Motor_parameter;

typedef struct
{
  float VX; //存放平滑控制处理后的数据
  float VY; //存放平滑控制处理后的数据
  float VZ; //存放平滑控制处理后的数据
} Smooth_Control;

extern long int ErrorCode;
extern int Check, Checking, Checked, CheckCount, CheckPhrase1, CheckPhrase2;
extern u8 Flag_Left, Flag_Right, Flag_sudu, Flag_Direction;
extern u8 PID_Send;
extern float RC_Velocity;
extern u8 PID_Send, Turn_Flag, Flag_Stop;
extern u8 APP_ON_Flag, PS2_ON_Flag, Remote_ON_Flag, CAN_ON_Flag, Usart_ON_Flag;
extern float Move_X, Move_Y, Move_Z;

extern float Velocity_KP, Velocity_KI;
extern float PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY;
extern unsigned char temp_show;
extern u8 Car_Mode;
extern int Divisor_Mode;
extern float Wheel_spacing;     //主动轮轮距
extern float Encoder_precision; //编码器精度
extern float Wheel_perimeter;   //轮子周长
extern float Axle_spacing;      //麦轮前后轴轴距
extern float Omni_turn_radiaus; //全向轮转弯半径
extern Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D;
extern int Servo; //舵机PWM值
extern Smooth_Control smooth_control;
extern int robot_mode_check_flag;

void systemInit(void);
#define CONTROL_DELAY 1000 //换算成实际时间是10秒
#define CAR_NUMBER 6       //一共多少个车
#define RATE_1_HZ 1
#define RATE_5_HZ 5
#define RATE_10_HZ 10
#define RATE_20_HZ 20
#define RATE_25_HZ 25
#define RATE_50_HZ 50
#define RATE_100_HZ 100
#define RATE_200_HZ 200
#define RATE_250_HZ 250
#define RATE_500_HZ 500
#define RATE_1000_HZ 1000

/*一些C库函数的相关头文件*/
#include <stdio.h>
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include "stdarg.h"
#endif /* __SYSTEM_H */

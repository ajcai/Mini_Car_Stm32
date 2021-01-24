/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
版本V1.1
修改时间：2020-07-04
All rights reserved
***********************************************/
#include "system.h"

//通过这个宏定义区分麦轮车还是全向轮车 Omni_or_Mec，0是麦轮车系列，1是全向轮系列

//Car_Mode for Mec
//0:高配麦轮无轴承座SENIOR_MEC_NO
//1:高配麦轮摆式悬挂SENIOR_MEC_BS
//2:高配麦轮独立悬挂SENIOR_MEC_DL
//3:顶配麦轮摆式悬挂常规型TOP_MEC_BS_18
//4:顶配麦轮摆式悬挂重载型TOP_MEC_BS_47
//5:顶配麦轮独立悬挂常规型TOP_MEC_DL_18

//Car_Mode for Omni
//0:高配全向轮三角极速版SENIOR_OMNI_FAST

/***********************初始化相关标志位****************************/
u8 Flag_Left, Flag_Right, Flag_Direction = 0, Turn_Flag;                                      //蓝牙遥控相关的标志位
u8 PID_Send;                                                                                  //APP通信的相关标志位
u8 PS2_ON_Flag = 0, APP_ON_Flag = 0, Remote_ON_Flag = 0, CAN_ON_Flag = 0, Usart_ON_Flag = 0;  //手柄、蓝牙、航模遥控控制标志位
u8 Car_Mode = 0;                                                                              //机器人选型标志位
u8 Flag_Stop = 1;                                                                             //机器人使能标志位
int Check = 0, Checking = 0, Checked = 0, CheckCount = 0, CheckPhrase1 = 0, CheckPhrase2 = 0; //自检标志位
long int ErrorCode = 0;                                                                       //错误代码
/****************************************************************/
float PS2_LX, PS2_LY, PS2_RX, PS2_RY, PS2_KEY; //PS2相关变量

float Velocity_KP = 300, Velocity_KI = 300; //速度控制PID参数
float RC_Velocity = 500;                    //设置遥控小车的速度 单位mm/s
float Move_X, Move_Y, Move_Z;               //小车各轴运动目标速度

unsigned char temp_show;
int Divisor_Mode;
float Encoder_precision;                            //编码器精度
float Wheel_perimeter;                              //轮子周长（单位：米）
float Wheel_spacing;                                //主动轮轮距 （单位：米）
float Axle_spacing;                                 //麦轮前后轴轴距
float Omni_turn_radiaus;                            //全向轮转弯半径
Motor_parameter MOTOR_A, MOTOR_B, MOTOR_C, MOTOR_D; //电机的参数结构体
int Servo;                                          //舵机PWM值
Smooth_Control smooth_control;                      //平滑控制中间变量
void systemInit(void)
{
  JTAG_Set(JTAG_SWD_DISABLE); //=====关闭JTAG接口
  JTAG_Set(SWD_ENABLE);       //=====打开SWD接口 可以利用主板的SWD接口调试

  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4); //设置系统中断优先级分组4
  delay_init();                                   //初始化延时函数
  LED_Init();                                     //初始化与 LED 连接的硬件接口
  Buzzer_Init();                                  //初始化蜂鸣器
  Enable_Pin();                                   //使能开关初始化
  OLED_Init();                                    //OLED初始化
  KEY_Init();                                     //按键初始化
  uart1_init(115200);                             //=====串口初始化为
  uart2_init(9600);                               //串口2初始化
  uart3_init(115200);                             //串口3初始化
  CAN1_Mode_Init(1, 2, 3, 6, 0);                  //=====CAN初始化

  Adc_Init();     //采集电池电压ADC引脚初始化
  Robot_Select(); //根据电位器的值判断目前正在运行的是哪一款机器人，然后进行对应的参数初始化

  if (Car_Mode == Akm_Car)
    Servo_PWM_Init(9999, 71); //初始化PWM
  else
    TIM1_Cap_Init(9999, 71); //初始化航模遥控接口（要注意的是4路预留PWM输出和航模遥控的引脚冲突）

  Encoder_Init_TIM2();           //=====编码器接口A初始化
  Encoder_Init_TIM3();           //=====编码器接口B初始化
  Encoder_Init_TIM4();           //=====编码器接口C初始化
  Encoder_Init_TIM5();           //=====编码器接口D初始化
  MiniBalance_Motor_Init();      //初始化控制电机正反转引脚
  MiniBalance_PWM_Init(7199, 0); //初始化PWM 10KHZ，用于驱动电机

  IIC_Init();     //IIC初始化
  MPU9250_Init(); //MPU9250初始化

  PS2_Init();    //ps2驱动端口初始化
  PS2_SetInit(); //ps2配置初始化,配置“红绿灯模式”，并选择是否可以修改
}

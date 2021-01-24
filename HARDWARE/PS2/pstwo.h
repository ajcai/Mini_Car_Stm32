#ifndef __PSTWO_H
#define __PSTWO_H
#include "delay.h"
#include "sys.h"
#include "system.h"

#define PS2_TASK_PRIO 5  //任务优先级
#define PS2_STK_SIZE 256 //任务堆栈大小

#define DI PEin(15) // 输入引脚

#define DO_H PDout(3) = 1 //命令位高
#define DO_L PDout(3) = 0 //命令位低

#define CS_H PBout(13) = 1 //CS拉高
#define CS_L PBout(13) = 0 //CS拉低

#define CLK_H PBout(12) = 1 //时钟拉高
#define CLK_L PBout(12) = 0 //时钟拉低

//These are our button constants
#define PSB_SELECT 1
#define PSB_L3 2
#define PSB_R3 3
#define PSB_START 4
#define PSB_PAD_UP 5
#define PSB_PAD_RIGHT 6
#define PSB_PAD_DOWN 7
#define PSB_PAD_LEFT 8
#define PSB_L2 9
#define PSB_R2 10
#define PSB_L1 11
#define PSB_R1 12
#define PSB_GREEN 13
#define PSB_RED 14
#define PSB_BLUE 15
#define PSB_PINK 16

#define PSB_TRIANGLE 13
#define PSB_CIRCLE 14
#define PSB_CROSS 15
#define PSB_SQUARE 16

#define PSS_RX 5 //右摇杆X轴数据
#define PSS_RY 6 //左摇杆Y轴数据
#define PSS_LX 7 //右摇杆X轴数据
#define PSS_LY 8 //左摇杆Y轴数据

extern u8 Data[9];
extern u16 MASK[16];
extern u16 Handkey;
void PS2_Read(void);
void PS2_Init(void);
u8 PS2_RedLight(void);                    //判断是否为红灯模式
void PS2_ReadData(void);                  //读手柄数据
void PS2_Cmd(u8 CMD);                     //向手柄发送命令
u8 PS2_DataKey(void);                     //按键值读取
u8 PS2_AnologData(u8 button);             //得到一个摇杆的模拟量
void PS2_ClearData(void);                 //清除数据缓冲区
void PS2_Vibration(u8 motor1, u8 motor2); //振动设置motor1  0xFF开，其他关，motor2  0x40~0xFF

void PS2_EnterConfing(void);     //进入配置
void PS2_TurnOnAnalogMode(void); //发送模拟量
void PS2_VibrationMode(void);    //振动设置
void PS2_ExitConfing(void);      //完成配置
void PS2_SetInit(void);          //配置初始化
void PS2_Receive(void);
void pstwo_task(void *pvParameters);
#endif

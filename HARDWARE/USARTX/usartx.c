#include "usartx.h"
SEND_DATA Send_Data;       //发送数据的
RECEIVE_DATA Receive_Data; //接收数据的
extern int Time_count;
/**************************************************************************
函数功能：串口2发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart2_send(u8 data)
{
  USART2->DR = data;
  while ((USART2->SR & 0x40) == 0)
    ;
}
/**************************************************************************
函数功能：串口2初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart2_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   // 需要使能AFIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);  //使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE); //使能USART3时钟
  GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);        //引脚重映射

  //USART_TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //PD5
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //USART_RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;     //PD6
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
  GPIO_Init(GPIOD, &GPIO_InitStructure);
  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1; //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;        //子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;           //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);                           //根据指定的参数初始化VIC寄存器
  //USART 初始化设置
  USART_InitStructure.USART_BaudRate = bound;                                     //串口波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
  USART_Init(USART2, &USART_InitStructure);                                       //初始化串口2
  USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);                                  //开启串口接受中断
  USART_Cmd(USART2, ENABLE);                                                      //使能串口2
}
/**************************************************************************
函数功能：串口2接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART2_IRQHandler(void)
{
  int Usart_Receive;
  if (USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //接收到数据
  {
    static u8 Flag_PID, i, j, Receive[50], Last_Usart_Receive;
    static float Data;

    Usart_Receive = USART2->DR;

    if (Deviation_Count < CONTROL_DELAY)
      return 0; //前期不进入中断

    if (Usart_Receive == 0x41 && Last_Usart_Receive == 0x41 && APP_ON_Flag == 0)
      PS2_ON_Flag = 0, Remote_ON_Flag = 0, APP_ON_Flag = 1, CAN_ON_Flag = 0, Usart_ON_Flag = 0; //使能串口控制开机10秒之后按APP的前进键进入
    if (Usart_Receive == 0x4B)
      Turn_Flag = 1; //进入转向控制界面
    else if (Usart_Receive == 0x49 || Usart_Receive == 0x4A)
      Turn_Flag = 0; //方向控制界面
    Last_Usart_Receive = Usart_Receive;

    if (Turn_Flag == 0) //摇杆控制界面
    {
      if (Usart_Receive >= 0x41 && Usart_Receive <= 0x48)
      {
        Flag_Direction = Usart_Receive - 0x40;
      }
      else if (Usart_Receive <= 8)
      {
        Flag_Direction = Usart_Receive;
      }
      else
        Flag_Direction = 0;
    }
    else if (Turn_Flag == 1) //按键控制界面
    {
      if (Usart_Receive == 0x43)
        Flag_Left = 0, Flag_Right = 1; //右自转
      else if (Usart_Receive == 0x47)
        Flag_Left = 1, Flag_Right = 0; //左自转
      else
        Flag_Left = 0, Flag_Right = 0;
      if (Usart_Receive == 0x41 || Usart_Receive == 0x45)
        Flag_Direction = Usart_Receive - 0x40;
      else
        Flag_Direction = 0;
    }
    if (Usart_Receive == 0x58)
      RC_Velocity = RC_Velocity + 100; //加速按键，100
    if (Usart_Receive == 0x59)
      RC_Velocity = RC_Velocity - 100; //减速按键，100

    //以下是与APP调试界面通讯
    if (Usart_Receive == 0x7B)
      Flag_PID = 1; //APP参数指令起始位
    if (Usart_Receive == 0x7D)
      Flag_PID = 2; //APP参数指令停止位

    if (Flag_PID == 1) //采集数据
    {
      Receive[i] = Usart_Receive;
      i++;
    }
    if (Flag_PID == 2) //分析数据
    {
      if (Receive[3] == 0x50)
        PID_Send = 1;
      else if (Receive[1] != 0x23)
      {
        for (j = i; j >= 4; j--)
        {
          Data += (Receive[j - 1] - 48) * pow(10, i - j);
        }
        switch (Receive[1])
        {
        case 0x30:
          RC_Velocity = Data;
          break;
        case 0x31:
          Velocity_KP = Data;
          break;
        case 0x32:
          Velocity_KI = Data;
          break;
        case 0x33:
          break;
        case 0x34:
          break;
        case 0x35:
          break;
        case 0x36:
          break;
        case 0x37:
          break;
        case 0x38:
          break; //预留
        }
      }
      Flag_PID = 0; //相关标志位清零
      i = 0;
      j = 0;
      Data = 0;
      memset(Receive, 0, sizeof(u8) * 50); //数组清零
    }
    if (RC_Velocity < 0)
      RC_Velocity = 0;
  }
  return 0;
}
/**************************************************************************
函数功能：串口3发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart3_send(u8 data)
{
  USART3->DR = data;
  while ((USART3->SR & 0x40) == 0)
    ;
}
/**************************************************************************
函数功能：串口1发送数据
入口参数：要发送的数据
返回  值：无
**************************************************************************/
void usart1_send(u8 data)
{
  USART1->DR = data;
  while ((USART1->SR & 0x40) == 0)
    ;
}
/**************************************************************************
函数功能：串口3初始化
入口参数：无
返回  值：无
**************************************************************************/
void uart3_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);   // 需要使能AFIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);  //使能GPIO时钟
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE); //使能USART3时钟
  GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE); //引脚重映射
                                                         //USART_TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;             //C10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //USART_RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;    //PC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //上拉输入
  GPIO_Init(GPIOC, &GPIO_InitStructure);
  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;                       //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                              //子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                 //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);                                                 //根据指定的参数初始化VIC寄存器
                                                                                  //USART 初始化设置
  USART_InitStructure.USART_BaudRate = bound;                                     //串口波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
  USART_Init(USART3, &USART_InitStructure);                                       //初始化串口3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);                                  //开启串口接受中断
  USART_Cmd(USART3, ENABLE);                                                      //使能串口3
}
/**************************************************************************
函数功能：串口3接收中断
入口参数：无
返回  值：无
**************************************************************************/
int USART3_IRQHandler(void)
{
  static u8 Count = 0;
  u8 Usart_Receive;

  if (USART_GetITStatus(USART3, USART_IT_RXNE) != RESET) //判断是否接收到数据
  {
    Usart_Receive = USART_ReceiveData(USART3); //读取数据
    if (Time_count < CONTROL_DELAY)
      return 0; //前期不进入中断
    Receive_Data.buffer[Count] = Usart_Receive;
    if (Usart_Receive == FRAME_HEADER || Count > 0)
      Count++;
    else
      Count = 0;
    if (Count == 11) //验证数据包的长度
    {
      Count = 0;                                 //重新开始接收
      if (Receive_Data.buffer[10] == FRAME_TAIL) //验证数据包的尾部校验信息
      {
        if (Receive_Data.buffer[9] == Check_Sum(9, 0)) //数据校验位计算，模式0是发送数据校验
        {
          PS2_ON_Flag = 0; //其他模式关停
          Remote_ON_Flag = 0;
          APP_ON_Flag = 0;
          CAN_ON_Flag = 0;
          Usart_ON_Flag = 0; //进入串口3中断，强行进入ROS模式，优先级最高
          Move_X = XYZ_Target_Speed_transition(Receive_Data.buffer[3], Receive_Data.buffer[4]);
          Move_Y = XYZ_Target_Speed_transition(Receive_Data.buffer[5], Receive_Data.buffer[6]);
          Move_Z = XYZ_Target_Speed_transition(Receive_Data.buffer[7], Receive_Data.buffer[8]);
        }
      }
    }
  }
  return 0;
}
/**************************************************************************
*  函数功能：串口1初始化
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/
void uart1_init(u32 bound)
{
  //GPIO端口设置
  GPIO_InitTypeDef GPIO_InitStructure;
  USART_InitTypeDef USART_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);  //使能GPIO时钟
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE); //使能USART时钟

  //USART_TX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //USART_RX
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;    //PA10
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU; //浮空输入
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  //UsartNVIC 配置
  NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;                       //抢占优先级
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;                              //子优先级
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;                                 //IRQ通道使能
  NVIC_Init(&NVIC_InitStructure);                                                 //根据指定的参数初始化VIC寄存器
                                                                                  //USART 初始化设置
  USART_InitStructure.USART_BaudRate = bound;                                     //串口波特率
  USART_InitStructure.USART_WordLength = USART_WordLength_8b;                     //字长为8位数据格式
  USART_InitStructure.USART_StopBits = USART_StopBits_1;                          //一个停止位
  USART_InitStructure.USART_Parity = USART_Parity_No;                             //无奇偶校验位
  USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None; //无硬件数据流控制
  USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;                 //收发模式
  USART_Init(USART1, &USART_InitStructure);                                       //初始化串口1
  USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);                                  //开启串口接受中断
  USART_Cmd(USART1, ENABLE);                                                      //使能串口1
}
/**************************************************************************
*  函数功能：串口1接收中断
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/
int USART1_IRQHandler(void)
{
  if (USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //接收到数据
  {
    u8 Usart_Receive;
    static u8 Count;
    static u8 rxbuf[11];
    int check = 0, error = 1, i;

    Usart_Receive = USART_ReceiveData(USART1); //读取数据
    if (Time_count < CONTROL_DELAY)
      return 0; //前期不进入中断

    //读取到完整数据，开始校验，校验成功则赋值XYZ目标速度
    rxbuf[Count] = Usart_Receive;
    if (Usart_Receive == FRAME_HEADER || Count > 0)
      Count++;
    else
      Count = 0;
    if (Count == 11) //验证数据包的长度
    {
      Count = 0;                   //重新开始接收
      if (rxbuf[10] == FRAME_TAIL) //验证数据包的尾部校验信息
      {

        for (i = 0; i < 9; i++)
        {
          check = rxbuf[i] ^ check; //异或，用于检测数据是否出错
        }
        if (check == rxbuf[9])
          error = 0; //检验成功

        if (error == 0) //数据校验位计算
        {
          if (Usart_ON_Flag == 0)
          {
            Usart_ON_Flag = 1;
            APP_ON_Flag = 0;
            PS2_ON_Flag = 0;
            Remote_ON_Flag = 0;
            CAN_ON_Flag = 0;
          }

          Move_X = (short)((rxbuf[3] << 8) + (rxbuf[4])); //求X轴速度 分高8位和低8位 单位mm/s
          Move_Y = (short)((rxbuf[5] << 8) + (rxbuf[6])); //求X轴速度 分高8位和低8位 单位mm/s
          Move_Z = (short)((rxbuf[7] << 8) + (rxbuf[8])); //求Z轴速度 分高8位和低8位 单位mm/s

          Move_X = Move_X / 1000; //单位m/s
          Move_Y = Move_Y / 1000; //单位m/s
          Move_Z = Move_Z / 1000; //单位m/s
        }
      }
    }
  }
  return 0;
}
/**************************************************************************
函数功能：串口3发送任务函数
入口参数：无
返回  值：无
**************************************************************************/
void data_task(void *pvParameters)
{
  u32 lastWakeTime = getSysTickCnt();

  while (1)
  {
    vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ)); //此任务以20Hz的频率运行
    data_transition();                               //对要进行发送的数据进行赋值
    USART3_SEND();                                   //串口3(ROS)发送数据
    CAN_SEND();                                      //CAN发送数据
    USART1_SEND();                                   //串口1发送数据需要关闭航模初始化TIM1_Cap_Init(0XFFFF,72-1);
  }
}
/**************************************************************************
函数功能：串口发送的数据进行赋值
入口参数：无
返回  值：无
**************************************************************************/
void data_transition(void)
{
  Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //帧头
  Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL;     //帧尾

  switch (Car_Mode)
  {
  case Mec_Car:                                                                                                                                            //麦克纳姆轮小车
    Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 4) * 1000;                                   //小车x轴速度
    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder - MOTOR_B.Encoder + MOTOR_C.Encoder - MOTOR_D.Encoder) / 4) * 1000;                                   //小车y轴速度
    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder - MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 4 / (Axle_spacing + Wheel_spacing)) * 1000; //小车z轴速度
    break;

  case Omni_Car:                                                                                                           //全向轮小车
    Send_Data.Sensor_Str.X_speed = ((MOTOR_C.Encoder - MOTOR_B.Encoder) / 2 / X_PARAMETER) * 1000;                         //小车x轴速度
    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder * 2 - MOTOR_B.Encoder - MOTOR_C.Encoder) / 3) * 1000;                 //小车y轴速度
    Send_Data.Sensor_Str.Z_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder) / 3 / Omni_turn_radiaus) * 1000; //小车z轴速度
    break;

  case Akm_Car:                                                                      //阿克曼小车
    Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000; //小车x轴速度
    Send_Data.Sensor_Str.Y_speed = 0;
    Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / Wheel_spacing) * 1000; //小车z轴速度
    break;

  case Diff_Car:                                                                     //两轮差速小车
    Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000; //小车x轴速度
    Send_Data.Sensor_Str.Y_speed = 0;
    Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / Wheel_spacing) * 1000; //小车z轴速度
    break;

  case FourWheel_Car:                                                                                                    //四驱车
    Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 4) * 1000; //小车x轴速度
    Send_Data.Sensor_Str.Y_speed = 0;
    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_B.Encoder - MOTOR_A.Encoder + MOTOR_C.Encoder + MOTOR_D.Encoder) / 2 / (Axle_spacing + Wheel_spacing)) * 1000; //小车z轴速度
    break;

  case Tank_Car:                                                                     //履带车
    Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder + MOTOR_B.Encoder) / 2) * 1000; //小车x轴速度
    Send_Data.Sensor_Str.Y_speed = 0;
    Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder - MOTOR_A.Encoder) / (Wheel_spacing)*1000); //小车z轴速度
    break;
  }

  //加速度计三轴加速度
  Send_Data.Sensor_Str.Accelerometer.X_data = accel[1];  //加速度计Y轴转换到ROS坐标X轴
  Send_Data.Sensor_Str.Accelerometer.Y_data = -accel[0]; //加速度计X轴转换到ROS坐标Y轴
  Send_Data.Sensor_Str.Accelerometer.Z_data = accel[2];

  //角速度计三轴角速度
  Send_Data.Sensor_Str.Gyroscope.X_data = gyro[1];  //角速度计Y轴转换到ROS坐标X轴
  Send_Data.Sensor_Str.Gyroscope.Y_data = -gyro[0]; //角速度计X轴转换到ROS坐标Y轴
  if (Flag_Stop == 0)
    Send_Data.Sensor_Str.Gyroscope.Z_data = gyro[2]; //如果电机控制位使能状态，那么正常发送Z轴角速度
  else
    Send_Data.Sensor_Str.Gyroscope.Z_data = 0; //如果机器人是静止的（电机控制位失能），那么发送的Z轴角速度为0

  Send_Data.Sensor_Str.Power_Voltage = Voltage * 1000; //电池电压(这里将浮点数放大一千倍传输，相应的在接收端在接收到数据后也会缩小一千倍)

  Send_Data.buffer[0] = Send_Data.Sensor_Str.Frame_Header; //帧头(固定值)
  Send_Data.buffer[1] = Flag_Stop;                         //电机状态

  Send_Data.buffer[2] = Send_Data.Sensor_Str.X_speed >> 8; //小车x轴速度
  Send_Data.buffer[3] = Send_Data.Sensor_Str.X_speed;      //小车x轴速度
  Send_Data.buffer[4] = Send_Data.Sensor_Str.Y_speed >> 8; //小车y轴速度
  Send_Data.buffer[5] = Send_Data.Sensor_Str.Y_speed;      //小车y轴速度
  Send_Data.buffer[6] = Send_Data.Sensor_Str.Z_speed >> 8; //小车z轴速度
  Send_Data.buffer[7] = Send_Data.Sensor_Str.Z_speed;      //小车z轴速度

  Send_Data.buffer[8] = Send_Data.Sensor_Str.Accelerometer.X_data >> 8; //加速度计三轴加速度
  Send_Data.buffer[9] = Send_Data.Sensor_Str.Accelerometer.X_data;      //加速度计三轴加速度
  Send_Data.buffer[10] = Send_Data.Sensor_Str.Accelerometer.Y_data >> 8;
  Send_Data.buffer[11] = Send_Data.Sensor_Str.Accelerometer.Y_data;
  Send_Data.buffer[12] = Send_Data.Sensor_Str.Accelerometer.Z_data >> 8;
  Send_Data.buffer[13] = Send_Data.Sensor_Str.Accelerometer.Z_data;

  Send_Data.buffer[14] = Send_Data.Sensor_Str.Gyroscope.X_data >> 8; //角速度计三轴角速度
  Send_Data.buffer[15] = Send_Data.Sensor_Str.Gyroscope.X_data;      //角速度计三轴角速度
  Send_Data.buffer[16] = Send_Data.Sensor_Str.Gyroscope.Y_data >> 8;
  Send_Data.buffer[17] = Send_Data.Sensor_Str.Gyroscope.Y_data;
  Send_Data.buffer[18] = Send_Data.Sensor_Str.Gyroscope.Z_data >> 8;
  Send_Data.buffer[19] = Send_Data.Sensor_Str.Gyroscope.Z_data;
  Send_Data.buffer[20] = Send_Data.Sensor_Str.Power_Voltage >> 8; //电池电压
  Send_Data.buffer[21] = Send_Data.Sensor_Str.Power_Voltage;      //电池电压

  Send_Data.buffer[22] = Check_Sum(22, 1); //数据校验位计算，模式1是发送数据校验

  Send_Data.buffer[23] = Send_Data.Sensor_Str.Frame_Tail; //帧尾（固定值）
}
/**************************************************************************
函数功能：串口3(ROS)发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART3_SEND(void)
{
  unsigned char i = 0;
  for (i = 0; i < SEND_DATA_SIZE; i++)
  {
    usart3_send(Send_Data.buffer[i]);
  }
}

/**************************************************************************
函数功能：串口发送数据
入口参数：无
返回  值：无
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;

  for (i = 0; i < SEND_DATA_SIZE; i++)
  {
    usart1_send(Send_Data.buffer[i]);
  }
}

/**************************************************************************
函数功能：计算发送的数据校验位
入口参数：
返回  值：检验位
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number, unsigned char Mode)
{
  unsigned char check_sum = 0, k;
  //发送数据的校验
  if (Mode == 1)
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Send_Data.buffer[k];
    }
  //接收数据的校验
  if (Mode == 0)
    for (k = 0; k < Count_Number; k++)
    {
      check_sum = check_sum ^ Receive_Data.buffer[k];
    }
  return check_sum;
}

/**************************************************************************
函数功能：将上位机发过来的高8位和低8位数据整合成一个short型数据后，再做单位还原换算
入口参数：高8位，低8位
返回  值：机器人X/Y/Z轴的目标速度
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High, u8 Low)
{
  short transition;                                       //数据转换的中间变量
  transition = ((High << 8) + Low);                       //将高8位和低8位整合成一个16位的short型数据
  return transition / 1000 + (transition % 1000) * 0.001; //发送端将数据发送前做了一个*1000的单位换算，这里接收数据后需要还原单位
}

/**************************************************************************
*  函数功能：CAN发送数据
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/
void CAN_SEND(void)
{
  u8 CAN_SENT[8], i;

  for (i = 0; i < 8; i++)
  {
    CAN_SENT[i] = Send_Data.buffer[i];
  }
  CAN1_Send_Num(0x101, CAN_SENT);

  for (i = 0; i < 8; i++)
  {
    CAN_SENT[i] = Send_Data.buffer[i + 8];
  }
  CAN1_Send_Num(0x102, CAN_SENT);

  for (i = 0; i < 8; i++)
  {
    CAN_SENT[i] = Send_Data.buffer[i + 16];
  }
  CAN1_Send_Num(0x103, CAN_SENT);
}

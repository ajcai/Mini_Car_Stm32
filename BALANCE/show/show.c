#include "show.h"
int Voltage_Show;
unsigned char i;          //计数变量
unsigned char Send_Count; //串口需要发送的数据个数
extern SEND_DATA Send_Data;//来源与usart.x，为根据车轮速度求出的三轴速度
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; //联合陀螺仪数据检测编码器与驱动
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;

/**************************************************************************
函数功能：OLED显示屏显示任务
入口参数：无
返回  值：无
**************************************************************************/
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
    {	
		int i=0;
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));//此任务以50Hz的频率运行
			if(Deviation_Count<50)Buzzer=1; //蜂鸣器
			else Buzzer=0;
		
		for(i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage_All=0;
		
//			Voltage_All+=Get_battery_volt();                //多次采样累积
//			if(++Voltage_Count==20) Voltage=Voltage_All/20,Voltage_All=0,Voltage_Count=0;//求平均值 获取电池电压
//		Voltage=Get_battery_volt();
			if(Voltage>=12.6)Voltage=12.6;//避免超电池电压
			else if(Voltage<10.5)Voltage=Voltage-0.5;//低电量的时候显示更低一点及时充电
				
		  if(Deviation_Count>=CONTROL_DELAY)				
			{
				if(Long_Press()==1)
				{
					OLED_Clear();
					Check=!Check;		
					if(Check==1)Flag_Stop=1;
					else Flag_Stop=0;					
					Checking=!Checking;
					Checked=0;
					CheckCount=0;
					ErrorCode=0;
					EncoderA_Count=0, EncoderB_Count=0, EncoderC_Count=0, EncoderD_Count=0;
					MPU9250SensorCountA=0, MPU9250SensorCountB=0, MPU9250SensorCountC=0, MPU9250SensorCountD=0;
				}
			}			
		  APP_Show();	    //向MiniBalance APP发送数据
			oled_show();    //显示屏打开
    }
}  

/**************************************************************************
函数功能：OLED显示
入口参数：无
返回  值：无
**************************************************************************/
void oled_show(void)
{  
   static int count=0;	 
	 int Car_Mode_Show, Divisor_Mode;
	 Divisor_Mode=2048/CAR_NUMBER+5;
	 Car_Mode_Show=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //采集电位器引脚信息	
	 if(Car_Mode_Show>5)Car_Mode_Show=5;
	 Voltage_Show=Voltage*100; 
	 count++;
	
	 if(Check==0)//没有进入自检模式时小车正常显示
	 {	
		 //显示屏第1行显示内容//
		 switch(Car_Mode_Show)
		 {
			case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; //麦克纳姆轮小车
			case Omni_Car:      OLED_ShowString(0,0,"Omni");  break; //全向轮小车
			case Akm_Car:       OLED_ShowString(0,0,"Akm ");  break; //阿克曼小车
			case Diff_Car:      OLED_ShowString(0,0,"Diff"); break; //两轮差速小车
			case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; //四驱车 
			case Tank_Car:      OLED_ShowString(0,0,"Tank"); break; //履带车
		 }
		 
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		 {
			 //麦轮、全向轮小车显示Z轴角速度
			 OLED_ShowString(55,0,"GZ");
			 if( gyro[2]<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-gyro[2],5,12);
			 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, gyro[2],5,12);		//z轴陀螺仪零点漂移数据	
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==FourWheel_Car||Car_Mode==Tank_Car)
		 {
			//阿克曼、差速、四驱、履带车显示Z轴角偏差值
			OLED_ShowString(55,0,"BIAS");
			if( Deviation_gyro[2]<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-Deviation_gyro[2],3,12);
			else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, Deviation_gyro[2],3,12);		//z轴陀螺仪零点漂移数据	
		 }
		 //显示屏第1行显示内容//
		 

		 //显示屏第2行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//麦轮、全向轮、四驱车显示电机A的状态
			OLED_ShowString(0,10,"A");
			if( MOTOR_A.Target<0)	OLED_ShowString(15,10,"-"),
														OLED_ShowNumber(20,10,-MOTOR_A.Target*1000,5,12);
			else                 	OLED_ShowString(15,10,"+"),
														OLED_ShowNumber(20,10, MOTOR_A.Target*1000,5,12); 
			
			if( MOTOR_A.Encoder<0)OLED_ShowString(60,10,"-"),
														OLED_ShowNumber(75,10,-MOTOR_A.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,10,"+"),
														OLED_ShowNumber(75,10, MOTOR_A.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //阿克曼、差速小车显示Z轴角速度
			 OLED_ShowString(00,10,"GYRO_Z:");
			 if( gyro[2]<0)  OLED_ShowString(60,10,"-"),
											 OLED_ShowNumber(75,10,-gyro[2],5,12);
			 else            OLED_ShowString(60,10,"+"),
											 OLED_ShowNumber(75,10, gyro[2],5,12);		//z轴陀螺仪数据	
		 }	 
		 //显示屏第2行显示内容//
		 
		 
		 //显示屏第3、4行显示内容//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//麦轮、全向轮、四驱车显示电机B的状态
			OLED_ShowString(0,20,"B");		
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
			
			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
			//麦轮、全向轮、四驱车显示电机C的状态	
			OLED_ShowString(0,30,"C");
			if( MOTOR_C.Target<0)	OLED_ShowString(15,30,"-"),
														OLED_ShowNumber(20,30,- MOTOR_C.Target*1000,5,12);
			else                 	OLED_ShowString(15,30,"+"),
														OLED_ShowNumber(20,30,  MOTOR_C.Target*1000,5,12); 
				
			if( MOTOR_C.Encoder<0)OLED_ShowString(60,30,"-"),
														OLED_ShowNumber(75,30,-MOTOR_C.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,30,"+"),
														OLED_ShowNumber(75,30, MOTOR_C.Encoder*1000,5,12);
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
		 {
			 //阿克曼、差速、履带车显示电机A的状态	
			 OLED_ShowString(0,20,"L:");
			 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
			 else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12); 
			
			 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
			 //阿克曼、差速、履带车显示电机B的状态
			 OLED_ShowString(0,30,"R:");
			 if( MOTOR_B.Target<0)	OLED_ShowString(15,30,"-"),
															OLED_ShowNumber(20,30,-MOTOR_B.Target*1000,5,12);
			 else                 	OLED_ShowString(15,30,"+"),
															OLED_ShowNumber(20,30,  MOTOR_B.Target*1000,5,12); 
				
			 if( MOTOR_B.Encoder<0)	OLED_ShowString(60,30,"-"),
															OLED_ShowNumber(75,30,-MOTOR_B.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,30,"+"),
															OLED_ShowNumber(75,30, MOTOR_B.Encoder*1000,5,12);
//       OLED_ShowString(55,20,"AX");
//			 if( accel[0]<0)  OLED_ShowString(80,20,"-"),OLED_ShowNumber(90,20,-accel[0],5,12);
//			 else             OLED_ShowString(80,20,"+"),OLED_ShowNumber(90,20, accel[0],5,12);		//z轴陀螺仪加速度计数据	
//			 OLED_ShowString(55,30,"AY");
//			 if( accel[1]<0)  OLED_ShowString(80,30,"-"),OLED_ShowNumber(90,30,-accel[1],5,12);
//			 else             OLED_ShowString(80,30,"+"),OLED_ShowNumber(90,30, accel[1],5,12);		//z轴陀螺仪加速度计数据	
		 }
		 //显示屏第3、4行显示内容//
		 
		 
		 //显示屏第5行显示内容
			if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
			{
				//麦轮小车显示电机D的状态
				OLED_ShowString(0,40,"D");
				if( MOTOR_D.Target<0)	OLED_ShowString(15,40,"-"),
															OLED_ShowNumber(20,40,- MOTOR_D.Target*1000,5,12);
				else                 	OLED_ShowString(15,40,"+"),
															OLED_ShowNumber(20,40,  MOTOR_D.Target*1000,5,12); 			
				if( MOTOR_D.Encoder<0)	OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(75,40,-MOTOR_D.Encoder*1000,5,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(75,40, MOTOR_D.Encoder*1000,5,12);
			}
			else if(Car_Mode==Omni_Car)
			{
				//全向轮小车显示Z轴速度(放大1000倍) 单位rad/s
				OLED_ShowString(0,40,"MOVE_Z"); 			
				if(Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
																						OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
				else                              	OLED_ShowString(60,40,"+"),
																						OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
			}
			else if(Car_Mode==Akm_Car)
			{
				//阿克曼小车显示舵机的PWM的数值
				OLED_ShowString(00,40,"SERVO:");
				if( Servo<0)		      OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(80,40,-Servo,4,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(80,40, Servo,4,12); 
			}
			else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			{
			 //差速小车显示左右电机的PWM的数值
															 OLED_ShowString(00,40,"MA");
			 if( MOTOR_A.Motor_Pwm<0)OLED_ShowString(20,40,"-"),
															 OLED_ShowNumber(30,40,-MOTOR_A.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(20,40,"+"),
															 OLED_ShowNumber(30,40, MOTOR_A.Motor_Pwm,4,12); 
															 OLED_ShowString(60,40,"MB");
			 if(MOTOR_B.Motor_Pwm<0) OLED_ShowString(80,40,"-"),
															 OLED_ShowNumber(90,40,-MOTOR_B.Motor_Pwm,4,12);
			 else                 	 OLED_ShowString(80,40,"+"),
															 OLED_ShowNumber(90,40, MOTOR_B.Motor_Pwm,4,12);
			}
			//显示屏第5行显示内容
			
			
			//显示屏第6行显示内容
//			OLED_ShowString(0,50,"Temp: ");//显示温度
//			OLED_ShowNumber(50,50, MPU_Get_Temperature(),4,12); 
			if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");//显示控制模式
			else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
			else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
			else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
			else if (Usart_ON_Flag==1) OLED_ShowString(0,50,"USART");
			else                       OLED_ShowString(0,50,"ROS  ");
			
			if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  //电机使能开关
			else                      OLED_ShowString(45,50,"OFF"); 
			
																OLED_ShowString(88,50,".");
																OLED_ShowString(110,50,"V");
																OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
																OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
			if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
		}
	 
		/*自检相关*/
		//显示自检确认界面
		if(Check==1&&!Checking&&Checked==0) 
		{
        OLED_ShowCheckConfirming();
		}
		//显示自检进行界面
		if(Check==1&&Checking&&Checked==0) 
		{	
        OLED_ShowChecking();
		}	
		//显示自检结果
		if(Check==1&&Checking&&Checked==1) 
		{
			//OLED_ShowString(0,30,"MPeB:");OLED_ShowNumber(40,30, MPU9250SensorCountB,3,12); 
			//OLED_ShowString(0,40,"SEeB:");OLED_ShowNumber(40,40, EncoderB_Count,3,12); 
			
      OLED_ShowCheckResult();
		}
		/*自检相关*/
		
		//=============刷新屏幕=======================//
		if(Check==0)OLED_Refresh_Gram();
    else {if(count>5)OLED_Refresh_Gram(),count=0;}

	  		
	}

/**************************************************************************
函数功能：OLED屏显示自检确认界面
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void OLED_ShowCheckConfirming(void)
{
	//按下用户按键开始自检
  for(i=0;i<8;i++)	
	{
	 OLED_ShowCHinese(i*15, 5, i, CNSizeWidth, CNSizeHeight);
	}
	for(i=8;i<12;i++)	
	{
	 OLED_ShowCHinese((i-8)*15, 3, i, CNSizeWidth, CNSizeHeight);
	}
	
	//无操作*秒后退出
	for(i=12;i<15;i++)	
	{
	 OLED_ShowCHinese((i-12)*15, 1, i, CNSizeWidth, CNSizeHeight);
	}
	OLED_ShowNumber(48,45,(5-CheckCount/100),1,12);
	for(i=15;i<19;i++)	
	{
	 OLED_ShowCHinese((i-12)*15+10, 1, i, CNSizeWidth, CNSizeHeight);
	}
}
/**************************************************************************
函数功能：OLED屏显示自检进行界面
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void OLED_ShowChecking(void)
{
	//按下用户按键开始自检
  for(i=19;i<24;i++)	
	{
	 OLED_ShowCHinese((i-19)*15, 5, i, CNSizeWidth, CNSizeHeight);
	}
	OLED_ShowNumber(5,28,1,1,12);
	for(i=24;i<28;i++)	
	{
	 OLED_ShowCHinese((i-24)*15+18, 3, i, CNSizeWidth, CNSizeHeight);
	}
	OLED_ShowNumber(5,28,(200*CheckPhrase2-CheckCount)/100,2,12);
//	OLED_ShowString(5, 43,"Z");
//	for(i=28;i<32;i++)	
//	{
//	 OLED_ShowCHinese((i-28)*15+15, 1, i, CNSizeWidth, CNSizeHeight);
//	}
//	OLED_ShowString(80, 43,":");
//	if( gyro[2]<0)  OLED_ShowString(85,43,"-"),OLED_ShowNumber(100,43,-gyro[2]/10,4,12);
//	else            OLED_ShowString(85,43,"+"),OLED_ShowNumber(100,43, gyro[2]/10,4,12);		//z轴陀螺仪零点漂移数据	
}
/**************************************************************************
函数功能：OLED屏显示自检完成界面
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void OLED_ShowCheckResult(void)
{
	int row=5;
//  for(i=32;i<36;i++)	
//	{
//	 OLED_ShowCHinese((i-32)*15, 5, i, CNSizeWidth, CNSizeHeight);
//	}
//	for(i=36;i<40;i++)	
//	{
//	 OLED_ShowCHinese((i-36)*15, 3, i, CNSizeWidth, CNSizeHeight);
//	}
//	
//	OLED_ShowString(65, 26,":");
//	OLED_ShowNumber(10,42,ErrorCode,9,16);
//	
//	OLED_ShowString( 0,00,"A");OLED_ShowNumber(20,00,EncoderA_Count,5,12);
//  OLED_ShowString(60,00,"B");OLED_ShowNumber(80,00,EncoderB_Count,5,12);
//  OLED_ShowString( 0,10,"C");OLED_ShowNumber(20,10,EncoderC_Count,5,12);
//  OLED_ShowString(60,10,"D");OLED_ShowNumber(80,10,EncoderD_Count,5,12);
//	OLED_ShowString( 0,20,"MPU");OLED_ShowNumber(80,20,MPU9250SensorCount,5,12);
//	OLED_ShowString( 0,30,"MPUErr");OLED_ShowNumber(80,30,MPU9250ErrorCount,5,12);
	
	if(ErrorCode&1)//电压过低
	{
	  for(i=40;i<44;i++)	
		{
		 OLED_ShowCHinese((i-40)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<1)//使能开关关闭，电机使能开关、急停开关
	{
	  for(i=44;i<50;i++)	
		{
		 OLED_ShowCHinese((i-44)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}

	
	if(ErrorCode&1<<3)//A驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<4)//A驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<5)//B驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<6)//B驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<7)//C驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<8)//C驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<9)//D驱动损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<10)//D驱动线反接
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<11)//电源不稳定
	{
	  for(i=64;i<69;i++)	
		{
		 OLED_ShowCHinese((i-64)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<12)//温度过高
	{
	  for(i=69;i<73;i++)	
		{
		 OLED_ShowCHinese((i-69)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<14||ErrorCode&1<<15)//电机线接错
	{
	  for(i=73;i<78;i++)	
		{
		 OLED_ShowCHinese((i-73)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<16)//A编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<17)//B编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<18)//C编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<19)//D编码器损坏
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<13)//MPU9250损坏 优先级较低，放在后面
	{
	  for(i=57;i<59;i++)	
		{
		 OLED_ShowCHinese((i-57)*15+80, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;	
		     if(row<1)OLED_ShowString( 20,44,"MPU9250");
		else if(row<3)OLED_ShowString( 20,27,"MPU9250");
		else if(row<5)OLED_ShowString( 20,10,"MPU9250");
	}
	
	if(ErrorCode==0)//错误代码为0，小车正常
	{
	  for(i=78;i<82;i++)	
		{
		 OLED_ShowCHinese((i-78)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;	
	}
	
	if(row<5)OLED_ShowNumber(0, 7,1,1,16),OLED_ShowString( 10,10,".");
	if(row<3)OLED_ShowNumber(0,24,2,1,16),OLED_ShowString( 10,27,".");
	if(row<1)OLED_ShowNumber(0,41,3,1,16),OLED_ShowString( 10,44,".");

}
/**************************************************************************
函数功能：向APP发送数据
入口参数：无
返回  值：无
作    者：WHEELTEC
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	 Voltage_Show=(Voltage*1000-10000)/27;//电池电压简单处理
	
	 if(Voltage_Show>100)Voltage_Show=100;   //对电压数据进行处理
	
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	 //对编码器数据就行数据处理便于图形化
	 Right_Figure=MOTOR_B.Encoder*100;  if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 flag_show=!flag_show;//错频处理，交替打印APP数据和显示波形
	 if(PID_Send==1)//发送PID参数
	  {
		printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);//打印到APP上面
		PID_Send=0;	
	  }	
	 else	if(flag_show==0)// 
	 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]); //打印到APP上面
	 else
	 printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);//打印到APP上面 显示波形

}



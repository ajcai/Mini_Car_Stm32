/***********************************************
公司：东莞市微宏智能科技有限公司
品牌：WHEELTEC
官网：wheeltec.net
淘宝店铺：shop114407458.taobao.com 
版本V1.1
修改时间：2020-07-04
All rights reserved
***********************************************/
#include "balance.h"
int Time_count=0;            //计时变量
int robot_mode_check_flag=0; //机器人模式是否出错检测标志位
int EncoderA_Count=0, EncoderB_Count=0, EncoderC_Count=0, EncoderD_Count=0;                               //联合陀螺仪数据检测编码器与驱动
int MPU9250ErrorCount, MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;//联合陀螺仪数据检测编码器与驱动
Encoder OriginalEncoder;     //编码器原始数据
/**************************************************************************
函数功能：对接收到数据进行处理
入口参数：X和Y Z轴方向的运动速度
返回  值：无
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5;
	
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //对输入速度进行平滑处理		
			Vx=smooth_control.VX;     //获取平滑处理后的数据
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
	  //麦克纳姆轮小车
	  if (Car_Mode==Mec_Car) 
    {
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); //速度限幅
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); //速度限幅
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); //速度限幅
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); //速度限幅
		} 
		
		//全向轮小车
		else if (Car_Mode==Omni_Car) 
		{
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); //速度限幅
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); //速度限幅
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); //速度限幅
			MOTOR_D.Target=0;			
		}
		
		//阿克曼小车
		else if (Car_Mode==Akm_Car) 
		{
			int K=1000;
			float Ratio=1, Angle;
			
			Angle=Vz;
			Angle=target_limit_float(Angle,-0.35f,0.35f);//舵机角度限幅
			if(Angle<0)Ratio=1.054;//舵机左右两边对称化处理
			else if(Angle>0)Ratio=0.838;
			else Ratio=0;
			
			MOTOR_A.Target   = Vx*(1-Wheel_spacing*tan(Angle)/2/Axle_spacing);//A电机目标
			MOTOR_B.Target   = Vx*(1+Wheel_spacing*tan(Angle)/2/Axle_spacing);//B电机目标
			Servo=(SERVO_INIT-Angle*K*Ratio); //舵机目标
			
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); //电机限幅
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); //电机限幅
			MOTOR_C.Target=0;
			MOTOR_D.Target=0;
			Servo=target_limit_int(Servo,900,2000);		
		}
		
		//差速小车
		else if (Car_Mode==Diff_Car) 
		{
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //计算出右轮的目标速度
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); //速度限幅
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); //速度限幅	
			MOTOR_C.Target=0;
			MOTOR_D.Target=0;
		}
		
		//四驱车
		else if(Car_Mode==FourWheel_Car) 
		{	
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出左轮的目标速度
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //计算出右轮的目标速度
					
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); //速度限幅
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); //速度限幅
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); //速度限幅
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); //速度限幅		
		}
		
		//履带车
		else if (Car_Mode==Tank_Car) 
		{
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //计算出左轮的目标速度
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //计算出右轮的目标速度
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); //速度限幅
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); //速度限幅
			MOTOR_C.Target=0;
			MOTOR_D.Target=0;
		}
}
/**************************************************************************
函数功能：核心控制相关
入口参数：
返回  值： 
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();
	  //Check=1,Checking=1,Checked=1,CheckCount=(1+200*(CheckPhrase2));
    while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); //此任务以100Hz的频率运行（10ms控制一次）
			if(Time_count<3000)Time_count++;
			Get_Velocity_Form_Encoder();   //获取编码器数据
			if(Check==0)
			{
				if(APP_ON_Flag)           Get_RC();                           //APP遥控
				else if(Remote_ON_Flag)   Remote_Control();                   //航模遥控
				else if(PS2_ON_Flag)      PS2_control();                      //PS2手柄控制
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);  //CAN、串口1、串口3(ROS)控制
				Key();                                                        //按键修改陀螺仪零点
				
				if(Turn_Off(Voltage)==0)               //===如果电池电压不存在异常
				 { 			 	
					MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===速度闭环控制计算电机A最终PWM
					MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===速度闭环控制计算电机B最终PWM
					MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===速度闭环控制计算电机C最终PWM
					MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===速度闭环控制计算电机D最终PWM
					//Limit_Pwm(5500);                     //===PWM限幅 最高是7200，考虑到安全，做了限幅，可以在完全调试好之后
					 
					switch(Car_Mode)
					{
						case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //麦克纳姆轮小车
						case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //全向轮小车
						case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //阿克曼小车
						case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //两轮差速小车
						case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //四驱车 
						case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //履带车
					}
				 }
				 else	Set_Pwm(0,0,0,0,0);    //===赋值给PWM寄存器 
			 }	
     else CheckTask();			
		 }  
}

/**************************************************************************
函数功能：自检任务
入口参数：无
返回  值：无
**************************************************************************/
int CheckTask(void)
{
	 static int A,B,C,D; //控制正转实际反转0001 控制反转实际正转0010 控制正转实际停转0100 控制反转实际停转1000 这里定义PWM给正为正转
	 static float MaxVoltage=0, MinVoltage=20, Temperature, LastTemperature=3000, TemperatureBias; //自检相关变量
	 static int WireWrong=0; //自检相关变量，单击线是否接错标志位
	 
	 if(Check)CheckCount++;  //自检确认倒计时
	 else CheckCount=0;
	
	 //确认成功，开始自检
	 if(Check&&Checking) 
	 {
		 int CheckPeriod=200, WaitPeriod=100; //每个阶段测2秒，等待1秒运动稳定后开始检测
		 
		 //电压波动检测
		 if(Voltage>MaxVoltage) MaxVoltage=Voltage;
		 if(Voltage<MinVoltage) MinVoltage=Voltage;
		 
		 //MPU9250检测
		 if(CheckCount<=(1+CheckPeriod*CheckPhrase2))
		 {
			 if(gyro[0]==0||gyro[1]==0||gyro[2]==0||accel[0]==0||accel[1]==0||accel[2]==0||MPU_Get_Temperature()==0)MPU9250ErrorCount++;
		 }
		 
		 //温度检测
		 TemperatureBias=MPU_Get_Temperature()-LastTemperature;
		 if(TemperatureBias<10&&TemperatureBias>-10)
		 {
			 Temperature=(MPU_Get_Temperature()+LastTemperature)/2;
			 LastTemperature=Temperature;
		 }
		 
		 //开环控制电机检测
		 if(0<CheckCount&&CheckCount<(CheckPeriod)) //测试A电机正转 
		 {
			 if(CheckCount==1)Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(2000, 0, 0, 0, 1500);
			 if(CheckCount>(0+CheckPeriod-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A<-3)A=A|1;    //电机反转		
         if(ZeroACount>90)			 A=A|1<<2; //电机停转
			 }			 
		 }
		 else if(CheckPeriod<CheckCount&&CheckCount<(CheckPeriod*(2))) //测试A电机反转
		 {
			 if(CheckCount==(1+CheckPeriod))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(-2000, 0, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*2-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A>3)A=A|1<<1; //电机正转 	
         if(ZeroACount>90)			A=A|1<<3; //电机停转				 
			 }		 
		 }
		 
		 else if(CheckPeriod*(2)<CheckCount&&CheckCount<(CheckPeriod*(3))) //测试B电机正转
		 {
			 if(CheckCount==(1+CheckPeriod*2))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*3-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B<-3)B=B|1;    //电机反转 
				 if(ZeroBCount>90)			 B=B|1<<2; //电机停转
			 }			 
		 }
		 else if(CheckPeriod*(3)<CheckCount&&CheckCount<(CheckPeriod*(4))) //测试B电机反转
		 {
			 if(CheckCount==(1+CheckPeriod*3))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, -2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*4-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B>3)B=B|1<<1; //电机正转
				 if(ZeroBCount>90)			B=B|1<<3; //电机停转
			 }			 
		 }
		 
		 else if(CheckPeriod*(4)<CheckCount&&CheckCount<(CheckPeriod*5)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*4))Set_Pwm(0,0,0,0,1500);    
			 Set_Pwm(0, 0, 2000, 0, 1500);//测试C电机正转
			 if(CheckCount>(CheckPeriod*5-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C<-3)C=C|1;    //电机反转	
         if(ZeroCCount>90)			 C=C|1<<2; //电机停转				 
			 }				 
		 }
		 else if(CheckPeriod*(5)<CheckCount&&CheckCount<(CheckPeriod*6)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*5))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, -2000, 0, 1500);//测试C电机反转
			 if(CheckCount>(CheckPeriod*6-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C>3)C=C|1<<1; //电机正转
				 if(ZeroCCount>90)			C=C|1<<3; //电机停转	
			 }				 
		 }
		 
		 else if(CheckPeriod*(6)<CheckCount&&CheckCount<(CheckPeriod*7)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*6))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, 2000, 1500);//测试D电机正转
			 if(CheckCount>(CheckPeriod*7-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D<-3)D=D|1;    //电机反转		
         if(ZeroDCount>90)			 D=D|1<<2; //电机停转					 
			 }	 
		 }
		 else if(CheckPeriod*(7)<CheckCount&&CheckCount<(CheckPeriod*8)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*7))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, -2000, 1500);//测试D电机反转
			 if(CheckCount>(CheckPeriod*8-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D>3)D=D|1<<1; //电机正转
				 if(ZeroDCount>90)			D=D|1<<3; //电机停转	
			 }				 
		 }
		 //开环控制电机检测		 
		 
		 //检测编码器A 四驱车同时检测AB
		 else if(CheckPeriod*(CheckPhrase1)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+1))) 
		 {			 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=-0.3, MOTOR_B.Target=0, MOTOR_C.Target=0, MOTOR_D.Target=0;
			 if(Car_Mode==FourWheel_Car)MOTOR_B.Target=-0.3;
			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   
			 if(Car_Mode==FourWheel_Car)
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);
			 else 
			 {
				 MOTOR_B.Motor_Pwm=0;   //===速度闭环控制计算电机B最终PWM
				 MOTOR_C.Motor_Pwm=0;   //===速度闭环控制计算电机C最终PWM
				 MOTOR_D.Motor_Pwm=0;   //===速度闭环控制计算电机D最终PWM
			 }
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //麦克纳姆轮小车
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //全向轮小车
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 1500 ); break; //阿克曼小车
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //两轮差速小车
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //四驱车 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //履带车
			 }
		 
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+1)-WaitPeriod))
			 {
				 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountA++;		
				 if(MOTOR_A.Encoder<0.02&&MOTOR_A.Encoder>-0.02)EncoderA_Count++;		
         if(Car_Mode==FourWheel_Car)		
				 {
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountB++;		
				   if(MOTOR_B.Encoder<0.02&&MOTOR_B.Encoder>-0.02)EncoderB_Count++;	
				 }					 
			 }				 
		 }
		 
		 //检测编码器B 四驱车同时检查CD,不检B
		 else if(CheckPeriod*(CheckPhrase1+1)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+2))) 
		 {					 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+1)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0.3, MOTOR_C.Target=0, MOTOR_D.Target=0;
			 if(Car_Mode==FourWheel_Car)MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=0.3, MOTOR_D.Target=0.3;
		 		 
			 if(Car_Mode==FourWheel_Car)
			 {
				 MOTOR_A.Motor_Pwm=0;   
				 MOTOR_B.Motor_Pwm=0;  
				 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target),   
				 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);  
			 }
			 else
			 {
				 MOTOR_A.Motor_Pwm=0;   
				 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   
				 MOTOR_C.Motor_Pwm=0;
				 MOTOR_D.Motor_Pwm=0;
			 }
			 
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //麦克纳姆轮小车
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //全向轮小车
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 1500 ); break; //阿克曼小车
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //两轮差速小车
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //四驱车 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //履带车
			 }
		 
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+2)-WaitPeriod))
			 {
				 if(Car_Mode==FourWheel_Car) 		
				 {
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountC++;		
				   if(MOTOR_C.Encoder<0.02&&MOTOR_C.Encoder>-0.02)EncoderC_Count++;	
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountD++;		
				   if(MOTOR_D.Encoder<0.02&&MOTOR_D.Encoder>-0.02)EncoderD_Count++;	
				 }
				 else
				 {
					 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountB++;		
				   if(MOTOR_B.Encoder<0.02&&MOTOR_B.Encoder>-0.02)EncoderB_Count++;	
				 }			 			 
			 }				 
		 }
		 
		 //检测编码器C
		 else if(CheckPeriod*(CheckPhrase1+2)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+3))&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car)) 
		 {				 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+2)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=-0.3, MOTOR_D.Target=0;
       		 
			 MOTOR_A.Motor_Pwm=0;   //===速度闭环控制计算电机A最终PWM
			 MOTOR_B.Motor_Pwm=0;   //===速度闭环控制计算电机B最终PWM
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===速度闭环控制计算电机C最终PWM
			 MOTOR_D.Motor_Pwm=0;   //===速度闭环控制计算电机D最终PWM
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //麦克纳姆轮小车
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //全向轮小车
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //阿克曼小车
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //两轮差速小车
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //四驱车 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //履带车
			 }
			 
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+3)-WaitPeriod))
			 {
				 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountC++;		
				 if(MOTOR_C.Encoder<0.02&&MOTOR_C.Encoder>-0.02)EncoderC_Count++;					 
			 }				 
		 }
		 
		 //检测编码器D
		 else if(CheckPeriod*(CheckPhrase1+3)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+4))&&(Car_Mode==Mec_Car)) 
		 {		 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+3)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=0, MOTOR_D.Target=0.3;
			 
			 MOTOR_A.Motor_Pwm=0;   //===速度闭环控制计算电机A最终PWM
			 MOTOR_B.Motor_Pwm=0;   //===速度闭环控制计算电机B最终PWM
			 MOTOR_C.Motor_Pwm=0;   //===速度闭环控制计算电机C最终PWM
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===速度闭环控制计算电机D最终PWM
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //麦克纳姆轮小车
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //全向轮小车
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //阿克曼小车
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //两轮差速小车
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //四驱车 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //履带车
			 }
			 
			 //9250Z轴速度值低次数统计(次数过高代表车没有被驱动)，编码器速度值低速度统计。
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+4)-WaitPeriod))
			 {
				 if(gyro[2]<400&&gyro[2]>-400)MPU9250SensorCountD++;	//麦轮D电机结构上可能带动小车转动的能力弱，检测宽容处理		
				 if(MOTOR_D.Encoder<0.02&&MOTOR_D.Encoder>-0.02)EncoderD_Count++;				 
			 }				 
		 }
		 
		 //检测电机线是否接错 
		 else if((CheckPeriod*(CheckPhrase2-2))<CheckCount && CheckCount<(CheckPeriod*(CheckPhrase2-1))&&Car_Mode==Mec_Car) //麦轮车测试两次电机线
		 {			 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase2-2)))Set_Pwm(0,0,0,0,1500);
			 else if(Car_Mode==Mec_Car)
			 {
				 Drive_Motor(0, 0.3, 0);
			 }
			 			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===速度闭环控制计算电机A最终PWM
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===速度闭环控制计算电机B最终PWM
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===速度闭环控制计算电机C最终PWM
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===速度闭环控制计算电机D最终PWM
			 Limit_Pwm(2000);

			 Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    );  //麦克纳姆轮小车
		 
			 if(CheckCount>(CheckPeriod*(CheckPhrase2-1)-WaitPeriod))
			 {
				 static int ZeroACount=0, ZeroBCount=0, ZeroCCount=0, ZeroDCount=0;
				 static int WireWrongCount=0;
				 
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 
				 if(gyro[2]>64000||gyro[2]<-64000)gyro[2]=501;

				 if((gyro[2]>500||gyro[2]<-500)) WireWrongCount++;
				 if(WireWrongCount>80)           WireWrong=1;//ErrorCode=ErrorCode|1<<15; //电机线接错，错误代码录入	 
			 }				 
		 }
		 else if((CheckPeriod*(CheckPhrase2-1))<CheckCount && CheckCount<(CheckPeriod*(CheckPhrase2))) 
		 {			 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase2-1)))Set_Pwm(0,0,0,0,1500);
			 if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			 {
				 Drive_Motor(0.15, 0, 0.5);
			 }
			 else if(Car_Mode==FourWheel_Car)
			 {
				 Drive_Motor(0.3,   0, PI/2);
			 }
			 else if(Car_Mode==Omni_Car)
			 {
				 Drive_Motor(0.3,   0, PI/2);
			 }
			 else if(Car_Mode==Mec_Car)
			 {
				 Drive_Motor(0.3,   0, 0);
			 }
			 			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===速度闭环控制计算电机A最终PWM
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===速度闭环控制计算电机B最终PWM
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===速度闭环控制计算电机C最终PWM
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===速度闭环控制计算电机D最终PWM
			 Limit_Pwm(2000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //麦克纳姆轮小车
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //全向轮小车
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //阿克曼小车
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //两轮差速小车
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //四驱车 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //履带车
			 }
		 
			 if(CheckCount>(CheckPeriod*(CheckPhrase2)-WaitPeriod))
			 {
				 static int ZeroACount=0, ZeroBCount=0, ZeroCCount=0, ZeroDCount=0;
				 static int WireWrongCount=0;
				 
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 
				 if(CheckCount==(CheckPeriod*(CheckPhrase2)-2))
				 {
					 //单独驱动时无编码器数据，整体运动时编码器有数据，说明驱动损坏
					 if((A&12)&&ZeroACount<30)ErrorCode=ErrorCode|1<<3; 
					 if((B&12)&&ZeroBCount<30)ErrorCode=ErrorCode|1<<5;
					 if((C&12)&&ZeroCCount<30)ErrorCode=ErrorCode|1<<7;
					 if((D&12)&&ZeroDCount<30)ErrorCode=ErrorCode|1<<9;
				 }
				 
				 //驱动、编码器不正常则确认电机线接错
				 if(gyro[2]>64000||gyro[2]<-64000)gyro[2]=501;
				 if(Car_Mode==Mec_Car)
				 {
				   if((gyro[2]>500||gyro[2]<-500))   WireWrongCount++;
					 if(WireWrongCount>80)             WireWrong=1;//ErrorCode=ErrorCode|1<<15; //电机线接错，错误代码录入
				 }
				 else
				 {
					 if((gyro[2]<100))                 WireWrong=1;//ErrorCode=ErrorCode|1<<14; //电机线接错，错误代码录入
				 }			 
			 }				 
		 }
		 
		 //检测电机线是否接错

		 //统计错误代码 需要用户先确认型号是否选对以及编码器是否接正确，该教程还在做
		 else if(CheckCount==(1+CheckPeriod*(CheckPhrase2)))
		 {			 		 
			 if(MPU9250ErrorCount>100*CheckPhrase2/2)     ErrorCode=ErrorCode|1<<13; //MPU9250损坏
			 if(Temperature>7000)                         ErrorCode=ErrorCode|1<<12; //温度过高
			 if((MaxVoltage-MinVoltage)>5)                ErrorCode=ErrorCode|1<<11; //电压波动过大，电源不稳定
			 if(Voltage<10)                               ErrorCode=ErrorCode|1;     //电压过低
			 if(EN==0)                                    ErrorCode=ErrorCode|1<<1;  //开关处于关闭状态，电源开关/急停开关
			 if(A==16&&B==16)                             ErrorCode=ErrorCode|1<<2;  //小车型号选错，右上角电位器调型号
			 
			 switch(A&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<3; break; //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<3; break; //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<4; break; //驱动线反接 0011											
				default:                                          break;
			 }
			 switch(B&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<5; break; //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<5; break; //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<6; break; //驱动线反接 0011											
				default:                                          break;
			 }
			 switch(C&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<7; break; //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<7; break; //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<8; break; //驱动线反接 0011											
				default:                                          break;
			 }
			 switch(D&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<9; break; //驱动损坏   0001
				case 2:                 ErrorCode=ErrorCode|1<<9; break; //驱动损坏   0010
				case 3:                 ErrorCode=ErrorCode|1<<10;break; //驱动线反接 0011											
				default:                                          break;
			 }
			 
			 //四驱车驱动单个轮子，无法使车子移动，导致陀螺仪无数据
			 if(MPU9250SensorCountA>=80){if(EncoderA_Count>80)                          ErrorCode=ErrorCode|1<<3; } //驱动A损坏
			 else                       {if(EncoderA_Count>80&&(!(ErrorCode&1<<3)))     ErrorCode=ErrorCode|1<<16;} //编码器A损坏 
			 if(MPU9250SensorCountB>=80){if(EncoderB_Count>80)                          ErrorCode=ErrorCode|1<<5; } //驱动B损坏
			 else                       {if(EncoderB_Count>80&&(!(ErrorCode&1<<5)))     ErrorCode=ErrorCode|1<<17;} //编码器B损坏 
			 if(MPU9250SensorCountC>=80){if(EncoderC_Count>80)                          ErrorCode=ErrorCode|1<<7; } //驱动C损坏
			 else                       {if(EncoderC_Count>80&&(!(ErrorCode&1<<7)))     ErrorCode=ErrorCode|1<<18;} //编码器C损坏 
			 if(MPU9250SensorCountD>=90){if(EncoderD_Count>80)                          ErrorCode=ErrorCode|1<<9; } //驱动D损坏
			 else                       {if(EncoderD_Count>80&&(!(ErrorCode&1<<9)))     ErrorCode=ErrorCode|1<<19;} //编码器D损坏 麦轮D电机结构上可能带动小车转动的能力弱，检测宽容处理	
			 //                          0B 1111 0000 0111 1111 1000
			 if(WireWrong==1&&(ErrorCode&0xf07f8)==0)ErrorCode=ErrorCode|1<<14;
			 OLED_Clear();
			 Checked=1;
	   }
		 //统计错误代码
			
		 //检测完成，电机置0
		 if(CheckCount>=(1+CheckPeriod*(CheckPhrase2)))
		 {
		  Set_Pwm(0,0,0,0,1500);
		  MOTOR_A.Target=0,   MOTOR_B.Target=0,   MOTOR_C.Target=0,   MOTOR_D.Target=0;
		  MOTOR_A.Motor_Pwm=0,MOTOR_B.Motor_Pwm=0,MOTOR_C.Motor_Pwm=0,MOTOR_D.Motor_Pwm=0;
		  MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===速度闭环控制计算电机A最终PWM
		  MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===速度闭环控制计算电机B最终PWM
		  MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===速度闭环控制计算电机C最终PWM
		  MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===速度闭环控制计算电机D最终PWM		
		 }
	 }
	 return 0;
}
/**************************************************************************
函数功能：赋值给PWM寄存器
入口参数：PWM
返回  值：无
**************************************************************************/
void Set_Pwm(int motor_a,int motor_b,int motor_c,int motor_d,int servo)
{
  if(motor_a<0)		AIN2=0,		AIN1=1;
	else				AIN2=1,		AIN1=0;
	PWMA=abs(motor_a);
	
	if(motor_b<0)		BIN2=1,		BIN1=0;
	else 	            BIN2=0,		BIN1=1;
	PWMB=abs(motor_b);
	
	if(motor_c>0)		CIN2=0,		CIN1=1;
	else 	            CIN2=1,		CIN1=0;
	PWMC=abs(motor_c);
	
	if(motor_d>0)		DIN2=0,		DIN1=1;
	else 	            DIN2=1,		DIN1=0;
	PWMD=abs(motor_d);
	
	Servo_PWM =servo;
//	TIM_SetCompare4(TIM1, servo);
//	TIM_SetCompare3(TIM1, 1500);
//	TIM_SetCompare2(TIM1, 1500);
//	TIM_SetCompare1(TIM1, 1500);
}

/**************************************************************************
函数功能：限制PWM赋值 
入口参数：幅值
返回  值：无
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
函数功能：限幅函数，设定高低阈值
入口参数：幅值
返回  值：
**************************************************************************/
float target_limit_float(float insert,float low,float high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
int target_limit_int(int insert,int low,int high)
{
    if (insert < low)
        return low;
    else if (insert > high)
        return high;
    else
        return insert;	
}
/**************************************************************************
函数功能：异常关闭电机
入口参数：电压
返回  值：1：异常  0：正常
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)//电池电压过低关闭电机
			{	                                                
				temp=1;      
				PWMA=0;
				PWMB=0;		
				PWMC=0;	
				PWMD=0;					
      }
			else
			temp=0;
			return temp;			
}
/**************************************************************************
函数功能：绝对值函数
入口参数：long int
返回  值：unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
函数功能：增量PI控制器
入口参数：编码器测量值，目标速度
返回  值：电机PWM
根据增量式离散PID公式 
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)代表本次偏差 
e(k-1)代表上一次的偏差  以此类推 
pwm代表增量输出
在我们的速度控制闭环系统里面，只使用PI控制
pwm+=Kp[e（k）-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	//增量式PI控制器
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   // /
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;//保存上一次偏差 
	 return Pwm;  //增量输出
}
int Incremental_PI_B (float Encoder,float Target)
{  //增量式PI控制器
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //保存上一次偏差 
	 return Pwm; //增量输出
}
int Incremental_PI_C (float Encoder,float Target)
{  //增量式PI控制器
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //保存上一次偏差 
	 return Pwm; //增量输出
}
int Incremental_PI_D (float Encoder,float Target)
{  //增量式PI控制器
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //计算偏差
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //保存上一次偏差 
	 return Pwm; //增量输出
}
/**************************************************************************
函数功能：通过串口指令对机器人进行遥控
入口参数：无
返回  值：无
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
//	float TurnParam;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
	{
	 switch(Flag_Direction)   //方向控制
	 { 
			case 1:      Move_X=RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 2:      Move_X=RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 3:      Move_X=0;      		 Move_Y=-RC_Velocity;  Flag_Move=1; 	 break;
			case 4:      Move_X=-RC_Velocity;  	 Move_Y=-RC_Velocity;  Flag_Move=1;    break;
			case 5:      Move_X=-RC_Velocity;  	 Move_Y=0;             Flag_Move=1;    break;
			case 6:      Move_X=-RC_Velocity;  	 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 7:      Move_X=0;     	 		 Move_Y=RC_Velocity;   Flag_Move=1;    break;
			case 8:      Move_X=RC_Velocity; 	 Move_Y=RC_Velocity;   Flag_Move=1;    break; 
			default:     Move_X=0;               Move_Y=0;             Flag_Move=0;    break;
	 }
	 if(Flag_Move==0)		//如果无方向控制指令	 ，检查转向控制状态
	 {	
				if     (Flag_Left ==1)  Move_Z= PI/4*(RC_Velocity/400); //左自旋   
				else if(Flag_Right==1)  Move_Z=-PI/4*(RC_Velocity/400); //右自旋		
				else 		                Move_Z=0;                       //停止
	 }
	}	
	else
	{
	 switch(Flag_Direction)   //方向控制
	 { 
			case 1:      Move_X=+RC_Velocity;  	 Move_Z=0;         break;
			case 2:      Move_X=+RC_Velocity;  	 Move_Z=-PI/4;   	 break;
			case 3:      Move_X=0;      				 Move_Z=-PI/4;   	 break;	 
			case 4:      Move_X=-RC_Velocity;  	 Move_Z=-PI/4;     break;		 
			case 5:      Move_X=-RC_Velocity;  	 Move_Z=0;         break;	 
			case 6:      Move_X=-RC_Velocity;  	 Move_Z=+PI/4;     break;	 
			case 7:      Move_X=0;     	 			 	 Move_Z=+PI/4;     break;
			case 8:      Move_X=+RC_Velocity; 	 Move_Z=+PI/4;     break; 
			default:     Move_X=0;               Move_Z=0;         break;
	 }
	 if     (Flag_Left ==1)  Move_Z= PI/4; //左自旋   
	 else if(Flag_Right==1)  Move_Z=-PI/4; //右自旋		

	}
	
	//Z轴数据转化
	if(Car_Mode==Akm_Car)
	{
		Move_Z=Move_Z*4/9;
	}
	else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
	{
	  if(Move_X<0) Move_Z=-Move_Z;
		Move_Z=Move_Z*RC_Velocity/400;
	}		
//	else if(Car_Mode==FourWheel_Car)
//	{
//	  TurnParam=750/RC_Velocity;
//	  if(TurnParam>=2)TurnParam=2;
//	  if(TurnParam<=0.8)TurnParam=0.8;
//	  Move_Z=Move_Z*TurnParam;
//	}
		
	Move_X=Move_X/1000;       Move_Y=Move_Y/1000;         Move_Z=Move_Z;
	Drive_Motor(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析

}

/**************************************************************************
函数功能：通过PS2有线手柄对机器人进行遥控
入口参数：无
返回  值：无
**************************************************************************/
void PS2_control(void)
{
//	  float TurnParam;
   	int LX,LY,RY;
		int Yuzhi=20;
			
		LY=-(PS2_LX-128); //获取偏差 PS2坐标系为 Y前后 X左右
		LX=-(PS2_LY-128); //获取偏差
		RY=-(PS2_RX-128); //获取偏差
		if(LX>-Yuzhi&&LX<Yuzhi)LX=0; //设置小角度的死区
		if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //设置小角度的死区
		if(RY>-Yuzhi&&RY<Yuzhi)RY=0; //设置小角度的死区
		if(LX==0) Move_X=Move_X/1.2;
		if(RY==0) Move_Z=Move_Z/1.2;
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //速度控制 加速
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //速度控制 减速	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
		Move_X=LX*RC_Velocity/128; //前进量单位mm/s
		Move_Y=LY*RC_Velocity/128; //前进量单位mm/s
	  Move_Z=RY*(PI/4)/128;      //转向量
	
	  //Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*RC_Velocity/400;
		}	
		else if(Car_Mode==Akm_Car)
		{
			Move_Z=Move_Z*4/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z;
			Move_Z=Move_Z*RC_Velocity/400;
		}	
//		else if(Car_Mode==FourWheel_Car)
//		{
//			TurnParam=750/RC_Velocity;
//			if(TurnParam>=2)TurnParam=2;
//			if(TurnParam<=0.8)TurnParam=0.8;
//			Move_Z=Move_Z*TurnParam;
//		}
		 
		Move_X=Move_X/1000;        //单位转换 mm/s->m/ss
		Move_Y=Move_Y/1000;        //单位转换 mm/s->m/ss
		Move_Z=Move_Z;
		
		Drive_Motor(Move_X,Move_Y,Move_Z);//得到控制目标值，进行运动学分析			 			
} 

/**************************************************************************
函数功能：通过航模遥控对机器人进行遥控
入口参数：无
返回  值：无
**************************************************************************/
void Remote_Control(void)
{
//	  float TurnParam;
    static u8 thrice=100;
    int Yuzhi=100;

    int LX,LY,RY,RX,Remote_RCvelocity; 
	  static float Target_LX,Target_LY,Target_RX,Target_RY;
		Remoter_Ch1=target_limit_int(Remoter_Ch1,1000,2000);
		Remoter_Ch2=target_limit_int(Remoter_Ch2,1000,2000);
		Remoter_Ch3=target_limit_int(Remoter_Ch3,1000,2000);
		Remoter_Ch4=target_limit_int(Remoter_Ch4,1000,2000);

    LX=Remoter_Ch2-1500;
    LY=Remoter_Ch4-1500;
	RX=Remoter_Ch3-1500;
    RY=Remoter_Ch1-1500;//自转

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
	if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
		
		if(LX==0) Target_LX=Target_LX/1.2;
		if(LY==0) Target_LY=Target_LY/1.2;
		if(RX==0) Target_RX=Target_RX/1.2;
		if(RY==0) Target_RY=Target_RY/1.2;
		
		Remote_RCvelocity=RC_Velocity+RX;
		
    Move_X= LX*Remote_RCvelocity/500; //前进量单位mm/s
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/4)/500;      //转向量
			 
		//Z轴数据转化
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Move_Z=Move_Z*Remote_RCvelocity/400;
		}	
		else if(Car_Mode==Akm_Car)
		{
			Move_Z=Move_Z*4/9;
		}
		else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car||Car_Mode==FourWheel_Car)
		{
			if(Move_X<0) Move_Z=-Move_Z;
			Move_Z=Move_Z*Remote_RCvelocity/400;
		}
//		else if(Car_Mode==FourWheel_Car)
//		{
//			TurnParam=750/RC_Velocity;
//			if(TurnParam>=2)TurnParam=2;
//			if(TurnParam<=0.8)TurnParam=0.8;
//			Move_Z=Move_Z*TurnParam;
//		}
		
		Move_X=Move_X/1000;        //单位转换 mm/s->m/ss
    Move_Y=Move_Y/1000;        //单位转换 mm/s->m/ss
		Move_Z=Move_Z;
		
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;//用于屏蔽航模遥控刚连接上的时候的干扰信息
				
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
函数功能：按键即时更新陀螺仪零点
入口参数：无
返回  值：无
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double_MPU6050(50); 
	if(tmp==2)memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));//双击更新陀螺仪零点
}
/**************************************************************************
函数功能：读取模式并采集编码器
入口参数：无
返回  值：无
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; //用于获取编码器的原始数据
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

		//这里采集回来的是编码器的原始数据，需要经过转换后才可以使用
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //麦克纳姆轮小车
			case Omni_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //全向轮小车
			case Akm_Car:       Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //阿克曼小车
			case Diff_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //两轮差速小车
			case FourWheel_Car: Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //四驱车 
			case Tank_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //履带车
		}
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  //编码器的数据转换
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; //编码器的数据转换
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  //编码器的数据转换
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; //编码器的数据转换
}
/**************************************************************************
函数功能：将机器人的目标速度做平滑控制处理
入口参数：机器人三轴目标速度
返回  值：无
**************************************************************************/
void Smooth_control(float vx,float vy,float vz)
{
	float step=0.01;

	if	(vx>0) 			smooth_control.VX+=step;
	else if(vx<0)		smooth_control.VX-=step;
	else if(vx==0)	smooth_control.VX=smooth_control.VX*0.9;
	
	if	(vy>0) 			smooth_control.VY+=step;
	else if(vy<0)		smooth_control.VY-=step;
	else if(vy==0)	smooth_control.VY=smooth_control.VY*0.9;
	
	if	(vz>0) 			smooth_control.VZ+=step;
	else if(vz<0)		smooth_control.VZ-=step;
	else if(vz==0)	smooth_control.VZ=smooth_control.VZ*0.9;
	
	smooth_control.VX=target_limit_float(smooth_control.VX,-float_abs(vx),float_abs(vx));
	smooth_control.VY=target_limit_float(smooth_control.VY,-float_abs(vy),float_abs(vy));
	smooth_control.VZ=target_limit_float(smooth_control.VZ,-float_abs(vz),float_abs(vz));
}
/**************************************************************************
函数功能：浮点型数据取绝对值
入口参数：浮点数
返回  值：输入数的绝对值
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

/**************************************************************************
函数功能：防止电位器选错模式，导致初始化出错引发电机乱转
入口参数：无
返回  值：无
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  //如果连续十次接近满幅输出，判断为电机乱转，让电机失能	
}

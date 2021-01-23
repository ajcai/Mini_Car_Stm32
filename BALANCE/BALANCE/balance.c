/***********************************************
��˾����ݸ��΢�����ܿƼ����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
�汾V1.1
�޸�ʱ�䣺2020-07-04
All rights reserved
***********************************************/
#include "balance.h"
int Time_count=0;            //��ʱ����
int robot_mode_check_flag=0; //������ģʽ�Ƿ�������־λ
int EncoderA_Count=0, EncoderB_Count=0, EncoderC_Count=0, EncoderD_Count=0;                               //�������������ݼ�������������
int MPU9250ErrorCount, MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;//�������������ݼ�������������
Encoder OriginalEncoder;     //������ԭʼ����
/**************************************************************************
�������ܣ��Խ��յ����ݽ��д���
��ڲ�����X��Y Z�᷽����˶��ٶ�
����  ֵ����
**************************************************************************/
void Drive_Motor(float Vx,float Vy,float Vz)
{
		float amplitude=3.5;
	
	  if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		{
			Smooth_control(Vx,Vy,Vz); //�������ٶȽ���ƽ������		
			Vx=smooth_control.VX;     //��ȡƽ������������
			Vy=smooth_control.VY;
			Vz=smooth_control.VZ;
		}
		
	  //�����ķ��С��
	  if (Car_Mode==Mec_Car) 
    {
			MOTOR_A.Target   = +Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_B.Target   = -Vy+Vx-Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_C.Target   = +Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
			MOTOR_D.Target   = -Vy+Vx+Vz*(Axle_spacing+Wheel_spacing);
		
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_D.Target=target_limit_float(MOTOR_D.Target,-amplitude,amplitude); //�ٶ��޷�
		} 
		
		//ȫ����С��
		else if (Car_Mode==Omni_Car) 
		{
			MOTOR_A.Target   =   Vy + Omni_turn_radiaus*Vz;
			MOTOR_B.Target   =  -X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
			MOTOR_C.Target   =  +X_PARAMETER*Vx - Y_PARAMETER*Vy + Omni_turn_radiaus*Vz;
		
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_C.Target=target_limit_float(MOTOR_C.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_D.Target=0;			
		}
		
		//������С��
		else if (Car_Mode==Akm_Car) 
		{
			int K=1000;
			float Ratio=1, Angle;
			
			Angle=Vz;
			Angle=target_limit_float(Angle,-0.35f,0.35f);//����Ƕ��޷�
			if(Angle<0)Ratio=1.054;//����������߶Գƻ�����
			else if(Angle>0)Ratio=0.838;
			else Ratio=0;
			
			MOTOR_A.Target   = Vx*(1-Wheel_spacing*tan(Angle)/2/Axle_spacing);//A���Ŀ��
			MOTOR_B.Target   = Vx*(1+Wheel_spacing*tan(Angle)/2/Axle_spacing);//B���Ŀ��
			Servo=(SERVO_INIT-Angle*K*Ratio); //���Ŀ��
			
			MOTOR_A.Target=target_limit_float(MOTOR_A.Target,-amplitude,amplitude); //����޷�
			MOTOR_B.Target=target_limit_float(MOTOR_B.Target,-amplitude,amplitude); //����޷�
			MOTOR_C.Target=0;
			MOTOR_D.Target=0;
			Servo=target_limit_int(Servo,900,2000);		
		}
		
		//����С��
		else if (Car_Mode==Diff_Car) 
		{
			MOTOR_A.Target  = Vx - Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * Wheel_spacing / 2.0f; //��������ֵ�Ŀ���ٶ�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); //�ٶ��޷�
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); //�ٶ��޷�	
			MOTOR_C.Target=0;
			MOTOR_D.Target=0;
		}
		
		//������
		else if(Car_Mode==FourWheel_Car) 
		{	
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_B.Target  = Vx - Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_C.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
			MOTOR_D.Target  = Vx + Vz * (Wheel_spacing +  Axle_spacing) / 2.0f; //��������ֵ�Ŀ���ٶ�
					
			MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_C.Target=target_limit_float( MOTOR_C.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_D.Target=target_limit_float( MOTOR_D.Target,-amplitude,amplitude); //�ٶ��޷�		
		}
		
		//�Ĵ���
		else if (Car_Mode==Tank_Car) 
		{
			MOTOR_A.Target  = Vx - Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
		  MOTOR_B.Target =  Vx + Vz * (Wheel_spacing) / 2.0f;    //��������ֵ�Ŀ���ٶ�
		  MOTOR_A.Target=target_limit_float( MOTOR_A.Target,-amplitude,amplitude); //�ٶ��޷�
	    MOTOR_B.Target=target_limit_float( MOTOR_B.Target,-amplitude,amplitude); //�ٶ��޷�
			MOTOR_C.Target=0;
			MOTOR_D.Target=0;
		}
}
/**************************************************************************
�������ܣ����Ŀ������
��ڲ�����
����  ֵ�� 
**************************************************************************/
void Balance_task(void *pvParameters)
{ 
	  u32 lastWakeTime = getSysTickCnt();
	  //Check=1,Checking=1,Checked=1,CheckCount=(1+200*(CheckPhrase2));
    while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_100_HZ)); //��������100Hz��Ƶ�����У�10ms����һ�Σ�
			if(Time_count<3000)Time_count++;
			Get_Velocity_Form_Encoder();   //��ȡ����������
			if(Check==0)
			{
				if(APP_ON_Flag)           Get_RC();                           //APPң��
				else if(Remote_ON_Flag)   Remote_Control();                   //��ģң��
				else if(PS2_ON_Flag)      PS2_control();                      //PS2�ֱ�����
				else                      Drive_Motor(Move_X, Move_Y, Move_Z);  //CAN������1������3(ROS)����
				Key();                                                        //�����޸����������
				
				if(Turn_Off(Voltage)==0)               //===�����ص�ѹ�������쳣
				 { 			 	
					MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===�ٶȱջ����Ƽ�����A����PWM
					MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===�ٶȱջ����Ƽ�����B����PWM
					MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===�ٶȱջ����Ƽ�����C����PWM
					MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===�ٶȱջ����Ƽ�����D����PWM
					//Limit_Pwm(5500);                     //===PWM�޷� �����7200�����ǵ���ȫ�������޷�����������ȫ���Ժ�֮��
					 
					switch(Car_Mode)
					{
						case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�����ķ��С��
						case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //ȫ����С��
						case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //������С��
						case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //���ֲ���С��
						case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //������ 
						case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�Ĵ���
					}
				 }
				 else	Set_Pwm(0,0,0,0,0);    //===��ֵ��PWM�Ĵ��� 
			 }	
     else CheckTask();			
		 }  
}

/**************************************************************************
�������ܣ��Լ�����
��ڲ�������
����  ֵ����
**************************************************************************/
int CheckTask(void)
{
	 static int A,B,C,D; //������תʵ�ʷ�ת0001 ���Ʒ�תʵ����ת0010 ������תʵ��ͣת0100 ���Ʒ�תʵ��ͣת1000 ���ﶨ��PWM����Ϊ��ת
	 static float MaxVoltage=0, MinVoltage=20, Temperature, LastTemperature=3000, TemperatureBias; //�Լ���ر���
	 static int WireWrong=0; //�Լ���ر������������Ƿ�Ӵ��־λ
	 
	 if(Check)CheckCount++;  //�Լ�ȷ�ϵ���ʱ
	 else CheckCount=0;
	
	 //ȷ�ϳɹ�����ʼ�Լ�
	 if(Check&&Checking) 
	 {
		 int CheckPeriod=200, WaitPeriod=100; //ÿ���׶β�2�룬�ȴ�1���˶��ȶ���ʼ���
		 
		 //��ѹ�������
		 if(Voltage>MaxVoltage) MaxVoltage=Voltage;
		 if(Voltage<MinVoltage) MinVoltage=Voltage;
		 
		 //MPU9250���
		 if(CheckCount<=(1+CheckPeriod*CheckPhrase2))
		 {
			 if(gyro[0]==0||gyro[1]==0||gyro[2]==0||accel[0]==0||accel[1]==0||accel[2]==0||MPU_Get_Temperature()==0)MPU9250ErrorCount++;
		 }
		 
		 //�¶ȼ��
		 TemperatureBias=MPU_Get_Temperature()-LastTemperature;
		 if(TemperatureBias<10&&TemperatureBias>-10)
		 {
			 Temperature=(MPU_Get_Temperature()+LastTemperature)/2;
			 LastTemperature=Temperature;
		 }
		 
		 //�������Ƶ�����
		 if(0<CheckCount&&CheckCount<(CheckPeriod)) //����A�����ת 
		 {
			 if(CheckCount==1)Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(2000, 0, 0, 0, 1500);
			 if(CheckCount>(0+CheckPeriod-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A<-3)A=A|1;    //�����ת		
         if(ZeroACount>90)			 A=A|1<<2; //���ͣת
			 }			 
		 }
		 else if(CheckPeriod<CheckCount&&CheckCount<(CheckPeriod*(2))) //����A�����ת
		 {
			 if(CheckCount==(1+CheckPeriod))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(-2000, 0, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*2-WaitPeriod))
			 {
				 static int ZeroACount=0;
				 if(OriginalEncoder.A==0)ZeroACount++;
				 if(OriginalEncoder.A>3)A=A|1<<1; //�����ת 	
         if(ZeroACount>90)			A=A|1<<3; //���ͣת				 
			 }		 
		 }
		 
		 else if(CheckPeriod*(2)<CheckCount&&CheckCount<(CheckPeriod*(3))) //����B�����ת
		 {
			 if(CheckCount==(1+CheckPeriod*2))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*3-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B<-3)B=B|1;    //�����ת 
				 if(ZeroBCount>90)			 B=B|1<<2; //���ͣת
			 }			 
		 }
		 else if(CheckPeriod*(3)<CheckCount&&CheckCount<(CheckPeriod*(4))) //����B�����ת
		 {
			 if(CheckCount==(1+CheckPeriod*3))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, -2000, 0, 0, 1500);
			 if(CheckCount>(CheckPeriod*4-WaitPeriod))
			 {
				 static int ZeroBCount=0;
				 if(OriginalEncoder.B==0)ZeroBCount++;
				 if(OriginalEncoder.B>3)B=B|1<<1; //�����ת
				 if(ZeroBCount>90)			B=B|1<<3; //���ͣת
			 }			 
		 }
		 
		 else if(CheckPeriod*(4)<CheckCount&&CheckCount<(CheckPeriod*5)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*4))Set_Pwm(0,0,0,0,1500);    
			 Set_Pwm(0, 0, 2000, 0, 1500);//����C�����ת
			 if(CheckCount>(CheckPeriod*5-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C<-3)C=C|1;    //�����ת	
         if(ZeroCCount>90)			 C=C|1<<2; //���ͣת				 
			 }				 
		 }
		 else if(CheckPeriod*(5)<CheckCount&&CheckCount<(CheckPeriod*6)&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*5))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, -2000, 0, 1500);//����C�����ת
			 if(CheckCount>(CheckPeriod*6-WaitPeriod))
			 {
				 static int ZeroCCount=0;
				 if(OriginalEncoder.C==0)ZeroCCount++;
				 if(OriginalEncoder.C>3)C=C|1<<1; //�����ת
				 if(ZeroCCount>90)			C=C|1<<3; //���ͣת	
			 }				 
		 }
		 
		 else if(CheckPeriod*(6)<CheckCount&&CheckCount<(CheckPeriod*7)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*6))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, 2000, 1500);//����D�����ת
			 if(CheckCount>(CheckPeriod*7-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D<-3)D=D|1;    //�����ת		
         if(ZeroDCount>90)			 D=D|1<<2; //���ͣת					 
			 }	 
		 }
		 else if(CheckPeriod*(7)<CheckCount&&CheckCount<(CheckPeriod*8)&&(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)) 
		 {
			 if(CheckCount==(1+CheckPeriod*7))Set_Pwm(0,0,0,0,1500);
			 Set_Pwm(0, 0, 0, -2000, 1500);//����D�����ת
			 if(CheckCount>(CheckPeriod*8-WaitPeriod))
			 {
				 static int ZeroDCount=0;
				 if(OriginalEncoder.D==0)ZeroDCount++;
				 if(OriginalEncoder.D>3)D=D|1<<1; //�����ת
				 if(ZeroDCount>90)			D=D|1<<3; //���ͣת	
			 }				 
		 }
		 //�������Ƶ�����		 
		 
		 //��������A ������ͬʱ���AB
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
				 MOTOR_B.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����B����PWM
				 MOTOR_C.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����C����PWM
				 MOTOR_D.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����D����PWM
			 }
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�����ķ��С��
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //ȫ����С��
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 1500 ); break; //������С��
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //���ֲ���С��
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //������ 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�Ĵ���
			 }
		 
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
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
		 
		 //��������B ������ͬʱ���CD,����B
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
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�����ķ��С��
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //ȫ����С��
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 1500 ); break; //������С��
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //���ֲ���С��
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //������ 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�Ĵ���
			 }
		 
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
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
		 
		 //��������C
		 else if(CheckPeriod*(CheckPhrase1+2)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+3))&&(Car_Mode==Mec_Car||Car_Mode==Omni_Car)) 
		 {				 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+2)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=-0.3, MOTOR_D.Target=0;
       		 
			 MOTOR_A.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����A����PWM
			 MOTOR_B.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����B����PWM
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===�ٶȱջ����Ƽ�����C����PWM
			 MOTOR_D.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����D����PWM
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�����ķ��С��
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //ȫ����С��
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //������С��
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //���ֲ���С��
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //������ 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�Ĵ���
			 }
			 
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+3)-WaitPeriod))
			 {
				 if(gyro[2]<500&&gyro[2]>-500)MPU9250SensorCountC++;		
				 if(MOTOR_C.Encoder<0.02&&MOTOR_C.Encoder>-0.02)EncoderC_Count++;					 
			 }				 
		 }
		 
		 //��������D
		 else if(CheckPeriod*(CheckPhrase1+3)<CheckCount&&CheckCount<(CheckPeriod*(CheckPhrase1+4))&&(Car_Mode==Mec_Car)) 
		 {		 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase1+3)))Set_Pwm(0,0,0,0,1500);
			 MOTOR_A.Target=0, MOTOR_B.Target=0, MOTOR_C.Target=0, MOTOR_D.Target=0.3;
			 
			 MOTOR_A.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����A����PWM
			 MOTOR_B.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����B����PWM
			 MOTOR_C.Motor_Pwm=0;   //===�ٶȱջ����Ƽ�����C����PWM
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===�ٶȱջ����Ƽ�����D����PWM
			 Limit_Pwm(3000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�����ķ��С��
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //ȫ����С��
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //������С��
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //���ֲ���С��
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //������ 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�Ĵ���
			 }
			 
			 //9250Z���ٶ�ֵ�ʹ���ͳ��(�������ߴ���û�б�����)���������ٶ�ֵ���ٶ�ͳ�ơ�
			 if(CheckCount>(CheckPeriod*(CheckPhrase1+4)-WaitPeriod))
			 {
				 if(gyro[2]<400&&gyro[2]>-400)MPU9250SensorCountD++;	//����D����ṹ�Ͽ��ܴ���С��ת�����������������ݴ���		
				 if(MOTOR_D.Encoder<0.02&&MOTOR_D.Encoder>-0.02)EncoderD_Count++;				 
			 }				 
		 }
		 
		 //��������Ƿ�Ӵ� 
		 else if((CheckPeriod*(CheckPhrase2-2))<CheckCount && CheckCount<(CheckPeriod*(CheckPhrase2-1))&&Car_Mode==Mec_Car) //���ֳ��������ε����
		 {			 
			 if(CheckCount==(1+CheckPeriod*(CheckPhrase2-2)))Set_Pwm(0,0,0,0,1500);
			 else if(Car_Mode==Mec_Car)
			 {
				 Drive_Motor(0, 0.3, 0);
			 }
			 			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===�ٶȱջ����Ƽ�����A����PWM
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===�ٶȱջ����Ƽ�����B����PWM
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===�ٶȱջ����Ƽ�����C����PWM
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===�ٶȱջ����Ƽ�����D����PWM
			 Limit_Pwm(2000);

			 Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    );  //�����ķ��С��
		 
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
				 if(WireWrongCount>80)           WireWrong=1;//ErrorCode=ErrorCode|1<<15; //����߽Ӵ��������¼��	 
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
			 			 
			 MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===�ٶȱջ����Ƽ�����A����PWM
			 MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===�ٶȱջ����Ƽ�����B����PWM
			 MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===�ٶȱջ����Ƽ�����C����PWM
			 MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===�ٶȱջ����Ƽ�����D����PWM
			 Limit_Pwm(2000);
			 switch(Car_Mode)
			 {
				 case Mec_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�����ķ��С��
				 case Omni_Car:      Set_Pwm( MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //ȫ����С��
				 case Akm_Car:       Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, Servo); break; //������С��
				 case Diff_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //���ֲ���С��
				 case FourWheel_Car: Set_Pwm(-MOTOR_A.Motor_Pwm, -MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //������ 
				 case Tank_Car:      Set_Pwm(-MOTOR_A.Motor_Pwm,  MOTOR_B.Motor_Pwm,  MOTOR_C.Motor_Pwm, MOTOR_D.Motor_Pwm, 0    ); break; //�Ĵ���
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
					 //��������ʱ�ޱ��������ݣ������˶�ʱ�����������ݣ�˵��������
					 if((A&12)&&ZeroACount<30)ErrorCode=ErrorCode|1<<3; 
					 if((B&12)&&ZeroBCount<30)ErrorCode=ErrorCode|1<<5;
					 if((C&12)&&ZeroCCount<30)ErrorCode=ErrorCode|1<<7;
					 if((D&12)&&ZeroDCount<30)ErrorCode=ErrorCode|1<<9;
				 }
				 
				 //��������������������ȷ�ϵ���߽Ӵ�
				 if(gyro[2]>64000||gyro[2]<-64000)gyro[2]=501;
				 if(Car_Mode==Mec_Car)
				 {
				   if((gyro[2]>500||gyro[2]<-500))   WireWrongCount++;
					 if(WireWrongCount>80)             WireWrong=1;//ErrorCode=ErrorCode|1<<15; //����߽Ӵ��������¼��
				 }
				 else
				 {
					 if((gyro[2]<100))                 WireWrong=1;//ErrorCode=ErrorCode|1<<14; //����߽Ӵ��������¼��
				 }			 
			 }				 
		 }
		 
		 //��������Ƿ�Ӵ�

		 //ͳ�ƴ������ ��Ҫ�û���ȷ���ͺ��Ƿ�ѡ���Լ��������Ƿ����ȷ���ý̳̻�����
		 else if(CheckCount==(1+CheckPeriod*(CheckPhrase2)))
		 {			 		 
			 if(MPU9250ErrorCount>100*CheckPhrase2/2)     ErrorCode=ErrorCode|1<<13; //MPU9250��
			 if(Temperature>7000)                         ErrorCode=ErrorCode|1<<12; //�¶ȹ���
			 if((MaxVoltage-MinVoltage)>5)                ErrorCode=ErrorCode|1<<11; //��ѹ�������󣬵�Դ���ȶ�
			 if(Voltage<10)                               ErrorCode=ErrorCode|1;     //��ѹ����
			 if(EN==0)                                    ErrorCode=ErrorCode|1<<1;  //���ش��ڹر�״̬����Դ����/��ͣ����
			 if(A==16&&B==16)                             ErrorCode=ErrorCode|1<<2;  //С���ͺ�ѡ�����Ͻǵ�λ�����ͺ�
			 
			 switch(A&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<3; break; //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<3; break; //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<4; break; //�����߷��� 0011											
				default:                                          break;
			 }
			 switch(B&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<5; break; //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<5; break; //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<6; break; //�����߷��� 0011											
				default:                                          break;
			 }
			 switch(C&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<7; break; //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<7; break; //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<8; break; //�����߷��� 0011											
				default:                                          break;
			 }
			 switch(D&0X0F)
			 {
				case 1:                 ErrorCode=ErrorCode|1<<9; break; //������   0001
				case 2:                 ErrorCode=ErrorCode|1<<9; break; //������   0010
				case 3:                 ErrorCode=ErrorCode|1<<10;break; //�����߷��� 0011											
				default:                                          break;
			 }
			 
			 //�����������������ӣ��޷�ʹ�����ƶ�������������������
			 if(MPU9250SensorCountA>=80){if(EncoderA_Count>80)                          ErrorCode=ErrorCode|1<<3; } //����A��
			 else                       {if(EncoderA_Count>80&&(!(ErrorCode&1<<3)))     ErrorCode=ErrorCode|1<<16;} //������A�� 
			 if(MPU9250SensorCountB>=80){if(EncoderB_Count>80)                          ErrorCode=ErrorCode|1<<5; } //����B��
			 else                       {if(EncoderB_Count>80&&(!(ErrorCode&1<<5)))     ErrorCode=ErrorCode|1<<17;} //������B�� 
			 if(MPU9250SensorCountC>=80){if(EncoderC_Count>80)                          ErrorCode=ErrorCode|1<<7; } //����C��
			 else                       {if(EncoderC_Count>80&&(!(ErrorCode&1<<7)))     ErrorCode=ErrorCode|1<<18;} //������C�� 
			 if(MPU9250SensorCountD>=90){if(EncoderD_Count>80)                          ErrorCode=ErrorCode|1<<9; } //����D��
			 else                       {if(EncoderD_Count>80&&(!(ErrorCode&1<<9)))     ErrorCode=ErrorCode|1<<19;} //������D�� ����D����ṹ�Ͽ��ܴ���С��ת�����������������ݴ���	
			 //                          0B 1111 0000 0111 1111 1000
			 if(WireWrong==1&&(ErrorCode&0xf07f8)==0)ErrorCode=ErrorCode|1<<14;
			 OLED_Clear();
			 Checked=1;
	   }
		 //ͳ�ƴ������
			
		 //�����ɣ������0
		 if(CheckCount>=(1+CheckPeriod*(CheckPhrase2)))
		 {
		  Set_Pwm(0,0,0,0,1500);
		  MOTOR_A.Target=0,   MOTOR_B.Target=0,   MOTOR_C.Target=0,   MOTOR_D.Target=0;
		  MOTOR_A.Motor_Pwm=0,MOTOR_B.Motor_Pwm=0,MOTOR_C.Motor_Pwm=0,MOTOR_D.Motor_Pwm=0;
		  MOTOR_A.Motor_Pwm=Incremental_PI_A(MOTOR_A.Encoder, MOTOR_A.Target);   //===�ٶȱջ����Ƽ�����A����PWM
		  MOTOR_B.Motor_Pwm=Incremental_PI_B(MOTOR_B.Encoder, MOTOR_B.Target);   //===�ٶȱջ����Ƽ�����B����PWM
		  MOTOR_C.Motor_Pwm=Incremental_PI_C(MOTOR_C.Encoder, MOTOR_C.Target);   //===�ٶȱջ����Ƽ�����C����PWM
		  MOTOR_D.Motor_Pwm=Incremental_PI_D(MOTOR_D.Encoder, MOTOR_D.Target);   //===�ٶȱջ����Ƽ�����D����PWM		
		 }
	 }
	 return 0;
}
/**************************************************************************
�������ܣ���ֵ��PWM�Ĵ���
��ڲ�����PWM
����  ֵ����
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
�������ܣ�����PWM��ֵ 
��ڲ�������ֵ
����  ֵ����
**************************************************************************/
void Limit_Pwm(int amplitude)
{	
	    MOTOR_A.Motor_Pwm=target_limit_float(MOTOR_A.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_B.Motor_Pwm=target_limit_float(MOTOR_B.Motor_Pwm,-amplitude,amplitude);
		  MOTOR_C.Motor_Pwm=target_limit_float(MOTOR_C.Motor_Pwm,-amplitude,amplitude);
	    MOTOR_D.Motor_Pwm=target_limit_float(MOTOR_D.Motor_Pwm,-amplitude,amplitude);
}	    
/**************************************************************************
�������ܣ��޷��������趨�ߵ���ֵ
��ڲ�������ֵ
����  ֵ��
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
�������ܣ��쳣�رյ��
��ڲ�������ѹ
����  ֵ��1���쳣  0������
**************************************************************************/
u8 Turn_Off( int voltage)
{
	    u8 temp;
			if(voltage<10||EN==0||Flag_Stop==1)//��ص�ѹ���͹رյ��
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
�������ܣ�����ֵ����
��ڲ�����long int
����  ֵ��unsigned int
**************************************************************************/
u32 myabs(long int a)
{ 		   
	  u32 temp;
		if(a<0)  temp=-a;  
	  else temp=a;
	  return temp;
}
/**************************************************************************
�������ܣ�����PI������
��ڲ���������������ֵ��Ŀ���ٶ�
����  ֵ�����PWM
��������ʽ��ɢPID��ʽ 
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)+Kd[e(k)-2e(k-1)+e(k-2)]
e(k)������ƫ�� 
e(k-1)������һ�ε�ƫ��  �Դ����� 
pwm�����������
�����ǵ��ٶȿ��Ʊջ�ϵͳ���棬ֻʹ��PI����
pwm+=Kp[e��k��-e(k-1)]+Ki*e(k)
**************************************************************************/
int Incremental_PI_A (float Encoder,float Target)
{ 	//����ʽPI������
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   // /
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias;//������һ��ƫ�� 
	 return Pwm;  //�������
}
int Incremental_PI_B (float Encoder,float Target)
{  //����ʽPI������
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //������һ��ƫ�� 
	 return Pwm; //�������
}
int Incremental_PI_C (float Encoder,float Target)
{  //����ʽPI������
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //������һ��ƫ�� 
	 return Pwm; //�������
}
int Incremental_PI_D (float Encoder,float Target)
{  //����ʽPI������
	 static float Bias,Pwm,Last_bias;
	 Bias=Target-Encoder;                //����ƫ��
	 Pwm+=Velocity_KP*(Bias-Last_bias)+Velocity_KI*Bias;   //
	 if(Pwm>7200)Pwm=7200;
	 if(Pwm<-7200)Pwm=-7200;
	 Last_bias=Bias; //������һ��ƫ�� 
	 return Pwm; //�������
}
/**************************************************************************
�������ܣ�ͨ������ָ��Ի����˽���ң��
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_RC(void)
{
	u8 Flag_Move=1;
//	float TurnParam;
	if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
	{
	 switch(Flag_Direction)   //�������
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
	 if(Flag_Move==0)		//����޷������ָ��	 �����ת�����״̬
	 {	
				if     (Flag_Left ==1)  Move_Z= PI/4*(RC_Velocity/400); //������   
				else if(Flag_Right==1)  Move_Z=-PI/4*(RC_Velocity/400); //������		
				else 		                Move_Z=0;                       //ֹͣ
	 }
	}	
	else
	{
	 switch(Flag_Direction)   //�������
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
	 if     (Flag_Left ==1)  Move_Z= PI/4; //������   
	 else if(Flag_Right==1)  Move_Z=-PI/4; //������		

	}
	
	//Z������ת��
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
	Drive_Motor(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����

}

/**************************************************************************
�������ܣ�ͨ��PS2�����ֱ��Ի����˽���ң��
��ڲ�������
����  ֵ����
**************************************************************************/
void PS2_control(void)
{
//	  float TurnParam;
   	int LX,LY,RY;
		int Yuzhi=20;
			
		LY=-(PS2_LX-128); //��ȡƫ�� PS2����ϵΪ Yǰ�� X����
		LX=-(PS2_LY-128); //��ȡƫ��
		RY=-(PS2_RX-128); //��ȡƫ��
		if(LX>-Yuzhi&&LX<Yuzhi)LX=0; //����С�Ƕȵ�����
		if(LY>-Yuzhi&&LY<Yuzhi)LY=0; //����С�Ƕȵ�����
		if(RY>-Yuzhi&&RY<Yuzhi)RY=0; //����С�Ƕȵ�����
		if(LX==0) Move_X=Move_X/1.2;
		if(RY==0) Move_Z=Move_Z/1.2;
	
	  if (PS2_KEY==11)		RC_Velocity+=5;  //�ٶȿ��� ����
	  else if(PS2_KEY==9)	RC_Velocity-=5;  //�ٶȿ��� ����	
	
		if(RC_Velocity<0)   RC_Velocity=0;
	
		Move_X=LX*RC_Velocity/128; //ǰ������λmm/s
		Move_Y=LY*RC_Velocity/128; //ǰ������λmm/s
	  Move_Z=RY*(PI/4)/128;      //ת����
	
	  //Z������ת��
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
		 
		Move_X=Move_X/1000;        //��λת�� mm/s->m/ss
		Move_Y=Move_Y/1000;        //��λת�� mm/s->m/ss
		Move_Z=Move_Z;
		
		Drive_Motor(Move_X,Move_Y,Move_Z);//�õ�����Ŀ��ֵ�������˶�ѧ����			 			
} 

/**************************************************************************
�������ܣ�ͨ����ģң�ضԻ����˽���ң��
��ڲ�������
����  ֵ����
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
    RY=Remoter_Ch1-1500;//��ת

    if(LX>-Yuzhi&&LX<Yuzhi)LX=0;
    if(LY>-Yuzhi&&LY<Yuzhi)LY=0;
    if(RX>-Yuzhi&&RX<Yuzhi)RX=0;
	if(RY>-Yuzhi&&RY<Yuzhi)RY=0;
		
		if(LX==0) Target_LX=Target_LX/1.2;
		if(LY==0) Target_LY=Target_LY/1.2;
		if(RX==0) Target_RX=Target_RX/1.2;
		if(RY==0) Target_RY=Target_RY/1.2;
		
		Remote_RCvelocity=RC_Velocity+RX;
		
    Move_X= LX*Remote_RCvelocity/500; //ǰ������λmm/s
		Move_Y=-LY*Remote_RCvelocity/500;
		Move_Z=-RY*(PI/4)/500;      //ת����
			 
		//Z������ת��
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
		
		Move_X=Move_X/1000;        //��λת�� mm/s->m/ss
    Move_Y=Move_Y/1000;        //��λת�� mm/s->m/ss
		Move_Z=Move_Z;
		
    if(thrice>0) Move_X=0,Move_Z=0,thrice--;//�������κ�ģң�ظ������ϵ�ʱ��ĸ�����Ϣ
				
		Drive_Motor(Move_X,Move_Y,Move_Z);
}
/**************************************************************************
�������ܣ�������ʱ�������������
��ڲ�������
����  ֵ����
**************************************************************************/
void Key(void)
{	
	u8 tmp;
	tmp=click_N_Double_MPU6050(50); 
	if(tmp==2)memcpy(Deviation_gyro,Original_gyro,sizeof(gyro));//˫���������������
}
/**************************************************************************
�������ܣ���ȡģʽ���ɼ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void Get_Velocity_Form_Encoder(void)
{
		float Encoder_A_pr,Encoder_B_pr,Encoder_C_pr,Encoder_D_pr; //���ڻ�ȡ��������ԭʼ����
		OriginalEncoder.A=Read_Encoder(2);	
		OriginalEncoder.B=Read_Encoder(3);	
		OriginalEncoder.C=Read_Encoder(4);	
		OriginalEncoder.D=Read_Encoder(5);	

		//����ɼ��������Ǳ�������ԭʼ���ݣ���Ҫ����ת����ſ���ʹ��
		switch(Car_Mode)
		{
			case Mec_Car:       Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //�����ķ��С��
			case Omni_Car:      Encoder_A_pr= OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //ȫ����С��
			case Akm_Car:       Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //������С��
			case Diff_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //���ֲ���С��
			case FourWheel_Car: Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr=-OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //������ 
			case Tank_Car:      Encoder_A_pr=-OriginalEncoder.A; Encoder_B_pr= OriginalEncoder.B; Encoder_C_pr= OriginalEncoder.C;  Encoder_D_pr= OriginalEncoder.D; break; //�Ĵ���
		}
		MOTOR_A.Encoder= Encoder_A_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  //������������ת��
		MOTOR_B.Encoder= Encoder_B_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; //������������ת��
		MOTOR_C.Encoder= Encoder_C_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision;  //������������ת��
		MOTOR_D.Encoder= Encoder_D_pr*CONTROL_FREQUENCY*Wheel_perimeter/Encoder_precision; //������������ת��
}
/**************************************************************************
�������ܣ��������˵�Ŀ���ٶ���ƽ�����ƴ���
��ڲ���������������Ŀ���ٶ�
����  ֵ����
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
�������ܣ�����������ȡ����ֵ
��ڲ�����������
����  ֵ���������ľ���ֵ
**************************************************************************/
float float_abs(float insert)
{
	if(insert>=0) return insert;
	else return -insert;
}

/**************************************************************************
�������ܣ���ֹ��λ��ѡ��ģʽ�����³�ʼ���������������ת
��ڲ�������
����  ֵ����
**************************************************************************/
void robot_mode_check(void)
{
	static u8 error=0;

	if(abs(MOTOR_A.Motor_Pwm)>2500||abs(MOTOR_B.Motor_Pwm)>2500||abs(MOTOR_C.Motor_Pwm)>2500||abs(MOTOR_D.Motor_Pwm)>2500)   error++;
	if(error>6) EN=0,Flag_Stop=1,robot_mode_check_flag=1;  //�������ʮ�νӽ�����������ж�Ϊ�����ת���õ��ʧ��	
}

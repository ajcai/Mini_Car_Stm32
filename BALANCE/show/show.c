#include "show.h"
int Voltage_Show;
unsigned char i;          //��������
unsigned char Send_Count; //������Ҫ���͵����ݸ���
extern SEND_DATA Send_Data;//��Դ��usart.x��Ϊ���ݳ����ٶ�����������ٶ�
extern int MPU9250ErrorCount, EncoderA_Count, EncoderB_Count, EncoderC_Count, EncoderD_Count; //�������������ݼ�������������
extern int MPU9250SensorCountA, MPU9250SensorCountB, MPU9250SensorCountC, MPU9250SensorCountD;

/**************************************************************************
�������ܣ�OLED��ʾ����ʾ����
��ڲ�������
����  ֵ����
**************************************************************************/
void show_task(void *pvParameters)
{
   u32 lastWakeTime = getSysTickCnt();
   while(1)
    {	
		int i=0;
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_50_HZ));//��������50Hz��Ƶ������
			if(Deviation_Count<50)Buzzer=1; //������
			else Buzzer=0;
		
		for(i=0;i<100;i++)
		{
			Voltage_All+=Get_battery_volt(); 
		}
		Voltage=Voltage_All/100;
		Voltage_All=0;
		
//			Voltage_All+=Get_battery_volt();                //��β����ۻ�
//			if(++Voltage_Count==20) Voltage=Voltage_All/20,Voltage_All=0,Voltage_Count=0;//��ƽ��ֵ ��ȡ��ص�ѹ
//		Voltage=Get_battery_volt();
			if(Voltage>=12.6)Voltage=12.6;//���ⳬ��ص�ѹ
			else if(Voltage<10.5)Voltage=Voltage-0.5;//�͵�����ʱ����ʾ����һ�㼰ʱ���
				
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
		  APP_Show();	    //��MiniBalance APP��������
			oled_show();    //��ʾ����
    }
}  

/**************************************************************************
�������ܣ�OLED��ʾ
��ڲ�������
����  ֵ����
**************************************************************************/
void oled_show(void)
{  
   static int count=0;	 
	 int Car_Mode_Show, Divisor_Mode;
	 Divisor_Mode=2048/CAR_NUMBER+5;
	 Car_Mode_Show=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //�ɼ���λ��������Ϣ	
	 if(Car_Mode_Show>5)Car_Mode_Show=5;
	 Voltage_Show=Voltage*100; 
	 count++;
	
	 if(Check==0)//û�н����Լ�ģʽʱС��������ʾ
	 {	
		 //��ʾ����1����ʾ����//
		 switch(Car_Mode_Show)
		 {
			case Mec_Car:       OLED_ShowString(0,0,"Mec "); break; //�����ķ��С��
			case Omni_Car:      OLED_ShowString(0,0,"Omni");  break; //ȫ����С��
			case Akm_Car:       OLED_ShowString(0,0,"Akm ");  break; //������С��
			case Diff_Car:      OLED_ShowString(0,0,"Diff"); break; //���ֲ���С��
			case FourWheel_Car: OLED_ShowString(0,0,"4WD "); break; //������ 
			case Tank_Car:      OLED_ShowString(0,0,"Tank"); break; //�Ĵ���
		 }
		 
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car)
		 {
			 //���֡�ȫ����С����ʾZ����ٶ�
			 OLED_ShowString(55,0,"GZ");
			 if( gyro[2]<0)  OLED_ShowString(80,0,"-"),OLED_ShowNumber(90,0,-gyro[2],5,12);
			 else            OLED_ShowString(80,0,"+"),OLED_ShowNumber(90,0, gyro[2],5,12);		//z�����������Ư������	
		 }
		 else if(Car_Mode==Akm_Car||Car_Mode==Diff_Car||Car_Mode==FourWheel_Car||Car_Mode==Tank_Car)
		 {
			//�����������١��������Ĵ�����ʾZ���ƫ��ֵ
			OLED_ShowString(55,0,"BIAS");
			if( Deviation_gyro[2]<0)  OLED_ShowString(90,0,"-"),OLED_ShowNumber(100,0,-Deviation_gyro[2],3,12);
			else                      OLED_ShowString(90,0,"+"),OLED_ShowNumber(100,0, Deviation_gyro[2],3,12);		//z�����������Ư������	
		 }
		 //��ʾ����1����ʾ����//
		 

		 //��ʾ����2����ʾ����//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//���֡�ȫ���֡���������ʾ���A��״̬
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
			 //������������С����ʾZ����ٶ�
			 OLED_ShowString(00,10,"GYRO_Z:");
			 if( gyro[2]<0)  OLED_ShowString(60,10,"-"),
											 OLED_ShowNumber(75,10,-gyro[2],5,12);
			 else            OLED_ShowString(60,10,"+"),
											 OLED_ShowNumber(75,10, gyro[2],5,12);		//z������������	
		 }	 
		 //��ʾ����2����ʾ����//
		 
		 
		 //��ʾ����3��4����ʾ����//
		 if(Car_Mode==Mec_Car||Car_Mode==Omni_Car||Car_Mode==FourWheel_Car)
		 {
			//���֡�ȫ���֡���������ʾ���B��״̬
			OLED_ShowString(0,20,"B");		
			if( MOTOR_B.Target<0)	OLED_ShowString(15,20,"-"),
														OLED_ShowNumber(20,20,-MOTOR_B.Target*1000,5,12);
			else                 	OLED_ShowString(15,20,"+"),
														OLED_ShowNumber(20,20, MOTOR_B.Target*1000,5,12); 
			
			if( MOTOR_B.Encoder<0)OLED_ShowString(60,20,"-"),
														OLED_ShowNumber(75,20,-MOTOR_B.Encoder*1000,5,12);
			else                 	OLED_ShowString(60,20,"+"),
														OLED_ShowNumber(75,20, MOTOR_B.Encoder*1000,5,12);
			//���֡�ȫ���֡���������ʾ���C��״̬	
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
			 //�����������١��Ĵ�����ʾ���A��״̬	
			 OLED_ShowString(0,20,"L:");
			 if( MOTOR_A.Target<0)	OLED_ShowString(15,20,"-"),
															OLED_ShowNumber(20,20,-MOTOR_A.Target*1000,5,12);
			 else                 	OLED_ShowString(15,20,"+"),
															OLED_ShowNumber(20,20, MOTOR_A.Target*1000,5,12); 
			
			 if( MOTOR_A.Encoder<0)	OLED_ShowString(60,20,"-"),
															OLED_ShowNumber(75,20,-MOTOR_A.Encoder*1000,5,12);
			 else                 	OLED_ShowString(60,20,"+"),
															OLED_ShowNumber(75,20, MOTOR_A.Encoder*1000,5,12);
			 //�����������١��Ĵ�����ʾ���B��״̬
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
//			 else             OLED_ShowString(80,20,"+"),OLED_ShowNumber(90,20, accel[0],5,12);		//z�������Ǽ��ٶȼ�����	
//			 OLED_ShowString(55,30,"AY");
//			 if( accel[1]<0)  OLED_ShowString(80,30,"-"),OLED_ShowNumber(90,30,-accel[1],5,12);
//			 else             OLED_ShowString(80,30,"+"),OLED_ShowNumber(90,30, accel[1],5,12);		//z�������Ǽ��ٶȼ�����	
		 }
		 //��ʾ����3��4����ʾ����//
		 
		 
		 //��ʾ����5����ʾ����
			if(Car_Mode==Mec_Car||Car_Mode==FourWheel_Car)
			{
				//����С����ʾ���D��״̬
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
				//ȫ����С����ʾZ���ٶ�(�Ŵ�1000��) ��λrad/s
				OLED_ShowString(0,40,"MOVE_Z"); 			
				if(Send_Data.Sensor_Str.X_speed<0)	OLED_ShowString(60,40,"-"),
																						OLED_ShowNumber(75,40,-Send_Data.Sensor_Str.X_speed,5,12);
				else                              	OLED_ShowString(60,40,"+"),
																						OLED_ShowNumber(75,40, Send_Data.Sensor_Str.X_speed,5,12);
			}
			else if(Car_Mode==Akm_Car)
			{
				//������С����ʾ�����PWM����ֵ
				OLED_ShowString(00,40,"SERVO:");
				if( Servo<0)		      OLED_ShowString(60,40,"-"),
															OLED_ShowNumber(80,40,-Servo,4,12);
				else                 	OLED_ShowString(60,40,"+"),
															OLED_ShowNumber(80,40, Servo,4,12); 
			}
			else if(Car_Mode==Diff_Car||Car_Mode==Tank_Car)
			{
			 //����С����ʾ���ҵ����PWM����ֵ
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
			//��ʾ����5����ʾ����
			
			
			//��ʾ����6����ʾ����
//			OLED_ShowString(0,50,"Temp: ");//��ʾ�¶�
//			OLED_ShowNumber(50,50, MPU_Get_Temperature(),4,12); 
			if(PS2_ON_Flag==1)         OLED_ShowString(0,50,"PS2  ");//��ʾ����ģʽ
			else if (APP_ON_Flag==1)   OLED_ShowString(0,50,"APP  ");
			else if (Remote_ON_Flag==1)OLED_ShowString(0,50,"R-C  ");
			else if (CAN_ON_Flag==1)   OLED_ShowString(0,50,"CAN  ");
			else if (Usart_ON_Flag==1) OLED_ShowString(0,50,"USART");
			else                       OLED_ShowString(0,50,"ROS  ");
			
			if(EN==1&&Flag_Stop==0)   OLED_ShowString(45,50,"O N");  //���ʹ�ܿ���
			else                      OLED_ShowString(45,50,"OFF"); 
			
																OLED_ShowString(88,50,".");
																OLED_ShowString(110,50,"V");
																OLED_ShowNumber(75,50,Voltage_Show/100,2,12);
																OLED_ShowNumber(98,50,Voltage_Show%100,2,12);
			if(Voltage_Show%100<10) 	OLED_ShowNumber(92,50,0,2,12);
		}
	 
		/*�Լ����*/
		//��ʾ�Լ�ȷ�Ͻ���
		if(Check==1&&!Checking&&Checked==0) 
		{
        OLED_ShowCheckConfirming();
		}
		//��ʾ�Լ���н���
		if(Check==1&&Checking&&Checked==0) 
		{	
        OLED_ShowChecking();
		}	
		//��ʾ�Լ���
		if(Check==1&&Checking&&Checked==1) 
		{
			//OLED_ShowString(0,30,"MPeB:");OLED_ShowNumber(40,30, MPU9250SensorCountB,3,12); 
			//OLED_ShowString(0,40,"SEeB:");OLED_ShowNumber(40,40, EncoderB_Count,3,12); 
			
      OLED_ShowCheckResult();
		}
		/*�Լ����*/
		
		//=============ˢ����Ļ=======================//
		if(Check==0)OLED_Refresh_Gram();
    else {if(count>5)OLED_Refresh_Gram(),count=0;}

	  		
	}

/**************************************************************************
�������ܣ�OLED����ʾ�Լ�ȷ�Ͻ���
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void OLED_ShowCheckConfirming(void)
{
	//�����û�������ʼ�Լ�
  for(i=0;i<8;i++)	
	{
	 OLED_ShowCHinese(i*15, 5, i, CNSizeWidth, CNSizeHeight);
	}
	for(i=8;i<12;i++)	
	{
	 OLED_ShowCHinese((i-8)*15, 3, i, CNSizeWidth, CNSizeHeight);
	}
	
	//�޲���*����˳�
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
�������ܣ�OLED����ʾ�Լ���н���
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void OLED_ShowChecking(void)
{
	//�����û�������ʼ�Լ�
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
//	else            OLED_ShowString(85,43,"+"),OLED_ShowNumber(100,43, gyro[2]/10,4,12);		//z�����������Ư������	
}
/**************************************************************************
�������ܣ�OLED����ʾ�Լ���ɽ���
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
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
	
	if(ErrorCode&1)//��ѹ����
	{
	  for(i=40;i<44;i++)	
		{
		 OLED_ShowCHinese((i-40)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<1)//ʹ�ܿ��عرգ����ʹ�ܿ��ء���ͣ����
	{
	  for(i=44;i<50;i++)	
		{
		 OLED_ShowCHinese((i-44)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}

	
	if(ErrorCode&1<<3)//A������
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<4)//A�����߷���
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<5)//B������
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<6)//B�����߷���
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<7)//C������
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<8)//C�����߷���
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<9)//D������
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=55;i<59;i++)	
		{
		 OLED_ShowCHinese((i-55)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<10)//D�����߷���
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=50;i<55;i++)	
		{
		 OLED_ShowCHinese((i-50)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<11)//��Դ���ȶ�
	{
	  for(i=64;i<69;i++)	
		{
		 OLED_ShowCHinese((i-64)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<12)//�¶ȹ���
	{
	  for(i=69;i<73;i++)	
		{
		 OLED_ShowCHinese((i-69)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<14||ErrorCode&1<<15)//����߽Ӵ�
	{
	  for(i=73;i<78;i++)	
		{
		 OLED_ShowCHinese((i-73)*15+15, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<16)//A��������
	{
		OLED_ShowString(20,(5-row)*8+10,"A");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<17)//B��������
	{
		OLED_ShowString(20,(5-row)*8+10,"B");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<18)//C��������
	{
		OLED_ShowString(20,(5-row)*8+10,"C");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	if(ErrorCode&1<<19)//D��������
	{
		OLED_ShowString(20,(5-row)*8+10,"D");
	  for(i=59;i<64;i++)	
		{
		 OLED_ShowCHinese((i-59)*15+30, row, i, CNSizeWidth, CNSizeHeight);
		}
		row=row-2;
	}
	
	if(ErrorCode&1<<13)//MPU9250�� ���ȼ��ϵͣ����ں���
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
	
	if(ErrorCode==0)//�������Ϊ0��С������
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
�������ܣ���APP��������
��ڲ�������
����  ֵ����
��    �ߣ�WHEELTEC
**************************************************************************/
void APP_Show(void)
{    
	 static u8 flag_show;
	 int Left_Figure,Right_Figure,Voltage_Show;
	 Voltage_Show=(Voltage*1000-10000)/27;//��ص�ѹ�򵥴���
	
	 if(Voltage_Show>100)Voltage_Show=100;   //�Ե�ѹ���ݽ��д���
	
	 Left_Figure=MOTOR_A.Encoder*100;  if(Left_Figure<0)Left_Figure=-Left_Figure;	 //�Ա��������ݾ������ݴ������ͼ�λ�
	 Right_Figure=MOTOR_B.Encoder*100;  if(Right_Figure<0)Right_Figure=-Right_Figure;
	
	 flag_show=!flag_show;//��Ƶ���������ӡAPP���ݺ���ʾ����
	 if(PID_Send==1)//����PID����
	  {
		printf("{C%d:%d:%d}$",(int)RC_Velocity,(int)Velocity_KP,(int)Velocity_KI);//��ӡ��APP����
		PID_Send=0;	
	  }	
	 else	if(flag_show==0)// 
	 printf("{A%d:%d:%d:%d}$",(u8)Left_Figure,(u8)Right_Figure,Voltage_Show,(int)gyro[2]); //��ӡ��APP����
	 else
	 printf("{B%d:%d:%d}$",(int)gyro[0],(int)gyro[1],(int)gyro[2]);//��ӡ��APP���� ��ʾ����

}



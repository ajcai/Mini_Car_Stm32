#include "usartx.h"
SEND_DATA Send_Data;//�������ݵ�
RECEIVE_DATA Receive_Data;//�������ݵ�
extern int Time_count;
/**************************************************************************
�������ܣ�����2��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart2_send(u8 data)
{
	USART2->DR = data;
	while((USART2->SR&0x40)==0);	
}
/**************************************************************************
�������ܣ�����2��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart2_init(u32 bound)
{  	 
	//GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOD, ENABLE);	//ʹ��GPIOʱ��
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART2, ENABLE);	//ʹ��USART3ʱ��
	GPIO_PinRemapConfig(GPIO_Remap_USART2, ENABLE);//������ӳ��

	//USART_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5; //PD5
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOD, &GPIO_InitStructure);   
	//USART_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_6;//PD6
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOD, &GPIO_InitStructure);
	//UsartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART2_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
	//USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART2, &USART_InitStructure);     //��ʼ������2
	USART_ITConfig(USART2, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART2, ENABLE);                    //ʹ�ܴ���2 
}
/**************************************************************************
�������ܣ�����2�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART2_IRQHandler(void)
{	
	int Usart_Receive;
	if(USART_GetITStatus(USART2, USART_IT_RXNE) != RESET) //���յ�����
	{	      
		static u8 Flag_PID,i,j,Receive[50],Last_Usart_Receive;
		static float Data;
				
		Usart_Receive=USART2->DR;
		
		if(Deviation_Count<CONTROL_DELAY)return 0;	//ǰ�ڲ������ж�

		if(Usart_Receive==0x41&&Last_Usart_Receive==0x41&&APP_ON_Flag==0)PS2_ON_Flag=0,Remote_ON_Flag=0,APP_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0;  //ʹ�ܴ��ڿ��ƿ���10��֮��APP��ǰ��������
	  if(Usart_Receive==0x4B) Turn_Flag=1;  //����ת����ƽ���
	  else	if(Usart_Receive==0x49||Usart_Receive==0x4A) 	 Turn_Flag=0;	//������ƽ���	
    Last_Usart_Receive=Usart_Receive;	
		
		if(Turn_Flag==0)//ҡ�˿��ƽ���
		{
				if(Usart_Receive>=0x41&&Usart_Receive<=0x48)  
				{	
					Flag_Direction=Usart_Receive-0x40;
				}
				else	if(Usart_Receive<=8)   
				{			
					Flag_Direction=Usart_Receive;
				}	
				else  Flag_Direction=0;
		}
		else if(Turn_Flag==1)//�������ƽ���
		{
			 if     (Usart_Receive==0x43) Flag_Left=0,Flag_Right=1; //����ת
			 else if(Usart_Receive==0x47) Flag_Left=1,Flag_Right=0; //����ת
			 else                         Flag_Left=0,Flag_Right=0;
			 if     (Usart_Receive==0x41||Usart_Receive==0x45) Flag_Direction=Usart_Receive-0x40;
			 else  Flag_Direction=0;
		}
		if(Usart_Receive==0x58)  RC_Velocity=RC_Velocity+100; //���ٰ�����100
		if(Usart_Receive==0x59)  RC_Velocity=RC_Velocity-100; //���ٰ�����100
	  
	 	//��������APP���Խ���ͨѶ
	 if(Usart_Receive==0x7B) Flag_PID=1;   //APP����ָ����ʼλ
	 if(Usart_Receive==0x7D) Flag_PID=2;   //APP����ָ��ֹͣλ

	 if(Flag_PID==1)  //�ɼ�����
	 {
		Receive[i]=Usart_Receive;
		i++;
	 }
	 if(Flag_PID==2)  //��������
	 {
					if(Receive[3]==0x50) 	 PID_Send=1;
				 else  if(Receive[1]!=0x23) 
				 {								
					for(j=i;j>=4;j--)
					{
						Data+=(Receive[j-1]-48)*pow(10,i-j);
					}
					switch(Receive[1])
					 {
						 case 0x30:  RC_Velocity=Data;break;
						 case 0x31:  Velocity_KP=Data;break;
						 case 0x32:  Velocity_KI=Data;break;
						 case 0x33:  break;
						 case 0x34:  break;
						 case 0x35:  break;
						 case 0x36:  break;
						 case 0x37:  break;
						 case 0x38:  break; 		//Ԥ��
					 }
				 }				 
				 Flag_PID=0;//��ر�־λ����
				 i=0;
				 j=0;
				 Data=0;
				 memset(Receive, 0, sizeof(u8)*50);//��������
	 }
   if(RC_Velocity<0)   RC_Velocity=0; 	 
   }
return 0;	
}
/**************************************************************************
�������ܣ�����3��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart3_send(u8 data)
{
	USART3->DR = data;
	while((USART3->SR&0x40)==0);	
}
/**************************************************************************
�������ܣ�����1��������
��ڲ�����Ҫ���͵�����
����  ֵ����
**************************************************************************/
void usart1_send(u8 data)
{
	USART1->DR = data;
	while((USART1->SR&0x40)==0);	
}
/**************************************************************************
�������ܣ�����3��ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void uart3_init(u32 bound)
{  	 
	  //GPIO�˿�����
  GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO,ENABLE);// ��Ҫʹ��AFIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);	//ʹ��GPIOʱ��
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);	//ʹ��USART3ʱ��
	GPIO_PinRemapConfig(GPIO_PartialRemap_USART3, ENABLE);//������ӳ��
	//USART_TX  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10; //C10
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
  GPIO_Init(GPIOC, &GPIO_InitStructure);   
    //USART_RX	  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;//PC11
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
  GPIO_Init(GPIOC, &GPIO_InitStructure);
    //UsartNVIC ����
  NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=2 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
  USART_Init(USART3, &USART_InitStructure);     //��ʼ������3
  USART_ITConfig(USART3, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
  USART_Cmd(USART3, ENABLE);                    //ʹ�ܴ���3 
}
/**************************************************************************
�������ܣ�����3�����ж�
��ڲ�������
����  ֵ����
**************************************************************************/
int USART3_IRQHandler(void)
{	
	static u8 Count=0;
	u8 Usart_Receive;

	if(USART_GetITStatus(USART3, USART_IT_RXNE) != RESET)  //�ж��Ƿ���յ�����
	{
		Usart_Receive = USART_ReceiveData(USART3);//��ȡ����
		if(Time_count<CONTROL_DELAY)return 0;	//ǰ�ڲ������ж�
    Receive_Data.buffer[Count]=Usart_Receive;
		if(Usart_Receive == FRAME_HEADER||Count>0) Count++; else Count=0;
		if (Count == 11)	//��֤���ݰ��ĳ���
		{   
				Count=0;//���¿�ʼ����
				if(Receive_Data.buffer[10] == FRAME_TAIL) //��֤���ݰ���β��У����Ϣ
				{
					if(Receive_Data.buffer[9] ==Check_Sum(9,0))	 //����У��λ���㣬ģʽ0�Ƿ�������У��
				  {			
						PS2_ON_Flag=0;//����ģʽ��ͣ
						Remote_ON_Flag=0;
						APP_ON_Flag=0;
						CAN_ON_Flag=0;
						Usart_ON_Flag=0;//���봮��3�жϣ�ǿ�н���ROSģʽ�����ȼ����	
						Move_X=XYZ_Target_Speed_transition(Receive_Data.buffer[3],Receive_Data.buffer[4]);
						Move_Y=XYZ_Target_Speed_transition(Receive_Data.buffer[5],Receive_Data.buffer[6]);
						Move_Z=XYZ_Target_Speed_transition(Receive_Data.buffer[7],Receive_Data.buffer[8]);
				  }
			}
		}
	} 
return 0;	
}
/**************************************************************************
*  �������ܣ�����1��ʼ��
*
*  ��ڲ�������
*
*  �� �� ֵ����
**************************************************************************/
void uart1_init(u32 bound)
{  	 
	  //GPIO�˿�����
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE);	//ʹ��GPIOʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);	//ʹ��USARTʱ��

	//USART_TX  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9; //PA9
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;	//�����������
	GPIO_Init(GPIOA, &GPIO_InitStructure);   
    //USART_RX	  
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;//PA10
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;//��������
	GPIO_Init(GPIOA, &GPIO_InitStructure);
    //UsartNVIC ����
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority=1 ;//��ռ���ȼ�
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;		//�����ȼ�
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;			//IRQͨ��ʹ��
	NVIC_Init(&NVIC_InitStructure);	//����ָ���Ĳ�����ʼ��VIC�Ĵ���
   //USART ��ʼ������
	USART_InitStructure.USART_BaudRate = bound;//���ڲ�����
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;//�ֳ�Ϊ8λ���ݸ�ʽ
	USART_InitStructure.USART_StopBits = USART_StopBits_1;//һ��ֹͣλ
	USART_InitStructure.USART_Parity = USART_Parity_No;//����żУ��λ
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;//��Ӳ������������
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;	//�շ�ģʽ
	USART_Init(USART1, &USART_InitStructure);     //��ʼ������1
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE);//�������ڽ����ж�
	USART_Cmd(USART1, ENABLE);                    //ʹ�ܴ���1
	
}
/**************************************************************************
*  �������ܣ�����1�����ж�
*
*  ��ڲ�������
*
*  �� �� ֵ����
**************************************************************************/
int USART1_IRQHandler(void)
{	
	if(USART_GetITStatus(USART1, USART_IT_RXNE) != RESET) //���յ�����
	{
		u8 Usart_Receive;
		static u8 Count;
		static u8 rxbuf[11];
		int check=0,error=1,i;
		
		Usart_Receive = USART_ReceiveData(USART1);//��ȡ����
		if(Time_count<CONTROL_DELAY)return 0;	//ǰ�ڲ������ж�		
			
		//��ȡ���������ݣ���ʼУ�飬У��ɹ���ֵXYZĿ���ٶ�
    rxbuf[Count]=Usart_Receive;
    if(Usart_Receive == FRAME_HEADER||Count>0) Count++; else Count=0;
		if (Count == 11)	//��֤���ݰ��ĳ���
		{   
				Count=0;//���¿�ʼ����
				if(rxbuf[10] == FRAME_TAIL) //��֤���ݰ���β��У����Ϣ
				{
					
					for(i=0; i<9; i++)
					{
						check=rxbuf[i]^check; //������ڼ�������Ƿ����
					}
					if(check==rxbuf[9]) error=0; //����ɹ�
					
					if(error==0)	 //����У��λ����
				  {		
            if(Usart_ON_Flag==0)
						{	
							Usart_ON_Flag=1;
							APP_ON_Flag=0;
							PS2_ON_Flag=0;
							Remote_ON_Flag=0;
							CAN_ON_Flag=0;
						}		
		
						Move_X=(short)((rxbuf[3]<<8)+(rxbuf[4])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Move_Y=(short)((rxbuf[5]<<8)+(rxbuf[6])); //��X���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						Move_Z=(short)((rxbuf[7]<<8)+(rxbuf[8])); //��Z���ٶ� �ָ�8λ�͵�8λ ��λmm/s
						
						Move_X=Move_X/1000; //��λm/s
						Move_Y=Move_Y/1000; //��λm/s
						Move_Z=Move_Z/1000; //��λm/s
					}
			  }
		 }
	}
		return 0;	
}
/**************************************************************************
�������ܣ�����3����������
��ڲ�������
����  ֵ����
**************************************************************************/
void data_task(void *pvParameters)
{
	 u32 lastWakeTime = getSysTickCnt();
	
   while(1)
    {	
			vTaskDelayUntil(&lastWakeTime, F2T(RATE_20_HZ));//��������20Hz��Ƶ������
			data_transition(); //��Ҫ���з��͵����ݽ��и�ֵ
			USART3_SEND();     //����3(ROS)��������
			CAN_SEND();        //CAN��������
			USART1_SEND();     //����1����������Ҫ�رպ�ģ��ʼ��TIM1_Cap_Init(0XFFFF,72-1);
		}

}
/**************************************************************************
�������ܣ����ڷ��͵����ݽ��и�ֵ
��ڲ�������
����  ֵ����
**************************************************************************/
void data_transition(void)
{
	Send_Data.Sensor_Str.Frame_Header = FRAME_HEADER; //֡ͷ
	Send_Data.Sensor_Str.Frame_Tail = FRAME_TAIL; //֡β
	
	switch(Car_Mode)
	{	
		case Mec_Car:      //�����ķ��С�� 
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000; //С��x���ٶ�
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder-MOTOR_D.Encoder)/4)*1000; //С��y���ٶ�
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_A.Encoder-MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4/(Axle_spacing+Wheel_spacing))*1000;//С��z���ٶ�            
		  break; 
		
    case Omni_Car: //ȫ����С��     
			Send_Data.Sensor_Str.X_speed = ((MOTOR_C.Encoder-MOTOR_B.Encoder)/2/X_PARAMETER)*1000; //С��x���ٶ�
	    Send_Data.Sensor_Str.Y_speed = ((MOTOR_A.Encoder*2-MOTOR_B.Encoder-MOTOR_C.Encoder)/3)*1000; //С��y���ٶ�
	    Send_Data.Sensor_Str.Z_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder)/3/Omni_turn_radiaus)*1000;//С��z���ٶ�       
		  break; 
    
		case Akm_Car:   //������С��
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; //С��x���ٶ�
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;//С��z���ٶ�
		  break; 
		
		case Diff_Car:  //���ֲ���С��
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; //С��x���ٶ�
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/Wheel_spacing)*1000;//С��z���ٶ�
			break; 
		
		case FourWheel_Car: //������ 
      Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/4)*1000; //С��x���ٶ�
	    Send_Data.Sensor_Str.Y_speed = 0;
	    Send_Data.Sensor_Str.Z_speed = ((-MOTOR_B.Encoder-MOTOR_A.Encoder+MOTOR_C.Encoder+MOTOR_D.Encoder)/2/(Axle_spacing+Wheel_spacing))*1000;//С��z���ٶ�
		 break; 
		
		case Tank_Car:   //�Ĵ���
			Send_Data.Sensor_Str.X_speed = ((MOTOR_A.Encoder+MOTOR_B.Encoder)/2)*1000; //С��x���ٶ�
			Send_Data.Sensor_Str.Y_speed = 0;
			Send_Data.Sensor_Str.Z_speed = ((MOTOR_B.Encoder-MOTOR_A.Encoder)/(Wheel_spacing)*1000);//С��z���ٶ�
			break; 
	}
	
	//���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Accelerometer.X_data= accel[1]; //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Accelerometer.Y_data=-accel[0]; //���ٶȼ�X��ת����ROS����Y��
	Send_Data.Sensor_Str.Accelerometer.Z_data= accel[2];
	
	//���ٶȼ�������ٶ�
	Send_Data.Sensor_Str.Gyroscope.X_data= gyro[1]; //���ٶȼ�Y��ת����ROS����X��
	Send_Data.Sensor_Str.Gyroscope.Y_data=-gyro[0]; //���ٶȼ�X��ת����ROS����Y��
	if(Flag_Stop==0) Send_Data.Sensor_Str.Gyroscope.Z_data=gyro[2];  //����������λʹ��״̬����ô��������Z����ٶ�
	else             Send_Data.Sensor_Str.Gyroscope.Z_data=0;       //����������Ǿ�ֹ�ģ��������λʧ�ܣ�����ô���͵�Z����ٶ�Ϊ0
	
	Send_Data.Sensor_Str.Power_Voltage = Voltage*1000; //��ص�ѹ(���ｫ�������Ŵ�һǧ�����䣬��Ӧ���ڽ��ն��ڽ��յ����ݺ�Ҳ����Сһǧ��)
	
	Send_Data.buffer[0]=Send_Data.Sensor_Str.Frame_Header; //֡ͷ(�̶�ֵ)
  Send_Data.buffer[1]=Flag_Stop;//���״̬
	
	Send_Data.buffer[2]=Send_Data.Sensor_Str.X_speed >>8; //С��x���ٶ�
	Send_Data.buffer[3]=Send_Data.Sensor_Str.X_speed ;    //С��x���ٶ�
	Send_Data.buffer[4]=Send_Data.Sensor_Str.Y_speed>>8;  //С��y���ٶ�
	Send_Data.buffer[5]=Send_Data.Sensor_Str.Y_speed;     //С��y���ٶ�
	Send_Data.buffer[6]=Send_Data.Sensor_Str.Z_speed >>8; //С��z���ٶ�
	Send_Data.buffer[7]=Send_Data.Sensor_Str.Z_speed ;    //С��z���ٶ�
	
	Send_Data.buffer[8]=Send_Data.Sensor_Str.Accelerometer.X_data>>8; //���ٶȼ�������ٶ�
	Send_Data.buffer[9]=Send_Data.Sensor_Str.Accelerometer.X_data;    //���ٶȼ�������ٶ�
	Send_Data.buffer[10]=Send_Data.Sensor_Str.Accelerometer.Y_data>>8;
	Send_Data.buffer[11]=Send_Data.Sensor_Str.Accelerometer.Y_data;
	Send_Data.buffer[12]=Send_Data.Sensor_Str.Accelerometer.Z_data>>8;
	Send_Data.buffer[13]=Send_Data.Sensor_Str.Accelerometer.Z_data;
	
	Send_Data.buffer[14]=Send_Data.Sensor_Str.Gyroscope.X_data>>8; //���ٶȼ�������ٶ�
	Send_Data.buffer[15]=Send_Data.Sensor_Str.Gyroscope.X_data; //���ٶȼ�������ٶ�
	Send_Data.buffer[16]=Send_Data.Sensor_Str.Gyroscope.Y_data>>8;
	Send_Data.buffer[17]=Send_Data.Sensor_Str.Gyroscope.Y_data;
	Send_Data.buffer[18]=Send_Data.Sensor_Str.Gyroscope.Z_data>>8;
	Send_Data.buffer[19]=Send_Data.Sensor_Str.Gyroscope.Z_data;
	Send_Data.buffer[20]=Send_Data.Sensor_Str.Power_Voltage >>8; //��ص�ѹ
	Send_Data.buffer[21]=Send_Data.Sensor_Str.Power_Voltage; //��ص�ѹ

	Send_Data.buffer[22]=Check_Sum(22,1); //����У��λ���㣬ģʽ1�Ƿ�������У��
	
	Send_Data.buffer[23]=Send_Data.Sensor_Str.Frame_Tail;//֡β���̶�ֵ��
}
/**************************************************************************
�������ܣ�����3(ROS)��������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART3_SEND(void)
{
  unsigned char i = 0;	
	for(i=0; i<24; i++)
	{
		usart3_send(Send_Data.buffer[i]);
	}	 
}

/**************************************************************************
�������ܣ����ڷ�������
��ڲ�������
����  ֵ����
**************************************************************************/
void USART1_SEND(void)
{
  unsigned char i = 0;	
	
	for(i=0; i<24; i++)
	{
		usart1_send(Send_Data.buffer[i]);
	}	 
}

/**************************************************************************
�������ܣ����㷢�͵�����У��λ
��ڲ�����
����  ֵ������λ
**************************************************************************/
u8 Check_Sum(unsigned char Count_Number,unsigned char Mode)
{
	unsigned char check_sum=0,k;
	//�������ݵ�У��
	if(Mode==1)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Send_Data.buffer[k];
	}
	//�������ݵ�У��
	if(Mode==0)
	for(k=0;k<Count_Number;k++)
	{
	check_sum=check_sum^Receive_Data.buffer[k];
	}
	return check_sum;
}

/**************************************************************************
�������ܣ�����λ���������ĸ�8λ�͵�8λ�������ϳ�һ��short�����ݺ�������λ��ԭ����
��ڲ�������8λ����8λ
����  ֵ��������X/Y/Z���Ŀ���ٶ�
**************************************************************************/
float XYZ_Target_Speed_transition(u8 High,u8 Low)
{
		short transition; //����ת�����м����
		transition=((High<<8)+Low); //����8λ�͵�8λ���ϳ�һ��16λ��short������
		return transition/1000+(transition%1000)*0.001;    //���Ͷ˽����ݷ���ǰ����һ��*1000�ĵ�λ���㣬����������ݺ���Ҫ��ԭ��λ
}

/**************************************************************************
*  �������ܣ�CAN��������
*
*  ��ڲ�������
*
*  �� �� ֵ����
**************************************************************************/
void CAN_SEND(void) 
{
	u8 CAN_SENT[8],i;
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i];
	}
	CAN1_Send_Num(0x101,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+8];
	}
	CAN1_Send_Num(0x102,CAN_SENT);
	
	for(i=0;i<8;i++)
	{
	  CAN_SENT[i]=Send_Data.buffer[i+16];
	}
	CAN1_Send_Num(0x103,CAN_SENT);
}



/***********************************************
��˾����ݸ��΢�����ܿƼ����޹�˾
Ʒ�ƣ�WHEELTEC
������wheeltec.net
�Ա����̣�shop114407458.taobao.com 
�汾V1.1
�޸�ʱ�䣺2020-07-04
All rights reserved
***********************************************/
#include "system.h"

//ͨ������궨���������ֳ�����ȫ���ֳ� Omni_or_Mec��0�����ֳ�ϵ�У�1��ȫ����ϵ��

//Car_Mode for Mec
//0:���������������SENIOR_MEC_NO  
//1:�������ְ�ʽ����SENIOR_MEC_BS  
//2:�������ֶ�������SENIOR_MEC_DL
//3:�������ְ�ʽ���ҳ�����TOP_MEC_BS_18
//4:�������ְ�ʽ����������TOP_MEC_BS_47
//5:�������ֶ������ҳ�����TOP_MEC_DL_18

//Car_Mode for Omni
//0:����ȫ�������Ǽ��ٰ�SENIOR_OMNI_FAST  

/***********************��ʼ����ر�־λ****************************/
u8 Flag_Left,Flag_Right,Flag_Direction=0,Turn_Flag;//����ң����صı�־λ
u8 PID_Send;                                       //APPͨ�ŵ���ر�־λ
u8 PS2_ON_Flag=0,APP_ON_Flag=0,Remote_ON_Flag=0,CAN_ON_Flag=0,Usart_ON_Flag=0;  //�ֱ�����������ģң�ؿ��Ʊ�־λ  
u8 Car_Mode=0;  //������ѡ�ͱ�־λ
u8 Flag_Stop=1; //������ʹ�ܱ�־λ
int Check=0, Checking=0, Checked=0, CheckCount=0, CheckPhrase1=0, CheckPhrase2=0; //�Լ��־λ
long int ErrorCode=0; //�������
/****************************************************************/
float PS2_LX,PS2_LY,PS2_RX,PS2_RY,PS2_KEY;         //PS2��ر���
                 
float Velocity_KP=300,Velocity_KI=300;    //�ٶȿ���PID����
float RC_Velocity=500;        				        //����ң��С�����ٶ� ��λmm/s
float Move_X, Move_Y, Move_Z;               //С�������˶�Ŀ���ٶ�

unsigned char temp_show;
int Divisor_Mode;
float Encoder_precision;//����������
float Wheel_perimeter; //�����ܳ�����λ���ף�
float Wheel_spacing; //�������־� ����λ���ף�
float Axle_spacing; //����ǰ�������
float Omni_turn_radiaus; //ȫ����ת��뾶
Motor_parameter MOTOR_A,MOTOR_B,MOTOR_C,MOTOR_D;  //����Ĳ����ṹ�� 
int Servo; //���PWMֵ
Smooth_Control smooth_control; //ƽ�������м����
void systemInit(void)
{
	JTAG_Set(JTAG_SWD_DISABLE);     //=====�ر�JTAG�ӿ�
	JTAG_Set(SWD_ENABLE);           //=====��SWD�ӿ� �������������SWD�ӿڵ���
	
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_4);//����ϵͳ�ж����ȼ�����4
	delay_init();					          //��ʼ����ʱ����
	LED_Init();                     //��ʼ���� LED ���ӵ�Ӳ���ӿ�
	Buzzer_Init();                  //��ʼ��������
	Enable_Pin();                   //ʹ�ܿ��س�ʼ��
	OLED_Init();                    //OLED��ʼ��	  
	KEY_Init();	                    //������ʼ��	
	uart1_init(115200);	            //=====���ڳ�ʼ��Ϊ
	uart2_init(9600);               //����2��ʼ��
	uart3_init(115200);             //����3��ʼ�� 
	CAN1_Mode_Init(1,2,3,6,0);      //=====CAN��ʼ��
	
	Adc_Init();                     //�ɼ���ص�ѹADC���ų�ʼ��	
	Robot_Select();                 //���ݵ�λ����ֵ�ж�Ŀǰ�������е�����һ������ˣ�Ȼ����ж�Ӧ�Ĳ�����ʼ��
	
	if(Car_Mode==Akm_Car) Servo_PWM_Init(9999,71);   		//��ʼ��PWM
	else 		TIM1_Cap_Init(9999,71);    //��ʼ����ģң�ؽӿڣ�Ҫע�����4·Ԥ��PWM����ͺ�ģң�ص����ų�ͻ��
	
	Encoder_Init_TIM2();            //=====�������ӿ�A��ʼ��
	Encoder_Init_TIM3();            //=====�������ӿ�B��ʼ��
	Encoder_Init_TIM4();            //=====�������ӿ�C��ʼ��
	Encoder_Init_TIM5();            //=====�������ӿ�D��ʼ��
	MiniBalance_Motor_Init();       //��ʼ�����Ƶ������ת����
	MiniBalance_PWM_Init(7199,0);   //��ʼ��PWM 10KHZ�������������

	IIC_Init();                     //IIC��ʼ��
  MPU9250_Init();                 //MPU9250��ʼ��		
	
	PS2_Init();											//ps2�����˿ڳ�ʼ��
	PS2_SetInit();		 							//ps2���ó�ʼ��,���á����̵�ģʽ������ѡ���Ƿ�����޸�
}

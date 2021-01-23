#include "motor.h"

void MiniBalance_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //ʹ�ܶ˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //ʹ�ܶ˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ�ܶ˿�ʱ��
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //ʹ�ܶ˿�ʱ��

	GPIO_InitStructure.GPIO_Pin = IN1_PIN_A;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTA, &GPIO_InitStructure);			//�����趨������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_A;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTA, &GPIO_InitStructure);			//�����趨������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_B;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTB, &GPIO_InitStructure);			//�����趨������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_B;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTB, &GPIO_InitStructure);			//�����趨������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_C;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTC, &GPIO_InitStructure);			//�����趨������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_C;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTC, &GPIO_InitStructure);			//�����趨������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_D;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTD, &GPIO_InitStructure);			//�����趨������ʼ��GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_D;			//�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//�������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTD, &GPIO_InitStructure);			//�����趨������ʼ��GPIO

	GPIO_ResetBits(IN1_PORTA,IN1_PIN_A);	//io���0����ֹ�����ת
	GPIO_ResetBits(IN2_PORTA,IN2_PIN_A);	//io���0����ֹ�����ת
	GPIO_ResetBits(IN1_PORTB,IN1_PIN_B);	//io���0����ֹ�����ת
	GPIO_ResetBits(IN2_PORTB,IN2_PIN_B);	//io���0����ֹ�����ת
	GPIO_ResetBits(IN1_PORTC,IN1_PIN_C);	//io���0����ֹ�����ת
	GPIO_ResetBits(IN2_PORTC,IN2_PIN_C);	//io���0����ֹ�����ת
	GPIO_ResetBits(IN1_PORTD,IN1_PIN_D);	//io���0����ֹ�����ת
	GPIO_ResetBits(IN2_PORTD,IN2_PIN_D);	//io���0����ֹ�����ת
}
/**************************************************************************
�������ܣ����PWM���ų�ʼ��
��ڲ�������
����  ֵ����
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //ʹ�ܶ�ʱ��8 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //ʹ��GPIO����ʱ��
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_A;  //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_B;  //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_C;  //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_D;  //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTD, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //��������һ�������¼�װ�����Զ���װ�ؼĴ������ڵ�ֵ	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //����������ΪTIMxʱ��Ƶ�ʳ�����Ԥ��Ƶֵ  ����Ƶ
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1Ԥװ��ʹ��	 
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2Ԥװ��ʹ��	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH3Ԥװ��ʹ��	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��	


	TIM_ARRPreloadConfig(TIM8, ENABLE); //ʹ��TIMx��ARR�ϵ�Ԥװ�ؼĴ���	
	
	TIM_Cmd(TIM8, ENABLE);  //ʹ��TIM8
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); //�߼���ʱ���������ʹ�����		
} 

/**************************************************************************
�������ܣ�ʹ�ܿ��ص����ų�ʼ��
��ڲ�������
����  ֵ���� 
**************************************************************************/
void Enable_Pin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //ʹ�ܶ˿�ʱ��
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;            //�˿�����
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //��������
  GPIO_Init(GPIOE, &GPIO_InitStructure);					      //�����趨������ʼ��GPIO
} 

/*****************   *********************************************************
�������ܣ����PWM��ʼ��
��ڲ�������ڲ�����arr���Զ���װֵ  psc��ʱ��Ԥ��Ƶ�� 
����  ֵ����
**************************************************************************/
void Servo_PWM_Init(u16 arr,u16 psc)	
{ 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;

	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//enable the afio
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);//timer1 remap all the pins
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  //ʹ�ܶ�ʱ��1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //ʹ��GPIO����ʱ��

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 ;  //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;      
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;  //�˿�����
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //�����������
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//��ʼ����ʱ�� TIM1   
	TIM_TimeBaseStructure.TIM_Period = arr; //�趨�������Զ���װֵ 
	TIM_TimeBaseStructure.TIM_Prescaler = psc;   //Ԥ��Ƶ�� 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //����ʱ�ӷָ�:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM���ϼ���ģʽ
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //����TIM_TimeBaseInitStruct��ָ���Ĳ�����ʼ��TIMx��ʱ�������λ

	//��ʼ��TIM1���벶����� ͨ��1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01   ѡ������� 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x11;    //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//��ʼ��TIM1���벶����� ͨ��2
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01   ѡ�������  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x11;    //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//��ʼ��TIM1���벶����� ͨ��3
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01   ѡ�������  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //�����ز���
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //���������Ƶ,����Ƶ 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;    //IC1F=0000 ���������˲��� ���˲�
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//  //��ʼ��TIM1���벶����� ͨ��4
	//  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01   ѡ������� 
	//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //�����ز���
	//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //���������Ƶ,����Ƶ 
	//  TIM_ICInitStructure.TIM_ICFilter = 0x11;    //IC1F=0000 ���������˲��� ���˲�
	//  TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //ѡ��ʱ��ģʽ:TIM�����ȵ���ģʽ1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //�Ƚ����ʹ��
	TIM_OCInitStructure.TIM_Pulse = 0;                            //���ô�װ�벶��ȽϼĴ���������ֵ
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //�������:TIM����Ƚϼ��Ը�
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //����TIM_OCInitStruct��ָ���Ĳ�����ʼ������TIMx
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4Ԥװ��ʹ��  


	//  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1�ж�
	//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //��ռ���ȼ�0��
	//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	//  NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� 

	//�жϷ����ʼ��
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  //TIM1�ж�
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //��ռ���ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //�����ȼ�0��
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQͨ����ʹ��
	NVIC_Init(&NVIC_InitStructure);   //����NVIC_InitStruct��ָ���Ĳ�����ʼ������NVIC�Ĵ��� \

	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3,  ENABLE);   //����������жϣ�����CC1IE,CC2IE,CC3IE,CC4IE�����ж�  
	TIM_CtrlPWMOutputs(TIM1,ENABLE); //�߼���ʱ���������ʹ�����    
	TIM_Cmd(TIM1, ENABLE);     //ʹ�ܶ�ʱ��

}


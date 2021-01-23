#include "motor.h"

void MiniBalance_Motor_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA, ENABLE); //使能端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE); //使能端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能端口时钟
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //使能端口时钟

	GPIO_InitStructure.GPIO_Pin = IN1_PIN_A;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTA, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_A;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTA, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_B;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTB, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_B;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTB, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_C;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTC, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_C;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTC, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN1_PIN_D;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN1_PORTD, &GPIO_InitStructure);			//根据设定参数初始化GPIO
	
	GPIO_InitStructure.GPIO_Pin = IN2_PIN_D;			//端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;	//推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;	//50M
	GPIO_Init(IN2_PORTD, &GPIO_InitStructure);			//根据设定参数初始化GPIO

	GPIO_ResetBits(IN1_PORTA,IN1_PIN_A);	//io输出0，防止电机乱转
	GPIO_ResetBits(IN2_PORTA,IN2_PIN_A);	//io输出0，防止电机乱转
	GPIO_ResetBits(IN1_PORTB,IN1_PIN_B);	//io输出0，防止电机乱转
	GPIO_ResetBits(IN2_PORTB,IN2_PIN_B);	//io输出0，防止电机乱转
	GPIO_ResetBits(IN1_PORTC,IN1_PIN_C);	//io输出0，防止电机乱转
	GPIO_ResetBits(IN2_PORTC,IN2_PIN_C);	//io输出0，防止电机乱转
	GPIO_ResetBits(IN1_PORTD,IN1_PIN_D);	//io输出0，防止电机乱转
	GPIO_ResetBits(IN2_PORTD,IN2_PIN_D);	//io输出0，防止电机乱转
}
/**************************************************************************
函数功能：电机PWM引脚初始化
入口参数：无
返回  值：无
**************************************************************************/
void MiniBalance_PWM_Init(u16 arr,u16 psc)
{		 					 
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef  TIM_TimeBaseStructure;
	TIM_OCInitTypeDef  TIM_OCInitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM8, ENABLE);  //使能定时器8 
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE); //使能GPIO外设时钟
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_A;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTA, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_B;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTB, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_C;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTC, &GPIO_InitStructure);
	
	GPIO_InitStructure.GPIO_Pin = PWM_PIN_D;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(PWM_PORTD, &GPIO_InitStructure);
	
	TIM_TimeBaseStructure.TIM_Period = arr; //设置在下一个更新事件装入活动的自动重装载寄存器周期的值	 
	TIM_TimeBaseStructure.TIM_Prescaler =psc; //设置用来作为TIMx时钟频率除数的预分频值  不分频
	TIM_TimeBaseStructure.TIM_ClockDivision = 1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM8, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

 	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC1Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC1PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH1预装载使能	 
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC2Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC2PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH2预装载使能	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC3Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC3PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH3预装载使能	
	
	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC4Init(TIM8, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4PreloadConfig(TIM8, TIM_OCPreload_Enable);  //CH4预装载使能	


	TIM_ARRPreloadConfig(TIM8, ENABLE); //使能TIMx在ARR上的预装载寄存器	
	
	TIM_Cmd(TIM8, ENABLE);  //使能TIM8
	
	TIM_CtrlPWMOutputs(TIM8,ENABLE); //高级定时器输出必须使能这句		
} 

/**************************************************************************
函数功能：使能开关的引脚初始化
入口参数：无
返回  值：无 
**************************************************************************/
void Enable_Pin(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //使能端口时钟
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0;            //端口配置
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;         //上拉输入
  GPIO_Init(GPIOE, &GPIO_InitStructure);					      //根据设定参数初始化GPIO
} 

/*****************   *********************************************************
函数功能：舵机PWM初始化
入口参数：入口参数：arr：自动重装值  psc：时钟预分频数 
返回  值：无
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
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  //使能定时器1
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //使能GPIO外设时钟

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 ;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;      
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14 ;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;      //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;    //50M
	GPIO_Init(GPIOE, &GPIO_InitStructure);

	//初始化定时器 TIM1   
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = psc;   //预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM1输入捕获参数 通道1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01   选择输入端 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x11;    //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//初始化TIM1输入捕获参数 通道2
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01   选择输入端  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x11;    //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//初始化TIM1输入捕获参数 通道3
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01   选择输入端  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;    //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//  //初始化TIM1输入捕获参数 通道4
	//  TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01   选择输入端 
	//  TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;  //上升沿捕获
	//  TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	//  TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;    //配置输入分频,不分频 
	//  TIM_ICInitStructure.TIM_ICFilter = 0x11;    //IC1F=0000 配置输入滤波器 不滤波
	//  TIM_ICInit(TIM1, &TIM_ICInitStructure);

	TIM_OCInitStructure.TIM_OCMode = TIM_OCMode_PWM1; //选择定时器模式:TIM脉冲宽度调制模式1
	TIM_OCInitStructure.TIM_OutputState = TIM_OutputState_Enable; //比较输出使能
	TIM_OCInitStructure.TIM_Pulse = 0;                            //设置待装入捕获比较寄存器的脉冲值
	TIM_OCInitStructure.TIM_OCPolarity = TIM_OCPolarity_High;     //输出极性:TIM输出比较极性高
	TIM_OCInitStructure.TIM_OCNIdleState = TIM_OCNIdleState_Set; 
	TIM_OC4Init(TIM1, &TIM_OCInitStructure);  //根据TIM_OCInitStruct中指定的参数初始化外设TIMx
	TIM_OC4PreloadConfig(TIM1, TIM_OCPreload_Enable);  //CH4预装载使能  


	//  NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1中断
	//  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	//  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	//  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	//  NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  //TIM1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 \

	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3,  ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断  
	TIM_CtrlPWMOutputs(TIM1,ENABLE); //高级定时器输出必须使能这句    
	TIM_Cmd(TIM1, ENABLE);     //使能定时器

}


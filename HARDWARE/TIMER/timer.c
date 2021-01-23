#include "timer.h"

u8 TIM1CH1_CAPTURE_STA = 0;	//通道1输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM1CH1_CAPTURE_UPVAL;
u16 TIM1CH1_CAPTURE_DOWNVAL;

u8 TIM1CH2_CAPTURE_STA = 0;	//通道2输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM1CH2_CAPTURE_UPVAL;
u16 TIM1CH2_CAPTURE_DOWNVAL;

u8 TIM1CH3_CAPTURE_STA = 0;	//通道3输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM1CH3_CAPTURE_UPVAL;
u16 TIM1CH3_CAPTURE_DOWNVAL;

u8 TIM1CH4_CAPTURE_STA = 0;	//通道4输入捕获标志，高两位做捕获标志，低6位做溢出标志		
u16 TIM1CH4_CAPTURE_UPVAL;
u16 TIM1CH4_CAPTURE_DOWNVAL;

u32 TIM1_T1;
u32 TIM1_T2;
u32 TIM1_T3;
u32 TIM1_T4;

//定时器2输入捕获配置
int pwmout1, pwmout2, pwmout3, pwmout4; 				//输出占空比
u32 Remoter_Ch1=1500,Remoter_Ch2=1500,Remoter_Ch3=1500,Remoter_Ch4=1500;//航模遥控采集相关变量
u32 L_Remoter_Ch1=1500,L_Remoter_Ch2=1500,L_Remoter_Ch3=1500,L_Remoter_Ch4=1500;  //航模遥控接收变量

/**************************************************************************
*  函数功能：航模遥控初始化函数
*
*  入口参数：arr：自动重装值  psc：时钟预分频数 
*
*  返 回 值：无
**************************************************************************/ 
void TIM1_Cap_Init(u16 arr, u16 psc)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	TIM_TimeBaseInitTypeDef TIM_TimeBaseStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	TIM_ICInitTypeDef TIM_ICInitStructure;
	
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);//enable the afio
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_TIM1, ENABLE);  //使能定时器1
 	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //使能GPIO外设时钟
	GPIO_PinRemapConfig(GPIO_FullRemap_TIM1,ENABLE);//timer1 remap all the pins
	
	
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14;  //端口配置
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPD;      
	GPIO_Init(GPIOE, &GPIO_InitStructure);
	//GPIO_ResetBits(GPIOE, GPIO_Pin_9 | GPIO_Pin_11 | GPIO_Pin_13 | GPIO_Pin_14);//输入   下拉

	//初始化定时器 TIM1	 
	TIM_TimeBaseStructure.TIM_Period = arr; //设定计数器自动重装值 
	TIM_TimeBaseStructure.TIM_Prescaler = psc; 	//预分频器 
	TIM_TimeBaseStructure.TIM_ClockDivision = TIM_CKD_DIV1; //设置时钟分割:TDTS = Tck_tim
	TIM_TimeBaseStructure.TIM_CounterMode = TIM_CounterMode_Up;  //TIM向上计数模式
	TIM_TimeBaseInit(TIM1, &TIM_TimeBaseStructure); //根据TIM_TimeBaseInitStruct中指定的参数初始化TIMx的时间基数单位

	//初始化TIM1输入捕获参数 通道1
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_1; //CC1S=01 	选择输入端 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x0F;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//初始化TIM1输入捕获参数 通道2
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_2; //CC1S=01 	选择输入端  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//初始化TIM1输入捕获参数 通道3
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_3; //CC1S=01 	选择输入端  
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	//初始化TIM1输入捕获参数 通道4
	TIM_ICInitStructure.TIM_Channel = TIM_Channel_4; //CC1S=01 	选择输入端 
	TIM_ICInitStructure.TIM_ICPolarity = TIM_ICPolarity_Rising;	//上升沿捕获
	TIM_ICInitStructure.TIM_ICSelection = TIM_ICSelection_DirectTI; //
	TIM_ICInitStructure.TIM_ICPrescaler = TIM_ICPSC_DIV1;	  //配置输入分频,不分频 
	TIM_ICInitStructure.TIM_ICFilter = 0x00;	  //IC1F=0000 配置输入滤波器 不滤波
	TIM_ICInit(TIM1, &TIM_ICInitStructure);

	NVIC_InitStructure.NVIC_IRQChannel = TIM1_UP_IRQn;  //TIM1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

	//中断分组初始化
	NVIC_InitStructure.NVIC_IRQChannel = TIM1_CC_IRQn;  //TIM1中断
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;  //先占优先级0级
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;  //从优先级0级
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE; //IRQ通道被使能
	NVIC_Init(&NVIC_InitStructure);   //根据NVIC_InitStruct中指定的参数初始化外设NVIC寄存器 

	TIM_ITConfig(TIM1, TIM_IT_CC1|TIM_IT_CC2|TIM_IT_CC3|TIM_IT_CC4,	ENABLE);   //不允许更新中断，允许CC1IE,CC2IE,CC3IE,CC4IE捕获中断	
	TIM_CtrlPWMOutputs(TIM1,ENABLE); //高级定时器输出必须使能这句		
	TIM_Cmd(TIM1, ENABLE); 		//使能定时器

}

/**************************************************************************
*  函数功能：航模遥控接收中断
*
*  入口参数：无
*
*  返 回 值：无
**************************************************************************/ 
void TIM1_CC_IRQHandler(void)
{
	//连接航模后，需要推下前进杆，才可以正式航模控制小车
  if(Remoter_Ch2>1600&&Remote_ON_Flag==0&&Deviation_Count>=CONTROL_DELAY){APP_ON_Flag=0;PS2_ON_Flag=0;Remote_ON_Flag=1,CAN_ON_Flag=0,Usart_ON_Flag=0; }
	  /////////////////////通道一///////////////////////////////
	if ((TIM1CH1_CAPTURE_STA & 0X80) == 0) 		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC1) != RESET) 		//捕获1发生捕获事件
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC1); 		//清除中断标志位
			if (TIM1CH1_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM1CH1_CAPTURE_DOWNVAL = TIM_GetCapture1(TIM1);//记录下此时的定时器计数值
				if (TIM1CH1_CAPTURE_DOWNVAL < TIM1CH1_CAPTURE_UPVAL)
				{
					TIM1_T1 = 9999;
				}
				else
					TIM1_T1 = 0;
				Remoter_Ch1 = TIM1CH1_CAPTURE_DOWNVAL - TIM1CH1_CAPTURE_UPVAL+ TIM1_T1;		//得到总的高电平的时间
				if(abs(Remoter_Ch1-L_Remoter_Ch1)>500) Remoter_Ch1=L_Remoter_Ch1; //滤波
					 L_Remoter_Ch1=Remoter_Ch1;
				
				TIM1CH1_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM1CH1_CAPTURE_UPVAL = TIM_GetCapture1(TIM1);		//获取上升沿数据
				TIM1CH1_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC1PolarityConfig(TIM1, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}
  /////////////////////通道二///////////////////////////////
	if ((TIM1CH2_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC2) != RESET)		//捕获2发生捕获事件
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC2);		//清除中断标志位
			if (TIM1CH2_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM1CH2_CAPTURE_DOWNVAL = TIM_GetCapture2(TIM1);//记录下此时的定时器计数值
				if (TIM1CH2_CAPTURE_DOWNVAL < TIM1CH2_CAPTURE_UPVAL)
				{
					TIM1_T2 = 9999;
				}
				else
					TIM1_T2 = 0;
				Remoter_Ch2 = TIM1CH2_CAPTURE_DOWNVAL - TIM1CH2_CAPTURE_UPVAL
						+ TIM1_T2;		//得到总的高电平的时间
				if(abs(Remoter_Ch2-L_Remoter_Ch2)>500)Remoter_Ch2=L_Remoter_Ch2;//滤波
				L_Remoter_Ch2=Remoter_Ch2;
				
				TIM1CH2_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM1CH2_CAPTURE_UPVAL = TIM_GetCapture2(TIM1);		//获取上升沿数据
				TIM1CH2_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC2PolarityConfig(TIM1, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}
  /////////////////////通道三///////////////////////////////
	if ((TIM1CH3_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC3) != RESET)		//捕获3发生捕获事件
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC3);		//清除中断标志位
			if (TIM1CH3_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM1CH3_CAPTURE_DOWNVAL = TIM_GetCapture3(TIM1);//记录下此时的定时器计数值
				if (TIM1CH3_CAPTURE_DOWNVAL < TIM1CH3_CAPTURE_UPVAL)
				{
					TIM1_T3 = 9999;
				}
				else
					TIM1_T3 = 0;
				Remoter_Ch3 = TIM1CH3_CAPTURE_DOWNVAL - TIM1CH3_CAPTURE_UPVAL
						+ TIM1_T3;		//得到总的高电平的时间
				if(abs(Remoter_Ch3-L_Remoter_Ch3)>500)Remoter_Ch3=L_Remoter_Ch3;//滤波
									L_Remoter_Ch3=Remoter_Ch3;
				TIM1CH3_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM1CH3_CAPTURE_UPVAL = TIM_GetCapture3(TIM1);		//获取上升沿数据
				TIM1CH3_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC3PolarityConfig(TIM1, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}
  /////////////////////通道四///////////////////////////////
	if ((TIM1CH4_CAPTURE_STA & 0X80) == 0)		//还未成功捕获	
	{
		if (TIM_GetITStatus(TIM1, TIM_IT_CC4) != RESET)		//捕获4发生捕获事件
		{
			TIM_ClearITPendingBit(TIM1, TIM_IT_CC4);		//清除中断标志位
			if (TIM1CH4_CAPTURE_STA & 0X40)		//捕获到一个下降沿
			{
				TIM1CH4_CAPTURE_DOWNVAL = TIM_GetCapture4(TIM1);//记录下此时的定时器计数值
				if (TIM1CH4_CAPTURE_DOWNVAL < TIM1CH4_CAPTURE_UPVAL)
				{
					TIM1_T4 = 9999;
				}
				else
					TIM1_T4 = 0;
				Remoter_Ch4 = TIM1CH4_CAPTURE_DOWNVAL - TIM1CH4_CAPTURE_UPVAL
						+ TIM1_T4;		//得到总的高电平的时间
				if(abs(Remoter_Ch4-L_Remoter_Ch4)>500)Remoter_Ch4=L_Remoter_Ch4;//滤波
				L_Remoter_Ch4=Remoter_Ch4;				
				TIM1CH4_CAPTURE_STA = 0;		//捕获标志位清零
				TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Rising); //设置为上升沿捕获		  
			}
			else //发生捕获时间但不是下降沿，第一次捕获到上升沿，记录此时的定时器计数值
			{
				TIM1CH4_CAPTURE_UPVAL = TIM_GetCapture4(TIM1);		//获取上升沿数据
				TIM1CH4_CAPTURE_STA |= 0X40;		//标记已捕获到上升沿
				TIM_OC4PolarityConfig(TIM1, TIM_ICPolarity_Falling);//设置为下降沿捕获
			}
		}
	}
}
void TIM1_UP_IRQHandler(void) 
{ 
  TIM1->SR&=~(1<<0);//清除中断标志位 	    
}


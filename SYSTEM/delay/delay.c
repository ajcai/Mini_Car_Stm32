#include "delay.h"
#include "sys.h"
////////////////////////////////////////////////////////////////////////////////// 	 

#if SYSTEM_SUPPORT_OS
#include "FreeRTOS.h"					//FreeRTOS使用的头文件		  
#include "task.h"             //FreeRTOS使用的头文件	
#endif

static u8  fac_us=0;							//us延时倍乘数			   
static u16 fac_ms=0;							//ms延时倍乘数,在ucos下,代表每个节拍的ms数
	
static u32 sysTickCnt=0;	
extern void xPortSysTickHandler(void);

/**************************************************************************
函数功能：初始化延迟函数
入口参数：无
返回  值：无
**************************************************************************/
void delay_init()
{
	u32 reload;
	SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);//选择外部时钟  HCLK
	fac_us=SystemCoreClock/1000000;				//不论是否使用OS,fac_us都需要使用
	reload=SystemCoreClock/1000000;				//每秒钟的计数次数 单位为M  
	reload*=1000000/configTICK_RATE_HZ;			//根据configTICK_RATE_HZ设定溢出时间
												//reload为24位寄存器,最大值:16777216,在72M下,约合0.233s左右	
	fac_ms=1000/configTICK_RATE_HZ;				//代表OS可以延时的最少单位	   

	SysTick->CTRL|=SysTick_CTRL_TICKINT_Msk;   	//开启SYSTICK中断
	SysTick->LOAD=reload; 						//每1/configTICK_RATE_HZ秒中断一次	
	SysTick->CTRL|=SysTick_CTRL_ENABLE_Msk;   	//开启SYSTICK    
}									    
/**************************************************************************
函数功能：延时nus的延迟函数
入口参数：延时的时长
返回  值：无
**************************************************************************/  								   
void delay_us(u32 nus)
{		
	u32 ticks;
	u32 told,tnow,tcnt=0;
	u32 reload=SysTick->LOAD;				//LOAD的值	    	 
	ticks=nus*fac_us; 						//需要的节拍数 
	told=SysTick->VAL;        				//刚进入时的计数器值
	while(1)
	{
		tnow=SysTick->VAL;	
		if(tnow!=told)
		{	    
			if(tnow<told)tcnt+=told-tnow;	//这里注意一下SYSTICK是一个递减的计数器就可以了.
			else tcnt+=reload-tnow+told;	    
			told=tnow;
			if(tcnt>=ticks)break;			//时间超过/等于要延迟的时间,则退出.
		}  
	};										    
}  
/**************************************************************************
函数功能：延时nms的延迟函数
入口参数：延时的时长
返回  值：无
**************************************************************************/  
void delay_ms(u32 nms)
{	
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
	{		
		if(nms>=fac_ms)						//延时的时间大于OS的最少时间周期 
		{ 
   			vTaskDelay(nms/fac_ms);	 		//FreeRTOS延时
		}
		nms%=fac_ms;						//OS已经无法提供这么小的延时了,采用普通方式延时    
	}
	delay_us((u32)(nms*1000));				//普通方式延时
}
/**************************************************************************
函数功能：延时ms的延迟函数
入口参数：延时的时长
返回  值：无
**************************************************************************/  
void delay_xms(u32 nms)
{
	u32 i;
	for(i=0;i<nms;i++) delay_us(1000);
}

/**************************************************************************
函数功能：滴答定时器中断服务函数
入口参数：延时的时长
返回  值：无
**************************************************************************/  
void SysTick_Handler(void)
{	
    if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)//系统已经运行
    {
        xPortSysTickHandler();	
    }
		else
		{
		sysTickCnt++;	/*调度开启之前计数*/
	}
}
/**************************************************************************
函数功能：获取上次执行函数的时间
入口参数：无
返回  值：上一次执行程序的时间
**************************************************************************/ 
u32 getSysTickCnt(void)
{
	if(xTaskGetSchedulerState()!=taskSCHEDULER_NOT_STARTED)	/*系统已经运行*/
		return xTaskGetTickCount();
	else
		return sysTickCnt;
}


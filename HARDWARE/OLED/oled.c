#include "oled.h"
#include "stdlib.h"
#include "oledfont.h"  	 
#include "delay.h"
		  
	

u8 OLED_GRAM[128][8];	 
void OLED_Refresh_Gram(void)
{
	u8 i,n;		    
	for(i=0;i<8;i++)  
	{  
		OLED_WR_Byte (0xb0+i,OLED_CMD);    //ÉèÖÃÒ³µØÖ·£¨0~7£©
		OLED_WR_Byte (0x00,OLED_CMD);      //ÉèÖÃÏÔÊ¾Î»ÖÃ¡ªÁÐµÍµØÖ·
		OLED_WR_Byte (0x10,OLED_CMD);      //ÉèÖÃÏÔÊ¾Î»ÖÃ¡ªÁÐ¸ßµØÖ·   
		for(n=0;n<128;n++)OLED_WR_Byte(OLED_GRAM[n][i],OLED_DATA); 
	}   
}
 /**************************************************************************
º¯Êý¹¦ÄÜ£ºÏòOLEDÐ´ÈëÒ»¸ö×Ö½Ú
Èë¿Ú²ÎÊý£//dat:ÒªÐ´ÈëµÄÊý¾Ý/ÃüÁî	//cmd:Êý¾Ý/ÃüÁî±êÖ¾ 0,±íÊ¾ÃüÁî;1,±íÊ¾Êý¾Ý;		  
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/  
void OLED_WR_Byte(u8 dat,u8 cmd)
{	
	u8 i;			  
	if(cmd)
	  OLED_RS_Set();
	else 
	  OLED_RS_Clr();		  
	for(i=0;i<8;i++)
	{			  
		OLED_SCLK_Clr();
		if(dat&0x80)
		   OLED_SDIN_Set();
		else 
		   OLED_SDIN_Clr();
		OLED_SCLK_Set();
		dat<<=1;   
	}				 		  
	OLED_RS_Set();   	  
} 
 /**************************************************************************
º¯Êý¹¦ÄÜ£º¿ªÆôOLEDÏÔÊ¾  
Èë¿Ú²ÎÊý£º//x,y :Æðµã×ø±ê	//t:1 Ìî³ä 0,Çå¿Õ			  
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/  
void OLED_Display_On(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDCÃüÁî
	OLED_WR_Byte(0X14,OLED_CMD);  //DCDC ON
	OLED_WR_Byte(0XAF,OLED_CMD);  //DISPLAY ON
}
/**************************************************************************
º¯Êý¹¦ÄÜ£º¹Ø±ÕOLEDÏÔÊ¾ 
Èë¿Ú²ÎÊý£º//x,y :Æðµã×ø±ê	//t:1 Ìî³ä 0,Çå¿Õ			  
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/  
void OLED_Display_Off(void)
{
	OLED_WR_Byte(0X8D,OLED_CMD);  //SET DCDCÃüÁî
	OLED_WR_Byte(0X10,OLED_CMD);  //DCDC OFF
	OLED_WR_Byte(0XAE,OLED_CMD);  //DISPLAY OFF
}	
/**************************************************************************
º¯Êý¹¦ÄÜ£ºÇåÆÁº¯Êý,ÇåÍêÆÁ,Õû¸öÆÁÄ»ÊÇºÚÉ«µÄ!ºÍÃ»µãÁÁÒ»Ñù!!!
Èë¿Ú²ÎÊý£º//x,y :Æðµã×ø±ê	//t:1 Ìî³ä 0,Çå¿Õ			  
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/  
void OLED_Clear(void)  
{  
	u8 i,n;  
	for(i=0;i<8;i++)for(n=0;n<128;n++)OLED_GRAM[n][i]=0X00;  
	OLED_Refresh_Gram();//¸üÐÂÏÔÊ¾
}
/**************************************************************************
º¯Êý¹¦ÄÜ£º»­µã 
Èë¿Ú²ÎÊý£º//x,y :Æðµã×ø±ê	//t:1 Ìî³ä 0,Çå¿Õ			  
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/ 
void OLED_DrawPoint(u8 x,u8 y,u8 t)
{
	u8 pos,bx,temp=0;
	if(x>127||y>63)return;//³¬³ö·¶Î§ÁË.
	pos=7-y/8;
	bx=y%8;
	temp=1<<(7-bx);
	if(t)OLED_GRAM[x][pos]|=temp;
	else OLED_GRAM[x][pos]&=~temp;	    
}
/**************************************************************************
º¯Êý¹¦ÄÜ£ºÔÚÖ¸¶¨Î»ÖÃÏÔÊ¾Ò»¸ö×Ö·û,°üÀ¨²¿·Ö×Ö·û
Èë¿Ú²ÎÊý£º//x,y :Æðµã×ø±ê	//len :Êý×ÖµÄÎ»Êý  //size:×ÖÌå´óÐ¡ // mode:0,·´°×ÏÔÊ¾;1,Õý³£ÏÔÊ¾	   
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void OLED_ShowChar(u8 x,u8 y,u8 chr,u8 size,u8 mode)
{      			    
	u8 temp,t,t1;
	u8 y0=y;
	chr=chr-' ';//µÃµ½Æ«ÒÆºóµÄÖµ				   
    for(t=0;t<size;t++)
    {   
		if(size==12)temp=oled_asc2_1206[chr][t];  //µ÷ÓÃ1206×ÖÌå
		else temp=oled_asc2_1608[chr][t];		 //µ÷ÓÃ1608×ÖÌå 	                          
        for(t1=0;t1<8;t1++)
		{
			if(temp&0x80)OLED_DrawPoint(x,y,mode);
			else OLED_DrawPoint(x,y,!mode);
			temp<<=1;
			y++;
			if((y-y0)==size)
			{
				y=y0;
				x++;
				break;
			}
		}  	 
    }          
}
/**************************************************************************
º¯Êý¹¦ÄÜ£ºÇómµÄn´Î·½µÄº¯Êý
Èë¿Ú²ÎÊý£ºm£ºµ×Êý£¬n£º´Î·½Êý
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
u32 oled_pow(u8 m,u8 n)
{
	u32 result=1;	 
	while(n--)result*=m;    
	return result;
}

/**************************************************************************
º¯Êý¹¦ÄÜ£ºÏÔÊ¾2¸öÊý×Ö
Èë¿Ú²ÎÊý£º//x,y :Æðµã×ø±ê	//len :Êý×ÖµÄÎ»Êý  //size:×ÖÌå´óÐ¡  //mode:Ä£Ê½	0,Ìî³äÄ£Ê½;1,µþ¼ÓÄ£Ê½  //num:ÊýÖµ(0~4294967295);	 
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void OLED_ShowNumber(u8 x,u8 y,u32 num,u8 len,u8 size)
{         	
	u8 t,temp;
	u8 enshow=0;						   
	for(t=0;t<len;t++)
	{
		temp=(num/oled_pow(10,len-t-1))%10;
		if(enshow==0&&t<(len-1))
		{
			if(temp==0)
			{
				OLED_ShowChar(x+(size/2)*t,y,' ',size,1);
				continue;
			}else enshow=1; 
		 	 
		}
	 	OLED_ShowChar(x+(size/2)*t,y,temp+'0',size,1); 
	}
} 
/**************************************************************************
º¯Êý¹¦ÄÜ£ºÏÔÊ¾×Ö·û´®
Èë¿Ú²ÎÊý£º//x,y :Æðµã×ø±ê	 // *p:×Ö·û´®ÆðÊ¼µØÖ· 
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/
void OLED_ShowString(u8 x,u8 y,const u8 *p)
{
#define MAX_CHAR_POSX 122
#define MAX_CHAR_POSY 58          
    while(*p!='\0')
    {       
        if(x>MAX_CHAR_POSX){x=0;y+=16;}
        if(y>MAX_CHAR_POSY){y=x=0;OLED_Clear();}
        OLED_ShowChar(x,y,*p,12,1);	 
        x+=8;
        p++;
    }  
}	 
/**************************************************************************
º¯Êý¹¦ÄÜ£º³õÊ¼»¯OLED	
Èë¿Ú²ÎÊý£wÎÞ 
·µ»Ø  Öµ£ºÎÞ
**************************************************************************/	    
void OLED_Init(void)
{ 	
GPIO_InitTypeDef GPIO_InitStructure;
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOE, ENABLE); //Ê¹ÄÜPB¶Ë¿ÚÊ±ÖÓ
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_5|GPIO_Pin_6|GPIO_Pin_7|GPIO_Pin_8;//¶Ë¿ÚÅäÖÃ
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;      //ÍÆÍìÊä³ö
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;      //2M
	GPIO_Init(GPIOE, &GPIO_InitStructure);					      //¸ù¾ÝÉè¶¨²ÎÊý³õÊ¼»¯GPIO


	PWR_BackupAccessCmd(ENABLE);//ÔÊÐíÐÞ¸ÄRTC ºÍºó±¸¼Ä´æÆ÷
	RCC_LSEConfig(RCC_LSE_OFF);//¹Ø±ÕÍâ²¿µÍËÙÍâ²¿Ê±ÖÓÐÅºÅ¹¦ÄÜ ºó£¬PC13 PC14 PC15 ²Å¿ÉÒÔµ±ÆÕÍ¨IOÓÃ¡£
	BKP_TamperPinCmd(DISABLE);//¹Ø±ÕÈëÇÖ¼ì²â¹¦ÄÜ£¬Ò²¾ÍÊÇ PC13£¬Ò²¿ÉÒÔµ±ÆÕÍ¨IO Ê¹ÓÃ
	PWR_BackupAccessCmd(DISABLE);//½ûÖ¹ÐÞ¸Äºó±¸¼Ä´æÆ÷

	OLED_RST_Clr();
	delay_ms(100);
	OLED_RST_Set(); 
				  
	OLED_WR_Byte(0xAE,OLED_CMD); //¹Ø±ÕÏÔÊ¾
	OLED_WR_Byte(0xD5,OLED_CMD); //ÉèÖÃÊ±ÖÓ·ÖÆµÒò×Ó,Õðµ´ÆµÂÊ
	OLED_WR_Byte(80,OLED_CMD);   //[3:0],·ÖÆµÒò×Ó;[7:4],Õðµ´ÆµÂÊ
	OLED_WR_Byte(0xA8,OLED_CMD); //ÉèÖÃÇý¶¯Â·Êý
	OLED_WR_Byte(0X3F,OLED_CMD); //Ä¬ÈÏ0X3F(1/64) 
	OLED_WR_Byte(0xD3,OLED_CMD); //ÉèÖÃÏÔÊ¾Æ«ÒÆ
	OLED_WR_Byte(0X00,OLED_CMD); //Ä¬ÈÏÎª0

	OLED_WR_Byte(0x40,OLED_CMD); //ÉèÖÃÏÔÊ¾¿ªÊ¼ÐÐ [5:0],ÐÐÊý.
													
	OLED_WR_Byte(0x8D,OLED_CMD); //µçºÉ±ÃÉèÖÃ
	OLED_WR_Byte(0x14,OLED_CMD); //bit2£¬¿ªÆô/¹Ø±Õ
	OLED_WR_Byte(0x20,OLED_CMD); //ÉèÖÃÄÚ´æµØÖ·Ä£Ê½
	OLED_WR_Byte(0x02,OLED_CMD); //[1:0],00£¬ÁÐµØÖ·Ä£Ê½;01£¬ÐÐµØÖ·Ä£Ê½;10,Ò³µØÖ·Ä£Ê½;Ä¬ÈÏ10;
	OLED_WR_Byte(0xA1,OLED_CMD); //¶ÎÖØ¶¨ÒåÉèÖÃ,bit0:0,0->0;1,0->127;
	OLED_WR_Byte(0xC0,OLED_CMD); //ÉèÖÃCOMÉ¨Ãè·½Ïò;bit3:0,ÆÕÍ¨Ä£Ê½;1,ÖØ¶¨ÒåÄ£Ê½ COM[N-1]->COM0;N:Çý¶¯Â·Êý
	OLED_WR_Byte(0xDA,OLED_CMD); //ÉèÖÃCOMÓ²¼þÒý½ÅÅäÖÃ
	OLED_WR_Byte(0x12,OLED_CMD); //[5:4]ÅäÖÃ
	 
	OLED_WR_Byte(0x81,OLED_CMD); //¶Ô±È¶ÈÉèÖÃ
	OLED_WR_Byte(0xEF,OLED_CMD); //1~255;Ä¬ÈÏ0X7F (ÁÁ¶ÈÉèÖÃ,Ô½´óÔ½ÁÁ)
	OLED_WR_Byte(0xD9,OLED_CMD); //ÉèÖÃÔ¤³äµçÖÜÆÚ
	OLED_WR_Byte(0xf1,OLED_CMD); //[3:0],PHASE 1;[7:4],PHASE 2;
	OLED_WR_Byte(0xDB,OLED_CMD); //ÉèÖÃVCOMH µçÑ¹±¶ÂÊ
	OLED_WR_Byte(0x30,OLED_CMD); //[6:4] 000,0.65*vcc;001,0.77*vcc;011,0.83*vcc;

	OLED_WR_Byte(0xA4,OLED_CMD); //È«¾ÖÏÔÊ¾¿ªÆô;bit0:1,¿ªÆô;0,¹Ø±Õ;(°×ÆÁ/ºÚÆÁ)
	OLED_WR_Byte(0xA6,OLED_CMD); //ÉèÖÃÏÔÊ¾·½Ê½;bit0:1,·´ÏàÏÔÊ¾;0,Õý³£ÏÔÊ¾	    						   
	OLED_WR_Byte(0xAF,OLED_CMD); //¿ªÆôÏÔÊ¾	 
	OLED_Clear();
}  


/*ÏÔÊ¾ºº×Ö
 *×¢Òâ£ºÓÃÎÒÕâÖÖ·½·¨À´ÏÔÊ¾ºº×ÖÒ»¶¨ÒªÂú×ãÓÃ×ÖÄ£Éú³ÉÈí¼þÉú³ÉµÄ×Ö¿íÓëµãÕó´óÐ¡ÏàÍ¬²ÅÐÐ£¬·ñÕßÈÝÒ×ÂÒÂë
 *x£º±íÊ¾ÏÔÊ¾µÄË®Æ½×ø±ê  
 *y: ±íÊ¾ÏÔÊ¾µÄ´¹Ö±×ø±ê 
 *no: ±íÊ¾ÒªÏÔÊ¾µÄºº×Ö£¨Ä£×é£©ÔÚhzk[][]Êý×éÖÐµÄÐÐºÅ,Í¨¹ýÐÐºÅÀ´È·¶¨ÔÚÊý×éÖÐÒªÏÔÊ¾µÄºº×Ö
 *ÕâÀï×ÖÌåµÄ¿ífont_widthµÄÖµ±ØÐëÓëÓÃ×ÖÄ£ÖÆ×÷Èí¼þÉú³É×ÖÄ£Ê±µÄµãÕóÖµ´óÐ¡Ò»ÖÂ
 *font_heightµÄÖµÎªÓÃ×ÖÄ£ÖÆ×÷Èí¼þÉú³É×ÖÄ£Ê±×ÖÌåµÄ¸ß,ÓÉÓÚÎÒµÄÆÁÏñËØÎª32*128-----0~7¹²8Ò³£¬Ã¿Ò³4¸öÎ»
 */
void OLED_ShowCHinese(u8 x,u8 y,u8 no,u8 font_width,u8 font_height)
{     			    
	 u8 t, i;
   for(i=0;i<(font_height/8);i++)				/*font_height×î´óÖµÎª32£¬ÕâÕÅÆÁÖ»ÓÐ8¸öÒ³£¨ÐÐ£©£¬Ã¿Ò³4¸öÎ»*/
	 {
			OLED_Set_Pos(x,y+i);	
			for(t=0;t<font_width;t++)		/*font_width×î´óÖµÎª128£¬ÆÁÄ»Ö»ÓÐÕâÃ´´ó*/
			{	
					OLED_WR_Byte(Hzk16[(font_height/8)*no+i][t],OLED_DATA);
			}		
	 }
}	 

/*ÉèÖÃºº×ÖÔÚÆÁÄ»ÉÏÏÔÊ¾µÄ×ø±ê£¨Î»ÖÃ£©*/
void OLED_Set_Pos(unsigned char x, unsigned char y)
{ 	
	 OLED_WR_Byte(0xb0+y,OLED_CMD);
	 OLED_WR_Byte(((x&0xf0)>>4)|0x10,OLED_CMD);
	 OLED_WR_Byte((x&0x0f),OLED_CMD); 
} 



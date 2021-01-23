#include "robot_select_init.h"

Robot_Parament_InitTypeDef  Robot_Parament;//初始化机器人参数结构体

/**************************************************************************
函数功能：根据电位器切换需要控制的小车类型
入口参数：无
返回  值：无
**************************************************************************/
void Robot_Select(void)
{
	Divisor_Mode=2048/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //采集电位器引脚信息	
  if(Car_Mode>5)Car_Mode=5;
	//Car_Mode=Mec_Car;
	switch(Car_Mode)
	{
		case Mec_Car:       Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     HALL_30F, Photoelectric_500, Mecanum_75);            break; //麦克纳姆轮小车
		case Omni_Car:      Robot_Init(0,                        0,                        Omni_Turn_Radiaus_109, HALL_30F, Photoelectric_500, FullDirecion_60);       break; //全向轮小车
		case Akm_Car:       Robot_Init(Akm_wheelspacing,         Akm_axlespacing,          0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //阿克曼小车
		case Diff_Car:      Robot_Init(Diff_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //两轮差速小车
		case FourWheel_Car: Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //四驱车 
		case Tank_Car:      Robot_Init(Tank_wheelSpacing,     0,                           0,                     HALL_30F, Photoelectric_500, Tank_WheelDiameter); break; //履带车
	}
	
	//自检相关参数
	switch(Car_Mode)
  {
	 case Mec_Car:       CheckPhrase1=8, CheckPhrase2=14; break; //麦克纳姆轮小车
	 case Omni_Car:      CheckPhrase1=6, CheckPhrase2=10; break; //全向轮小车
	 case Akm_Car:       CheckPhrase1=4, CheckPhrase2=7;  break; //阿克曼小车
	 case Diff_Car:      CheckPhrase1=4, CheckPhrase2=7;  break; //两轮差速小车
	 case FourWheel_Car: CheckPhrase1=8, CheckPhrase2=11; break; //四驱车 
	 case Tank_Car:      CheckPhrase1=4, CheckPhrase2=7;  break; //履带车
  }
}

/**************************************************************************
函数功能：初始化小车参数
入口参数：轮距 轴距 自转半径 电机减速比 电机编码器精度 轮胎直径
返回  值：无
**************************************************************************/
void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, int gearratio,float Accuracy,float tyre_diameter) // 
{
  Robot_Parament.WheelSpacing=wheelspacing;         //半轮距
  Robot_Parament.AxleSpacing=axlespacing;           //半轴距
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus; //全向轮小车旋转半径	
  Robot_Parament.GearRatio=gearratio;               //电机减速比
  Robot_Parament.EncoderAccuracy=Accuracy;          //编码器精度(电机驱动线数)
  Robot_Parament.WheelDiameter=tyre_diameter;       //主动轮轮径
	
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;//编码器精度
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;  //轮子周长
  Wheel_spacing=Robot_Parament.WheelSpacing;        //轮距 麦轮车为半轮距
	Axle_spacing=Robot_Parament.AxleSpacing;          //轴距 麦轮车为半轴距
	Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus; //全向轮小车旋转半径
}



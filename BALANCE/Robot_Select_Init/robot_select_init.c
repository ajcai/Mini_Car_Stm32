#include "robot_select_init.h"

Robot_Parament_InitTypeDef  Robot_Parament;//��ʼ�������˲����ṹ��

/**************************************************************************
�������ܣ����ݵ�λ���л���Ҫ���Ƶ�С������
��ڲ�������
����  ֵ����
**************************************************************************/
void Robot_Select(void)
{
	Divisor_Mode=2048/CAR_NUMBER+5;
	Car_Mode=(int) ((Get_adc_Average(Potentiometer,10))/Divisor_Mode); //�ɼ���λ��������Ϣ	
  if(Car_Mode>5)Car_Mode=5;
	//Car_Mode=Mec_Car;
	switch(Car_Mode)
	{
		case Mec_Car:       Robot_Init(MEC_wheelspacing,         MEC_axlespacing,          0,                     HALL_30F, Photoelectric_500, Mecanum_75);            break; //�����ķ��С��
		case Omni_Car:      Robot_Init(0,                        0,                        Omni_Turn_Radiaus_109, HALL_30F, Photoelectric_500, FullDirecion_60);       break; //ȫ����С��
		case Akm_Car:       Robot_Init(Akm_wheelspacing,         Akm_axlespacing,          0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //������С��
		case Diff_Car:      Robot_Init(Diff_wheelSpacing,        0,                        0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //���ֲ���С��
		case FourWheel_Car: Robot_Init(Four_Mortor_wheelSpacing, Four_Mortor__axlespacing, 0,                     HALL_30F, Photoelectric_500, Black_WheelDiameter);   break; //������ 
		case Tank_Car:      Robot_Init(Tank_wheelSpacing,     0,                           0,                     HALL_30F, Photoelectric_500, Tank_WheelDiameter); break; //�Ĵ���
	}
	
	//�Լ���ز���
	switch(Car_Mode)
  {
	 case Mec_Car:       CheckPhrase1=8, CheckPhrase2=14; break; //�����ķ��С��
	 case Omni_Car:      CheckPhrase1=6, CheckPhrase2=10; break; //ȫ����С��
	 case Akm_Car:       CheckPhrase1=4, CheckPhrase2=7;  break; //������С��
	 case Diff_Car:      CheckPhrase1=4, CheckPhrase2=7;  break; //���ֲ���С��
	 case FourWheel_Car: CheckPhrase1=8, CheckPhrase2=11; break; //������ 
	 case Tank_Car:      CheckPhrase1=4, CheckPhrase2=7;  break; //�Ĵ���
  }
}

/**************************************************************************
�������ܣ���ʼ��С������
��ڲ������־� ��� ��ת�뾶 ������ٱ� ������������� ��ֱ̥��
����  ֵ����
**************************************************************************/
void Robot_Init(float wheelspacing, float axlespacing, float omni_turn_radiaus, int gearratio,float Accuracy,float tyre_diameter) // 
{
  Robot_Parament.WheelSpacing=wheelspacing;         //���־�
  Robot_Parament.AxleSpacing=axlespacing;           //�����
  Robot_Parament.OmniTurnRadiaus=omni_turn_radiaus; //ȫ����С����ת�뾶	
  Robot_Parament.GearRatio=gearratio;               //������ٱ�
  Robot_Parament.EncoderAccuracy=Accuracy;          //����������(�����������)
  Robot_Parament.WheelDiameter=tyre_diameter;       //�������־�
	
	Encoder_precision=EncoderMultiples*Robot_Parament.EncoderAccuracy*Robot_Parament.GearRatio;//����������
	Wheel_perimeter=Robot_Parament.WheelDiameter*PI;  //�����ܳ�
  Wheel_spacing=Robot_Parament.WheelSpacing;        //�־� ���ֳ�Ϊ���־�
	Axle_spacing=Robot_Parament.AxleSpacing;          //��� ���ֳ�Ϊ�����
	Omni_turn_radiaus=Robot_Parament.OmniTurnRadiaus; //ȫ����С����ת�뾶
}



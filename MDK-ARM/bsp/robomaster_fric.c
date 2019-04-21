/**
  ******************************************************************************
  * File Name          : robomaster_fric.c
	* author						 : 周恒
	* Date							 :2019.1.23
  * Description        :摩擦轮驱动
	
  ******************************************************************************
  ******************************************************************************
  */

#include "robomaster_fric.h"
#ifdef INFANTRY_1
int Close_Fric_ON=1100,Mid_Fric_ON=1150,Remote_Fric_ON=1210,Intercontinental_Fric_ON=1270,Fric_OFF=1000;//1100对应于10m/s 1200对应于20m/s 1300对应于30m/s
#endif
#ifdef INFANTRY_2
int Close_Fric_ON=1250,Mid_Fric_ON=1280,Remote_Fric_ON=1330,Intercontinental_Fric_ON=1370,Fric_OFF=1000;//1100对应于10m/s 1200对应于20m/s 1300对应于30m/s
#endif

void init_Fric_PWM()
{	
	TIM_SetTIM5Compare(Fric_OFF);
}

void TIM_SetTIM5Compare(uint16_t compare)
{
	TIM5->CCR3=compare;//摩擦轮
	TIM5->CCR4=compare;
}

void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,int duty)
	{
		switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = duty;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = duty;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = duty;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = duty;break;
	}
	
}
/**********************************************************************************************************
*函 数 名: close_fire
*功能说明: 近程制导 10m/s
*形    参: 
*返 回 值: 
**********************************************************************************************************/	
void close_fire()
{
	fric_ramp.max_value=Close_Fric_ON;
	ramp_calc(&fric_ramp,1000);
	PWM_SetDuty(&htim5,FRIC1,fric_ramp.out);
	PWM_SetDuty(&htim5,FRIC2,fric_ramp.out);
}	
/**********************************************************************************************************
*函 数 名: mid_fire
*功能说明: 中程制导 15m/s
*形    参: 
*返 回 值: 
**********************************************************************************************************/	
void mid_fire()
{
	fric_ramp.max_value=Mid_Fric_ON;
	ramp_calc(&fric_ramp,1000);
	PWM_SetDuty(&htim5,FRIC1,fric_ramp.out);
	PWM_SetDuty(&htim5,FRIC2,fric_ramp.out);
}
/**********************************************************************************************************
*函 数 名: remote_fire
*功能说明: 远程制导 20m/s
*形    参: 
*返 回 值: 
**********************************************************************************************************/
void remote_fire()
{
	fric_ramp.max_value=Remote_Fric_ON;
	ramp_calc(&fric_ramp,1000);
	PWM_SetDuty(&htim5,FRIC1,fric_ramp.out);
	PWM_SetDuty(&htim5,FRIC2,fric_ramp.out);
}

/**********************************************************************************************************
*函 数 名: intercontinental_fire
*功能说明: 洲际制导 28m/s
*形    参: 
*返 回 值: 
**********************************************************************************************************/
void intercontinental_fire()
{
	fric_ramp.max_value=Intercontinental_Fric_ON;
	ramp_calc(&fric_ramp,1000);
	PWM_SetDuty(&htim5,FRIC1,fric_ramp.out);
	PWM_SetDuty(&htim5,FRIC2,fric_ramp.out);
}



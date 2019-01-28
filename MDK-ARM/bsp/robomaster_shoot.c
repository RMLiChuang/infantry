/**
  ******************************************************************************
  * File Name          : robomaster_shoot.c
	* author						 : 周恒
	* Date							 :2019.1.23
  * Description        :拨弹轮驱动
	
  ******************************************************************************
  ******************************************************************************
  */

#include "robomaster_shoot.h"

static Shoot_Motor_t trigger_motor;          //射击数据
static shoot_mode_e shoot_mode = SHOOT_STOP; //射击状态机
/**********************************************************************************************************
*函 数 名: single_shot
*功能说明: 拨弹轮单发
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void single_shoot()
{
//	if(remote_control.switch_right==1)
//	{
//		//moto_chassis[6].round_cnt=0;
//		HeadTxData[4]=0;
//		HeadTxData[5]=0;
//		init_TIM5_PWM();
//	}
	if(remote_control.switch_right==2)
	{
		moto_chassis[6].round_cnt=0;
		HeadTxData[4]=0;
		HeadTxData[5]=0;
		PWM_SetDuty(&htim5,TIM_CHANNEL_1,1500);
		PWM_SetDuty(&htim5,TIM_CHANNEL_2,1500);
		PWM_SetDuty(&htim5,TIM_CHANNEL_3,1500);
		PWM_SetDuty(&htim5,TIM_CHANNEL_4,1500);
	}
	if(remote_control.switch_right==3)
	{
		if(moto_chassis[6].round_cnt<4)
			{
					motor_pid[6].target=2500;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
			}
		else
		{
			HeadTxData[4]=0;
			HeadTxData[5]=0;
		}
			PWM_SetDuty(&htim5,TIM_CHANNEL_1,1500);
			PWM_SetDuty(&htim5,TIM_CHANNEL_2,1500);
			PWM_SetDuty(&htim5,TIM_CHANNEL_3,1500);
			PWM_SetDuty(&htim5,TIM_CHANNEL_4,1500);
	}
}



char BD_zhuangtai=0;
char BD_tuidan=0;
int i,keep;
int pwm_output;
void shoot_control()
{
	if(remote_control.switch_left!=3)
		{
				
				
//					motor_pid[6].target=3000;
//					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
//					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
//					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
					single_shoot();
					
					if(motor_pid[6].err<500&&moto_chassis[6].speed_rpm>600)		//拨弹轮工作状态稳定
					{
						BD_zhuangtai=1;
					}
					if((BD_zhuangtai==1)&&(motor_pid[6].err>400)&&(moto_chassis[6].speed_rpm<600))//拨弹轮由工作稳定到卡弹
					{
						BD_zhuangtai=2;
						BD_tuidan=1;
					}
					if(BD_tuidan==1)
					{
						BD_tuidan=2;
						moto_chassis[6].round_cnt=0;
					}
					if(BD_tuidan==2)
					{
						if(moto_chassis[6].round_cnt>-1)
						{
							motor_pid[6].target=-2500;
							motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
							HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
						}
						else
						{
							BD_tuidan=0;
							BD_zhuangtai=3;
						}
					}
					if(BD_zhuangtai==3)
					{
						keep++;
						if(keep>500)
							{BD_zhuangtai=0;keep=0;}
					}
					
					
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,1800);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,1800);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,1800);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,1800);
				}
				if(remote_control.switch_right==1)
				{
					HeadTxData[4]=0;
					HeadTxData[5]=0;//拨弹轮电流值
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					//set_rammer_current(&hcan1,0);
					init_TIM5_PWM();
				}

		if(remote_control.switch_left==3)
		{
			HeadTxData[4]=0;
			HeadTxData[5]=0;//拨弹轮电流值
			//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
			//set_rammer_current(&hcan1,0);
			init_TIM5_PWM();
		}
}


//单发程序
void single_shoot1(void)
{
	int s=0;
	if(remote_control.switch_left!=1)
	{
			PWM_SetDuty(&htim5,TIM_CHANNEL_1,1.4);
			PWM_SetDuty(&htim5,TIM_CHANNEL_2,1.4);
			PWM_SetDuty(&htim5,TIM_CHANNEL_3,1.4);
			PWM_SetDuty(&htim5,TIM_CHANNEL_4,1.4);
				
		if(remote_control.switch_right==2)//
		{	
			motor_pid[6].target=2200;
			s=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
			if(k&&(!s))
				motor_pid[6].target=0;
			j=0;
		}
		
		else 
			if(remote_control.switch_right==1)//
			{
				motor_pid[6].target=2200;
				s=HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_1);
				if(j&&!s)
				motor_pid[6].target=0;
			}
		
		if(remote_control.switch_right==3)//
		{	
				motor_pid[6].target=0;
				moto_chassis[6].round_cnt=0;
				j=0;
				k=0;
		}
	}
	
	else
	{
		PWM_SetDuty(&htim5,TIM_CHANNEL_1,1.0);
		PWM_SetDuty(&htim5,TIM_CHANNEL_2,1.0);
		PWM_SetDuty(&htim5,TIM_CHANNEL_3,1.0);
		PWM_SetDuty(&htim5,TIM_CHANNEL_4,1.0);
	}
	
		motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
		//motor_pid[7].f_cal_pid(&motor_pid[7],moto_chassis[7].speed_rpm);
		HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
		HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF);
		//HeadTxData[6]=(uint8_t)((motor_pid[7].output>>8)&0xFF);//拨弹电机电流值
		//HeadTxData[7]=(uint8_t)(motor_pid[7].output&0xFF);
		CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
}


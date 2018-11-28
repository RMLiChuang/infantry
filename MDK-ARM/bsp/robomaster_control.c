/**
  *@file robomaster_control.c
  *@date 2018-10-5
  *@author izh20
  *@brief 
  */


#include "robomaster_control.h"
#include "robomaster_common.h"


void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty)
	{
	
	switch(tim_channel){	
		case TIM_CHANNEL_1: tim->Instance->CCR1 = (10000*duty) - 1;break;
		case TIM_CHANNEL_2: tim->Instance->CCR2 = (10000*duty) - 1;break;
		case TIM_CHANNEL_3: tim->Instance->CCR3 = (10000*duty) - 1;break;
		case TIM_CHANNEL_4: tim->Instance->CCR4 = (10000*duty) - 1;break;
	}
	
}

void shoot_control()
{
	if(remote_control.switch_left!=3)
		{
				if(remote_control.switch_right==3)
				{
				  motor_pid[6].target=4000;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					set_rammer_current(&hcan1,motor_pid[6].output);
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.14);
				}
				if(remote_control.switch_right==2)
				{	
					motor_pid[6].target=7000;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					
					set_rammer_current(&hcan1,motor_pid[6].output);
				}
				if(remote_control.switch_right==1)
				{
					set_rammer_current(&hcan1,0);
					init_TIM5_PWM();
				}

		}else
		{
			set_rammer_current(&hcan1,0);
			init_TIM5_PWM();
		}
}

//IMU_Type chassis_imu;
/**********************************************************************************************************
*函 数 名: chassis_control
*功能说明: 底盘控制程序，加入了imu的z轴(yaw轴)的角度换和角速度环控制，底盘加入速度环进行控制
*形    参: 需要yaw轴角度，角速度，电机速度反馈
*返 回 值: 电流输出
**********************************************************************************************************/
extern int16_t moto_ctr[6];
int32_t set_spd = 0;//速度参数
int32_t turn=0;     //转弯
long yaw_flag=0;
extern int cnt1;
extern int cnt2;
char cnt_steer=0;
int yaw_cnt=0;
int chassis_yaw_correct=0;
void chassis_control()
{
	if(remote_control.switch_left!=3)
	{
			if(cnt1==100)//0.5s进入一次，使第4个led以2HZ频率闪烁，判断底盘程序正常运行
			{
					HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_4);
					cnt1=0;
			}
			DBUS_Deal();//获取遥控器的数据并将数据赋值给电机的目标转速
			
			cnt_steer++;
			if(cnt_steer==4)
			{
				steer_control();
				cnt_steer=0;
			}
			
			
			if(remote_control.switch_left==1)	
			{
          /*************************yaw轴控制    begin**********/		
	
					if(remote_control.ch3==0)//偏航杆置于中位
					{
//						if(yaw_cnt<500)//步兵上电后一段时间锁定偏航角，磁力计、陀螺仪融合需要一段时间，这里取500
//						yaw_cnt++;
							if(chassis_yaw.target==0)  //回中时赋角度期望值
							{
								chassis_yaw.target=imu.yaw;
							}
							PID_Control_Yaw(&chassis_yaw,imu.yaw);//该函数解决0度到360度的突变
								//chassis_yaw.f_cal_pid(&chassis_yaw,imu.yaw);    //偏航角度控制
								chassis_yaw_speed.target=chassis_yaw.output;//偏航角速度环期望，来源于偏航角度控制器输出
					}
			
					else//波动偏航方向杆后，只进行内环角速度控制
					{
							chassis_yaw.target=0;//偏航角期望给0,不进行角度控制
							chassis_yaw_speed.target=remote_control.ch3*300/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
					}
		
					/*************************yaw轴控制  end**********/		
			
					motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。
					motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //根据设定值进行PID计算。        速度为反馈值
					motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //根据设定值进行PID计算。
					motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //根据设定值进行PID计算。
	
	

					chassis_yaw.f_cal_pid(&chassis_yaw_speed,-imu.gz);	
	
					set_moto_current(&hcan1,motor_pid[0].output+chassis_yaw_speed.output,   //将PID的计算结果通过CAN发送到电机
																	motor_pid[1].output+chassis_yaw_speed.output,
																	motor_pid[2].output+chassis_yaw_speed.output,
																	motor_pid[3].output+chassis_yaw_speed.output);
			}
			
			
		if(remote_control.switch_left==2)  //遥控器给定数值直接作为速度目标
		{
						motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。
						motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //根据设定值进行PID计算。        速度为反馈值
						motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //根据设定值进行PID计算。
						motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //根据设定值进行PID计算。
						set_moto_current(&hcan1,motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
															motor_pid[1].output,
															motor_pid[2].output,
															motor_pid[3].output);
		}
	
	}
		else
		{
			chassis_yaw.target=imu.yaw;  //上电后标定底盘的初始yaw值，
		  set_moto_current(&hcan1,0,0,0,0);
	  }
}
/******************************************************
                      底盘电机控制2017(上一届)

1, 就算出四种姿态的输出
2，四种姿态的叠加
3，准备CAN的数据输出
4，开始电机控制
__________________________________________________________________
| 电机位置 |电机号| 输出\方向 | 前 | 后 | 左 | 右 | 顺 | 逆 | 停 | 													前			后
``````````````````````````````````````````````````````````````````
|   左前   |0X201 |moto_ctr[0]| >0 | <0 | <0 | >0 | >0 | <0 | =0 |      									>0	|		<0
``````````````````````````````````````````````````````````````````	
|   右前   |0X202 |moto_ctr[1]| <0 | >0 | <0 | >0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````   //2018.10.6 修改 												2018.10.7（经测试，原版没问题）
|   左后   |0X203 |moto_ctr[2]| <0 | >0 | >0 | <0 | >0 | <0 | =0 |												>0  |		<0
``````````````````````````````````````````````````````````````````	
|   右后   |0X204 |moto_ctr[3]| >0 | <0 | >0 | <0 | >0 | <0 | =0 |												<0	|		>0
``````````````````````````````````````````````````````````````````
|  备注:|moto_ctr[0]|=|moto_ctr[1]|=|moto_ctr[2]|=|moto_ctr[3]|  |
``````````````````````````````````````````````````````````````````											
*************************** ***************************/
//static float record=0.0;
//void walk_straight(void){
//	if(moto_ctr[0]>0&&moto_ctr[1]>0&&moto_ctr[2]>0&&moto_ctr[3]>0){
//			record=imu.yaw;
//	}else if(moto_ctr[0]<0&&moto_ctr[1]<0&&moto_ctr[2]<0&&moto_ctr[3]<0){
//	}
//}


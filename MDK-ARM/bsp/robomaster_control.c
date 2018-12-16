/**
  *@file robomaster_control.c
  *@date 2018-10-5
  *@author izh20
  *@brief 
  */


#include "robomaster_control.h"
#include "robomaster_common.h"

#define twist_speed 1000

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
				  //motor_pid[6].target=4000;
					//motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					//set_rammer_current(&hcan1,motor_pid[6].output);
					//set_rammer_current(&hcan1,0);
					HeadTxData[4]=0;
					HeadTxData[5]=0;//拨弹轮电流值
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.16);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.16);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.16);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.16);
				}
				if(remote_control.switch_right==2)
				{	
					motor_pid[6].target=2500;
					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
					//set_rammer_current(&hcan1,motor_pid[6].output);
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					PWM_SetDuty(&htim5,TIM_CHANNEL_1,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_2,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_3,0.14);
					PWM_SetDuty(&htim5,TIM_CHANNEL_4,0.14);
				}
				if(remote_control.switch_right==1)
				{
					HeadTxData[4]=0;
					HeadTxData[5]=0;//拨弹轮电流值
					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
					//set_rammer_current(&hcan1,0);
					init_TIM5_PWM();
				}

		}else
		{
			HeadTxData[4]=0;
			HeadTxData[5]=0;//拨弹轮电流值
			//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
			//set_rammer_current(&hcan1,0);
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
	if(remote_control.switch_right!=3)
	{
//			if(cnt1==100)//0.5s进入一次，使第4个led以2HZ频率闪烁，判断底盘程序正常运行
//			{
//					HAL_GPIO_TogglePin(LED_USER_GPIO_PORT,LED_G_Pin);
//					cnt1=0;
//			}
			Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_G_Pin,1);//设置ledG闪烁频率
			DBUS_Deal();//获取遥控器的数据并将数据赋值给电机的目标转速
			
			cnt_steer++;
			if(cnt_steer==4)
			{
				steer_control();
				cnt_steer=0;
			}
			
			
			if(remote_control.switch_right==2)	
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
	
//					set_moto_current(&hcan1,motor_pid[0].output+chassis_yaw_speed.output,   //将PID的计算结果通过CAN发送到电机
//																	motor_pid[1].output+chassis_yaw_speed.output,
//																	motor_pid[2].output+chassis_yaw_speed.output,
//																	motor_pid[3].output+chassis_yaw_speed.output);
			}
			
			
		if(remote_control.switch_right==1)  //遥控器给定数值直接作为速度目标
		{
			
			
			
			

						motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。
						motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //根据设定值进行PID计算。        速度为反馈值
						motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //根据设定值进行PID计算。
						motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //根据设定值进行PID计算。
						set_moto_current(&hcan1,motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
															motor_pid[1].output,
															motor_pid[2].output,
															motor_pid[3].output);
			//set_moto_current(&hcan1,500,500,0,500);
		}
	
	}
		else
		{
			//chassis_yaw.target=imu.yaw;  //上电后标定底盘的初始yaw值，
		  set_moto_current(&hcan1,0,0,0,0);
	  }
}
/**********************************************************************************************************
*函 数 名: chassis_speed_control
*功能说明: 底盘速度环控制
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void chassis_speed_control()
{
//		DBUS_Deal();//获取遥控器的数据并将数据赋值给电机的目标转速
		motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。
		motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //根据设定值进行PID计算。        速度为反馈值
		motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //根据设定值进行PID计算。
		motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //根据设定值进行PID计算。
	
//		//将底盘电机的速度解码发送
//	MotorTxData[0] = ((motor_pid[0].output>>8)&0xFF);
//	MotorTxData[1] = (motor_pid[0].output&0xFF); 
//	MotorTxData[2] = ((motor_pid[1].output>>8)&0xFF);
//	MotorTxData[3] = (motor_pid[1].output&0xFF); 
//	MotorTxData[4] = ((motor_pid[2].output>>8)&0xFF);
//	MotorTxData[5] = (motor_pid[2].output&0xFF); 
//	MotorTxData[6] = ((motor_pid[3].output>>8)&0xFF);
//	MotorTxData[7] = (motor_pid[3].output&0xFF); 

}
/**********************************************************************************************************
*函 数 名: chassis_current_mix
*功能说明: 底盘电流输出融合
*形    参: 需要速度环电流，位置换电流，功率环电流
*返 回 值: 电流输出
**********************************************************************************************************/
void chassis_current_mix()
{
	MotorTxData[0] = (((motor_pid[0].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[1] = ((motor_pid[0].output+chassis_yaw_angle.output)&0xFF); 
	MotorTxData[2] = (((motor_pid[1].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[3] = ((motor_pid[1].output+chassis_yaw_angle.output)&0xFF); 
	MotorTxData[4] = (((motor_pid[2].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[5] = ((motor_pid[2].output+chassis_yaw_angle.output)&0xFF); 
	MotorTxData[6] = (((motor_pid[3].output+chassis_yaw_angle.output)>>8)&0xFF);
	MotorTxData[7] = ((motor_pid[3].output+chassis_yaw_angle.output)&0xFF); 
}

/**********************************************************************************************************
*函 数 名: set_current_zero
*功能说明: 将电流值设置为0
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void set_current_zero()
{
	char i;
	for(i=0;i<8;i++)
	{
		MotorTxData[i]=0;
		HeadTxData[i]=0;
	}
	CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
	//CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);  //向底盘电机发送给定的电流值
}
/**********************************************************************************************************
*函 数 名: chassis_twist_control
*功能说明: 底盘与云台相结合的扭腰程序 （熟称猫步）
*形    参: 需要yaw轴角度，角速度，电机速度反馈
*返 回 值: 电流输出
**********************************************************************************************************/


void chassis_twist_control()
{
	if(remote_control.switch_left!=3)
	{
		Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_G_Pin,1);//设置ledG闪烁频率
		DBUS_Deal();//获取遥控器的数据并将数据赋值给电机的目标转速
		if(robot_status.mode!=TWIST)
		{
			chassis_yaw_angle.initial=moto_chassis[5].angle;//开启猫步时，记录底盘初始值为云台yaw轴所在机械角度值
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			
			pan_tilt_pitch.initial=moto_chassis[4].angle;
			robot_status.mode=TWIST;//将步兵模式设置为猫步模式
		}
			
			
			chassis_speed_control();//底盘速度环控制
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
			pan_tilt_lock_control(); //云台锁头程序启动
			chassis_current_mix();
			
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
			
		}
	else if(remote_control.switch_left==3)//使步兵底盘与云台之间的角度变成初始化状态
	{
		robot_status.mode=STANDBY;//步兵进入待命状态
//		chassis_yaw_angle.target=chassis_yaw_angle.initial;
//		PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
//		chassis_speed_control();//底盘速度环控制
//		//set_moto_current(&hcan1,chassis_yaw_angle.output,chassis_yaw_angle.output,chassis_yaw_angle.output,chassis_yaw_angle.output);
//		chassis_current_mix();
//		CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
		set_current_zero();
	}
	
}
/**********************************************************************************************************
*函 数 名: chassis_follow_pan_tilt_control
*功能说明: 底盘跟随云台，熟成走直线，视角为云台电机视角，可以轻松实现走直线操作
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
bool follow_flag=0;
void chassis_follow_pan_tilt_control()
{
	
	if(robot_status.mode!=FOLLOW)//步兵从非跟随模式进入跟随模式时，需要记录底盘与云台的绝对角度
		{
			chassis_yaw_angle.initial=moto_chassis[5].angle;//开启底盘跟随时，记录底盘初始值为云台yaw轴所在机械角度值
			robot_status.mode=FOLLOW;//步兵处于底盘跟谁模式
			//chassis_yaw.target=chassis_yaw.initial;
		}
		chassis_speed_control();//底盘速度环控制
		chassis_yaw_angle.target=chassis_yaw_angle.initial;
		PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
		pan_tilt_lock_control(); //云台锁头程序启动
		
		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
		set_moto_current(&hcan1,motor_pid[0].output+chassis_yaw_angle.output,
														motor_pid[1].output+chassis_yaw_angle.output,
														motor_pid[2].output+chassis_yaw_angle.output,
														0);
		
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


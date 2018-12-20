#include "pan_tilt_control.h"
float steer_output;
void steer_control()
{
	
		steer_output=15+(remote_control.ch4*10/660);
		PWM_SetDuty(&htim5,TIM_CHANNEL_1,steer_output/100);
}

/**********************************************************************************************************
*函 数 名: pan_tilt_control
*功能说明: 云台控制程序，
*形    参: 需要yaw轴角度，角速度，电机速度反馈
*返 回 值: 电流输出
**********************************************************************************************************/
int cnt_cat=0;
void pan_tilt_control()
{
	if(remote_control.switch_left==2)
	{
//		
//		/********************步兵相对于底盘控制，反馈信号为电机的机械角度***********************/
//		motor_pid[4].target=remote_control.ch2*1000/660+5440;//修正     pitch轴    步兵云台为yaw
//		motor_pid[4].f_cal_pid(&motor_pid[4],moto_chassis[4].angle);//机械角度环
//		pan_tilt_yaw_speed.target=motor_pid[4].output;
//		motor_pid[4].f_cal_pid(&pan_tilt_yaw_speed,imu.gz);
//		
//		
//		
//		motor_pid[5].target=remote_control.ch4+4900;//修正     yaw轴       步兵云台为pitch
//		motor_pid[5].f_cal_pid(&motor_pid[5],moto_chassis[5].angle);//机械角度环
//		pan_tilt_pithch_speed.target=motor_pid[5].output;
//		motor_pid[5].f_cal_pid(&pan_tilt_pithch_speed,imu.gy);//角速度环
//		set_pan_tilt_current(&hcan1,pan_tilt_yaw_speed.output,pan_tilt_pithch_speed.output);
		
	}
	if(remote_control.switch_left==1)
	{	
		cnt_cat++;
		/********************YAW轴控制***********************/
		if(remote_control.ch2==0)//偏航杆置于中位
		{
			if(pan_tilt_yaw.target==0)  //回中时赋角度期望值
			{
				pan_tilt_yaw.target=imu.yaw;
				
			}
			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
			//motor_pid[5].f_cal_pid(&motor_pid[5],imu.yaw);//imu角度环
			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
	  }
		else
		{
			pan_tilt_yaw.target=0;////偏航角期望给0,不进行角度控制
			pan_tilt_yaw_speed.target=remote_control.ch2*300/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
		}
		
		pan_tilt_yaw.f_cal_pid(&pan_tilt_yaw_speed,imu.gz);
		
		/*************************PITCH轴控制**************************/		
		
		
		if(remote_control.ch3==0)//偏航杆置于中位
		{
			if(motor_pid[4].target==0)  //回中时赋角度期望值
			{
				motor_pid[4].target=moto_chassis[4].angle;
				
			}
			PID_Control_Yaw(&motor_pid[4],moto_chassis[4].angle);
			pan_tilt_pitch_speed.target=motor_pid[4].output;
	  }
		else
		{
			motor_pid[4].target=0;////偏航角期望给0,不进行角度控制
			pan_tilt_pitch_speed.target=remote_control.ch3*300/660;//pitch_control;//偏航角速度环期望，直接来源于遥控器打杆量
		}
		
		
		
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gx);//角速度环
		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
		
		if(cnt_cat<200)
			set_moto_current(&hcan1,1000,1000,1000,1000);
		if(cnt_cat>200&&cnt_cat<400)
		{
			set_moto_current(&hcan1,-1000,-1000,-1000,-1000);
			
		}
		else if(cnt_cat>400)
			cnt_cat=0;
	}		
	
	else
	{
		set_pan_tilt_current(&hcan1,0,0);
		set_moto_current(&hcan1,0,0,0,0);
	}
}

/**********************************************************************************************************
*函 数 名: pan_tilt_machine_home
*功能说明: 云台yaw轴和pitch轴机械归中，用于程序初始化
*形    参: 需要yaw轴imu的角度，角速度(imu.yaw,imu.gz)
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_machine_home(void)
{
	if(remote_control.switch_left!=3)
	{
		pan_tilt_yaw_mechanical_angle_control();
		pan_tilt_pitch_mechanical_angle_control();
		if(remote_control.switch_right!=3)
		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
		else
			set_pan_tilt_current(&hcan1,0,0);
		//set_pan_tilt_current(&hcan1,0,pan_tilt_yaw_speed.output);
	}
	else
		set_pan_tilt_current(&hcan1,0,0);
}

/**********************************************************************************************************
*函 数 名: pan_tilt_lock_control
*功能说明: 云台yaw轴绝对位置锁定,熟称锁头
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_lock_control()
{
	pan_tilt_yaw_imu_angle_control();
	pan_tilt_pitch_imu_angle_control();
	
	HeadTxData[0]=(uint8_t)((pan_tilt_pitch_speed.output>>8)&0xFF);
  HeadTxData[1]=(uint8_t)(pan_tilt_pitch_speed.output&0xFF);           //205   pitch
	  
	HeadTxData[2]=(uint8_t)((pan_tilt_yaw_speed.output>>8)&0xFF);
  HeadTxData[3]=(uint8_t)(pan_tilt_yaw_speed.output&0xFF);           //206  yaw 
	if(remote_control.switch_left!=3)
	{
		CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
	}
	else if(remote_control.switch_left==3)
	{
		HeadTxData[0]=0;
		HeadTxData[1]=0;           //205   pitch
		HeadTxData[2]=0;
		HeadTxData[3]=0;           //206  yaw 
		CAN_Send_Msg(&hcan1,0,HEADID,8);
	}
	
	//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
}


/**********************************************************************************************************
*函 数 名: pan_tilt_yaw_imu_angle_control
*功能说明: 云台yaw轴控制，需要imu的yaw轴角度作为反馈值
*形    参: 需要yaw轴imu的角度，角速度(imu.yaw,imu.gz)
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_yaw_imu_angle_control(void)
{	
	if(remote_control.switch_left==2)
	{
	if(remote_control.ch3==0)//偏航杆置于中位
		{
			if(pan_tilt_yaw.target==0)  //回中时赋角度期望值
			{
				pan_tilt_yaw.target=imu.yaw;
				
			}
			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
	  }
	
		else
		{
			pan_tilt_yaw.target=0;////偏航角期望给0,不进行角度控制
			pan_tilt_yaw_speed.target=remote_control.ch3*120/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
		}
	}
	else if(remote_control.switch_left==1)
	{
	if(remote_control.ch4==0)//偏航杆置于中位
		{
			if(pan_tilt_yaw.target==0)  //回中时赋角度期望值
			{
				pan_tilt_yaw.target=imu.yaw;
				
			}
			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
	  }
	
		else
		{
			pan_tilt_yaw.target=0;////偏航角期望给0,不进行角度控制
			pan_tilt_yaw_speed.target=remote_control.ch4*120/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
		}
	}
	
		
		pan_tilt_yaw.f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
		
}
/**********************************************************************************************************
*函 数 名: pan_tilt_yaw_mechanical_angle_control
*功能说明: 云台yaw轴控制，需要电机编码器的角度作为反馈值
*形    参: 需要yaw轴电机编码器的角度，imu的角速度（moto_chassis[4].angle,imu.gz）
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_yaw_mechanical_angle_control(void)
{
	/*************************测试用**************************/
//	if(remote_control.ch2==0)//偏航杆置于中位
//		{
//			if(motor_pid[5].target==0)  //回中时赋角度期望值
//			{
//				motor_pid[5].target=moto_chassis[4].angle;
//				
//			}
//			PID_Control_Pitch(&motor_pid[5],moto_chassis[4].angle);
//			pan_tilt_yaw_speed.target=motor_pid[5].output;
//	  }
//		else
//		{
//			motor_pid[5].target=0;////偏航角期望给0,不进行角度控制
//			pan_tilt_yaw_speed.target=remote_control.ch2*300/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
//		}
	
	/*************************机械归中用***********************/
	if(motor_pid[5].target==0)  //回中时赋角度期望值
	{
		motor_pid[5].target=moto_chassis[5].angle;
	}
		PID_Control_Yaw(&motor_pid[5],moto_chassis[5].angle);
		pan_tilt_yaw_speed.target=-motor_pid[5].output;
	/**********************************************************/
		motor_pid[5].f_cal_pid(&pan_tilt_yaw_speed,-imu.gz);
}



/**********************************************************************************************************
*函 数 名: pan_tilt_pitch_imu_angle_control
*功能说明: 云台pitch轴控制，需要imu的pitch轴角度作为反馈值
*形    参: 需要yaw轴imu的角度，角速度(imu.pitch,imu.gx)
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_pitch_imu_angle_control(void)
{
//	if(remote_control.ch3==0)//偏航杆置于中位
//		{
//			if(pan_tilt_pitch.target==0)  //回中时赋角度期望值
//			{
//				pan_tilt_pitch.target=imu.pit;
//			}
//			PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
//			pan_tilt_pitch_speed.target=pan_tilt_pitch.output;
//	  }
//		else
//		{
//			pan_tilt_pitch.target=0;////偏航角期望给0,不进行角度控制
//			pan_tilt_pitch_speed.target=remote_control.ch3*300/660;//pitch_control;//偏航角速度环期望，直接来源于遥控器打杆量
//		}
		pan_tilt_pitch.target=185;//pan_tilt_pitch.initial;
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
		pan_tilt_pitch_speed.target=-pan_tilt_pitch.output;
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//角速度环
		pan_tilt_pitch_speed.output=GildeAverageValueFilter(pan_tilt_pitch_speed.output,Data);
}
/**********************************************************************************************************
*函 数 名: pan_tilt_pitch_mechanical_angle_control
*功能说明: 云台pitch轴控制，需要电机编码器的角度作为反馈值
*形    参: 需要yaw轴imu的角度，角速度(moto_chassis[4].angle,imu.gx)
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_pitch_mechanical_angle_control(void)
{
	/*************************测试用**************************/
//	if(remote_control.ch3==0)//偏航杆置于中位
//		{
//			if(motor_pid[4].target==0)  //回中时赋角度期望值
//			{
//				motor_pid[4].target=moto_chassis[4].angle;
//			}
//			PID_Control_Pitch(&motor_pid[4],moto_chassis[4].angle);
//			pan_tilt_pitch_speed.target=motor_pid[4].output;
//	  }
//		else
//		{
//			motor_pid[4].target=0;////偏航角期望给0,不进行角度控制
//			pan_tilt_pitch_speed.target=remote_control.ch3*300/660;//pitch_control;//偏航角速度环期望，直接来源于遥控器打杆量
//		}
	/*************************机械归中用***********************/
	  motor_pid[4].target=3310;
		PID_Control_Pitch(&motor_pid[4],moto_chassis[4].angle);
		pan_tilt_pitch_speed.target=-motor_pid[4].output;
		
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//角速度环
		
}


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
		/********************步兵相对于地面控制，反馈信号为电机的机械角度***********************/
		if(remote_control.ch3==0)//偏航杆置于中位
		{
			if(motor_pid[5].target==0)  //回中时赋角度期望值
			{
				motor_pid[5].target=imu.yaw;
				
			}
			PID_Control_Yaw(&motor_pid[5],imu.yaw);
			//motor_pid[5].f_cal_pid(&motor_pid[5],imu.yaw);//imu角度环
			pan_tilt_yaw_speed.target=motor_pid[5].output;
	  }
		else
		{
			motor_pid[5].target=0;////偏航角期望给0,不进行角度控制
			pan_tilt_yaw_speed.target=remote_control.ch3*300/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
		}
		
		motor_pid[5].f_cal_pid(&pan_tilt_yaw_speed,imu.gz);
		
		
		
		motor_pid[4].target=remote_control.ch4+4900;//修正     yaw轴       步兵云台为pitch
		motor_pid[4].f_cal_pid(&motor_pid[4],moto_chassis[4].angle);//机械角度环
		pan_tilt_pithch_speed.target=motor_pid[4].output;
		motor_pid[4].f_cal_pid(&pan_tilt_pithch_speed,imu.gy);//角速度环
		set_pan_tilt_current(&hcan1,pan_tilt_pithch_speed.output,pan_tilt_yaw_speed.output);
	}		
	
	else
		set_pan_tilt_current(&hcan1,0,0);
}
/**
  *@file robomaster_control.c
  *@date 2019-1-6
  *@author izh20
  *@brief 
  */


#include "robomaster_control.h"
#include "robomaster_common.h"

ramp_function_source_t chassis_ramp;
#define CHASSIS_CONTROL_TIME	0.005f
//#define twist_speed        1000
#define chassis_limit      1000       //走猫步时的底盘限位机械角度
#define chassis_dead_band  10        //底盘机械角度的死区
#define twist_dead_band    100				//用于猫步中识别遥控器操作时底盘与云台位置的误差
//#define PI 3.1415926
#define CHASSIS_MID_VALUE 4190//云台位于底盘中间时，yaw轴电机机械角度为4190
#define MAX_ANGEL 45//云台相对于底盘转动的最大角度值

	
//char BD_zhuangtai=0;
//char BD_tuidan=0;
//int i,keep;
//int pwm_output;
//void shoot_control()
//{
//	if(remote_control.switch_left!=3)
//		{
//				
//				
////					motor_pid[6].target=3000;
////					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
////					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
////					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
//				single_shot();
//					
//					if(motor_pid[6].err<500&&moto_chassis[6].speed_rpm>600)		//拨弹轮工作状态稳定
//					{
//						BD_zhuangtai=1;
//					}
//					if((BD_zhuangtai==1)&&(motor_pid[6].err>400)&&(moto_chassis[6].speed_rpm<600))//拨弹轮由工作稳定到卡弹
//					{
//						BD_zhuangtai=2;
//						BD_tuidan=1;
//					}
//					if(BD_tuidan==1)
//					{
//						BD_tuidan=2;
//						moto_chassis[6].round_cnt=0;
//					}
//					if(BD_tuidan==2)
//					{
//						if(moto_chassis[6].round_cnt>-1)
//						{
//							motor_pid[6].target=-2500;
//							motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
//							HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
//					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
//						}
//						else
//						{
//							BD_tuidan=0;
//							BD_zhuangtai=3;
//						}
//					}
//					if(BD_zhuangtai==3)
//					{
//						keep++;
//						if(keep>500)
//							{BD_zhuangtai=0;keep=0;}
//					}
//					
//					
//					PWM_SetDuty(&htim5,TIM_CHANNEL_1,1800);
//					PWM_SetDuty(&htim5,TIM_CHANNEL_2,1800);
//					PWM_SetDuty(&htim5,TIM_CHANNEL_3,1800);
//					PWM_SetDuty(&htim5,TIM_CHANNEL_4,1800);
//				}
//				if(remote_control.switch_right==1)
//				{
//					HeadTxData[4]=0;
//					HeadTxData[5]=0;//拨弹轮电流值
//					//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//					//set_rammer_current(&hcan1,0);
//					init_TIM5_PWM();
//				}

//		if(remote_control.switch_left==3)
//		{
//			HeadTxData[4]=0;
//			HeadTxData[5]=0;//拨弹轮电流值
//			//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
//			//set_rammer_current(&hcan1,0);
//			init_TIM5_PWM();
//		}
//}

///**********************************************************************************************************
//*函 数 名: single_shot
//*功能说明: 拨弹轮单发
//*形    参: 
//*返 回 值: 电流输出
//**********************************************************************************************************/
//void single_shot()
//{
////	if(remote_control.switch_right==1)
////	{
////		//moto_chassis[6].round_cnt=0;
////		HeadTxData[4]=0;
////		HeadTxData[5]=0;
////		init_TIM5_PWM();
////	}
//	if(remote_control.switch_right==2)
//	{
//		moto_chassis[6].round_cnt=0;
//		HeadTxData[4]=0;
//		HeadTxData[5]=0;
//		PWM_SetDuty(&htim5,TIM_CHANNEL_1,1500);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_2,1500);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_3,1500);
//		PWM_SetDuty(&htim5,TIM_CHANNEL_4,1500);
//	}
//	if(remote_control.switch_right==3)
//	{
//		if(moto_chassis[6].round_cnt<4)
//			{
//					motor_pid[6].target=2500;
//					motor_pid[6].f_cal_pid(&motor_pid[6],moto_chassis[6].speed_rpm);
//					HeadTxData[4]=(uint8_t)((motor_pid[6].output>>8)&0xFF);//拨弹电机电流值
//					HeadTxData[5]=(uint8_t)(motor_pid[6].output&0xFF); 
//			}
//		else
//		{
//			HeadTxData[4]=0;
//			HeadTxData[5]=0;
//		}
//			PWM_SetDuty(&htim5,TIM_CHANNEL_1,1500);
//			PWM_SetDuty(&htim5,TIM_CHANNEL_2,1500);
//			PWM_SetDuty(&htim5,TIM_CHANNEL_3,1500);
//			PWM_SetDuty(&htim5,TIM_CHANNEL_4,1500);
//	}
//}


//IMU_Type chassis_imu;
///**********************************************************************************************************
//*函 数 名: chassis_control
//*功能说明: 底盘控制程序，加入了imu的z轴(yaw轴)的角度换和角速度环控制，底盘加入速度环进行控制
//*形    参: 需要yaw轴角度，角速度，电机速度反馈
//*返 回 值: 电流输出
//**********************************************************************************************************/
//extern int16_t moto_ctr[6];
//int32_t set_spd = 0;//速度参数
//int32_t turn=0;     //转弯
//long yaw_flag=0;
//extern int cnt1;
//extern int cnt2;
//char cnt_steer=0;
//int yaw_cnt=0;
//int chassis_yaw_correct=0;
//void chassis_control()
//{
//	if(remote_control.switch_right!=3)
//	{
////			if(cnt1==100)//0.5s进入一次，使第4个led以2HZ频率闪烁，判断底盘程序正常运行
////			{
////					HAL_GPIO_TogglePin(LED_USER_GPIO_PORT,LED_G_Pin);
////					cnt1=0;
////			}
//			Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_G_Pin,1);//设置ledG闪烁频率
//			DBUS_Deal();//获取遥控器的数据并将数据赋值给电机的目标转速
//			
//			cnt_steer++;
//			if(cnt_steer==4)
//			{
//				steer_control();
//				cnt_steer=0;
//			}
//			
//			
//			if(remote_control.switch_right==2)	
//			{
//          /*************************yaw轴控制    begin**********/		
//	
//					if(remote_control.ch3==0)//偏航杆置于中位
//					{
////						if(yaw_cnt<500)//步兵上电后一段时间锁定偏航角，磁力计、陀螺仪融合需要一段时间，这里取500
////						yaw_cnt++;
//							if(chassis_yaw.target==0)  //回中时赋角度期望值
//							{
//								chassis_yaw.target=imu.yaw;
//							}
//							PID_Control_Yaw(&chassis_yaw,imu.yaw);//该函数解决0度到360度的突变
//								//chassis_yaw.f_cal_pid(&chassis_yaw,imu.yaw);    //偏航角度控制
//								chassis_yaw_speed.target=chassis_yaw.output;//偏航角速度环期望，来源于偏航角度控制器输出
//					}
//			
//					else//波动偏航方向杆后，只进行内环角速度控制
//					{
//							chassis_yaw.target=0;//偏航角期望给0,不进行角度控制
//							chassis_yaw_speed.target=remote_control.ch3*300/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
//					}
//		
//					/*************************yaw轴控制  end**********/		
//			
//					motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。
//					motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //根据设定值进行PID计算。        速度为反馈值
//					motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //根据设定值进行PID计算。
//					motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //根据设定值进行PID计算。
//	
//	

//					chassis_yaw.f_cal_pid(&chassis_yaw_speed,-imu.gz);	
//	
////					set_moto_current(&hcan1,motor_pid[0].output+chassis_yaw_speed.output,   //将PID的计算结果通过CAN发送到电机
////																	motor_pid[1].output+chassis_yaw_speed.output,
////																	motor_pid[2].output+chassis_yaw_speed.output,
////																	motor_pid[3].output+chassis_yaw_speed.output);
//			}
//			
//			
//		if(remote_control.switch_right==1)  //遥控器给定数值直接作为速度目标
//		{
//			
//			
//			
//			

//						motor_pid[0].f_cal_pid(&motor_pid[0],moto_chassis[0].speed_rpm);    //根据设定值进行PID计算。
//						motor_pid[1].f_cal_pid(&motor_pid[1],moto_chassis[1].speed_rpm);    //根据设定值进行PID计算。        速度为反馈值
//						motor_pid[2].f_cal_pid(&motor_pid[2],moto_chassis[2].speed_rpm);    //根据设定值进行PID计算。
//						motor_pid[3].f_cal_pid(&motor_pid[3],moto_chassis[3].speed_rpm);    //根据设定值进行PID计算。
//						set_moto_current(&hcan1,motor_pid[0].output,   //将PID的计算结果通过CAN发送到电机
//															motor_pid[1].output,
//															motor_pid[2].output,
//															motor_pid[3].output);
//			//set_moto_current(&hcan1,500,500,0,500);
//		}
//	
//	}
//		else
//		{
//			//chassis_yaw.target=imu.yaw;  //上电后标定底盘的初始yaw值，
//		  set_moto_current(&hcan1,0,0,0,0);
//	  }
//}

/**********************************************************************************************************
*函 数 名: chassis_init
*功能说明: 底盘斜坡函数初始化，用于猫步控制
*形    参: 
*返 回 值: 
**********************************************************************************************************/
void chassis_init()
{
	ramp_init(&chassis_ramp, CHASSIS_CONTROL_TIME, 3500, 5500);//3500和5500为扭腰时yaw电机编码器极限位置
}
/**********************************************************************************************************
*函 数 名: get_chassis_to_pan_tilt_rad
*功能说明: 获取底盘相对于云台相对角度（）
*形    参: 
*返 回 值: 弧度
**********************************************************************************************************/
float pan_tilt_rad;
void get_chassis_to_pan_tilt_rad()
{
	pan_tilt_rad=(chassis_yaw_angle.initial-pan_tilt_yaw_motor.angle)/8192*360/PI;
}
/**********************************************************************************************************
*函 数 名: get_chassis_to_pan_tilt_rad
*功能说明: 获取底盘相对于云台相对角度（）
*形    参: 
*返 回 值: 角度
**********************************************************************************************************/
float pan_tilt_angle;
void get_chassis_to_pan_tilt_angle()
{
	pan_tilt_angle=(chassis_yaw_angle.initial-pan_tilt_yaw_motor.angle)/8192*360;
	if(int_abs(pan_tilt_angle)>MAX_ANGEL)
	{
		robot_status.chassis_control=OUT_OF_CONTROL;
	}
	else 
	{
		robot_status.chassis_control=CONTROL;
	}
}
/**********************************************************************************************************
*函 数 名: GildeAverageValueFilter
*功能说明: 底盘速度环控制
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
float Data[N];
float GildeAverageValueFilter(float NewValue,float *Data)
{
  float max,min;
  float sum;
  unsigned char i;
  Data[0]=NewValue;
  max=Data[0];
  min=Data[0];
  sum=Data[0];
  for(i=N-1;i!=0;i--)
  {
    if(Data[i]>max) max=Data[i];
    else if(Data[i]<min) min=Data[i];
    sum+=Data[i];
    Data[i]=Data[i-1];
  }
  i=N-2;
  sum=sum-max-min;
  sum=sum/i;
  return(sum);
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
void chassis_current_mix(int16_t *output)
{
//	MotorTxData[0] = (((motor_pid[0].output+chassis_yaw_angle.output)>>8)&0xFF);
//	MotorTxData[1] = ((motor_pid[0].output+chassis_yaw_angle.output)&0xFF); 
//	MotorTxData[2] = (((motor_pid[1].output+chassis_yaw_angle.output)>>8)&0xFF);
//	MotorTxData[3] = ((motor_pid[1].output+chassis_yaw_angle.output)&0xFF); 
//	MotorTxData[4] = (((motor_pid[2].output+chassis_yaw_angle.output)>>8)&0xFF);
//	MotorTxData[5] = ((motor_pid[2].output+chassis_yaw_angle.output)&0xFF); 
//	MotorTxData[6] = (((motor_pid[3].output+chassis_yaw_angle.output)>>8)&0xFF);
//	MotorTxData[7] = ((motor_pid[3].output+chassis_yaw_angle.output)&0xFF); 
	
	MotorTxData[0] = output[0]>>8&0xFF;
	MotorTxData[1] = output[0]&0xFF;
	MotorTxData[2] = output[1]>>8&0xFF;
	MotorTxData[3] = output[1]&0xFF;
	MotorTxData[4] = output[2]>>8&0xFF;
	MotorTxData[5] = output[2]&0xFF;
	MotorTxData[6] = output[3]>>8&0xFF;
	MotorTxData[7] = output[3]&0xFF;
	
	
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
	CAN_Send_Msg(&hcan1, HeadTxData, HEADID, 8);  //向底盘电机发送给定的电流值
}
/**********************************************************************************************************
*函 数 名: chassis_twist_control
*功能说明: 底盘与云台相结合的扭腰程序 （熟称猫步）
*形    参: 需要yaw轴角度，角速度，电机速度反馈
*返 回 值: 电流输出
**********************************************************************************************************/
bool chassis_position_flag=0;
void chassis_twist_control()  //在进行猫步过程中，需要屏蔽遥控器的输出
{
		int16_t output[4]={0};//用于超级电容速度限制
		DBUS_Deal();
		if(robot_status.mode!=TWIST)
		{
			//Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_B_Pin,1);//设置ledB闪烁频率
			pan_tilt_lock_control(); //云台锁头程序启动
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
			
			//chassis_current_mix();
			output[0]=(motor_pid[0].output+chassis_yaw_angle.output);
			output[1]=(motor_pid[1].output+chassis_yaw_angle.output);
			output[2]=(motor_pid[2].output+chassis_yaw_angle.output);
			output[3]=(motor_pid[3].output+chassis_yaw_angle.output);
			Super_Cap_control(output);//电容闭环		
			chassis_current_mix(output);
			
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
			if((moto_chassis[5].angle<(chassis_yaw_angle.initial+chassis_dead_band))&&(moto_chassis[5].angle>(chassis_yaw_angle.initial-chassis_dead_band)))
			robot_status.mode=TWIST;//将步兵模式设置为猫步模式
		}
		
		
		if(robot_status.mode==TWIST)
		{	
			//Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_B_Pin,1);//设置ledB闪烁频率
				chassis_speed_control();//底盘速度环控制
				if(//(remote_control.ch2+remote_control.ch1)==0||
					(moto_chassis[5].angle>(chassis_yaw_angle.initial+twist_dead_band))||
					(moto_chassis[5].angle<(chassis_yaw_angle.initial-twist_dead_band)))//当遥感没有前后左右运动或者底盘与云台有一定位置差，进行猫步
				{
					if(chassis_position_flag==0)//正转
					{
						if(robot_status.chassis_control==CONTROL)
						{
							chassis_yaw_angle.target=(chassis_yaw_angle.initial+chassis_limit);//4500+1000 ramp_calc(chassis_ramp,(chassis_yaw_angle.initial+chassis_limit))
							PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
//						chassis_yaw_speed.target=chassis_yaw_angle.output;
//						PID_Control_Yaw(&chassis_yaw_speed,-imu_9250.gz);
							if((moto_chassis[5].angle<(chassis_yaw_angle.target+chassis_dead_band))&&   //4500  3500+100 5500-100 
								(moto_chassis[5].angle>(chassis_yaw_angle.target-chassis_dead_band)))//当yaw轴电机机械角度接近目标值时，切换底盘旋转方向
								chassis_position_flag=1;
						}
						if(robot_status.chassis_control==OUT_OF_CONTROL)
						{
							chassis_yaw_angle.output=0;
						}
//						if(moto_chassis[5].angle>(chassis_yaw_angle.target))//当yaw轴电机机械角度超过目标值时
//							chassis_position_flag=1;//进行反转处理
				
					}
					if(chassis_position_flag==1)//反转
					{
						if(robot_status.chassis_control==CONTROL)
						{
							chassis_yaw_angle.target=(chassis_yaw_angle.initial-chassis_limit);//ramp_calc(chassis_ramp,(chassis_yaw_angle.initial+chassis_limit))
							PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
//						chassis_yaw_speed.target=chassis_yaw_angle.output;
//						PID_Control_Yaw(&chassis_yaw_speed,-imu_9250.gz);
						
							if((moto_chassis[5].angle<(chassis_yaw_angle.target+chassis_dead_band))&&
								(moto_chassis[5].angle>(chassis_yaw_angle.target-chassis_dead_band)))
								chassis_position_flag=0;
						}
						if(robot_status.chassis_control==OUT_OF_CONTROL)
						{
							chassis_yaw_angle.output=0;
						}
//						if(moto_chassis[5].angle<(chassis_yaw_angle.target))//当yaw轴电机机械角度超过目标值时
//							chassis_position_flag=0;//进行反转处理
					}
				}
//			
				
			pan_tilt_lock_control(); //云台锁头程序启动
			
			output[0]=(motor_pid[0].output+chassis_yaw_angle.output);
			output[1]=(motor_pid[1].output+chassis_yaw_angle.output);
			output[2]=(motor_pid[2].output+chassis_yaw_angle.output);
			output[3]=(motor_pid[3].output+chassis_yaw_angle.output);
			Super_Cap_control(output);//电容闭环		
			chassis_current_mix(output);
				
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
		}
}
/**********************************************************************************************************
*函 数 名: chassis_angle_speed_control
*功能说明: 底盘与角度与角速度闭环
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void chassis_angle_speed_control()
{
		chassis_yaw_angle.target=chassis_yaw_angle.initial;
		PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
		chassis_yaw_speed.target=chassis_yaw_angle.output;
		//chassis_yaw_speed.target=remote_control.ch3*chassis_pan_tilt_max_rotate_speed/660;
		PID_Control_Yaw(&chassis_yaw_speed,-imu_9250.gz);
}
/**********************************************************************************************************
*函 数 名: infantry_control
*功能说明: 底盘与云台相结合的控制程序 
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/

bool calibrate_flag=0;
void infantry_control()
{
	if(calibrate_flag==0)//初始化云台机械结构位置
	{
		calibrate_initial_position();
		calibrate_flag=1;
	}
	if(robot_status.control_mode==REMOTE_CONTROL)//遥控器控制模式
	{
		if(remote_control.switch_left==2)//底盘跟随云台模式
		{
			reset_camera();
			chassis_follow_pan_tilt_control();		
		}
		else if(remote_control.switch_left==1)//猫步模式
		{
			chassis_twist_control();
		}
		else if(remote_control.switch_left==3)//使步兵底盘与云台之间的角度变成初始化状态
		{
			detect_cartridge();
			robot_status.mode=STANDBY;//步兵进入待命状态
			set_current_zero();//屏蔽掉所有电机
		}
	}
	else if(robot_status.control_mode==KEYBOARD_CONTROL)
	{
		Keyboard_Init();
//		if(kb.c_flag==1)
//		{
//			chassis_twist_control();
//		}
//		else if(kb.c_flag!=1)
//		{
			
			chassis_follow_pan_tilt_control();
		//}
	}
}
/**********************************************************************************************************
*函 数 名: chassis_follow_pan_tilt_control
*功能说明: 底盘跟随云台，熟成走直线，视角为云台电机视角，可以轻松实现走直线操作
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
//bool follow_flag=0;
void chassis_follow_pan_tilt_control()
{
		int16_t output[4]={0};//用于超级电容速度限制
		//Bling_Set(&Light_G,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_A_Pin,1);//设置ledA闪烁频率
		DBUS_Deal();//获取遥控器的数据并将数据赋值给电机的目标转速
		if(robot_status.mode!=FOLLOW)
		{
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			
			//pan_tilt_pitch.initial=moto_chassis[4].angle;
			robot_status.mode=FOLLOW;//将步兵模式设置为底盘跟随云台模式
		}
		if(robot_status.mode==FOLLOW)
		{	
			
			chassis_speed_control();//底盘速度环控制
//			if(remote_control.ch3!=0)//但转弯的时候，屏蔽底盘速度环
//			{
//				motor_pid[0].output=0;
//				motor_pid[1].output=0;
//				motor_pid[2].output=0;
//				motor_pid[3].output=0;
//			}
			//底盘角度和角速度环
			chassis_yaw_angle.target=chassis_yaw_angle.initial;
			PID_Control_Yaw(&chassis_yaw_angle,moto_chassis[5].angle);//yaw电机编码器获得的角度作为反馈值
			//chassis_yaw_speed.target=chassis_yaw_angle.output;
				//chassis_yaw_speed.target=remote_control.ch3*chassis_pan_tilt_max_rotate_speed/660;
			//PID_Control_Yaw(&chassis_yaw_speed,-imu_9250.gz);
			
			pan_tilt_lock_control(); //云台锁头程序启动
			
			output[0]=(motor_pid[0].output+chassis_yaw_angle.output);
			output[1]=(motor_pid[1].output+chassis_yaw_angle.output);
			output[2]=(motor_pid[2].output+chassis_yaw_angle.output);
			output[3]=(motor_pid[3].output+chassis_yaw_angle.output);
			Super_Cap_control(output);//电容闭环
			
			chassis_current_mix(output);
			CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
		}
}

/**********************************************************************************************************
*函 数 名: calibrate_initial_position
*功能说明: 标定初始化的云台与底盘位置，用于底盘回正
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void calibrate_initial_position() //yaw轴机械角度中间值为4190
{
	//chassis_yaw_angle.initial=moto_chassis[5].angle;//记录上电时刻底盘相对云台的初始位置
	chassis_yaw_angle.initial=CHASSIS_MID_VALUE;
}
/**********************************************************************************************************
*函 数 名: set_chassis_moto_target_zero
*功能说明:设置底盘电机目标值为0  当步兵失控时会调用此函数
*形    参: 
*返 回 值: 
**********************************************************************************************************/
void set_chassis_moto_target_zero()
{
	char i;
	for(i=0;i<8;i++)
	{
		MotorTxData[i]=0;
		//HeadTxData[i]=0;
		//CAN_Send_Msg(&hcan1,0,HEADID,8);
		
	}
	CAN_Send_Msg(&hcan1, MotorTxData, MOTORID, 8);  //向底盘电机发送给定的电流值
	//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
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


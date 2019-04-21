#include "pan_tilt_control.h"
#define PAN_TILT_YAW_NUM 0.07f
#define PAN_TILT_PIT_NUM 0.07f
//底盘控制周期
#define PAN_TILT_CONTROL_TIME	0.005f

/**********************************************************************************************************
*函 数 名: pan_tilt_init
*功能说明: 云台初始化
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_init()
{
		pitch_angle_target=179.0f;																																																																																
    const static fp32 pan_tilt_yaw_order_filter[1] = {PAN_TILT_YAW_NUM};
    const static fp32 pan_tilt_pit_order_filter[1] = {PAN_TILT_PIT_NUM};
		init_Fric_PWM();			//初始化摩擦轮的PWM
    //用一阶滤波代替斜波函数生成
    first_order_filter_init(&pan_tilt_cmd_slow_set_yaw, PAN_TILT_CONTROL_TIME, pan_tilt_yaw_order_filter);
    first_order_filter_init(&pan_tilt_cmd_slow_set_pit, PAN_TILT_CONTROL_TIME, pan_tilt_pit_order_filter);
		//摩擦轮斜坡函数初始化
		ramp_init(&fric_ramp,SHOOT_CONTROL_TIME * 0.001f, Close_Fric_ON, Fric_OFF);
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
//		if(remote_control.switch_right!=3)
//		set_pan_tilt_current(&hcan1,pan_tilt_pitch_speed.output,pan_tilt_yaw_speed.output);
//		else
//			set_pan_tilt_current(&hcan1,0,0);
		//set_pan_tilt_current(&hcan1,0,pan_tilt_yaw_speed.output);
//	}
//	else
//		set_pan_tilt_current(&hcan1,0,0);
		
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
}
}

/**********************************************************************************************************
*函 数 名: pan_tilt_lock_control
*功能说明: 云台yaw轴绝对位置锁定,熟称锁头
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
void pan_tilt_lock_control()
{
	if(robot_status.vision_status==VISION_SUCCESS&&
		robot_status.vision_mode==ACTIVATE)
		//int_abs(VISION_YAW_TARGET-Armour_attack.pan_tilt_angel_err.Yaw_Err)<60&&
		//int_abs(VISION_PIT_TARGET-Armour_attack.pan_tilt_angel_err.Pitch_Err)<60)//如果视觉没有离线并且左边开关为1，进行视觉打击
	{
		armour_attack();
		//pan_tilt_pitch_imu_angle_control();
	}
	else 
	{
		pan_tilt_yaw_imu_angle_control();
		pan_tilt_pitch_imu_angle_control();
	}
	HeadTxData[0]=(uint8_t)((pan_tilt_pitch_speed.output>>8)&0xFF);
  HeadTxData[1]=(uint8_t)(pan_tilt_pitch_speed.output&0xFF);           //205   pitch
	  
	HeadTxData[2]=(uint8_t)((pan_tilt_yaw_speed.output>>8)&0xFF);
  HeadTxData[3]=(uint8_t)(pan_tilt_yaw_speed.output&0xFF);           //206  yaw 
	HeadTxData[4]=(uint8_t)((shoot_control_loop()>>8)&0xFF); //摩擦轮控制  
	HeadTxData[5]=(uint8_t)(shoot_control_loop()&0xFF);
	if(robot_status.control_mode==REMOTE_CONTROL)
	{
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
			if(remote_control.switch_right!=3)//不在键盘模式时，关闭摩擦轮
			{
				HeadTxData[4]=0;
				HeadTxData[5]=0; 
			}
			CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
		}
	}
	else if(robot_status.control_mode==KEYBOARD_CONTROL)
	{
		CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
	}
	
	//CAN_Send_Msg(&hcan1,HeadTxData,HEADID,8);
}


/**********************************************************************************************************
*函 数 名: pan_tilt_yaw_imu_angle_control
*功能说明: 云台yaw轴控制，需要imu的yaw轴角度作为反馈值
*形    参: 需要yaw轴imu的角度，角速度(imu.yaw,imu.gz)
*返 回 值: 电流输出
**********************************************************************************************************/
int yaw_velocity_target;//yaw轴角速度输入目标值
void pan_tilt_yaw_imu_angle_control(void)
{	
//	if(remote_control.switch_left==2)
//	{
	if(yaw_velocity_target==0)//偏航杆置于中位
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
			pan_tilt_yaw_speed.target=yaw_velocity_target*chassis_pan_tilt_max_rotate_speed/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
		}
	//}
		
//	else if(remote_control.switch_left==1)
//	{
//	if(remote_control.ch4==0)//偏航杆置于中位
//		{
//			if(pan_tilt_yaw.target==0)  //回中时赋角度期望值
//			{
//				pan_tilt_yaw.target=imu.yaw;
//				
//			}
//			PID_Control_Yaw(&pan_tilt_yaw,imu.yaw);
//			pan_tilt_yaw_speed.target=pan_tilt_yaw.output;
//	  }
//	
//		else
//		{
//			pan_tilt_yaw.target=0;////偏航角期望给0,不进行角度控制
//			pan_tilt_yaw_speed.target=remote_control.ch4*120/660;//yaw_control;//偏航角速度环期望，直接来源于遥控器打杆量
//		}
//	}
	
		
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
*函 数 名: PID_Pitch_control
*功能说明: 云台pitch轴增量式角度环控制，
*形    参: 
*返 回 值: 电流输出
**********************************************************************************************************/
	float error[3]={0},Dbuf[3]={0};
void PID_Pitch_control()
{	
	error[2]=error[1];
	error[1]=error[0];
	error[0]=pan_tilt_pitch.target-imu.pit;
	if((int_abs(pan_tilt_pitch.err) > pan_tilt_pitch.DeadBand))
	{
		pan_tilt_pitch.pout = pan_tilt_pitch.kp *(error[0]-error[1]);
		pan_tilt_pitch.iout = pan_tilt_pitch.ki * error[0];
//		if(pan_tilt_pitch.iout > pan_tilt_pitch.IntegralLimit)
//			pan_tilt_pitch.iout = pan_tilt_pitch.IntegralLimit;
//		if(pan_tilt_pitch.iout < - pan_tilt_pitch.IntegralLimit)
//			pan_tilt_pitch.iout = - pan_tilt_pitch.IntegralLimit;
		Dbuf[2] = Dbuf[1];
    Dbuf[1] = Dbuf[0];
    Dbuf[0] = (error[0] - 2.0f * error[1] + error[2]);
    pan_tilt_pitch.dout = pan_tilt_pitch.kd * Dbuf[0];
		pan_tilt_pitch.output += pan_tilt_pitch.pout + pan_tilt_pitch.iout + pan_tilt_pitch.dout;
		if(pan_tilt_pitch.output>pan_tilt_pitch.MaxOutput)         
		{
			pan_tilt_pitch.output = pan_tilt_pitch.MaxOutput;
		}
		if(pan_tilt_pitch.output < -(pan_tilt_pitch.MaxOutput))
		{
			pan_tilt_pitch.output = -(pan_tilt_pitch.MaxOutput);
		}
	}
}

/**********************************************************************************************************
*函 数 名: pan_tilt_pitch_imu_angle_control
*功能说明: 云台pitch轴控制，需要imu的pitch轴角度作为反馈值
*形    参: 需要yaw轴imu的角度，角速度(imu.pitch,imu.gx)
*返 回 值: 电流输出
**********************************************************************************************************/
int pitch_velocity_target;//pitch轴角速度输入目标值
float pitch_angle_target=179.0f;//pitch轴角速输入目标值
void pan_tilt_pitch_imu_angle_control(void)
{
	if(robot_status.control_mode==REMOTE_CONTROL)
	{
		if(pitch_velocity_target==0)//俯仰杆置于中位
		{
			if(pan_tilt_pitch.target==0)  //回中时赋角度期望值
			{
				pan_tilt_pitch.target=imu.pit;
			}
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
			pan_tilt_pitch_speed.target=pan_tilt_pitch.output;
	  }
		else
		{
			pan_tilt_pitch.target=0;////偏航角期望给0,不进行角度控制
			pan_tilt_pitch_speed.target=pitch_velocity_target*300/660;//pitch_control;//偏航角速度环期望，直接来源于遥控器打杆量
		}
		#ifdef INFANTRY_1
		PID_Control_Pitch(&pan_tilt_pitch_speed,-imu.gy);//角速度环
		#endif
		#ifdef INFANTRY_2
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//角速度环
		#endif
		//pan_tilt_pitch_speed.output=GildeAverageValueFilter(pan_tilt_pitch_speed.output,Data);//滤波？
	}
	else if(robot_status.control_mode==KEYBOARD_CONTROL)
	{
		pan_tilt_pitch.target=pitch_angle_target;//来源于鼠标控制
		
		//PID_Pitch_control();
		#ifdef INFANTRY_1
		pan_tilt_pitch.target=fp32_constrain(pan_tilt_pitch.target,150.0f,190.0f);//限幅
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
		#endif
		#ifdef INFANTRY_2
		pan_tilt_pitch.target=fp32_constrain(pan_tilt_pitch.target,165.0f,197.0f);//限幅
		PID_Control_Pitch(&pan_tilt_pitch,imu.pit);
		#endif
		pan_tilt_pitch_speed.target=pan_tilt_pitch.output;
		#ifdef INFANTRY_1
		PID_Control_Pitch(&pan_tilt_pitch_speed,-imu.gy);//角速度环
		#endif
		#ifdef INFANTRY_2
		PID_Control_Pitch(&pan_tilt_pitch_speed,imu.gy);//角速度环
		#endif
	}
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


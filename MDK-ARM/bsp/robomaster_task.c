#include "robomaster_task.h"
//#include "mpu9250.h"
#include "robomaster_common.h"

/**
  * @brief  TIM2中断处理函数主要运行的程序，5MS执行一次
  * @param  无
  * @retval 
  * @usage  用于底盘的驱动，结合imu进行底盘控制，后期可用来走猫步        
  *               
  */
int	cnt1=0,cnt2=0;
float mypitch=0.0,myroll=0.0,myyaw=0.0;
int testmpu;
int init_yaw=0;
void task() 
{	
	cnt1++;	
	if(cnt1%100==0)//0.5秒进入一次
		oled_clear(Pen_Clear);//清屏
	
	Bling_Working(Bling_Mode);
	mpu_get_data();//获得imu原始数据
	imu_ahrs_update(&imu);//更新四元数和imu姿态
	
	//mpu9250_get_data();
	//imu_ahrs_update(&imu_9250);

	get_chassis_to_pan_tilt_angle();
	if(robot_status.anomaly==NORMAL)
	{
		infantry_control();//步兵控制
		//shoot_control();			//摩擦轮以及拨弹电机的控制
		
		single_shoot();
		//single_shoot1();
	}
	robot_status_detection();//步兵状态检测 OLED显示
	select_mode();//控制模式选择：1遥控器	2键盘
	remap_variable();//变量重定义，目的用于键盘和遥控器对车的操作

//	display();
}

void select_mode()
{
	if(remote_control.switch_left==3&&remote_control.switch_right==3)
	{
		robot_status.control_mode=KEYBOARD_CONTROL;//键盘模式
	}
	else
	{
		robot_status.control_mode=REMOTE_CONTROL;//遥控器模式
	}
	
}

void remap_variable()
{
	if(robot_status.control_mode==REMOTE_CONTROL)//键盘模式
	{
		translation=remote_control.ch1;//平移重定向
		straight=remote_control.ch2;//直行重定向
		yaw_velocity_target=remote_control.ch3;//旋转重定向
		pitch_velocity_target=remote_control.ch4;//pit重定向
	}
	if(robot_status.control_mode==KEYBOARD_CONTROL)
	{
		yaw_velocity_target=remote_control.mouse.x*8;
		pitch_velocity_target=remote_control.mouse.y*8;
		switch(robot_status.fric_mode)
		{
			case STOP:									init_Fric_PWM();				 break;
			case CLOSE_FIRE:							close_fire();						 break;
			case MID_FIRE:								mid_fire();							 break;
			case REMOTE_FIRE:							remote_fire(); 					 break;
			case INTERCONTINENTAL_FIRE:		intercontinental_fire(); break;
		}
		
	}
}


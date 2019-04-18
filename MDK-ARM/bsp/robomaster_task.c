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
	mpu_get_data();//获得imu原始数据
	imu_ahrs_update(&imu);//更新四元数和imu姿态
	if(robot_status.anomaly==NORMAL)
	{
			pan_tilt_lock_control(); //云台锁头程序启动
			chassis_task();//底盘控制
	}
	get_armour_err();
	robot_status_detection();//步兵状态检测 OLED显示
	select_mode();//控制模式选择：1遥控器	2键盘
	remap_variable();//变量重定义，目的用于键盘和遥控器对车的操作
	Bling_Working(Bling_Mode);
	if(cnt1%100==0)//0.5秒进入一次
		oled_clear(Pen_Clear);//清屏
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
	if(robot_status.control_mode==REMOTE_CONTROL)//遥控模式
	{
		yaw_velocity_target=remote_control.ch3;//旋转重定向
		pitch_velocity_target=remote_control.ch4;//pit重定向
	}
	if(robot_status.control_mode==KEYBOARD_CONTROL)//键盘模式
	{
		Keyboard_Control();
	}
}


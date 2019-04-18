#include "Keyboard_Control.h"

/*
W 1 	S 2 	A 4		D 8 	SHIFT 16 	CTRL 32
Q 64	E 128 	R 256		F 512		G 1024
Z 2048		X 4096 		C 8192 		V 16384  	B 32768


*/
#define YAW_MOUSE_SENSITIVITY 13.0f//鼠标控制云台YAW的灵敏度
#define PITCH_MOUSE_SENSITIVITY 0.025f//鼠标控制云台PIT的灵敏度
char key_board_mode=0;//用于键盘切换模式
float Twist_speed=150.0f;//扭腰速度
char keyboard_count=0;
char fric_count=0;
void Keyboard_Control()
{
	//一阶低通滤波代替斜波作为底盘速度输入
    first_order_filter_cali(&pan_tilt_cmd_slow_set_yaw, remote_control.mouse.x*YAW_MOUSE_SENSITIVITY);
    first_order_filter_cali(&pan_tilt_cmd_slow_set_pit, remote_control.mouse.y*PITCH_MOUSE_SENSITIVITY);
		yaw_velocity_target=pan_tilt_cmd_slow_set_yaw.out;
		//pitch_velocity_target=pan_tilt_cmd_slow_set_pit.out;
		pitch_angle_target+=pan_tilt_cmd_slow_set_pit.out;
	if(pitch_angle_target>190.0f)
		pitch_angle_target=190.0f;
	if(pitch_angle_target<150.0f)
		pitch_angle_target=150.0f;
		switch(robot_status.fric_mode)//摩擦轮控制
		{
			case STOP:										init_Fric_PWM();				 break;
			case CLOSE_FIRE:							close_fire();						 break;
			case MID_FIRE:								mid_fire();							 break;
			case REMOTE_FIRE:							remote_fire(); 					 break;
			case INTERCONTINENTAL_FIRE:		intercontinental_fire(); break;
		}
		if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_Z)                               //按下“Z”。
	{
		key_board_mode=0;//切换为底盘跟随云台角度
	}
	else if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_X)                               //按下“E”
	{
		key_board_mode=1;//切换为底盘跟随底盘角度
	}
	else if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_Q)                               //按下“E”
	{
		Twist_speed+=0.2f;
		if(Twist_speed>250.0f)
			Twist_speed=250.0f;
	}
	else if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_E)                               //按下“E”
	{
		Twist_speed-=0.2f;
		if(Twist_speed<80.0f)
			Twist_speed=80.0f;
	}
	else if (remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_R)                               //按下“r”。
	{
		open_cap();//取弹，比赛过程中只需打开弹舱，摄像头要保持前方
		//detect_cartridge();//打开弹舱并查看弹药
	}
	else if(remote_control.keyBoard.key_code	& KEY_PRESSED_OFFSET_B)																//按下“b”,检测弹药
	{
		detect_cartridge();//打开弹舱并查看弹药
	}		
	else if(remote_control.keyBoard.key_code	& KEY_PRESSED_OFFSET_V)//V
	{
			keyboard_count++;
		if(keyboard_count==10)
		{
			fric_count++;
		}
		if(fric_count>4)
		{
			fric_count=0;
		}
		#ifdef DEBUG_MODE//调试模式可以关闭摩擦轮，比赛过程中不准关闭摩擦轮
		if(fric_count==0&&shoot_mode == SHOOT_STOP)//只有在拨弹轮停止时才能关闭摩擦轮
		{
			robot_status.fric_mode=STOP;
		}
		#endif
			if(fric_count==1)
		{
			robot_status.fric_mode=CLOSE_FIRE;
		}
			if(fric_count==2)
		{
			robot_status.fric_mode=MID_FIRE;
		}
			if(fric_count==3)
		{
			robot_status.fric_mode=REMOTE_FIRE;
		}
			if(fric_count==4)
		{
			robot_status.fric_mode=INTERCONTINENTAL_FIRE;
		}
	}
//	else if(remote_control.mouse.press_left==1)
//	{
////		kb.mousel_flag   =   1;
////		kb.mousel        =   0;
//	}
	
	else if(remote_control.mouse.press_right==1)
	{
		robot_status.vision_mode=ACTIVATE;//激活视觉识别
	}
	else                                                                            //否则速度停止
	{
		keyboard_count=0;//清0，实现按键按一次只加一个数
		reset_camera();//复位图传位置
		robot_status.vision_mode=DORMANT;//屏蔽视觉识别
	}
}

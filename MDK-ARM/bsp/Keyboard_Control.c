#include "Keyboard_Control.h"

/*
W 1 	S 2 	A 4		D 8 	SHIFT 16 	CTRL 32
Q 64	E 128 	R 256		F 512		G 1024
Z 2048		X 4096 		C 8192 		V 16384  	B 32768


*/
#define YAW_MOUSE_SENSITIVITY 13.0f//��������̨YAW��������
#define PITCH_MOUSE_SENSITIVITY 0.025f//��������̨PIT��������
char key_board_mode=0;//���ڼ����л�ģʽ
float Twist_speed=150.0f;//Ť���ٶ�
char keyboard_count=0;
char fric_count=0;
void Keyboard_Control()
{
	//һ�׵�ͨ�˲�����б����Ϊ�����ٶ�����
    first_order_filter_cali(&pan_tilt_cmd_slow_set_yaw, remote_control.mouse.x*YAW_MOUSE_SENSITIVITY);
    first_order_filter_cali(&pan_tilt_cmd_slow_set_pit, remote_control.mouse.y*PITCH_MOUSE_SENSITIVITY);
		yaw_velocity_target=pan_tilt_cmd_slow_set_yaw.out;
		//pitch_velocity_target=pan_tilt_cmd_slow_set_pit.out;
		pitch_angle_target+=pan_tilt_cmd_slow_set_pit.out;
	if(pitch_angle_target>190.0f)
		pitch_angle_target=190.0f;
	if(pitch_angle_target<150.0f)
		pitch_angle_target=150.0f;
		switch(robot_status.fric_mode)//Ħ���ֿ���
		{
			case STOP:										init_Fric_PWM();				 break;
			case CLOSE_FIRE:							close_fire();						 break;
			case MID_FIRE:								mid_fire();							 break;
			case REMOTE_FIRE:							remote_fire(); 					 break;
			case INTERCONTINENTAL_FIRE:		intercontinental_fire(); break;
		}
		if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_Z)                               //���¡�Z����
	{
		key_board_mode=0;//�л�Ϊ���̸�����̨�Ƕ�
	}
	else if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_X)                               //���¡�E��
	{
		key_board_mode=1;//�л�Ϊ���̸�����̽Ƕ�
	}
	else if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_Q)                               //���¡�E��
	{
		Twist_speed+=0.2f;
		if(Twist_speed>250.0f)
			Twist_speed=250.0f;
	}
	else if(remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_E)                               //���¡�E��
	{
		Twist_speed-=0.2f;
		if(Twist_speed<80.0f)
			Twist_speed=80.0f;
	}
	else if (remote_control.keyBoard.key_code & KEY_PRESSED_OFFSET_R)                               //���¡�r����
	{
		open_cap();//ȡ��������������ֻ��򿪵��գ�����ͷҪ����ǰ��
		//detect_cartridge();//�򿪵��ղ��鿴��ҩ
	}
	else if(remote_control.keyBoard.key_code	& KEY_PRESSED_OFFSET_B)																//���¡�b��,��ⵯҩ
	{
		detect_cartridge();//�򿪵��ղ��鿴��ҩ
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
		#ifdef DEBUG_MODE//����ģʽ���Թر�Ħ���֣����������в�׼�ر�Ħ����
		if(fric_count==0&&shoot_mode == SHOOT_STOP)//ֻ���ڲ�����ֹͣʱ���ܹر�Ħ����
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
		robot_status.vision_mode=ACTIVATE;//�����Ӿ�ʶ��
	}
	else                                                                            //�����ٶ�ֹͣ
	{
		keyboard_count=0;//��0��ʵ�ְ�����һ��ֻ��һ����
		reset_camera();//��λͼ��λ��
		robot_status.vision_mode=DORMANT;//�����Ӿ�ʶ��
	}
}

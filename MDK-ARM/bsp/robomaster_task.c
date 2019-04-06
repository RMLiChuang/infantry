#include "robomaster_task.h"
//#include "mpu9250.h"
#include "robomaster_common.h"

/**
  * @brief  TIM2�жϴ�������Ҫ���еĳ���5MSִ��һ��
  * @param  ��
  * @retval 
  * @usage  ���ڵ��̵����������imu���е��̿��ƣ����ڿ�������è��        
  *               
  */
int	cnt1=0,cnt2=0;
float mypitch=0.0,myroll=0.0,myyaw=0.0;
int testmpu;
int init_yaw=0;
void task() 
{	
	cnt1++;	
	if(cnt1%100==0)//0.5�����һ��
		oled_clear(Pen_Clear);//����
	
	Bling_Working(Bling_Mode);
	mpu_get_data();//���imuԭʼ����
	imu_ahrs_update(&imu);//������Ԫ����imu��̬
	
	//mpu9250_get_data();
	//imu_ahrs_update(&imu_9250);

	get_chassis_to_pan_tilt_angle();
	if(robot_status.anomaly==NORMAL)
	{
		infantry_control();//��������
		//shoot_control();			//Ħ�����Լ���������Ŀ���
		
		single_shoot();
		//single_shoot1();
	}
	robot_status_detection();//����״̬��� OLED��ʾ
	select_mode();//����ģʽѡ��1ң����	2����
	remap_variable();//�����ض��壬Ŀ�����ڼ��̺�ң�����Գ��Ĳ���

//	display();
}

void select_mode()
{
	if(remote_control.switch_left==3&&remote_control.switch_right==3)
	{
		robot_status.control_mode=KEYBOARD_CONTROL;//����ģʽ
	}
	else
	{
		robot_status.control_mode=REMOTE_CONTROL;//ң����ģʽ
	}
	
}

void remap_variable()
{
	if(robot_status.control_mode==REMOTE_CONTROL)//����ģʽ
	{
		translation=remote_control.ch1;//ƽ���ض���
		straight=remote_control.ch2;//ֱ���ض���
		yaw_velocity_target=remote_control.ch3;//��ת�ض���
		pitch_velocity_target=remote_control.ch4;//pit�ض���
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


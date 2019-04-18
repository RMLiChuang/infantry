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
	mpu_get_data();//���imuԭʼ����
	imu_ahrs_update(&imu);//������Ԫ����imu��̬
	if(robot_status.anomaly==NORMAL)
	{
			pan_tilt_lock_control(); //��̨��ͷ��������
			chassis_task();//���̿���
	}
	get_armour_err();
	robot_status_detection();//����״̬��� OLED��ʾ
	select_mode();//����ģʽѡ��1ң����	2����
	remap_variable();//�����ض��壬Ŀ�����ڼ��̺�ң�����Գ��Ĳ���
	Bling_Working(Bling_Mode);
	if(cnt1%100==0)//0.5�����һ��
		oled_clear(Pen_Clear);//����
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
	if(robot_status.control_mode==REMOTE_CONTROL)//ң��ģʽ
	{
		yaw_velocity_target=remote_control.ch3;//��ת�ض���
		pitch_velocity_target=remote_control.ch4;//pit�ض���
	}
	if(robot_status.control_mode==KEYBOARD_CONTROL)//����ģʽ
	{
		Keyboard_Control();
	}
}


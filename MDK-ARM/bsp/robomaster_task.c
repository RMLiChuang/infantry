#include "robomaster_task.h"
#include "mpu9250.h"
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
	Bling_Working(Bling_Mode);
	mpu_get_data();//���imuԭʼ����
	imu_ahrs_update(&imu);//������Ԫ����imu��̬
	//mpu9250_get_data();
	//imu_ahrs_update(&imu_9250);
	
	pan_tilt_control();
	//chassis_control();		//���̵���Ŀ���
	
	//shoot_control();			//Ħ�����Լ���������Ŀ���
	
	
}

#include "robomaster_task.h"
#include "mpu9250.h"
#include "inv_mpu.h"


/**
  * @brief  TIM2�жϴ�������Ҫ���еĳ���5MSִ��һ��
  * @param  ��
  * @retval 
  * @usage  ���ڵ��̵����������imu���е��̿��ƣ����ڿ�������è��        
  *               
  */
int	cnt=0,cnt1=0,cnt2=0;
float mypitch=0.0,myroll=0.0,myyaw=0.0;
int testmpu;
int init_yaw=0;
void task() 
{	
	cnt++;
	cnt1++;
	
	if(cnt==100)    //0.5s����һ�Σ�ʹ��7������2HZƵ����˸���Ӷ��ж��жϳ�����������
	{
		cnt=0;
		HAL_GPIO_TogglePin(GPIOG,GPIO_PIN_7);
	}
	
	mpu_get_data();//���imuԭʼ����
	imu_ahrs_update(&imu);//������Ԫ����imu��̬
//	MPU_Get_Accelerometer(&mpu9250_data);
//	MPU_Get_Gyroscope(&mpu9250_data);
	mpu9250_get_data();
	imu_ahrs_update(&imu_9250);
	
	
	chassis_control();		//���̵���Ŀ���
	//shoot_control();			//Ħ�����Լ���������Ŀ���
	
	///mpu_dmp_get_data(&mypitch,&myroll,&myyaw);//dmp��ȡŷ����
}

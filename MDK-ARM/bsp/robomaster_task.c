#include "robomaster_task.h"
#include "mpu9250.h"
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
	Bling_Working(Bling_Mode);
	mpu_get_data();//获得imu原始数据
	imu_ahrs_update(&imu);//更新四元数和imu姿态
	//mpu9250_get_data();
	//imu_ahrs_update(&imu_9250);
	
	pan_tilt_control();
	//chassis_control();		//底盘电机的控制
	
	//shoot_control();			//摩擦轮以及拨弹电机的控制
	
	
}

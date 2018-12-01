/**
  ******************************************************************************
  * @file    pid.c
  * @author  izh20(github)
  * @version V1.0.0
  * @date    2018/11/25
  * @brief   对每一个pid结构体都要先进行函数的连接，再进行初始化
  ******************************************************************************
  * @attention 应该是用二阶差分(d)云台会更加稳定
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "stm32f4xx.h"
#include <main.h>
#include "usart.h"
#define ABS(x)		((x>0)? x: -x) 
PID_TypeDef motor_pid[7];//初始化7个电机的pid结构体 其中0 1 2 3对应底盘电机，4 5 对应云台电机 6对应拨弹电机，目前只用到了0 1 2 3 6号电机
PID_TypeDef pan_tilt_pitch,pan_tilt_pithch_speed,pan_tilt_roll,pan_tilt_roll_speed,pan_tilt_yaw,pan_tilt_yaw_speed;//定义了云台pitch，yaw轴的角度，角速度结构体
PID_TypeDef chassis_yaw_speed,chassis_yaw;//定义了底盘yaw轴的角速度和角度结构体
PID_TypeDef chassis_imu_temperature;//定义了底盘imu温度结构体 用于温度补偿
//extern int isMove;

/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(
	PID_TypeDef * pid, 
	PID_ID   id,
	uint16_t maxout,
	uint16_t intergral_limit,
	float deadband,
	uint16_t period,
	int16_t  max_err,
	int16_t  target,

	float 	kp, 
	float 	ki, 
	float 	kd)
{
	pid->id = id;		
	
	pid->ControlPeriod = period;             //没用到
	pid->DeadBand = deadband;
	pid->IntegralLimit = intergral_limit;
	pid->MaxOutput = maxout;
	pid->Max_Err = max_err;
	pid->target = target;
	
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
	
	pid->output = 0;
}

/*中途更改底盘参数设定--------------------------------------------------------------*/
void pid_reset_chassis(u8 motornum,u32 kp, u32 ki, u32 kd)
{
	//main函数里面定义了7个pid结构
	if(motornum<7){
		motor_pid[motornum].kp = (float)kp/1000.0;
		motor_pid[motornum].ki = (float)ki/1000.0;
		motor_pid[motornum].kd = (float)kd/1000.0;
	}
}
void pid_reset_all_chassis(u32 kp, u32 ki, u32 kd)
{
	//main函数里面定义了7个pid结构
		for(unsigned int i=0;i<4;++i)
		{
			pid_reset_chassis(i,kp,ki,kd);
		}
}
void show_pid(u8 motornum){
	printf("\r\n number %d motornum kp is %.4f , ki is %.4f, kd is %.4f",motornum,motor_pid[motornum].kp,motor_pid[motornum].ki,motor_pid[motornum].kd);
}
void show_all_pid(void){
	for(unsigned short i=0;i<4;++i){
		show_pid(i);
	}
}
/*中途更改参数设定--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid计算-----------------------------------------------------------------------*/

	
float pid_calculate(PID_TypeDef* pid, float measure)//, int16_t target)
{
//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	//是否进入死区
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//积分是否超出限制
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}


	return pid->output;
}
/**********************************************************************************************************
*函 数 名: PID_Control_Yaw
*功能说明: 底盘yaw轴控制  增加了偏航角0和360临界突变的处理
*形    参: 控制对象的pid结构体  反馈值
*返 回 值: pid输出
**********************************************************************************************************/
float PID_Control_Yaw(PID_TypeDef* pid, float measure)
{
	//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
	pid->err = pid->target - pid->measure;
	
	/***********************偏航角在0度/360度附近的处理*****************************/
	if(pid->err<-300)  pid->err=pid->err+360;
  if(pid->err>300)  pid->err=pid->err-360;
	
	//是否进入死区
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//积分是否超出限制
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid输出和
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //滤波？
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}


	return pid->output;
	
}


/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}

void all_pid_init()
{
	/*< 初始化PID参数 >*/
	
	//底盘电机初始化
  for(int i=0; i<4; i++)
  {	

    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],
																	PID_Speed,					
																	2500,							//maxOutput												//输出限幅
																	1000,								//integralLimit										//积分限幅
																	10,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	1000,								//max_err													//最大误差
																	0,									//target
																	2,								//kp
																	0.05,							//ki	
																	0.01);							//kd
    																	
  }
	
	
//拨单电机pid初始化
    pid_init(&motor_pid[6]);
    motor_pid[6].f_param_init(&motor_pid[6],
																	PID_Speed,					
																	10000,							//maxOutput												//输出限幅
																	2000,								//integralLimit										//积分限幅
																	10,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	5000,								//max_err													//最大误差
																	0,									//target
																	2,								//kp
																	0.05,							//ki	
																	3);							//kd
	
//chassis_yaw pid初始化  用于走直线，角度外环控制
	pid_init(&chassis_yaw);
	chassis_yaw.f_param_init(&chassis_yaw,
																	PID_Speed,					
																	150,							//maxOutput												//输出限幅
																	50,								//integralLimit										//积分限幅
																	0.5,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	30,								//max_err													//最大误差
																	0,									//target
																	5,								//kp
																	0,							//ki	
																	0);							//kd
	

//chassis_yaw_speed pid初始化  用于走直线，角度外环控制
	pid_init(&chassis_yaw_speed);
	chassis_yaw_speed.f_param_init(&chassis_yaw_speed,
																	PID_Speed,					
																	4000,							//maxOutput												//输出限幅
																	1000,								//integralLimit										//积分限幅
																	1,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	150,								//max_err													//最大误差
																	0,									//target
																	22,								//kp
																	0.1,							//ki	
																	0.3);							//kd
	#ifdef QUADROTOR_PAN_TILT
	//pitch轴电机角度环pid初始化  
	 pid_init(&motor_pid[4]);
    motor_pid[4].f_param_init(&motor_pid[4],
																	PID_Speed,					
																	29000,							//maxOutput												//输出限幅
																	2000,								//integralLimit										//积分限幅
																	10,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	5000,								//max_err													//最大误差
																	0,									//target
																	50,								//kp
																	10,							//ki	
																	65);							//kd
	//yaw轴电机角度环pid初始化    
	 pid_init(&motor_pid[5]);
    motor_pid[5].f_param_init(&motor_pid[5],
																	PID_Speed,					
																	29000,							//maxOutput												//输出限幅
																	2000,								//integralLimit										//积分限幅
																	10,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	5000,								//max_err													//最大误差
																	0,									//target
																	50,								//kp
																	10,							//ki	
																	65);							//kd

	#endif
	#ifdef INFANTRY_PAN_TILT
	//YAW轴电机角度环pid初始化  
	 pid_init(&motor_pid[4]);
    motor_pid[4].f_param_init(&motor_pid[4],
																	PID_Speed,					
																	16384,							//maxOutput												//输出限幅
																	2000,								//integralLimit										//积分限幅
																	10,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	5000,								//max_err													//最大误差
																	0,									//target
																	2,								//kp
																	0,							//ki	
																	0);							//kd
	//PITCH轴电机角度环pid初始化    
	 pid_init(&motor_pid[5]);
    motor_pid[5].f_param_init(&motor_pid[5],
																	PID_Speed,					
																	16384,							//maxOutput												//输出限幅
																	2000,								//integralLimit										//积分限幅
																	10,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	5000,								//max_err													//最大误差
																	0,									//target
																	5,							//kp
																	0,							//ki	
																	0);							//kd
	
	//PITCH轴电机角速度环pid初始化    
	 pid_init(&pan_tilt_pithch_speed);
    pan_tilt_pithch_speed.f_param_init(&pan_tilt_pithch_speed,
																	PID_Speed,					
																	6000,							//maxOutput												//输出限幅
																	1500,								//integralLimit										//积分限幅
																	12,									//deadband												//死区（绝对值）
																	0,									//controlPeriod										//控制周期
																	300,								//max_err													//最大误差
																	0,									//target
																	3,								//kp    3.2
																	0.02,							//ki	    0.02
																	0.1);							//kd			0.1
	#endif																
																	
																	
																	
	
}






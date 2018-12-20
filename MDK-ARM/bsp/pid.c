/**
  ******************************************************************************
  * @file    pid.c
  * @author  izh20(github)
  * @version V1.0.0
  * @date    2018/11/25
  * @brief   ��ÿһ��pid�ṹ�嶼Ҫ�Ƚ��к��������ӣ��ٽ��г�ʼ��
  ******************************************************************************
  * @attention Ӧ�����ö��ײ��(d)��̨������ȶ�
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "pid.h"
#include "stm32f4xx.h"
#include <main.h>
#include "usart.h"
#define ABS(x)		((x>0)? x: -x) 
PID_TypeDef motor_pid[7];//��ʼ��7�������pid�ṹ�� ����0 1 2 3��Ӧ���̵����4 5 ��Ӧ��̨��� 6��Ӧ���������Ŀǰֻ�õ���0 1 2 3 6�ŵ��
PID_TypeDef pan_tilt_pitch,pan_tilt_pitch_speed,pan_tilt_roll,pan_tilt_roll_speed,pan_tilt_yaw,pan_tilt_yaw_speed;//��������̨pitch��yaw��ĽǶȣ����ٶȽṹ��
PID_TypeDef chassis_yaw_angle,chassis_yaw_speed,chassis_yaw;//�����˵���yaw��ı������Ƕȣ�imu�Ľ��ٶȺ�imu�ĽǶȽṹ��
PID_TypeDef chassis_imu_temperature;//�����˵���imu�¶Ƚṹ�� �����¶Ȳ���
//extern int isMove;

/*������ʼ��--------------------------------------------------------------*/
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
	
	pid->ControlPeriod = period;             //û�õ�
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

/*��;���ĵ��̲����趨--------------------------------------------------------------*/
void pid_reset_chassis(u8 motornum,u32 kp, u32 ki, u32 kd)
{
	//main�������涨����7��pid�ṹ
	if(motornum<7){
		motor_pid[motornum].kp = (float)kp/1000.0;
		motor_pid[motornum].ki = (float)ki/1000.0;
		motor_pid[motornum].kd = (float)kd/1000.0;
	}
}
void pid_reset_all_chassis(u32 kp, u32 ki, u32 kd)
{
	//main�������涨����7��pid�ṹ
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
/*��;���Ĳ����趨--------------------------------------------------------------*/
static void pid_reset(PID_TypeDef * pid, float kp, float ki, float kd)
{
	pid->kp = kp;
	pid->ki = ki;
	pid->kd = kd;
}

/*pid����-----------------------------------------------------------------------*/

	
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
	
	//�Ƿ��������
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//�����Ƿ񳬳�����
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid�����
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //�˲���
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}
//	/***************��errС������ʱ������pid����*********************/
//	else if((ABS(pid->err) < pid->DeadBand))
//	{	
//		pid->err=0;
//		pid->output=0;
//	}


	return pid->output;
}
/**********************************************************************************************************
*�� �� ��: PID_Control_Yaw
*����˵��: ���̣���̨yaw�����  ������ƫ����0��360�ٽ�ͻ��Ĵ���
*��    ��: ���ƶ����pid�ṹ��  ����ֵ
*�� �� ֵ: pid���
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
	
//	/***********************ƫ������0��/360�ȸ����Ĵ���*****************************/
//	if(pid->err<-300)  pid->err=pid->err+360;
//  if(pid->err>300)  pid->err=pid->err-360;
	
	//�Ƿ��������
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//�����Ƿ񳬳�����
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid�����
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //�˲���
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}
	/***************��errС������ʱ������pid����*********************/
	else if((ABS(pid->err) < pid->DeadBand))
	{	
		pid->err=0;
		pid->output=0;
	}


	return pid->output;
	
}

/**********************************************************************************************************
*�� �� ��: PID_Control_Yaw
*����˵��: ��̨pitch�����  ��8189��0��ͻ����д���
*��    ��: ���ƶ����pid�ṹ��  ����ֵ
*�� �� ֵ: pid���
**********************************************************************************************************/
float PID_Control_Pitch(PID_TypeDef* pid, float measure)
{
	//	uint32_t time,lasttime;
	
	pid->lasttime = pid->thistime;
	pid->thistime = HAL_GetTick();
	pid->dtime = pid->thistime-pid->lasttime;
	pid->measure = measure;
  //	pid->target = target;
		
	pid->last_err  = pid->err;
	pid->last_output = pid->output;
	
//	/***********************pitch���е�Ƕ���0/8189�Ĵ���*****************************/
//	if(pid->target>8000&&pid->measure<200)  pid->err=pid->target-8189-pid->measure;
//  if(pid->target<200&&pid->measure>8000)  pid->err=pid->target+8189-pid->measure;
//	else 
		pid->err = pid->target - pid->measure;
	
	
	
	//�Ƿ��������
	if((ABS(pid->err) > pid->DeadBand))
	{
		pid->pout = pid->kp * pid->err;
		pid->iout += (pid->ki * pid->err);
		

		pid->dout =  pid->kd * (pid->err - pid->last_err); 
		
		//�����Ƿ񳬳�����
		if(pid->iout > pid->IntegralLimit)
			pid->iout = pid->IntegralLimit;
		if(pid->iout < - pid->IntegralLimit)
			pid->iout = - pid->IntegralLimit;
		
		//pid�����
		pid->output = pid->pout + pid->iout + pid->dout;
		

		//pid->output = pid->output*0.7f + pid->last_output*0.3f;  //�˲���
		if(pid->output>pid->MaxOutput)         
		{
			pid->output = pid->MaxOutput;
		}
		if(pid->output < -(pid->MaxOutput))
		{
			pid->output = -(pid->MaxOutput);
		}
	
	}
	/***************��errС������ʱ������pid����*********************/
	else if((ABS(pid->err) < pid->DeadBand))
	{	
		pid->err=0;
		pid->output=0;
	}


	return pid->output;
	
}


/*pid�ṹ���ʼ����ÿһ��pid������Ҫ����һ��-----------------------------------------------------*/
void pid_init(PID_TypeDef* pid)
{
	pid->f_param_init = pid_param_init;
	pid->f_pid_reset = pid_reset;
	pid->f_cal_pid = pid_calculate;
}

void all_pid_init()
{
	/*< ��ʼ��PID���� >*/
	
	//���̵����ʼ��
  for(int i=0; i<4; i++)
  {	

    pid_init(&motor_pid[i]);
    motor_pid[i].f_param_init(&motor_pid[i],
																	PID_Speed,					
																	2500,							//maxOutput												//����޷�
																	1000,								//integralLimit										//�����޷�
																	10,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	1000,								//max_err													//������
																	0,									//target
																	2,								//kp
																	0.05,							//ki	
																	0.01);							//kd
    																	
  }
	
	
//�������pid��ʼ��
    pid_init(&motor_pid[6]);
    motor_pid[6].f_param_init(&motor_pid[6],
																	PID_Speed,					
																	6000,							//maxOutput												//����޷�
																	2000,								//integralLimit										//�����޷�
																	10,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	5000,								//max_err													//������
																	0,									//target
																	5,								//kp 2
																	0,							//ki	 0.05
																	0);							//kd  3
	
	//chassis_yaw_angle pid��ʼ��  
	pid_init(&chassis_yaw_angle);//��Դ��yaw������е�Ƕȵ���ֵ
	chassis_yaw_angle.f_param_init(&chassis_yaw_angle,
																	PID_Speed,					
																	3300,							//maxOutput												//����޷�
																	0,								//integralLimit										//�����޷�
																	0.5,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	500,								//max_err													//������
																	0,									//target
																	16,								//kp
																	0,							//ki	
																	0);							//kd
//chassis_yaw pid��ʼ��  ������ֱ�ߣ��Ƕ��⻷����
	pid_init(&chassis_yaw);//��Դ��imu��yaw��Ƕ�
	chassis_yaw.f_param_init(&chassis_yaw,
																	PID_Speed,					
																	150,							//maxOutput												//����޷�
																	50,								//integralLimit										//�����޷�
																	0.5,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	30,								//max_err													//������
																	0,									//target
																	5,								//kp
																	0,							//ki	
																	0);							//kd
	

//chassis_yaw_speed pid��ʼ��  ������ֱ�ߣ��Ƕ��⻷����
	pid_init(&chassis_yaw_speed);
	chassis_yaw_speed.f_param_init(&chassis_yaw_speed,
																	PID_Speed,					
																	4000,							//maxOutput												//����޷�
																	1000,								//integralLimit										//�����޷�
																	1,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	150,								//max_err													//������
																	0,									//target
																	22,								//kp
																	0.1,							//ki	
																	0.3);							//kd
	#ifdef QUADROTOR_PAN_TILT
	//pitch�����ǶȻ�pid��ʼ��  
	 pid_init(&motor_pid[4]);
    motor_pid[4].f_param_init(&motor_pid[4],
																	PID_Speed,					
																	29000,							//maxOutput												//����޷�
																	2000,								//integralLimit										//�����޷�
																	10,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	5000,								//max_err													//������
																	0,									//target
																	50,								//kp
																	10,							//ki	
																	65);							//kd
	//yaw�����ǶȻ�pid��ʼ��    
	 pid_init(&motor_pid[5]);
    motor_pid[5].f_param_init(&motor_pid[5],
																	PID_Speed,					
																	29000,							//maxOutput												//����޷�
																	2000,								//integralLimit										//�����޷�
																	10,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	5000,								//max_err													//������
																	0,									//target
																	50,								//kp
																	10,							//ki	
																	65);							//kd

	#endif
	#ifdef INFANTRY_PAN_TILT
	//YAW�����ǶȻ�pid��ʼ��  //����ֵ��imu���
	 pid_init(&pan_tilt_yaw);
    pan_tilt_yaw.f_param_init(&pan_tilt_yaw,
																	PID_Speed,					
																	120,							//maxOutput												//����޷�
																	0,								//integralLimit										//�����޷�
																	0,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	30,								//max_err													//������
																	0,									//target
																  6.5,								//kp
																	0,							//ki	
																	0);							//kd
																	
	//YAW�����ǶȻ�pid��ʼ��  //����ֵ�ɵ�����������
	 pid_init(&motor_pid[5]);
    motor_pid[5].f_param_init(&motor_pid[5],
																	PID_Speed,					
																	120,							//maxOutput												//����޷�
																	0,								//integralLimit										//�����޷�
																	0,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	100,								//max_err													//������
																	0,									//target
																  0.8,								//kp 0.8
																	0,							//ki	
																	0);							//kd
																	
	//YAW�������ٶȻ�pid��ʼ��		//����ֵ��imu���														
	  pid_init(&pan_tilt_yaw_speed);
    pan_tilt_yaw_speed.f_param_init(&pan_tilt_yaw_speed,
																	PID_Speed,					
																	5000,							//maxOutput												//����޷�
																	2000,								//integralLimit										//�����޷�
																	0,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	60,								//max_err													//������
																	0,									//target
																	75,								//kp    50
																	0,							//ki	    0.003
																	2);							//kd			1																
	//PITCH������е�ǶȻ�pid��ʼ��������ֵ��imu��ȡ    
	 pid_init(&pan_tilt_pitch);
    pan_tilt_pitch.f_param_init(&pan_tilt_pitch,
																	PID_Speed,					
																	200,							//maxOutput												//����޷�
																	50,								//integralLimit										//�����޷�
																	0,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	60,								//max_err													//������
																	0,									//target
																	11,							//kp 10
																	0,							//ki	
																	0);							//kd 0						
	//PITCH������е�ǶȻ�pid��ʼ��������ֵ�ɵ����������ȡ    
	 pid_init(&motor_pid[4]);
    motor_pid[4].f_param_init(&motor_pid[4],
																	PID_Speed,					
																	200,							//maxOutput												//����޷�
																	50,								//integralLimit										//�����޷�
																	0,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	300,								//max_err													//������
																	0,									//target
																	0,							//kp 0.32
																	0,							//ki	
																	0);							//kd 0
	
	//PITCH�������ٶȻ�pid��ʼ��    
	 pid_init(&pan_tilt_pitch_speed);
    pan_tilt_pitch_speed.f_param_init(&pan_tilt_pitch_speed,
																	PID_Speed,					
																	6000,							//maxOutput												//����޷�
																	3000,								//integralLimit										//�����޷�
																	0,									//deadband												//����������ֵ��
																	0,									//controlPeriod										//��������
																	80,								//max_err													//������
																	0,									//target
																	37,								//kp    55
																	0.8,							//ki	   0.0001
																	3);							//kd			0.5
	#endif																
																	
																	
																	
	
}






/**
  ******************************************************************************
  * @file		 pid.h
  * @author  Ginger
  * @version V1.0.0
  * @date    2015/11/14
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef _PID_H
#define _PID_H

#include "stdint.h"
#include "mytype.h"

#define pan_tilt_yaw_motor 		moto_chassis[5]
#define pan_tilt_pitch_motor 		moto_chassis[4]
typedef enum
{

	PID_Position,
	PID_Speed
	
}PID_ID;

typedef struct _PID_TypeDef
{
	PID_ID id;
	
	float target;							//Ŀ��ֵ
	float initial;            //��ʼֵ
	float lastNoneZeroTarget;
	float kp;
	float ki;
	float kd;
	
	float   measure;					//����ֵ
	float   err;							//���
	float   last_err;      		//�ϴ����
	
	int16_t pout;
	int16_t iout;
	int16_t dout;
	
	int16_t output;						//�������
	int16_t last_output;			//�ϴ����
	
	int16_t MaxOutput;				//����޷�
	float IntegralLimit;		//�����޷�
	float DeadBand;			  //����������ֵ��
	float ControlPeriod;		//��������
	float  Max_Err;					//������
	
					  float thistime;
					float lasttime;
						uint8_t dtime;	
	
	void (*f_param_init)(struct _PID_TypeDef *pid,  //PID������ʼ��
				   PID_ID id,
				   int16_t maxOutput,
				   int16_t integralLimit,
				   int16_t deadband,
				   int16_t controlPeriod,
					float max_err,     
					float  target,
				   float kp,
				   float ki,
				   float kd);
				   
	void (*f_pid_reset)(struct _PID_TypeDef *pid, float kp,float ki, float kd);		//pid���������޸�
	float (*f_cal_pid)(struct _PID_TypeDef *pid, float measure);   //pid����
}PID_TypeDef;
void all_pid_init(void);
void pid_init(PID_TypeDef* pid);
void pid_reset_chassis(u8 motornum,u32 kp, u32 ki, u32 kd);
void pid_reset_all_chassis(u32 kp, u32 ki, u32 kd);
void show_pid(u8 motornum);
void show_all_pid(void);
float PID_Control_Pitch(PID_TypeDef* pid, float measure);
float PID_Control_Yaw(PID_TypeDef* pid, float measure);
#endif

//extern PID_TypeDef pid_pitch;    
extern PID_TypeDef motor_pid[7];
extern PID_TypeDef chassis_yaw_angle,chassis_yaw_speed,chassis_yaw,chassis_angle_pid;
extern PID_TypeDef pan_tilt_pitch,pan_tilt_pitch_speed,pan_tilt_roll,pan_tilt_roll_speed,pan_tilt_yaw,pan_tilt_yaw_speed;
extern PID_TypeDef vision_yaw,vision_pitch;

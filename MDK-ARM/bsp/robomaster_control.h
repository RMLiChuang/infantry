#ifndef __ROBOMASTER_CONTROL
#define __ROBOMASTER_CONTROL

#include "robomaster_common.h"
//typedef struct {
//	float init;    //底盘上电标定yaw值
//	float mid;     //当左右摇杆回中时标定底盘的yaw值
//	
//	
//}IMU_Type;

#define N                  4         //滑动平均滤波参数

extern float Data[N];
extern int32_t set_spd;
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,int duty);
void shoot_control(void);
//void chassis_control(void);
void chassis_twist_control(void);
void chassis_follow_pan_tilt_control(void);
void chassis_speed_control(void);
void chassis_current_mix(void);
void set_current_zero(void);
void infantry_control(void);
void calibrate_initial_position(void);
float GildeAverageValueFilter(float NewValue,float *Data);
void set_chassis_moto_target_zero(void);
#endif


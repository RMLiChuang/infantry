#ifndef __ROBOMASTER_CONTROL
#define __ROBOMASTER_CONTROL

#include "robomaster_common.h"
//typedef struct {
//	float init;    //底盘上电标定yaw值
//	float mid;     //当左右摇杆回中时标定底盘的yaw值
//	
//	
//}IMU_Type;




extern int32_t set_spd;
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty);
void shoot_control(void);
void chassis_control(void);
void chassis_twist_control(void);
void chassis_follow_pan_tilt_control(void);
void chassis_speed_control(void);
void chassis_current_mix(void);
void set_current_zero(void);
#endif


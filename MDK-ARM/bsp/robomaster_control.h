#ifndef __ROBOMASTER_CONTROL
#define __ROBOMASTER_CONTROL

#include "robomaster_common.h"
//typedef struct {
//	float init;    //�����ϵ�궨yawֵ
//	float mid;     //������ҡ�˻���ʱ�궨���̵�yawֵ
//	
//	
//}IMU_Type;




extern int32_t set_spd;
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,float duty);
void shoot_control(void);
void chassis_control(void);
void walk_straight(void);
#endif

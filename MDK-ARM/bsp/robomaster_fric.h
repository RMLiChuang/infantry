#ifndef __ROBOMASTER_FRIC_H
#define __ROBOMASTER_FRIC_H
#include "robomaster_common.h"

//#define Fric_UP  1500
//#define Fric_ON  1200
//#define Fric_OFF 1000
extern int Close_Fric_ON,Mid_Fric_ON,Remote_Fric_ON,Intercontinental_Fric_ON,Fric_OFF;//1100对应于10m/s 1200对应于20m/s 1300对应于30m/s
void init_Fric_PWM(void);
void TIM_SetTIM5Compare(uint16_t compare);
void PWM_SetDuty(TIM_HandleTypeDef *tim,uint32_t tim_channel,int duty);
void close_fire(void);
void mid_fire(void);
void remote_fire(void);
void intercontinental_fire(void);
#endif



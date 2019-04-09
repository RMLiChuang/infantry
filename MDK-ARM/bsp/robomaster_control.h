#ifndef __ROBOMASTER_CONTROL
#define __ROBOMASTER_CONTROL

#include "robomaster_common.h"
//typedef struct {
//	float init;    //�����ϵ�궨yawֵ
//	float mid;     //������ҡ�˻���ʱ�궨���̵�yawֵ
//	
//	
//}IMU_Type;

#define N                  4         //����ƽ���˲�����
#define chassis_pan_tilt_max_rotate_speed 	150			//������̨�����ת�ٶ� ��λ��




extern float Data[N];
extern int32_t set_spd;




extern float pan_tilt_angle,pan_tilt_pit_angle,pan_tilt_yaw_angle;




//void chassis_control(void);
void chassis_twist_control(void);
void chassis_follow_pan_tilt_control(void);
void chassis_speed_control(void);
void chassis_current_mix(int16_t *output);
void set_current_zero(void);
void infantry_control(void);
void calibrate_initial_position(void);
float GildeAverageValueFilter(float NewValue,float *Data);
void set_chassis_moto_target_zero(void);

void chassis_angle_speed_control(void);
void get_chassis_to_pan_tilt_rad(void);
void get_chassis_to_pan_tilt_angle(void);
#endif


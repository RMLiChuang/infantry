#ifndef _PAN_TILT_CONTROL
#define _PAN_TILT_CONTROL
#include "robomaster_common.h"
extern int yaw_velocity_target;//yaw����ٶ�����Ŀ��ֵ
extern int pitch_velocity_target;//pitch����ٶ�����Ŀ��ֵ
extern float pitch_angle_target;//pitch��Ƕ�����Ŀ��ֵ
void PID_Pitch_control(void);
void pan_tilt_machine_home(void);
void pan_tilt_yaw_imu_angle_control(void);
void pan_tilt_yaw_mechanical_angle_control(void);
void pan_tilt_pitch_imu_angle_control(void);
void pan_tilt_pitch_mechanical_angle_control(void);
void pan_tilt_lock_control(void);
void pan_tilt_init(void);
#endif

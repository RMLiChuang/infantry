#ifndef _PAN_TILT_CONTROL
#define _PAN_TILT_CONTROL
#include "robomaster_common.h"
extern float steer_output;
extern int yaw_velocity_target;//yaw轴角速度输入目标值
extern int pitch_velocity_target;//pitch轴角速度输入目标值
void steer_control(void);
void pan_tilt_control(void);
void pan_tilt_machine_home(void);
void pan_tilt_yaw_imu_angle_control(void);
void pan_tilt_yaw_mechanical_angle_control(void);
void pan_tilt_pitch_imu_angle_control(void);
void pan_tilt_pitch_mechanical_angle_control(void);
void pan_tilt_lock_control(void);
#endif

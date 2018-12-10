#ifndef _PAN_TILT_CONTROL
#define _PAN_TILT_CONTROL
#include "robomaster_common.h"
extern float steer_output;
void steer_control(void);
void pan_tilt_control(void);
void pan_tilt_machine_home(void);
void pan_tilt_yaw_imu_angle_control(void);
void pan_tilt_yaw_mechanical_angle_control(void);
void pan_tilt_pitch_imu_angle_control(void);
void pan_tilt_pitch_mechanical_angle_control(void);
void pan_tilt_lock_control(void);
#endif

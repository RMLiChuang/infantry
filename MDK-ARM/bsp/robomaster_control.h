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
#define chassis_pan_tilt_max_rotate_speed 	150			//底盘云台最大旋转速度 单位度





typedef struct
{
  //const RC_ctrl_t *chassis_RC;               //底盘使用的遥控器指针
  //const Gimbal_Motor_t *chassis_yaw_motor;   //底盘使用到yaw云台电机的相对角度来计算底盘的欧拉角
  //const Gimbal_Motor_t *chassis_pitch_motor; //底盘使用到pitch云台电机的相对角度来计算底盘的欧拉角
  //const fp32 *chassis_INS_angle;             //获取陀螺仪解算出的欧拉角指针
  //chassis_mode_e chassis_mode;               //底盘控制状态机
  //chassis_mode_e last_chassis_mode;          //底盘上次控制状态机
  //Chassis_Motor_t motor_chassis[4];          //底盘电机数据
  //PidTypeDef motor_speed_pid[4];             //底盘电机速度pid
  //PidTypeDef chassis_angle_pid;              //底盘跟随角度pid

  //first_order_filter_type_t chassis_cmd_slow_set_vx;
  //first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;                         //底盘速度 前进方向 前为正，单位 m/s
  fp32 vy;                         //底盘速度 左右方向 左为正  单位 m/s
  fp32 wz;                         //底盘旋转角速度，逆时针为正 单位 rad/s
  fp32 vx_set;                     //底盘设定速度 前进方向 前为正，单位 m/s
  fp32 vy_set;                     //底盘设定速度 左右方向 左为正，单位 m/s
  fp32 wz_set;                     //底盘设定旋转角速度，逆时针为正 单位 rad/s
  fp32 chassis_relative_angle;     //底盘与云台的相对角度，单位 rad/s
  fp32 chassis_relative_angle_set; //设置相对云台控制角度
  fp32 chassis_yaw_set;

  fp32 vx_max_speed;  //前进方向最大速度 单位m/s
  fp32 vx_min_speed;  //前进方向最小速度 单位m/s
  fp32 vy_max_speed;  //左右方向最大速度 单位m/s
  fp32 vy_min_speed;  //左右方向最小速度 单位m/s
  fp32 chassis_yaw;   //陀螺仪和云台电机叠加的yaw角度
  fp32 chassis_pitch; //陀螺仪和云台电机叠加的pitch角度
  fp32 chassis_roll;  //陀螺仪和云台电机叠加的roll角度

} chassis_move_t;


extern float Data[N];
extern int32_t set_spd;




extern float pan_tilt_angle;




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


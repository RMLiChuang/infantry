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





typedef struct
{
  //const RC_ctrl_t *chassis_RC;               //����ʹ�õ�ң����ָ��
  //const Gimbal_Motor_t *chassis_yaw_motor;   //����ʹ�õ�yaw��̨�������ԽǶ���������̵�ŷ����
  //const Gimbal_Motor_t *chassis_pitch_motor; //����ʹ�õ�pitch��̨�������ԽǶ���������̵�ŷ����
  //const fp32 *chassis_INS_angle;             //��ȡ�����ǽ������ŷ����ָ��
  //chassis_mode_e chassis_mode;               //���̿���״̬��
  //chassis_mode_e last_chassis_mode;          //�����ϴο���״̬��
  //Chassis_Motor_t motor_chassis[4];          //���̵������
  //PidTypeDef motor_speed_pid[4];             //���̵���ٶ�pid
  //PidTypeDef chassis_angle_pid;              //���̸���Ƕ�pid

  //first_order_filter_type_t chassis_cmd_slow_set_vx;
  //first_order_filter_type_t chassis_cmd_slow_set_vy;

  fp32 vx;                         //�����ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy;                         //�����ٶ� ���ҷ��� ��Ϊ��  ��λ m/s
  fp32 wz;                         //������ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 vx_set;                     //�����趨�ٶ� ǰ������ ǰΪ������λ m/s
  fp32 vy_set;                     //�����趨�ٶ� ���ҷ��� ��Ϊ������λ m/s
  fp32 wz_set;                     //�����趨��ת���ٶȣ���ʱ��Ϊ�� ��λ rad/s
  fp32 chassis_relative_angle;     //��������̨����ԽǶȣ���λ rad/s
  fp32 chassis_relative_angle_set; //���������̨���ƽǶ�
  fp32 chassis_yaw_set;

  fp32 vx_max_speed;  //ǰ����������ٶ� ��λm/s
  fp32 vx_min_speed;  //ǰ��������С�ٶ� ��λm/s
  fp32 vy_max_speed;  //���ҷ�������ٶ� ��λm/s
  fp32 vy_min_speed;  //���ҷ�����С�ٶ� ��λm/s
  fp32 chassis_yaw;   //�����Ǻ���̨������ӵ�yaw�Ƕ�
  fp32 chassis_pitch; //�����Ǻ���̨������ӵ�pitch�Ƕ�
  fp32 chassis_roll;  //�����Ǻ���̨������ӵ�roll�Ƕ�

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


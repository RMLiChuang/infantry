/**
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 * @file       bsp_imu.h
 * @brief      this file contains the common defines and functions prototypes for 
 *             the bsp_imu.c driver
 * @note         
 * @Version    V1.0.0
 * @Date       Jan-30-2018      
 ***************************************(C) COPYRIGHT 2018 DJI***************************************
 */

#ifndef __MPU_H__
#define __MPU_H__

#include "mytype.h"
#define MPU_DELAY(x) HAL_Delay(x)


typedef struct {
  int16_t Mag_Data[3];
  float Mag_Data_Correct[3];//��������������ȡ�Ĳ���
  float thx; //������ǲ������ֵ
  float thy;//������ǲ������ֵ
  int16_t x;
  int16_t y;
  int16_t z;
  float Angle_Mag;
}IST8310;

//extern IST8310 Mag_IST8310;


typedef struct
{
	int16_t ax;
	int16_t ay;
	int16_t az;

	int16_t mx;
	int16_t my;
	int16_t mz;

	int16_t temp;

	int16_t gx;
	int16_t gy;
	int16_t gz;
	
	int16_t ax_offset;
	int16_t ay_offset;
	int16_t az_offset;

	int16_t gx_offset;
	int16_t gy_offset;
	int16_t gz_offset;
} mpu_data_t;

typedef struct
{
	int16_t ax;   //���ٶ�
	int16_t ay;
	int16_t az;

	int16_t mx;   //������
	int16_t my;
	int16_t mz;

	float temp;

	float wx;    //���ٶ�  ������  ԭʼ����   /*!< omiga, +- 2000dps => +-32768  so gx/16.384/57.3 =	rad/s */
	float wy;
	float wz;  
   
	float gx;//���ٶ�  �Ƕ��� ԭʼ����
	float gy;
	float gz;
	
	float vx;
	float vy;
	float vz;

	float rol;
	float pit;
	float yaw;
} imu_t;
extern char mag_flag;

//extern bool HEAT_FINISH;//�¶Ȳ������ ��־λ
extern float X_g_av,Y_g_av,Z_g_av;//���õļ��ٶȼ�ֵ
extern float X_g_av_bpf,Y_g_av_bpf,Z_g_av_bpf;//�����˲�����õļ��ٶȼ�ֵ
extern float X_m_av,Y_m_av,Z_m_av;//�����˲���Ĵ����ƶ���
extern mpu_data_t mpu_data;
extern imu_t      imu;
extern float halfT;
uint8_t   mpu_device_init(void);
extern float mag_field_intensity;//�ų�ǿ��
void init_quaternion(void);
void mpu_get_data(void);
void imu_ahrs_update(void);
void imu_attitude_update(void);
void imu_cloud_getdata(void);
void mpu_offset_call(void);
void imu_cloud_cal(void);
#endif



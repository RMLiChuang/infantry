#ifndef __ROBOMASTER_VISION_H
#define __ROBOMASTER_VISION_H
#include "robomaster_common.h"
#define VISION_YAW_TARGET 310 //视觉yaw的目标值
#define VISION_PIT_TARGET 270 //视觉pit的目标值

#define VISION_DETECT 1 //使能视觉识别
#define ENEMYDATABUFFERLENGHT   60 //敌人数据帧
//角度数据结构
typedef struct
{
		int16_t origin_yaw;
		int16_t origin_pitch;
    int16_t Yaw_Err;//yaw偏差
    int16_t Pitch_Err;
		
		
}Angle_Err_Struct;
//点数据结构
typedef struct
{
    float X;
    float Y;
    float Z;
}Point_Struct;
//视觉打击结构体
typedef struct
{
    Angle_Err_Struct pan_tilt_angel_err;//云台与装甲板的yaw，pitch轴偏差
		bool recognition_success_flag; //识别成功标志位
		int check_sum;//检验数据是否正确
		uint32_t last_time;//上一帧接收视觉的时间
		uint32_t current_time;//此时接收视觉的时间
		uint16_t delta_time;//接收两帧数据的间隔时间
		uint8_t vision_frame;//视觉帧率
		
}Vision_Attack;
//视觉数据结构
typedef struct
{
    uint16_t X;//对应云台的YAW
		uint16_t Y;//对应于云台的PIT
		uint16_t Z;//对应于相机相对于装甲的距离
    int TimeStamp;
    int Time;
    char ID;
    uint32_t Tick;
}Enemy_Struct;
extern Enemy_Struct EnemyDataBuffer[ENEMYDATABUFFERLENGHT];
extern uint8_t EnemyDataBufferPoint;//主机解码数据缓存指针
extern uint8_t UART6_Date[8];//串口6接收pid调参数据
extern Vision_Attack Armour_attack;//装甲板识别结构体
extern uint8_t USART6_RX_BUF;
void armour_attack(void);
void get_armour_err(void);
uint8_t ForcastCore(uint16_t SampleTime, uint16_t ForcastTime, Point_Struct *ForcastPoint);
#endif


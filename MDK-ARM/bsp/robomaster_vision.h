#ifndef __ROBOMASTER_VISION_H
#define __ROBOMASTER_VISION_H
#include "robomaster_common.h"


#define VISION_DETECT 1 //使能视觉识别

//角度数据结构
typedef struct
{
		int16_t origin_yaw;
		int16_t origin_pitch;
    int16_t Yaw_Err;
    int16_t Pitch_Err;
		
}Angle_Err_Struct;

//视觉打击结构体
typedef struct
{
    Angle_Err_Struct pan_tilt_angel_err;//云台与装甲板的yaw，pitch轴偏差
		bool recognition_success_flag; //识别成功标志位
		int check_sum;//检验数据是否正确
}Vision_Attack;

extern uint32_t  Latest_Vision_Control_Pack_Time;
extern uint32_t vision_time;//用于检测使视觉识别是否离线
extern uint16_t UART6_Date[8];//串口6接收pid调参数据
extern u8 Usart_Flag;
extern Vision_Attack Armour_attack;//装甲板识别结构体
extern uint8_t USART6_RX_BUF;
void armour_attack(void);
void get_armour_err(void);
#endif


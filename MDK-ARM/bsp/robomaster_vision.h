#ifndef __ROBOMASTER_VISION_H
#define __ROBOMASTER_VISION_H
#include "robomaster_common.h"
#define VISION_YAW_TARGET 310 //�Ӿ�yaw��Ŀ��ֵ
#define VISION_PIT_TARGET 270 //�Ӿ�pit��Ŀ��ֵ

#define VISION_DETECT 1 //ʹ���Ӿ�ʶ��
#define ENEMYDATABUFFERLENGHT   60 //��������֡
//�Ƕ����ݽṹ
typedef struct
{
		int16_t origin_yaw;
		int16_t origin_pitch;
    int16_t Yaw_Err;//yawƫ��
    int16_t Pitch_Err;
		
		
}Angle_Err_Struct;
//�����ݽṹ
typedef struct
{
    float X;
    float Y;
    float Z;
}Point_Struct;
//�Ӿ�����ṹ��
typedef struct
{
    Angle_Err_Struct pan_tilt_angel_err;//��̨��װ�װ��yaw��pitch��ƫ��
		bool recognition_success_flag; //ʶ��ɹ���־λ
		int check_sum;//���������Ƿ���ȷ
		uint32_t last_time;//��һ֡�����Ӿ���ʱ��
		uint32_t current_time;//��ʱ�����Ӿ���ʱ��
		uint16_t delta_time;//������֡���ݵļ��ʱ��
		uint8_t vision_frame;//�Ӿ�֡��
		
}Vision_Attack;
//�Ӿ����ݽṹ
typedef struct
{
    uint16_t X;//��Ӧ��̨��YAW
		uint16_t Y;//��Ӧ����̨��PIT
		uint16_t Z;//��Ӧ����������װ�׵ľ���
    int TimeStamp;
    int Time;
    char ID;
    uint32_t Tick;
}Enemy_Struct;
extern Enemy_Struct EnemyDataBuffer[ENEMYDATABUFFERLENGHT];
extern uint8_t EnemyDataBufferPoint;//�����������ݻ���ָ��
extern uint8_t UART6_Date[8];//����6����pid��������
extern Vision_Attack Armour_attack;//װ�װ�ʶ��ṹ��
extern uint8_t USART6_RX_BUF;
void armour_attack(void);
void get_armour_err(void);
uint8_t ForcastCore(uint16_t SampleTime, uint16_t ForcastTime, Point_Struct *ForcastPoint);
#endif


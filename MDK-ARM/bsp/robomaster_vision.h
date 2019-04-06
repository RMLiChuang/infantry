#ifndef __ROBOMASTER_VISION_H
#define __ROBOMASTER_VISION_H
#include "robomaster_common.h"


#define VISION_DETECT 1 //ʹ���Ӿ�ʶ��

//�Ƕ����ݽṹ
typedef struct
{
		int16_t origin_yaw;
		int16_t origin_pitch;
    int16_t Yaw_Err;
    int16_t Pitch_Err;
		
}Angle_Err_Struct;

//�Ӿ�����ṹ��
typedef struct
{
    Angle_Err_Struct pan_tilt_angel_err;//��̨��װ�װ��yaw��pitch��ƫ��
		bool recognition_success_flag; //ʶ��ɹ���־λ
		int check_sum;//���������Ƿ���ȷ
}Vision_Attack;

extern uint32_t  Latest_Vision_Control_Pack_Time;
extern uint32_t vision_time;//���ڼ��ʹ�Ӿ�ʶ���Ƿ�����
extern uint16_t UART6_Date[8];//����6����pid��������
extern u8 Usart_Flag;
extern Vision_Attack Armour_attack;//װ�װ�ʶ��ṹ��
extern uint8_t USART6_RX_BUF;
void armour_attack(void);
void get_armour_err(void);
#endif


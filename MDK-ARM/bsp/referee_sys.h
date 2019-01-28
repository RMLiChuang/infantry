/*************************************************************************
 * Copyright (c) 2018, ����RM 
 * All rights reserved.
 * 
 * File name    :   referee_sys.h
 * Brief        :   referee_sys.c��ͷ�ļ�
 * Revision     :   1.0
 * Author       :   ���¸�
 * Date         :   2018.03.24
 �޸ģ�2019.1.14�����벻ͬ���ݱ�־λ
*************************************************************************/
#ifndef __REFEREE_SYS_H
#define __REFEREE_SYS_H

/* Includes -----------------------------------------------------------*/
#include "usart.h"

/* �궨��������󳤶�*/
#define datamaxlen 255
/* �궨�����ݰ�ͷ*/
#define DataHeader 0xA5


/* �궨�����ݰ�CMD*/
#define RM_status      0x01
#define RM_damage_data 0x02
#define RM_shoot_data  0x03
#define RM_WQ_data     0x04
#define RM_RFID_data   0x05
#define Match_results  0x06
#define Buff_status    0x07
#define RM_self_data   0x08

/* �궨�����ݰ�CMD*/
#define CMD1_LEN      8
#define CMD2_LEN      1
#define CMD3_LEN      6
#define CMD4_LEN     20
#define CMD5_LEN      2
#define CMD6_LEN      1
#define CMD7_LEN      2
#define CMD8_LEN     16

#define cal_cmd_len(x)    5 + 2 + x + 2     //FH = 5 2��CMD x�����ȵ����� 2��CRC16У��

/* ����������״̬--0x001*/
typedef __packed struct
{
	uint16_t stageremaintime;   //��ǰʣ��ʱ�䣬��λS

	uint16_t gameprogress;      //�������̣�0-δ��ʼ������1-׼���׶Σ�2-�Լ�׶Σ�3-5s����ʱ��4-��ս�׶Σ�5-����������

  uint8_t  robotlevel;        //�����˵�ǰ�ȼ�

	uint16_t remainHP;          //�����˵�ǰѪ��

  uint16_t maxHP;             //��������Ѫ��


}extGameRobotState_t;



/* �˺�����--0x002
armorType װ��ID(0-3bit)
0x0--0��װ�ף�ǰ��
0x1--1��װ�ף���
0x2--2��װ�ף���
0x3--3��װ�ף��ң�
0x4--4��װ�ף���1��
0x5--5��װ�ף���2��

hurtType   �˺�����(4-7bit)
0x0-װ���˺�
0x1-ģ�����

*/
typedef __packed struct
{

    uint8_t  armorType;
    uint8_t  hurtType;

}extRobotHurt_t;


/* ʵʱ�����Ϣ--0x003
bulletType����������
1: 17mm����
2: 43mm����

bulletFreq����ÿ��

bulletSpeed����ÿ��
*/
typedef __packed struct
{

    uint8_t  bulletType;
    uint8_t  bulletFreq;
    float bulletSpeed;

}extShoot_t;


/* ʵʱ������������--0x004
chassisVolt:���������ѹ ��
chassisCurrent:����������� ��
chassisPower:����������� ��
chassisPowerBuffer:���̹��ʻ��� ��
ǹ������
ShooterHeat_17mm:
ShooterHeat_42mm:
*/
typedef __packed struct
{
	float chassisVolt;
    float chassisCurrent;
    float chassisPower;
    float chassisPowerBuffer;

    uint16_t  ShooterHeat_17mm;
    uint16_t  ShooterHeat_42mm;


}extPowerHeatData_t;

/* ���ؽ�������--0x005
cardType��������
0�������ӳɿ�
1�������ӳɿ�
2���췽��Ѫ��
3��������Ѫ��
4���췽���ƿ�
5���������ƿ�
6���췽��ȴ��
7��������ȴ��
8���ﱤ��
9������
10����Դ����
11��ICRA���������ش���㿨

cardldx����������
*/
typedef __packed struct
{
    uint8_t  cardType;
    uint8_t  cardldx;

}extRfidDetect_t;

/* ����ʤ������--0x006
0:ƽ��
1:�췽ʤ
2:����ʤ
*/
typedef __packed struct
{
    uint8_t  winner;

}extGameResult_t;

/* Buff��ȡ����--0x007
bit0: ��Ѫ���Ѫ
bit1: ���̻����˻�Ѫ
bit2: ���ƿ���Ѫ
bit3: ��Դ������
bit4: �������������������
bit5: �з����������������
bit6: ����������С��������
bit7: �з�������С��������
bit8: ������ȴ
bit9: �ﱤ����
bit10: �ٷְٷ���
bit11: ���ڱ����ط���
bit12: ���ڱ����ط���

*/
typedef __packed struct
{

    uint16_t  buffMusk;

}extBuffMusk_t;



/* ������λ�ó�����Ϣ--0x008
�ֽ�ƫ��
0       --λ��x��
4       --λ��y��
8       --λ��z��
12      --ǹ�ڳ���Ƕ�ֵ����
*/
typedef __packed struct
{
	  float x;
    float y;
    float z;
    float yaw;

}extGameRobotPos_t;

typedef __packed struct
{
    extGameRobotState_t GameRobotState_t;

    extRobotHurt_t RobotHurt_t;

    extShoot_t Shoot_t;

    extPowerHeatData_t PowerHeatData_t;

    extRfidDetect_t RfidDetect_t;

    extBuffMusk_t BuffMusk_t;

    extGameRobotPos_t GameRobotPos_t;

}refDataStruct;

/* Declaration ------------------��������--------------------------------------*/
void get_referee_data(void);
void uart_receive_idle(UART_HandleTypeDef *huart);
void ref_sys_init(void);
void display(void);
extern refDataStruct  refSysData; 
#endif


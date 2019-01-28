/*************************************************************************
 * Copyright (c) 2018, ����RM
 * All rights reserved.
 *
 * File name    :   referee_sys.c
 * Brief        :   ����ϵͳAPI��������
 *                  ��ȡ����ϵͳ����������
 * Revision     :   1.0
 * Author       :   ���¸�
 * Date         :   2018.03.24
 *************************************************************************/
#include "referee_sys.h"
#include "stdio.h"

#include "string.h"
#include <stdlib.h>



uint8_t data1[cal_cmd_len(CMD1_LEN)];
uint8_t data2[cal_cmd_len(CMD2_LEN)];
uint8_t data3[cal_cmd_len(CMD3_LEN)];
uint8_t data4[cal_cmd_len(CMD4_LEN)];
uint8_t data5[cal_cmd_len(CMD5_LEN)];
uint8_t data6[cal_cmd_len(CMD6_LEN)];
uint8_t data7[cal_cmd_len(CMD7_LEN)];
uint8_t data8[cal_cmd_len(CMD8_LEN)];

uint8_t data_flag = 0;          //���ݸ��±�־

uint8_t usart2_rx_flag = 0;
uint8_t usart2_rx_buffer[datamaxlen];
uint8_t usart2_rx_len = 0, rx_len_cal = 0;
uint8_t dis_flag = 0;           //���Ա�־λ
uint8_t status_flag=0,damage_flag=0,shoot_flag=0,WQ_flag=0,RFID_flag=0,results_flag=0,Buff_flag=0,self_flag=0; //���ձ�־λ

char eeror_data = 1;
char sysflag=0;    //���ڵ��߱�־λ

refDataStruct  refSysData;      //�������ݽṹ�壬����ÿ����������ʲô���Ѿ���.h�ļ�����˵��

//crc8У���
const uint8_t CRC8_TAB[] =
{
	0x0, 0x5e, 0xbc, 0xe2, 0x61, 0x3f, 0xdd, 0x83, 0xc2, 0x9c, 0x7e, 0x20, 0xa3, 0xfd, 0x1f, 0x41,
	0x9d, 0xc3, 0x21, 0x7f, 0xfc, 0xa2, 0x40, 0x1e, 0x5f, 0x1, 0xe3, 0xbd, 0x3e, 0x60, 0x82, 0xdc,
	0x23, 0x7d, 0x9f, 0xc1, 0x42, 0x1c, 0xfe, 0xa0, 0xe1, 0xbf, 0x5d, 0x3, 0x80, 0xde, 0x3c, 0x62,
	0xbe, 0xe0, 0x2, 0x5c, 0xdf, 0x81, 0x63, 0x3d, 0x7c, 0x22, 0xc0, 0x9e, 0x1d, 0x43, 0xa1, 0xff,
	0x46, 0x18, 0xfa, 0xa4, 0x27, 0x79, 0x9b, 0xc5, 0x84, 0xda, 0x38, 0x66, 0xe5, 0xbb, 0x59, 0x7,
	0xdb, 0x85, 0x67, 0x39, 0xba, 0xe4, 0x6, 0x58, 0x19, 0x47, 0xa5, 0xfb, 0x78, 0x26, 0xc4, 0x9a,
	0x65, 0x3b, 0xd9, 0x87, 0x4, 0x5a, 0xb8, 0xe6, 0xa7, 0xf9, 0x1b, 0x45, 0xc6, 0x98, 0x7a, 0x24,
	0xf8, 0xa6, 0x44, 0x1a, 0x99, 0xc7, 0x25, 0x7b, 0x3a, 0x64, 0x86, 0xd8, 0x5b, 0x5, 0xe7, 0xb9,
	0x8c, 0xd2, 0x30, 0x6e, 0xed, 0xb3, 0x51, 0xf, 0x4e, 0x10, 0xf2, 0xac, 0x2f, 0x71, 0x93, 0xcd,
	0x11, 0x4f, 0xad, 0xf3, 0x70, 0x2e, 0xcc, 0x92, 0xd3, 0x8d, 0x6f, 0x31, 0xb2, 0xec, 0xe, 0x50,
	0xaf, 0xf1, 0x13, 0x4d, 0xce, 0x90, 0x72, 0x2c, 0x6d, 0x33, 0xd1, 0x8f, 0xc, 0x52, 0xb0, 0xee,
	0x32, 0x6c, 0x8e, 0xd0, 0x53, 0xd, 0xef, 0xb1, 0xf0, 0xae, 0x4c, 0x12, 0x91, 0xcf, 0x2d, 0x73,
	0xca, 0x94, 0x76, 0x28, 0xab, 0xf5, 0x17, 0x49, 0x8, 0x56, 0xb4, 0xea, 0x69, 0x37, 0xd5, 0x8b,
	0x57, 0x9, 0xeb, 0xb5, 0x36, 0x68, 0x8a, 0xd4, 0x95, 0xcb, 0x29, 0x77, 0xf4, 0xaa, 0x48, 0x16,
	0xe9, 0xb7, 0x55, 0xb, 0x88, 0xd6, 0x34, 0x6a, 0x2b, 0x75, 0x97, 0xc9, 0x4a, 0x14, 0xf6, 0xa8,
	0x74, 0x2a, 0xc8, 0x96, 0x15, 0x4b, 0xa9, 0xf7, 0xb6, 0xe8, 0xa, 0x54, 0xd7, 0x89, 0x6b, 0x35
};

//crc16 У���
const uint16_t CRC16_TAB[256] =
{
	0x0000, 0x1189, 0x2312, 0x329b, 0x4624, 0x57ad, 0x6536, 0x74bf,
	0x8c48, 0x9dc1, 0xaf5a, 0xbed3, 0xca6c, 0xdbe5, 0xe97e, 0xf8f7,
	0x1081, 0x0108, 0x3393, 0x221a, 0x56a5, 0x472c, 0x75b7, 0x643e,
	0x9cc9, 0x8d40, 0xbfdb, 0xae52, 0xdaed, 0xcb64, 0xf9ff, 0xe876,
	0x2102, 0x308b, 0x0210, 0x1399, 0x6726, 0x76af, 0x4434, 0x55bd,
	0xad4a, 0xbcc3, 0x8e58, 0x9fd1, 0xeb6e, 0xfae7, 0xc87c, 0xd9f5,
	0x3183, 0x200a, 0x1291, 0x0318, 0x77a7, 0x662e, 0x54b5, 0x453c,
	0xbdcb, 0xac42, 0x9ed9, 0x8f50, 0xfbef, 0xea66, 0xd8fd, 0xc974,
	0x4204, 0x538d, 0x6116, 0x709f, 0x0420, 0x15a9, 0x2732, 0x36bb,
	0xce4c, 0xdfc5, 0xed5e, 0xfcd7, 0x8868, 0x99e1, 0xab7a, 0xbaf3,
	0x5285, 0x430c, 0x7197, 0x601e, 0x14a1, 0x0528, 0x37b3, 0x263a,
	0xdecd, 0xcf44, 0xfddf, 0xec56, 0x98e9, 0x8960, 0xbbfb, 0xaa72,
	0x6306, 0x728f, 0x4014, 0x519d, 0x2522, 0x34ab, 0x0630, 0x17b9,
	0xef4e, 0xfec7, 0xcc5c, 0xddd5, 0xa96a, 0xb8e3, 0x8a78, 0x9bf1,
	0x7387, 0x620e, 0x5095, 0x411c, 0x35a3, 0x242a, 0x16b1, 0x0738,
	0xffcf, 0xee46, 0xdcdd, 0xcd54, 0xb9eb, 0xa862, 0x9af9, 0x8b70,
	0x8408, 0x9581, 0xa71a, 0xb693, 0xc22c, 0xd3a5, 0xe13e, 0xf0b7,
	0x0840, 0x19c9, 0x2b52, 0x3adb, 0x4e64, 0x5fed, 0x6d76, 0x7cff,
	0x9489, 0x8500, 0xb79b, 0xa612, 0xd2ad, 0xc324, 0xf1bf, 0xe036,
	0x18c1, 0x0948, 0x3bd3, 0x2a5a, 0x5ee5, 0x4f6c, 0x7df7, 0x6c7e,
	0xa50a, 0xb483, 0x8618, 0x9791, 0xe32e, 0xf2a7, 0xc03c, 0xd1b5,
	0x2942, 0x38cb, 0x0a50, 0x1bd9, 0x6f66, 0x7eef, 0x4c74, 0x5dfd,
	0xb58b, 0xa402, 0x9699, 0x8710, 0xf3af, 0xe226, 0xd0bd, 0xc134,
	0x39c3, 0x284a, 0x1ad1, 0x0b58, 0x7fe7, 0x6e6e, 0x5cf5, 0x4d7c,
	0xc60c, 0xd785, 0xe51e, 0xf497, 0x8028, 0x91a1, 0xa33a, 0xb2b3,
	0x4a44, 0x5bcd, 0x6956, 0x78df, 0x0c60, 0x1de9, 0x2f72, 0x3efb,
	0xd68d, 0xc704, 0xf59f, 0xe416, 0x90a9, 0x8120, 0xb3bb, 0xa232,
	0x5ac5, 0x4b4c, 0x79d7, 0x685e, 0x1ce1, 0x0d68, 0x3ff3, 0x2e7a,
	0xe70e, 0xf687, 0xc41c, 0xd595, 0xa12a, 0xb0a3, 0x8238, 0x93b1,
	0x6b46, 0x7acf, 0x4854, 0x59dd, 0x2d62, 0x3ceb, 0x0e70, 0x1ff9,
	0xf78f, 0xe606, 0xd49d, 0xc514, 0xb1ab, 0xa022, 0x92b9, 0x8330,
	0x7bc7, 0x6a4e, 0x58d5, 0x495c, 0x3de3, 0x2c6a, 0x1ef1, 0x0f78
};

/******************************************************
* Brief     : 16����תΪ������
* Parameter :
*      pdata: ����ȥ4��С�������ڴ洢��õ�ʮ�����Ƶ�ַ
* Return    : ����õ�С��
*******************************************************/
static float hex_to_float(uint8_t *pdata)
{
	uint8_t i;
	union  hex{
		uint8_t buf[4];
		float tmp;
	};
	union  hex hex_foloat;
	for (i = 0; i < 4; i++)
		hex_foloat.buf[i] = *(pdata++);
	return  hex_foloat.tmp;
}
/******************************************************
* Brief     : ��ȡCRC8��У��ֵ
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ����ö�Ӧ���ݵ�CRC8��ֵ
*******************************************************/
uint8_t get_crc8(uint8_t* pdata, uint8_t nLen)
{
	uint8_t crc8 = 0xff;

	while (nLen--)
	{
		crc8 = CRC8_TAB[crc8 ^ (*pdata++)];
	}
	return(crc8);
}

/******************************************************
* Brief     : ��ȡCRC16��У��ֵ
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ����ö�Ӧ���ݵ�CRC16��ֵ
*******************************************************/
uint16_t get_crc16(uint8_t* pData, uint8_t nLen)
{
	uint16_t crc16 = 0xffff;    // ��ʼ��
	while (nLen--)
	{
		crc16 = (crc16 >> 8) ^ CRC16_TAB[(crc16^ *pData++) & 0xff];
	}
	return crc16;
}

/******************************************************
* Brief     : �˲����ϵͳ���͹����������Ƿ���ȷ
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : 0�����ݴ���  1��������ȷ
*******************************************************/
uint8_t verify_check(uint8_t *pdata, uint8_t nlen)
{
	uint8_t tmp[4];
	uint16_t tmp1;
	uint8_t num;

	if (pdata == NULL || nlen <= 2 || nlen > 45)						//���жϴ���������Ƿ�Ϸ�
		return 0;

	for (num = 0; num < 4; num++)									//��ȡ��ͷ
		tmp[num] = *(pdata++);

	if (*pdata == get_crc8(tmp, sizeof(tmp)))					//��У��ͷcrc8
	{
		pdata -= 4; // ��ָ��ָ�ؿ�ʼ
		tmp1 = ((*(pdata + (nlen - 1))) << 8) | (*(pdata + (nlen - 2))); // ��ȡ��������crc16
		if (tmp1 == get_crc16(pdata, (nlen - 2)))				//��У��crc16
			return 1;
		else
			return 0;
	}
	else
		return 0;
}

/******************************************************
* Brief     : ��������ϵͳ���ݰ���1��CMD-0x0001
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ��
*******************************************************/
static char cal_ref_data1(unsigned char *pdata, unsigned char nlen)
{
	if (verify_check(pdata, nlen))
	{
		refSysData.GameRobotState_t.stageremaintime = (*(pdata + 8)) << 8 | *(pdata + 7);
		refSysData.GameRobotState_t.gameprogress = *(pdata + 9);
		refSysData.GameRobotState_t.robotlevel = *(pdata + 10);
		refSysData.GameRobotState_t.remainHP = (*(pdata + 12)) << 8 | *(pdata + 11);
		refSysData.GameRobotState_t.maxHP = (*(pdata + 14)) << 8 | *(pdata + 13);
        return 1;
	}
    else
    {
        return 0;
    }
}

/******************************************************
* Brief     : ��������ϵͳ���ݰ���2��CMD-0x0002
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ��
*******************************************************/
static char cal_ref_data2(unsigned char *pdata, unsigned char nlen)
{
	if (verify_check(pdata, nlen))
	{
		refSysData.RobotHurt_t.armorType = *(pdata + 7) & 0x0f;
		refSysData.RobotHurt_t.hurtType = *(pdata + 7) & 0xf0;
        return 1;

	}
    else
    {
        return 0;
    
    }

}


/******************************************************
* Brief     : ��������ϵͳ���ݰ���3��CMD-0x0003
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ��
*******************************************************/
static char cal_ref_data3(unsigned char *pdata, unsigned char nlen)
{
	if (verify_check(pdata, nlen))
	{
		refSysData.Shoot_t.bulletType = *(pdata + 7);
		refSysData.Shoot_t.bulletFreq = *(pdata + 9);
		refSysData.Shoot_t.bulletSpeed = hex_to_float(pdata + 10);
        return 1;
	}
    else
    {
        return 0;
    }
}

/******************************************************
* Brief     : ��������ϵͳ���ݰ���4��CMD-0x0004       8
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ��
*******************************************************/
static char cal_ref_data4(unsigned char *pdata, unsigned char nlen)
{
	if (verify_check(pdata, nlen))
	{
		refSysData.PowerHeatData_t.chassisVolt = hex_to_float(pdata + 7);
		refSysData.PowerHeatData_t.chassisCurrent = hex_to_float(pdata + 11);
		refSysData.PowerHeatData_t.chassisPower = hex_to_float(pdata + 15);
		refSysData.PowerHeatData_t.chassisPowerBuffer = hex_to_float(pdata + 19);
		refSysData.PowerHeatData_t.ShooterHeat_17mm = (*(pdata + 24)) << 8 | *(pdata + 23);
		refSysData.PowerHeatData_t.ShooterHeat_42mm = (*(pdata + 26)) << 8 | *(pdata + 25);
        return 1;

	}
    else
    {
        return 0;
    
    }

}

/******************************************************
* Brief     : ��������ϵͳ���ݰ���5��CMD-0x0005
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ��
*******************************************************/
static char cal_ref_data5(unsigned char *pdata, unsigned char nlen)
{
	if (verify_check(pdata, nlen))
	{
		refSysData.RfidDetect_t.cardldx = *(pdata + 7);
		refSysData.RfidDetect_t.cardType = *(pdata + 8);
        return 1;

	}
    else
    {
        return 0;
    
    }

}

/******************************************************
* Brief     : ��������ϵͳ���ݰ���7��CMD-0x0007
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ��
*******************************************************/
static char cal_ref_data7(unsigned char *pdata, unsigned char nlen)
{
	if (verify_check(pdata, nlen))
	{
		refSysData.BuffMusk_t.buffMusk = *(pdata + 7);
        return 1;

	}
    else
    {
        return 0;
    
    }

}

/******************************************************
* Brief     : ��������ϵͳ���ݰ���8��CMD-0x0008
* Parameter :
*      pdata: ��ҪУ��������ַ
*      nLen : ����ȥ�����ݳ���
* Return    : ��
*******************************************************/
static char cal_ref_data8(unsigned char *pdata, unsigned char nlen)
{
	if (verify_check(pdata, nlen))
	{
		refSysData.GameRobotPos_t.x = hex_to_float(pdata + 7);
		refSysData.GameRobotPos_t.y = hex_to_float(pdata + 11);
		refSysData.GameRobotPos_t.z = hex_to_float(pdata + 15);
		refSysData.GameRobotPos_t.yaw = (hex_to_float(pdata + 19))*10;
        return 1;

	}
    else
    {
        return 0;
    
    }

}

/******************************************************
* Brief     : ����ϵͳ���ݳ�ʼ��
* Parameter : ��
* Return    : ��
*******************************************************/
static void ref_sys()
{

	/*�Խṹ������б������г�ʼ��*/
	refSysData.GameRobotState_t.gameprogress = 0;
	refSysData.GameRobotState_t.maxHP = 0;
	refSysData.GameRobotState_t.remainHP = 0;
	refSysData.GameRobotState_t.robotlevel = 0;
	refSysData.GameRobotState_t.stageremaintime = 0;

	refSysData.RobotHurt_t.armorType = 0;
	refSysData.RobotHurt_t.hurtType = 0;

	refSysData.Shoot_t.bulletFreq = 0;
	refSysData.Shoot_t.bulletSpeed = 0;
	refSysData.Shoot_t.bulletType = 0;

	refSysData.PowerHeatData_t.chassisCurrent = 0;
	refSysData.PowerHeatData_t.chassisPower = 0;
	refSysData.PowerHeatData_t.chassisPowerBuffer = 0;
	refSysData.PowerHeatData_t.chassisVolt = 0;
	refSysData.PowerHeatData_t.ShooterHeat_17mm = 0;
	refSysData.PowerHeatData_t.ShooterHeat_42mm = 0;


	refSysData.RfidDetect_t.cardldx = 0;
	refSysData.RfidDetect_t.cardType = 0;

	refSysData.BuffMusk_t.buffMusk = 0;

	refSysData.GameRobotPos_t.x = 0;
	refSysData.GameRobotPos_t.y = 0;
	refSysData.GameRobotPos_t.z = 0;
	refSysData.GameRobotPos_t.yaw = 0;
}
/******************************************************
* Brief     : ���²���ϵͳ���ݽṹ��
* Parameter : ��
* Return    : ��
*******************************************************/
void get_referee_data(void)
{
	/*�ж������Ƿ����*/
	if (usart2_rx_flag == 1)
	{
		/*���ݱ�־λȷ���Ƿ���Ҫ������һ����*/
//		if (data_flag == 1)
//		{
//			cal_ref_data1(data1, 44);
//			data_flag = 0;
//		}
//		else if (data_flag == 2)
//		{
//			cal_ref_data1(data1, 44);
//			cal_ref_data2(data2, 12);
//			data_flag = 0;
//		}
//		else if (data_flag == 3)
//		{
//			cal_ref_data1(data1, 44);
//			cal_ref_data3(data3, 25);
//			data_flag = 0;
//		}
//		else if (data_flag == 4)
//		{
//			cal_ref_data1(data1, 44);
//			cal_ref_data2(data2, 12);
//			cal_ref_data3(data3, 25);
//			data_flag = 0;
//		}
		usart2_rx_flag = 0;
	}
}
/******************************************************
* Brief     : ��ʾ���ݺ���
* Parameter : ��
* Return    : ��
*******************************************************/
void display(void)
{
//	printf("out");
	if (dis_flag == 1)
	{
//		cal_ref_data4(data4,20);
//		printf("ʣ��Ѫ��:%d\n", refSysData.GameRobotState_t.remainHP);
//		printf("���̵�ѹ:%0.2f\n", refSysData.PowerHeatData_t.chassisVolt);
//		printf("���̵���:%0.2f\n", refSysData.PowerHeatData_t.chassisCurrent);
		if(damage_flag==1)
		{
			damage_flag=0;
			printf("����װ��:%d\n", refSysData.RobotHurt_t.armorType);
		}
		printf("���̹���:%0.2f\n", refSysData.PowerHeatData_t.chassisPower);
//		printf("���ʻ���:%0.2f\n", refSysData.PowerHeatData_t.chassisPowerBuffer);
//		printf("\n");
//		printf("����װ��:%d\n", refSysData.RobotHurt_t.armorType);
//		printf("���˷�ʽ:%d\n", refSysData.RobotHurt_t.hurtType);
//		//printf("Ѫ���仯ֵ:%d\n",refSysData.GameRobotState_t.);
//    	printf("ǹ�ڳ���Ƕ�ֵ:%f\n", refSysData.GameRobotPos_t.yaw);
//    
//		printf("\n");
//		if (refSysData.Shoot_t.bulletType == 1)
//		{
//			printf("�����ӵ��ٶ�:%d\n", refSysData.Shoot_t.bulletFreq);
//			printf("�����ӵ�Ƶ��:%0.2f\n", refSysData.Shoot_t.bulletSpeed);
//		}
//		if (refSysData.Shoot_t.bulletType == 2)
//		{
//			printf("Ӣ���ӵ��ٶ�:%d\n", refSysData.Shoot_t.bulletFreq);
//			printf("Ӣ���ӵ�Ƶ��:%0.2f\n", refSysData.Shoot_t.bulletSpeed);
//		}
//		printf("\n");
		dis_flag = 0;
	}
}

//---------------------------������س�ʼ��----------------------------------------------

DMA_HandleTypeDef hdma_usart2_rx;    //DMA���   ������INIT_DMA.c�ļ������������ｫ�����ε�
UART_HandleTypeDef huart2;          //uart2��� ������INIT_USART.c�ļ������������ｫ�����ε�

/******************************************************
* Brief     : �����ʼ��
* Parameter : ��
* Return    : ��
*******************************************************/
void hal_uart2_MspInit(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	__HAL_RCC_USART2_CLK_ENABLE();
	__HAL_RCC_DMA1_CLK_ENABLE();

	/**USART2 GPIO Configuration
	PD6     ------> USART2_RX
	PD5     ------> USART2_TX
	*/
	//    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5;
	//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	//    GPIO_InitStruct.Pull = GPIO_PULLUP;
	//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	//    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

	GPIO_InitStruct.Pin = GPIO_PIN_2 | GPIO_PIN_3;
	GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);


	/* USART2 DMA Init */
	/* USART2_RX Init */
	hdma_usart2_rx.Instance = DMA1_Stream5;
	hdma_usart2_rx.Init.Channel = DMA_CHANNEL_4;
	hdma_usart2_rx.Init.Direction = DMA_PERIPH_TO_MEMORY;
	hdma_usart2_rx.Init.PeriphInc = DMA_PINC_DISABLE;
	hdma_usart2_rx.Init.MemInc = DMA_MINC_ENABLE;
	hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
	hdma_usart2_rx.Init.MemDataAlignment = DMA_MDATAALIGN_BYTE;
	hdma_usart2_rx.Init.Mode = DMA_NORMAL;
	hdma_usart2_rx.Init.Priority = DMA_PRIORITY_LOW;
	hdma_usart2_rx.Init.FIFOMode = DMA_FIFOMODE_DISABLE;
	if (HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK)
	{
		Error_Handler();
	}

	__HAL_LINKDMA(&huart2, hdmarx, hdma_usart2_rx);


	/* USART2 interrupt Init */
	HAL_NVIC_SetPriority(USART2_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(USART2_IRQn);


	/**USART2 GPIO Configuration
	PD6     ------> USART2_RX
	PD5     ------> USART2_TX
	*/
	//    GPIO_InitStruct.Pin = GPIO_PIN_6|GPIO_PIN_5;
	//    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
	//    GPIO_InitStruct.Pull = GPIO_NOPULL;
	//    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_VERY_HIGH;
	//    GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
	//    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

}


/******************************************************
* Brief     : ���ڳ�ʼ��
* Parameter : ��
* Return    : ��
*******************************************************/
void uart2_init(void)
{
	huart2.Instance = USART2;
	huart2.Init.BaudRate = 115200;
	huart2.Init.WordLength = UART_WORDLENGTH_8B;
	huart2.Init.StopBits = UART_STOPBITS_1;
	huart2.Init.Parity = UART_PARITY_NONE;
	huart2.Init.Mode = UART_MODE_TX_RX;
	huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
	huart2.Init.OverSampling = UART_OVERSAMPLING_16;
	if (HAL_MultiProcessor_Init(&huart2, 0, UART_WAKEUPMETHOD_IDLELINE) != HAL_OK)
	{
		Error_Handler();
	}
}
/******************************************************
* Brief     : ����ϵͳ�����ʼ��
* Parameter : ��
* Return    : ��
*******************************************************/
void ref_sys_init(void)
{
	ref_sys();
	uart2_init();
	hal_uart2_MspInit();
	if (HAL_UART_Receive_DMA(&huart2, (uint8_t *)&usart2_rx_buffer, datamaxlen) != HAL_OK)
	{
		Error_Handler();
	}
	__HAL_UART_ENABLE_IT(&huart2, UART_IT_IDLE);

}
/******************************************************
* Brief     : �����жϺ�������
* Parameter : ��
* Return    : ��
*******************************************************/
void uart_receive_idle(UART_HandleTypeDef *huart)
{
	uint16_t remian = 0;
	if((__HAL_UART_GET_FLAG(huart, UART_FLAG_IDLE) != RESET))
	{
		if (huart->Instance == USART2)
		{
			sysflag=1;
			__HAL_UART_CLEAR_IDLEFLAG(huart);
			remian = hdma_usart2_rx.Instance->NDTR;//���ճ���
			HAL_UART_DMAStop(huart);
			/* �˴��������ݣ���Ҫ�ǿ�������λ��־λ */
			if (usart2_rx_flag == 0)
			{
				eeror_data =1;
				// usart2_rx_flag = 1;
				usart2_rx_len = datamaxlen - remian;
				//usart2_rx_len = remian;
				rx_len_cal = usart2_rx_len;

				while(rx_len_cal > 9&&eeror_data == 1)//��СCMD����
				{
					if(usart2_rx_buffer[usart2_rx_len - rx_len_cal] == 0xA5)
					{
						switch (usart2_rx_buffer[usart2_rx_len - rx_len_cal + 5])//��λ��ǰ
						{
							case RM_status://������״̬
							memcpy(data1,&usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD1_LEN));
                            rx_len_cal -=cal_cmd_len(CMD1_LEN);
                            eeror_data = cal_ref_data1(data1, cal_cmd_len(CMD1_LEN)); 
							dis_flag = 1;
							status_flag=1;
							break;
							case RM_damage_data://�˺�����
							memcpy(data2, &usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD2_LEN));
                            rx_len_cal -=cal_cmd_len(CMD2_LEN);
                            eeror_data = cal_ref_data2(data2, cal_cmd_len(CMD2_LEN)); 
							dis_flag = 1;
							damage_flag=1;
							break;
							case RM_shoot_data://ʵʱ�������
							memcpy(data3, &usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD3_LEN));
                            rx_len_cal -=cal_cmd_len(CMD3_LEN);
                            eeror_data = cal_ref_data3(data3, cal_cmd_len(CMD3_LEN)); 
							dis_flag = 1;
							shoot_flag=1;
							break;
							case RM_WQ_data://ʵʱ������������
							memcpy(data4, &usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD4_LEN));
                            rx_len_cal -=cal_cmd_len(CMD4_LEN);
                            eeror_data = cal_ref_data4(data4, cal_cmd_len(CMD4_LEN)); 
							dis_flag = 1;
							WQ_flag=1;
							break;
							case RM_RFID_data://���ؽ�������
							memcpy(data5, &usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD5_LEN));
                            rx_len_cal -=cal_cmd_len(CMD5_LEN);
                            eeror_data = cal_ref_data5(data5, cal_cmd_len(CMD5_LEN)); 
							dis_flag = 1;
							RFID_flag=1;
							break;
							case Match_results://����������ݣ�����ʱ����һ��
							memcpy(data6, &usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD6_LEN));
                            rx_len_cal -=cal_cmd_len(CMD6_LEN);
							//  cal_ref_data6(data6, cal_cmd_len(CMD6_LEN)); 
							dis_flag = 1;
							results_flag=1;
							break;
							case Buff_status://buff״̬
							memcpy(data7, &usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD7_LEN));
                            rx_len_cal -=cal_cmd_len(CMD7_LEN);
                            eeror_data =  cal_ref_data7(data7, cal_cmd_len(CMD7_LEN)); 
							dis_flag = 1;
							Buff_flag=1;
							break;
							case RM_self_data://λ�ü�ǹ�ڳ�����Ϣ
							memcpy(data8, &usart2_rx_buffer[usart2_rx_len - rx_len_cal], cal_cmd_len(CMD8_LEN));
                            rx_len_cal -=cal_cmd_len(CMD8_LEN);
                            eeror_data = cal_ref_data8(data8, cal_cmd_len(CMD8_LEN)); 
							dis_flag = 1;
							self_flag=1;
							break;
						}
					}
				  else
					break;
				}
				/* ��ջ��棬���½��� */
				memset(usart2_rx_buffer, 0x00, datamaxlen);
				HAL_UART_Receive_DMA(huart, (uint8_t *)&usart2_rx_buffer, datamaxlen);
			}
		}
	}
}

void USART2_IRQHandler(void)                	
{ 
	uart_receive_idle(&huart2);
} 



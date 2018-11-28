/*
* @�ļ�     robot_status.c
 * @˵��     ������״̬��������
 * @�汾  	 V1.0
 * @����     izh20
 * @��վ     
 * @����     2018.11.25
*/
#include "robot_status.h"






ROBOT_STATUS_t robot_status;
/**********************************************************************************************************
*�� �� ��: PlaceStausCheck
*����˵��: ����״̬��⣺��ֹ���˶�
*��    ��: ���ٶ�
*�� �� ֵ: ��
**********************************************************************************************************/
void PlaceStausCheck(Vector3f_t gyro_status)
{
    Vector3f_t gyroDiff;
    static Vector3f_t lastGyro;
    static float threshold = 1.0f;
    static uint16_t checkNum = 0;
    static int16_t count = 0;

    gyroDiff.x = gyro_status.x - lastGyro.x;
    gyroDiff.y = gyro_status.y - lastGyro.y;
    gyroDiff.z = gyro_status.z - lastGyro.z;
    lastGyro = gyro_status;

    if(count < 30)
    {
        count++;
        //��������ֵ�仯������ֵ
        if(abs(gyroDiff.x) > threshold || abs(gyroDiff.y) > threshold || abs(gyroDiff.z) > threshold)
        {
            checkNum++;
        }
    }
    else
    {
        //���������ݶ�����������һ��ֵʱ��Ϊ�ɻ������ھ�ֹ״̬
        if(checkNum > 10)
            robot_status.placement = MOTIONAL;
        else
            robot_status.placement = STATIC;

        checkNum = 0;
        count = 0;
    }
}

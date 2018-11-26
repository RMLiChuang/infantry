#ifndef __ROBOT_STATUS_
#define __ROBOT_STATUS_
#include "robomaster_common.h"
#include "vector3.h"

typedef struct
{
    uint8_t  init;           //��ʼ��״̬
    uint8_t  failsafe;       //ʧ�ر���״̬
    uint8_t  armed;          //�������״̬
    uint8_t  flight;         //����״̬
    uint8_t  placement;      //����״̬
    uint8_t  altControl;     //�߶ȿ���״̬
    uint8_t  posControl;     //λ�ÿ���״̬
    uint8_t  mode;
    uint32_t initFinishTime; //��ʼ�����ʱ��
} ROBOT_STATUS_t;


//��ʼ��״̬
enum
{
    HEATING,		        //������
    HEAT_FINISH,		    //�������
    INIT_FINISH             //��ʼ����� ����ɼ��ٶ���ƫ���㣩
};

//����״̬
enum
{
    STATIC,		            //��ֹ
    MOTIONAL			    //�˶�
};
extern ROBOT_STATUS_t robot_status;
void PlaceStausCheck(Vector3f_t gyro_status);
#endif


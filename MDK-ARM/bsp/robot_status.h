#ifndef __ROBOT_STATUS_
#define __ROBOT_STATUS_
#include "robomaster_common.h"
#include "vector3.h"

typedef struct
{
    uint8_t  init;           //��ʼ��״̬
    uint8_t  imu_status;		 //imu����״̬
		uint8_t	 anomaly;				 //�쳣���״̬
    uint8_t  armed;          //�������״̬
    uint8_t  placement;      //����״̬
    uint8_t  posControl;     //λ�ÿ���״̬
    uint8_t  mode;					 //��������ģʽ
    uint32_t initFinishTime; //��ʼ�����ʱ��
} ROBOT_STATUS_t;


//imu��ʼ��״̬
enum
{		
    HEATING,		        //������
    HEAT_FINISH,		    //�������
    INIT_FINISH             //��ʼ����� ����ɼ��ٶ���ƫ���㣩
};

//����״̬
enum
{		
    MOTIONAL,		    			//�˶�
		STATIC 		            //��ֹ
};

//�����˶�ģʽ
enum
{	
	INITIAL,         			 //��ʼ��ģʽ
	STANDBY,          		 //��������״̬
	FOLLOW,           		 //���̸�����̨
	TWIST,             		 //Ť��
	KEY_BOARD_CONTROL			 //���̿���״̬
};
//�����쳣ģʽ
enum
{
	NORMAL,								 //����״̬
	REMOTE_CONTROL_OFFLINE,//ң��������
	CHASSIS_MOTOR_OFFLINE, //���̵������
	PAN_TILT_OFFLINE			 //��̨�������
	
};
extern ROBOT_STATUS_t robot_status;
void PlaceStausCheck(Vector3f_t gyro_status);
void robot_status_init(void);
void robot_status_detection(void);
#endif


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
		uint8_t  vision_status;    //�Ӿ�ʶ��״̬
		uint8_t  chassis_control; //���̿���״̬ ��Ϊ�ɿ���ǿɿ�״̬
	
		uint8_t  control_mode;    //С������ģʽ����Ϊң�������ƺͼ��̿���
		uint8_t  vision_mode;     //�Ӿ�ʶ��ģʽ  
		uint8_t  fric_mode;				//Ħ����ģʽ
		uint8_t  rammer_mode;			//������ģʽ
} ROBOT_STATUS_t;

//������ģʽ
enum
{			
    SAFETY,						//��ȫģʽ
		SINGLE_SHOT,		    //����
		THRICE_SHOT,				//���Զ�,����M16 3����
		AUTO_SHOT						//�Զ�
};

//Ħ����ģʽ  ���������
enum
{		
		STOP,		        				//ֹͣ
    CLOSE_FIRE,							//�����Ƶ� 10m/s
		MID_FIRE,								//�г��Ƶ� 15m/s
		REMOTE_FIRE,						//Զ���Ƶ� 20m/s
		INTERCONTINENTAL_FIRE		//�޼��Ƶ� 28m/s
};
//�Ӿ�ʶ��ģʽ  ���������
enum
{			
    DORMANT,						//����
		ACTIVATE		        //����
};
//С������ģʽ����Ϊң�������ƺͼ��̿���
enum
{		
		REMOTE_CONTROL,		        //ң��������
    KEYBOARD_CONTROL		    	//���̿���
};

//��������̨����ԽǶȵĿɿ�״̬  ����ֵ����35��ʱ�޷�����
enum
{		
		CONTROL,		        //�ɿ�
    OUT_OF_CONTROL,		    //ʧ��
};

//imu��ʼ��״̬
enum
{		
    HEATING,		        //������
    HEAT_FINISH,		    //�������
    INIT_FINISH             //��ʼ����� ����ɼ��ٶ���ƫ���㣩
};
//visionʶ��״̬
enum
{		
		VISION_SUCCESS,						//�Ӿ�ʶ��ɹ�
    VISION_LOSE		    				//δʶ��
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
	PAN_TILT_OFFLINE,			 //��̨�������
	VISION_OFFLINE
	
};
extern ROBOT_STATUS_t robot_status;
void PlaceStausCheck(Vector3f_t gyro_status);
void robot_status_init(void);
void robot_status_detection(void);
void robot_status_display(void);
#endif


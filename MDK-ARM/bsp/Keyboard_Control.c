#include "Keyboard_Control.h"

/*
W 1 	S 2 	A 4		D 8 	SHIFT 16 	CTRL 32
Q 64	E 128 	R 256		F 512		G 1024
Z 2048		X 4096 		C 8192 		V 16384  	B 32768


*/



Keybroad_Control kb;
#define kbspeed  350           //���̿����ٶ�ϵ��
float s_up = 1.5 ;                //���̼���ϵ���������¡�shift����
float kbshootspeed=1.4;

void Keyboard_value_Init(void)                                                  //���̽ṹ�������ʼֵ
{
	kb.c=1;
	kb.c_flag=0;
	kb.b=1;
	kb.b_flag=0;
	kb.speed=kbspeed;
	
	kb.mousel_flag   =   0;
	kb.mouser_flag   =   0;
}

void Keyboard_Init(void)
{
	keyboard_chassis();
	keyboard_mouse();
	if(remote_control.keyBoard.key_code==0x21)                                      //����"CTRL+W"��Ħ���ּ���
	{
	
	}	
	else if(remote_control.keyBoard.key_code==0x22)                                 //����"CTRL+S"��Ħ���ּ���
	{
	
	}
	
	if(remote_control.keyBoard.key_code==0x2000&&kb.c==1)                                     //����"V"��è������/�ر�
	{
		kb.c_flag=!kb.c_flag;
		kb.c=0;
	}	
	else if(remote_control.keyBoard.key_code!=0x2000)
		kb.c=1;
	
	
	if(remote_control.keyBoard.key_code==0x8000&&kb.b==1)                                      //����"B"��Ħ���ֿ���/�ر�
	{
		kb.b_flag=!kb.b_flag;
		kb.b=0;
	}	
	else if(remote_control.keyBoard.key_code!=0x8000)
		kb.b=1;
		


//		if (remote_control.keyBoard.key_code==0x20)                               //���¡�CTRL����

}

void keyboard_chassis(void)
{
	if (remote_control.keyBoard.key_code==0x01)                                  //���¡�W��������ǰ��
	{
		straight=kbspeed;
	}
	
	else if (remote_control.keyBoard.key_code==0x02)                             //���¡�S�������Ӻ���
	{
		straight=-kbspeed;
	}			

	else if (remote_control.keyBoard.key_code==0x04)                              //���¡�A������������
	{
		translation=-kbspeed;
	}

	else if (remote_control.keyBoard.key_code==0x08)                              //���¡�D������������
	{
		translation=kbspeed;
	}
	
	else if(remote_control.keyBoard.key_code==0x40)                               //���¡�Q����������ʱ����ת
	{
		
	}


	else if(remote_control.keyBoard.key_code==0x80)                               //���¡�E��������˳ʱ����ת
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x11)                               //���¡�SHIFT+W��������ǰ��
	{
		straight=kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x12)                               //���¡�SHIFT+S�������ٺ���
	{
		straight=-kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x14)                               //���¡�SHIFT+A������������
	{
		translation=-kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x18)                               //���¡�SHIFT+D������������
	{
		translation=kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x05)                               //���¡�W+A��������ƽ�ƶ�
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x09)                               //���¡�W+D��������ƽ�ƶ�
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x06)                               //���¡�A+S��������ƽ�ƶ�
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x0A)                               //���¡�D+S��������ƽ�ƶ�
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x41)                               //���¡�W+Q����תǰ��
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x81)                               //���¡�W+E������תǰ��
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x15)                               //���¡�W+A+SHIIFT������������ƽ�ƶ�
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x19)                               //���¡�W+D+SHIFT������������ƽ�ƶ�
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x16)                               //���¡�A+S+SHIFT������������ƽ�ƶ�
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x1A)                               //���¡�D+S+SHIFT������������ƽ�ƶ�
	{
	
	}                                                                              
	
	else if (remote_control.keyBoard.key_code==0x51)                               //���¡�W+Q+SHIIFT����������תǰ��
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x91)                               //���¡�W+E+SHIFT����������תǰ��
	{
		
	}	
	else if (remote_control.keyBoard.key_code==0x100)                               //���¡�r�����Ӿ�ʶ��
	{
		detect_cartridge();//�򿪵��ղ��鿴��ҩ
	}
	else if(remote_control.keyBoard.key_code==0x8000)																//���¡�b��,�л����ģʽ
	{
		detect_cartridge();//�򿪵��ղ��鿴��ҩ
	}		
	
	else                                                                            //�����ٶ�ֹͣ
	{
		straight=0;//ǰ���ٶ�Ϊ��
		translation=0;//ƽ���ٶ�Ϊ��
		reset_camera();//��λͼ��λ��
		robot_status.vision_mode=DORMANT;//�����Ӿ�ʶ��
		motor_pid[0].target =       0;
		motor_pid[1].target =       0;
		motor_pid[2].target =       0;
		motor_pid[3].target =       0;
		kb.yaw              =       0;
	}
	
}

void keyboard_mouse(void)
{
	if(remote_control.mouse.press_left==1)
	{
		kb.mousel_flag   =   1;
		kb.mousel        =   0;
	}
	
	else if(remote_control.mouse.press_right==1)
	{
		robot_status.vision_mode=ACTIVATE;//�����Ӿ�ʶ��
//		kb.mouser_flag   =   1;
//		kb.mouser        =   0;
	}
	
	else
	{
		robot_status.vision_mode=DORMANT;//�����Ӿ�ʶ��
		kb.mousel_flag   =   0;
		kb.mousel        =   1;
		kb.mouser_flag   =   0;
		kb.mouser        =   1;
	}

}



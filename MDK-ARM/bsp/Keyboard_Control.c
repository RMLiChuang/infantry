#include "Keyboard_Control.h"

/*
W 1 	S 2 	A 4		D 8 	SHIFT 16 	CTRL 32
Q 64	E 128 	R 256		F 512		G 1024
Z 2048		X 4096 		C 8192 		V 16384  	B 32768


*/



Keybroad_Control kb;
#define kbspeed  350           //键盘控制速度系数
float s_up = 1.5 ;                //键盘加速系数，即按下“shift”键
float kbshootspeed=1.4;

void Keyboard_value_Init(void)                                                  //键盘结构体变量初始值
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
	if(remote_control.keyBoard.key_code==0x21)                                      //按下"CTRL+W"。摩擦轮加速
	{
	
	}	
	else if(remote_control.keyBoard.key_code==0x22)                                 //按下"CTRL+S"。摩擦轮减速
	{
	
	}
	
	if(remote_control.keyBoard.key_code==0x2000&&kb.c==1)                                     //按下"V"。猫步开启/关闭
	{
		kb.c_flag=!kb.c_flag;
		kb.c=0;
	}	
	else if(remote_control.keyBoard.key_code!=0x2000)
		kb.c=1;
	
	
	if(remote_control.keyBoard.key_code==0x8000&&kb.b==1)                                      //按下"B"。摩擦轮开启/关闭
	{
		kb.b_flag=!kb.b_flag;
		kb.b=0;
	}	
	else if(remote_control.keyBoard.key_code!=0x8000)
		kb.b=1;
		


//		if (remote_control.keyBoard.key_code==0x20)                               //按下“CTRL”。

}

void keyboard_chassis(void)
{
	if (remote_control.keyBoard.key_code==0x01)                                  //按下“W”。车子前进
	{
		straight=kbspeed;
	}
	
	else if (remote_control.keyBoard.key_code==0x02)                             //按下“S”。车子后退
	{
		straight=-kbspeed;
	}			

	else if (remote_control.keyBoard.key_code==0x04)                              //按下“A”。车子左移
	{
		translation=-kbspeed;
	}

	else if (remote_control.keyBoard.key_code==0x08)                              //按下“D”。车子右移
	{
		translation=kbspeed;
	}
	
	else if(remote_control.keyBoard.key_code==0x40)                               //按下“Q”。车子逆时针旋转
	{
		
	}


	else if(remote_control.keyBoard.key_code==0x80)                               //按下“E”。车子顺时针旋转
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x11)                               //按下“SHIFT+W”。加速前进
	{
		straight=kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x12)                               //按下“SHIFT+S”。加速后退
	{
		straight=-kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x14)                               //按下“SHIFT+A”。加速左移
	{
		translation=-kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x18)                               //按下“SHIFT+D”。加速右移
	{
		translation=kbspeed*s_up;
	}
	
	else if (remote_control.keyBoard.key_code==0x05)                               //按下“W+A”。左上平移动
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x09)                               //按下“W+D”。右上平移动
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x06)                               //按下“A+S”。左下平移动
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x0A)                               //按下“D+S”。右下平移动
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x41)                               //按下“W+Q”左转前进
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x81)                               //按下“W+E”。右转前进
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x15)                               //按下“W+A+SHIIFT”。加速左上平移动
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x19)                               //按下“W+D+SHIFT”。加速右上平移动
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x16)                               //按下“A+S+SHIFT”。加速左下平移动
	{
		
	}
	
	else if (remote_control.keyBoard.key_code==0x1A)                               //按下“D+S+SHIFT”。加速右下平移动
	{
	
	}                                                                              
	
	else if (remote_control.keyBoard.key_code==0x51)                               //按下“W+Q+SHIIFT”。加速左转前进
	{
		
	}	
	
	else if (remote_control.keyBoard.key_code==0x91)                               //按下“W+E+SHIFT”。加速右转前进
	{
		
	}	
	else if (remote_control.keyBoard.key_code==0x100)                               //按下“r”。视觉识别
	{
		detect_cartridge();//打开弹舱并查看弹药
	}
	else if(remote_control.keyBoard.key_code==0x8000)																//按下“b”,切换射击模式
	{
		detect_cartridge();//打开弹舱并查看弹药
	}		
	
	else                                                                            //否则速度停止
	{
		straight=0;//前后速度为零
		translation=0;//平移速度为零
		reset_camera();//复位图传位置
		robot_status.vision_mode=DORMANT;//屏蔽视觉识别
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
		robot_status.vision_mode=ACTIVATE;//激活视觉识别
//		kb.mouser_flag   =   1;
//		kb.mouser        =   0;
	}
	
	else
	{
		robot_status.vision_mode=DORMANT;//屏蔽视觉识别
		kb.mousel_flag   =   0;
		kb.mousel        =   1;
		kb.mouser_flag   =   0;
		kb.mouser        =   1;
	}

}



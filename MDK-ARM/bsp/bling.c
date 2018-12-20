/**********************************************************************************************************
                               力创RM实验室robomaster代码
                                Github: https://github.com/RMLiChuang
                                技术讨论：342874929
 * @文件     bling.c
 * @说明     led灯闪烁，用于程序调试
 * @版本  	 V1.0
 * @作者     izh20
 * @网站     
 * @日期     2018.11.29
**********************************************************************************************************/


#include "bling.h"

Bling_Light Light_H,Light_G,Light_F,Light_E,Light_D,Light_C,Light_B,Light_A;
uint16_t Bling_Mode=0;
/***************************************************
函数名: void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               GPIO_TypeDef* Port,
               uint16_t Pin
               ,uint8_t Flag)
说明:	状态指示灯设置函数
入口:	时间、周期、占空比、端口等
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//持续时间
               uint16_t Period,//周期100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               GPIO_TypeDef* Port,
               uint16_t Pin,
               uint8_t Flag)
{
  Light->Bling_Contiune_Time=(uint16_t)(Continue_time/5);//持续时间
  Light->Bling_Period=Period;//周期
  Light->Bling_Percent=Percent;//占空比
  Light->Bling_Cnt=Cnt;
  Light->Port=Port;//端口
  Light->Pin=Pin;//引脚
  Light->Endless_Flag=Flag;//无尽模式
}

/***************************************************
函数名: void Bling_Process(Bling_Light *Light)//闪烁运行线程
说明:	状态指示灯实现
入口:	状态灯结构体
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Process(Bling_Light *Light)//闪烁运行线程
{
  if(Light->Bling_Contiune_Time>=1)  Light->Bling_Contiune_Time--;
  else HAL_GPIO_WritePin(Light->Port,Light->Pin,GPIO_PIN_SET);//置高,灭
  if(Light->Bling_Contiune_Time!=0//总时间未清0
     ||Light->Endless_Flag==1)//判断无尽模式是否开启
  {
    Light->Bling_Cnt++;
    if(5*Light->Bling_Cnt>=Light->Bling_Period) Light->Bling_Cnt=0;//计满清零          //以周期的50%时间亮
    if(5*Light->Bling_Cnt<=Light->Bling_Period*Light->Bling_Percent)
       HAL_GPIO_WritePin(Light->Port,Light->Pin,GPIO_PIN_SET);//置高，灭
    else HAL_GPIO_WritePin(Light->Port,Light->Pin,GPIO_PIN_RESET);//置低，亮
  }
}
/***************************************************
函数名: void Quad_Start_Bling(void)
说明:	LED初始化后开机闪烁
入口:	无
出口:	无
备注:	上电初始化，运行一次
****************************************************/
void Infantry_Start_Bling()
{
   Bling_Set(&Light_A,2000,200,0.5,0,LED_USER_GPIO_PORT,LED_A_Pin,0);
   Bling_Set(&Light_B,2000,500,0.5,0,LED_USER_GPIO_PORT,LED_B_Pin,0);
   Bling_Set(&Light_C,2000,800,0.5,0,LED_USER_GPIO_PORT,LED_C_Pin,0);
   Bling_Set(&Light_H,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_H_Pin,1);

}

/***************************************************
函数名: Bling_Working(uint16 bling_mode)
说明:	状态指示灯状态机
入口:	当前模式
出口:	无
备注:	程序初始化后、始终运行
****************************************************/
void Bling_Working(uint16_t bling_mode)
{
      if(bling_mode==0)//全灭
      {
          Bling_Process(&Light_A);
          Bling_Process(&Light_B);
          Bling_Process(&Light_C);
      }
			
			
		
		Bling_Process(&Light_A);//判断底盘跟随云台模式是否正常
		Bling_Process(&Light_B);//判断猫步程序是否正常
		Bling_Process(&Light_C);
		Bling_Process(&Light_D);
		Bling_Process(&Light_E);
		Bling_Process(&Light_F);
		Bling_Process(&Light_G);
		Bling_Process(&Light_H);//判断中断2是否正常
}

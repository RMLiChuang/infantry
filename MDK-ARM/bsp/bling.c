/**********************************************************************************************************
                               ����RMʵ����robomaster����
                                Github: https://github.com/RMLiChuang
                                �������ۣ�342874929
 * @�ļ�     bling.c
 * @˵��     led����˸�����ڳ������
 * @�汾  	 V1.0
 * @����     izh20
 * @��վ     
 * @����     2018.11.29
**********************************************************************************************************/


#include "bling.h"

Bling_Light Light_H,Light_G,Light_F,Light_E,Light_D,Light_C,Light_B,Light_A;
uint16_t Bling_Mode=0;
/***************************************************
������: void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//����ʱ��
               uint16_t Period,//����100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               GPIO_TypeDef* Port,
               uint16_t Pin
               ,uint8_t Flag)
˵��:	״ָ̬ʾ�����ú���
���:	ʱ�䡢���ڡ�ռ�ձȡ��˿ڵ�
����:	��
��ע:	�����ʼ����ʼ������
****************************************************/
void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//����ʱ��
               uint16_t Period,//����100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               GPIO_TypeDef* Port,
               uint16_t Pin,
               uint8_t Flag)
{
  Light->Bling_Contiune_Time=(uint16_t)(Continue_time/5);//����ʱ��
  Light->Bling_Period=Period;//����
  Light->Bling_Percent=Percent;//ռ�ձ�
  Light->Bling_Cnt=Cnt;
  Light->Port=Port;//�˿�
  Light->Pin=Pin;//����
  Light->Endless_Flag=Flag;//�޾�ģʽ
}

/***************************************************
������: void Bling_Process(Bling_Light *Light)//��˸�����߳�
˵��:	״ָ̬ʾ��ʵ��
���:	״̬�ƽṹ��
����:	��
��ע:	�����ʼ����ʼ������
****************************************************/
void Bling_Process(Bling_Light *Light)//��˸�����߳�
{
  if(Light->Bling_Contiune_Time>=1)  Light->Bling_Contiune_Time--;
  else HAL_GPIO_WritePin(Light->Port,Light->Pin,GPIO_PIN_SET);//�ø�,��
  if(Light->Bling_Contiune_Time!=0//��ʱ��δ��0
     ||Light->Endless_Flag==1)//�ж��޾�ģʽ�Ƿ���
  {
    Light->Bling_Cnt++;
    if(5*Light->Bling_Cnt>=Light->Bling_Period) Light->Bling_Cnt=0;//��������          //�����ڵ�50%ʱ����
    if(5*Light->Bling_Cnt<=Light->Bling_Period*Light->Bling_Percent)
       HAL_GPIO_WritePin(Light->Port,Light->Pin,GPIO_PIN_SET);//�øߣ���
    else HAL_GPIO_WritePin(Light->Port,Light->Pin,GPIO_PIN_RESET);//�õͣ���
  }
}
/***************************************************
������: void Quad_Start_Bling(void)
˵��:	LED��ʼ���󿪻���˸
���:	��
����:	��
��ע:	�ϵ��ʼ��������һ��
****************************************************/
void Infantry_Start_Bling()
{
   Bling_Set(&Light_A,2000,200,0.5,0,LED_USER_GPIO_PORT,LED_A_Pin,0);
   Bling_Set(&Light_B,2000,500,0.5,0,LED_USER_GPIO_PORT,LED_B_Pin,0);
   Bling_Set(&Light_C,2000,800,0.5,0,LED_USER_GPIO_PORT,LED_C_Pin,0);
   Bling_Set(&Light_H,2000,1000,0.5,0,LED_USER_GPIO_PORT,LED_H_Pin,1);

}

/***************************************************
������: Bling_Working(uint16 bling_mode)
˵��:	״ָ̬ʾ��״̬��
���:	��ǰģʽ
����:	��
��ע:	�����ʼ����ʼ������
****************************************************/
void Bling_Working(uint16_t bling_mode)
{
      if(bling_mode==0)//ȫ��
      {
          Bling_Process(&Light_A);
          Bling_Process(&Light_B);
          Bling_Process(&Light_C);
      }
			
			
		
		Bling_Process(&Light_A);//�жϵ��̸�����̨ģʽ�Ƿ�����
		Bling_Process(&Light_B);//�ж�è�������Ƿ�����
		Bling_Process(&Light_C);
		Bling_Process(&Light_D);
		Bling_Process(&Light_E);
		Bling_Process(&Light_F);
		Bling_Process(&Light_G);
		Bling_Process(&Light_H);//�ж��ж�2�Ƿ�����
}

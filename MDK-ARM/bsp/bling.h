#ifndef __BLING_H
#define __BLING_H
#include "robomaster_common.h"

typedef struct
{
  uint16_t Bling_Contiune_Time;//��˸����ʱ��
  uint16_t Bling_Period;//��˸����
  float  Bling_Percent;//��˸ռ�ձ�
  uint16_t  Bling_Cnt;//��˸������
  GPIO_TypeDef* Port; //�˿�
  uint16_t Pin;//����
  uint8_t Endless_Flag;//�޾�ģʽ
}Bling_Light;

void Bling_Set(Bling_Light *Light,
               uint32_t Continue_time,//����ʱ��
               uint16_t Period,//����100ms~1000ms
               float Percent,//0~100%
               uint16_t  Cnt,
               GPIO_TypeDef* Port,
               uint16_t Pin
               ,uint8_t Flag);
void Bling_Process(Bling_Light *Light);
void Infantry_Start_Bling(void);
void Bling_Working(uint16_t bling_mode);
extern Bling_Light Light_H,Light_G,Light_F,Light_E,Light_D,Light_C,Light_B,Light_A;
extern uint16_t Bling_Mode;						 
#endif
		

							 
							 






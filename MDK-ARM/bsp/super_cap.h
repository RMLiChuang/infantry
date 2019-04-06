/**
  ******************************************************************************
  * @file	super_cap.h
  * @author  松小罗
  * @version V1.0.0
  * @date    2019/1/20
  * @brief   
  ******************************************************************************
  * @attention
  *
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/

#ifndef __SUPER_CAP_H
#define __SUPER_CAP_H

#include "stm32f4xx_hal.h"
//#include "stdint.h"
//#include "stdlib.h"
#include "robomaster_common.h"
#include "stm32f4xx_hal_adc.h"
//__STM32F4xx_ADC_H


extern u8 Cap_In_flag;//充电标志位
extern u8 Cap_Out_flag;//充电标志位
extern u8 control_state;//控制状态标志

#define		Super_Cap_Pin_OUT		GPIO_PIN_1//放电
#define		Super_Cap_Pin_IN		GPIO_PIN_0//充电
#define		Super_Cap_Port			GPIOF

#define		Super_Cap_OUT_ON()		HAL_GPIO_WritePin(Super_Cap_Port, Super_Cap_Pin_OUT, GPIO_PIN_SET)
#define		Super_Cap_OUT_OFF()		HAL_GPIO_WritePin(Super_Cap_Port, Super_Cap_Pin_OUT, GPIO_PIN_RESET)
#define		Super_Cap_IN_ON()		HAL_GPIO_WritePin(Super_Cap_Port, Super_Cap_Pin_IN, GPIO_PIN_SET)
#define		Super_Cap_IN_OFF()		HAL_GPIO_WritePin(Super_Cap_Port, Super_Cap_Pin_IN, GPIO_PIN_RESET)


void 	Super_Cap_Init(void);
void  	MX_ADC1_Init(void);
float 	MY_ADC_GetValue(void);

float Calc_Cap_Power(float voltage,u8 flag);
void Super_Cap_control(int16_t *output);

#endif



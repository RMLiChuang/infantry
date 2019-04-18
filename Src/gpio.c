/**
  ******************************************************************************
  * File Name          : gpio.c
  * Description        : This file provides code for the configuration
  *                      of all used GPIO pins.
  ******************************************************************************
  ******************************************************************************
  */

/* Includes ------------------------------------------------------------------*/
#include "gpio.h"
/* USER CODE BEGIN 0 */

/* USER CODE END 0 */

/*----------------------------------------------------------------------------*/
/* Configure GPIO                                                             */
/*----------------------------------------------------------------------------*/
/* USER CODE BEGIN 1 */

/* USER CODE END 1 */

/** Configure pins as 
        * Analog 
        * Input 
        * Output
        * EVENT_OUT
        * EXTI
*/
void MX_GPIO_Init(void)
{

  GPIO_InitTypeDef GPIO_InitStruct;

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOG_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOI_CLK_ENABLE();
  __HAL_RCC_GPIOH_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOE_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LASER_GPIO_Port, LASER_Pin, GPIO_PIN_SET);
	
	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(IMU_TEMP_Port, IMU_TEMP_Pin, GPIO_PIN_RESET);//初始化imu电阻模块，设置输出为0
	
  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOH, GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_USER_GPIO_PORT, LED_H_Pin|LED_G_Pin|LED_F_Pin|LED_E_Pin 
                          |LED_D_Pin|LED_C_Pin|LED_B_Pin|LED_A_Pin, GPIO_PIN_SET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_S_GPIO_Port, GPIO_PIN_6|LED_S_Pin, GPIO_PIN_SET);//编号为S的LED灯

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(LED_T_GPIO_Port, LED_T_Pin, GPIO_PIN_SET);//编号为T的led灯

  /*Configure GPIO pins : PGPin PG8 PG7 PG6 
                           PG5 PG4 PG3 PG2 
                           PG1 */
  GPIO_InitStruct.Pin = LASER_Pin|LED_H_Pin|LED_G_Pin|LED_F_Pin         //用户灯
                          |LED_E_Pin|LED_D_Pin|LED_C_Pin|LED_B_Pin 
                          |LED_A_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_USER_GPIO_PORT, &GPIO_InitStruct);

  /*Configure GPIO pins : PH2 PH3 PH4 PH5 */
  GPIO_InitStruct.Pin = GPIO_PIN_2|GPIO_PIN_3|GPIO_PIN_4|GPIO_PIN_5;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOH, &GPIO_InitStruct);

  /*Configure GPIO pins : PD14 PDPin */
  GPIO_InitStruct.Pin = GPIO_PIN_14|KEY_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);
	
	GPIO_InitStruct.Pin = GPIO_PIN_15;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */  //编号为S的灯
  GPIO_InitStruct.Pin =  GPIO_PIN_6|LED_S_Pin; 
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_S_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pin : PtPin */    //编号为T的灯
  GPIO_InitStruct.Pin = LED_T_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(LED_T_GPIO_Port, &GPIO_InitStruct);
	
	/*Configure GPIO pin : PtPin */                 //初始化imu_电阻加热模块
	GPIO_InitStruct.Pin = IMU_TEMP_Pin|OLED_DC_Pin|OLED_RST_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(IMU_TEMP_Port, &GPIO_InitStruct);
	
	//外部中断，用于射击时单发操作
	GPIO_InitStruct.Pin = GPIO_PIN_1;						//设置引脚，第一引脚
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;	//设置中断触发态，上升沿中断
  GPIO_InitStruct.Pull = GPIO_PULLDOWN;				//设置IO口电平，高电平
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);			//GPIO初始化配置，初始化GPIO中A组
  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);			//设置外部中断优先级
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);
	
}

/* USER CODE BEGIN 2 */

/* USER CODE END 2 */

/**
  * @}
  */

/**
  * @}
  */

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

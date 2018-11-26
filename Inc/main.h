/**
  ******************************************************************************
  * File Name          : main.hpp
  * Description        : This file contains the common defines of the application
  ******************************************************************************
  ******************************************************************************
  */
/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __MAIN_H
#define __MAIN_H
  /* Includes ------------------------------------------------------------------*/

/* Includes ------------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LASER_Pin 							GPIO_PIN_13
#define LASER_GPIO_Port 				GPIOG
#define KEY_Pin 								GPIO_PIN_10
#define KEY_GPIO_Port 					GPIOD
#define LED1_Pin 								GPIO_PIN_14	//绿灯
#define LED1_GPIO_Port 					GPIOF
#define LED2_Pin 								GPIO_PIN_11  //红灯
#define LED2_GPIO_Port 					GPIOE
#define IMU_TEMP_Pin       			GPIO_PIN_5 //imu电阻加热为PB5
#define IMU_TEMP_Port						GPIOB
/* ########################## Assert Selection ############################## */
/**
  * @brief Uncomment the line below to expanse the "assert_param" macro in the 
  *        HAL drivers code
  */
/* #define USE_FULL_ASSERT    1U */

/* USER CODE BEGIN Private defines */

/* USER CODE END Private defines */

#ifdef __cplusplus
 extern "C" {
#endif
void _Error_Handler(char *, int);

#define Error_Handler() _Error_Handler(__FILE__, __LINE__)
#ifdef __cplusplus
}
#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

#endif /* __MAIN_H */
/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

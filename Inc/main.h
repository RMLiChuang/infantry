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
//#define QUADROTOR_PAN_TILT 		//四轴云台
#define INFANTRY_PAN_TILT     //步兵云台
typedef unsigned char uint8_t;
typedef unsigned short int uint16_t;
typedef unsigned int uint32_t;
typedef unsigned long long uint64_t;
typedef unsigned char bool_t;
typedef float fp32;
typedef double fp64;
/* USER CODE END Includes */

/* Private define ------------------------------------------------------------*/

#define LASER_Pin 							GPIO_PIN_13
#define LASER_GPIO_Port 				GPIOG
#define KEY_Pin 								GPIO_PIN_10
#define KEY_GPIO_Port 					GPIOD
#define LED_S_Pin 								GPIO_PIN_14	
#define LED_S_GPIO_Port 					GPIOF//绿灯 编号为S
#define LED_T_Pin 								GPIO_PIN_11  
#define LED_T_GPIO_Port 					GPIOE//红灯 编号为T
#define LED_H_Pin									GPIO_PIN_1
#define LED_G_Pin									GPIO_PIN_2
#define LED_F_Pin									GPIO_PIN_3
#define LED_E_Pin									GPIO_PIN_4
#define LED_D_Pin									GPIO_PIN_5
#define LED_C_Pin									GPIO_PIN_6
#define LED_B_Pin									GPIO_PIN_7
#define LED_A_Pin									GPIO_PIN_8
#define LED_USER_GPIO_PORT				GPIOG        //用户灯

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

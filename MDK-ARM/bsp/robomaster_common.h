#ifndef __ROBOMASTER_COMMON
#define __ROBOMASTER_COMMON

/* Includes ------------------------------------------------------------------*/
#include "stm32f4xx.h"
#include "stm32f4xx_it.h"
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "mytype.h"
#include <math.h>
/* USER CODE BEGIN Includes */
#include "bsp_can.h"
#include "pid.h"
#include "myiic.h"
#include "Remote_Control.h"
#include "bsp_imu.h"
#include "ist8310_reg.h"
#include "mpu6500_reg.h"
#include "robomaster_vcan.h"
#include "robomaster_control.h"
#include "robomaster_task.h"
#include "robomaster_filter.h"
#include "vector3.h"
#include "robomaster_calibrate.h"
#include "robot_status.h"
#include "pan_tilt_control.h"
#include "bling.h"
#include "delay.h"
/* USER CODE END Includes */

#endif


/**
  ******************************************************************************
  * File Name          : main.c
	* author						 : 周恒
	* Date							 :2018.10.07
  * Description        ;while循环中只运行山外的上位机程序，姿态解算，底盘电机的驱动和摩擦轮，拨弹电机的控制全在robomaster_task.c文件中运行
												目前程序只进行了底盘电机的速度闭环，拨弹电机的速度闭环，云台电机还未进行控制，下一步可将角度信息融合到底盘的控制中去
	
  ******************************************************************************
  ******************************************************************************
  */
	
	/*！！！！！！！！！！！！！！在自己写的每一函数上面都需要复制下面的函数说明，增加可读性！！！！！！！！！！！！！！！！！！*/
	/**********************************************************************************************************
*函 数 名: 
*功能说明: 
*形    参: 
*返 回 值: 
**********************************************************************************************************/
	

/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "stm32f4xx_hal.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h" 
#include "mpu9250.h"
//#include "inv_mpu.h"
#include "delay.h"
#include "usmart.h"
/* USER CODE BEGIN Includes */
#include "robomaster_common.h"
/* USER CODE END Includes */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
/* Private variables ---------------------------------------------------------*/


uint16_t TIM_COUNT[2];

#define SpeedStep 500

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);

/* USER CODE BEGIN PFP */
/* Private function prototypes -----------------------------------------------*/

/* USER CODE END PFP */

/* USER CODE BEGIN 0 */

/* USER CODE END 0 */
unsigned int testx;
extern int testmpu;
extern float mypitch;
extern float myroll;
extern float myyaw;
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration----------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_CAN1_Init();//can通信
  MX_USART1_UART_Init();//遥控器接收，串口中断
  MX_TIM1_Init();//用来计时，测试程序运行的时间
  MX_TIM5_Init();//产生4路pwm波来控制摩擦轮
  MX_USART6_UART_Init();//用于上位机调试
	MX_ADC1_Init();
  MX_SPI1_Init();
//	HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);//该函数会开启接收中断：标志位UART_IT_RXNE，并且设置接收缓冲以及接收缓冲接收最大数据量
  MX_SPI5_Init();//用于imu驱动
  MX_TIM2_Init();//中断控制程序  主要程序在TIM2中运行
  MX_TIM3_Init();//未用
	MX_UART8_Init();
//  MX_TIM4_Init();//USMART占用
//  MX_USART2_UART_Init();//蓝牙串口  未用
	ref_sys_init();//裁判系统初始化，内部已设置好串口相关的东西
  MX_USART3_UART_Init();//大疆SDK串口 未用
  delay_init(168);//延时函数初始化
	//MPU9250_Init();
  //usmart_dev.init(84);//USMART初始化
  /* USER CODE BEGIN 2 */
	/**TIM5 GPIO Configuration    
	PI0     ------> TIM5_CH4
	PH12     ------> TIM5_CH3
	PH11     ------> TIM5_CH2
	PH10     ------> TIM5_CH1 
	*/

	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim5,TIM_CHANNEL_4);
	
  my_can_filter_init_recv_all(&hcan1);     //配置CAN过滤器
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //启动CAN接收中断
  HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100);   //启动串口接收
	
	ramp_init(&fric_ramp,SHOOT_CONTROL_TIME * 0.001f, Close_Fric_ON, Fric_OFF);
	oled_init();
	init_Fric_PWM();			//初始化摩擦轮的PWM
	Infantry_Start_Bling();//步兵状态指示灯
	mpu_device_init();   //在初始化imu的时候，要先初始化SPI5和GPIOF6 不然无法初始化imu
	robot_status_init();//步兵模式初始化
	init_quaternion();	//初始化四元数，用于姿态解算
	all_pid_init();			//所有电机的pid初始化
    																	
	//ramp_init(&trigger_motor.fric2_ramp, SHOOT_CONTROL_TIME * 0.005f, Fric_DOWN, Fric_OFF);
	
	Keyboard_value_Init();//键盘初始化
	
	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	HAL_TIM_Base_Start_IT(&htim2);//开启定时器2  程序主要任务都在定时器2中断里完成呢
	
  /* USER CODE END 2 */
	Super_Cap_Init();//超级电容初始化，内部已初始化好ADC
	
	 //底盘初始化
    chassis_init(&chassis_move);
  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {	
		
  /* USER CODE END WHILE */
		oled_shownum(3,4,k,0,1);
		
    //oled_LOGO();
    oled_refresh_gram();
  /* USER CODE BEGIN 3 */

		debug_program();
	
	

	

  /* USER CODE END 3 */

	}
}

/** System Clock Configuration
*/
void SystemClock_Config(void)
{

  RCC_OscInitTypeDef RCC_OscInitStruct;
  RCC_ClkInitTypeDef RCC_ClkInitStruct;

    /**Configure the main internal regulator output voltage 
    */
  __HAL_RCC_PWR_CLK_ENABLE();

  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLM = 6;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Initializes the CPU, AHB and APB busses clocks 
    */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    _Error_Handler(__FILE__, __LINE__);
  }

    /**Configure the Systick interrupt time 
    */
  HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq()/1000);

    /**Configure the Systick 
    */
  HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  Period elapsed callback in non blocking mode
  * @note   This function is called  when TIM6 interrupt took place, inside
  * HAL_TIM_IRQHandler(). It makes a direct call to HAL_IncTick() to increment
  * a global variable "uwTick" used as application time base.
  * @param  htim : TIM handle
  * @retval None
  */
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
/* USER CODE BEGIN Callback 0 */

/* USER CODE END Callback 0 */
  if (htim->Instance == TIM6) {
    HAL_IncTick();
  }
/* USER CODE BEGIN Callback 1 */

/* USER CODE END Callback 1 */
}

/**
  * @brief  This function is executed in case of error occurrence.
  * @param  None
  * @retval None
  */
void _Error_Handler(char * file, int line)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  while(1) 
  {
  }
  /* USER CODE END Error_Handler_Debug */ 
}

#ifdef USE_FULL_ASSERT

/**
   * @brief Reports the name of the source file and the source line number
   * where the assert_param error has occurred.
   * @param file: pointer to the source file name
   * @param line: assert_param error line source number
   * @retval None
   */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
    ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */

}

#endif

/**
  * @}
  */ 

/**
  * @}
*/ 

/************************ (C) COPYRIGHT STMicroelectronics *****END OF FILE****/

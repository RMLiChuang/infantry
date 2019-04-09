/**
  ******************************************************************************
  * File Name          : main.c
	* author						 : �ܺ�
	* Date							 :2018.10.07
  * Description        ;whileѭ����ֻ����ɽ�����λ��������̬���㣬���̵����������Ħ���֣���������Ŀ���ȫ��robomaster_task.c�ļ�������
												Ŀǰ����ֻ�����˵��̵�����ٶȱջ�������������ٶȱջ�����̨�����δ���п��ƣ���һ���ɽ��Ƕ���Ϣ�ںϵ����̵Ŀ�����ȥ
	
  ******************************************************************************
  ******************************************************************************
  */
	
	/*�������������������������������Լ�д��ÿһ�������涼��Ҫ��������ĺ���˵�������ӿɶ��ԣ�����������������������������������*/
	/**********************************************************************************************************
*�� �� ��: 
*����˵��: 
*��    ��: 
*�� �� ֵ: 
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
  MX_CAN1_Init();//canͨ��
  MX_USART1_UART_Init();//ң�������գ������ж�
  MX_TIM1_Init();//������ʱ�����Գ������е�ʱ��
  MX_TIM5_Init();//����4·pwm��������Ħ����
  MX_USART6_UART_Init();//������λ������
	MX_ADC1_Init();
  MX_SPI1_Init();
//	HAL_UART_Receive_IT(&huart6, uart6_rx_buff, 1);//�ú����Ὺ�������жϣ���־λUART_IT_RXNE���������ý��ջ����Լ����ջ���������������
  MX_SPI5_Init();//����imu����
  MX_TIM2_Init();//�жϿ��Ƴ���  ��Ҫ������TIM2������
  MX_TIM3_Init();//δ��
	MX_UART8_Init();
//  MX_TIM4_Init();//USMARTռ��
//  MX_USART2_UART_Init();//��������  δ��
	ref_sys_init();//����ϵͳ��ʼ�����ڲ������úô�����صĶ���
  MX_USART3_UART_Init();//��SDK���� δ��
  delay_init(168);//��ʱ������ʼ��
	//MPU9250_Init();
  //usmart_dev.init(84);//USMART��ʼ��
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
	
  my_can_filter_init_recv_all(&hcan1);     //����CAN������
  HAL_CAN_Receive_IT(&hcan1, CAN_FIFO0);   //����CAN�����ж�
  HAL_UART_Receive_IT_IDLE(&huart1,UART_Buffer,100);   //�������ڽ���
	
	ramp_init(&fric_ramp,SHOOT_CONTROL_TIME * 0.001f, Close_Fric_ON, Fric_OFF);
	oled_init();
	init_Fric_PWM();			//��ʼ��Ħ���ֵ�PWM
	Infantry_Start_Bling();//����״ָ̬ʾ��
	mpu_device_init();   //�ڳ�ʼ��imu��ʱ��Ҫ�ȳ�ʼ��SPI5��GPIOF6 ��Ȼ�޷���ʼ��imu
	robot_status_init();//����ģʽ��ʼ��
	init_quaternion();	//��ʼ����Ԫ����������̬����
	all_pid_init();			//���е����pid��ʼ��
    																	
	//ramp_init(&trigger_motor.fric2_ramp, SHOOT_CONTROL_TIME * 0.005f, Fric_DOWN, Fric_OFF);
	
	Keyboard_value_Init();//���̳�ʼ��
	
	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	HAL_TIM_Base_Start_IT(&htim2);//������ʱ��2  ������Ҫ�����ڶ�ʱ��2�ж��������
	
  /* USER CODE END 2 */
	Super_Cap_Init();//�������ݳ�ʼ�����ڲ��ѳ�ʼ����ADC
	
	 //���̳�ʼ��
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

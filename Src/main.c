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
	
	
	

  
	init_TIM5_PWM();			//��ʼ��TIM5��PWM
	Infantry_Start_Bling();//����״ָ̬ʾ��
	mpu_device_init();   //�ڳ�ʼ��imu��ʱ��Ҫ�ȳ�ʼ��SPI5��GPIOF6 ��Ȼ�޷���ʼ��imu
	robot_status_init();//����ģʽ��ʼ��
	init_quaternion();	//��ʼ����Ԫ����������̬����
	all_pid_init();			//���е����pid��ʼ��
    																	
	//ramp_init(&trigger_motor.fric2_ramp, SHOOT_CONTROL_TIME * 0.005f, Fric_DOWN, Fric_OFF);
	
	HAL_TIM_IC_Start_DMA(&htim1,TIM_CHANNEL_2,(uint32_t *)TIM_COUNT,2);
	HAL_TIM_Base_Start_IT(&htim2);//������ʱ��2  ������Ҫ�����ڶ�ʱ��2�ж��������
	
	//RM2312_Init();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
//  	uint8_t buf[]="asd";

  while (1)
  {	
		 
		
  /* USER CODE END WHILE */

  /* USER CODE BEGIN 3 */
/*		wave_form_data[0] =motor_pid[6].output;
	  wave_form_data[1] =motor_pid[6].target;
	  wave_form_data[2] =moto_chassis[6].speed_rpm;
	  wave_form_data[3] =moto_chassis[6].angle;
	  wave_form_data[4] =moto_chassis[6].total_angle;//moto_chassis[0].real_current;
	  wave_form_data[5] =moto_chassis[6].real_current;//moto_chassis[0].hall;
	*/  
//	  wave_form_data[0] =(short)imu.yaw;
//	  wave_form_data[1] =(short)chassis_yaw.target;
//	  wave_form_data[2] =(short)((imu.mx/imu.my)*100);//chassis_yaw.output;
//	  wave_form_data[3] =(short)chassis_yaw_speed.output;
//	  wave_form_data[4] =(short)HAL_GetTick();//imu.gz;
//	  wave_form_data[5] =(short)chassis_yaw_speed.target;
		
//		wave_form_data[0] =(short)imu.yaw;
//	  wave_form_data[1] =(short)chassis_yaw.target;//MPU_Set_Accel_Fsr(2);
//	  wave_form_data[2] =(short)-imu.gz;//chassis_yaw.output;
//	  wave_form_data[3] =(short)chassis_yaw_speed.target;//mag_field_intensity;//�ų�ǿ��;
//	  wave_form_data[4] =(short)chassis_yaw.output;//imu.gz;
//	  wave_form_data[5] =(short)chassis_yaw_speed.output;

//���̵��������̨�ջ�
//			wave_form_data[0] =(short)chassis_yaw_angle.target;    //YAW ID:206
//	    wave_form_data[1] =(short)chassis_yaw_angle.output;
//	    wave_form_data[2] =(short)moto_chassis[5].angle;
//	    wave_form_data[3] =(short)imu.gz;
//	    wave_form_data[4] =(short)pan_tilt_yaw_speed.output;      //PITCH ID:205
//	    wave_form_data[5] =(short)imu.pit;

//yaw��imu�Ƕȷ�������
//    wave_form_data[0] =(short)pan_tilt_yaw.target;    //YAW ID:206
//	  wave_form_data[1] =(short)motor_pid[5].output;
//	  wave_form_data[2] =(short)pan_tilt_yaw_speed.target;
//	  wave_form_data[3] =(short)-imu.gz;
//		wave_form_data[4] =(short)pan_tilt_yaw_speed.output;      //PITCH ID:205
//	  wave_form_data[5] =(short)imu.yaw;

////pitch��imu�Ƕȷ�������
//    wave_form_data[0] =(short)pan_tilt_pitch.target;      //YAW ID:206
//	  wave_form_data[1] =(short)pan_tilt_pitch.output;
//	  wave_form_data[2] =(short)pan_tilt_pitch_speed.target;
//	  wave_form_data[3] =(short)imu.gy;//pan_tilt_yaw_speed.output;
//		wave_form_data[4] =(short)pan_tilt_pitch_speed.output;//motor_pid[5].err;      //PITCH ID:205
//	  wave_form_data[5] =(short)imu.pit;//moto_chassis[4].angle;

////��̨���е�yaw��pitch��е�Ƕȵ���
//		wave_form_data[0] =(short)motor_pid[5].target;    //YAW ID:206
//	  wave_form_data[1] =(short)motor_pid[5].err;
//	  wave_form_data[2] =(short)moto_chassis[5].angle;
//	  wave_form_data[3] =(short)motor_pid[4].target; 
//		wave_form_data[4] =(short)motor_pid[4].output;     //PITCH ID:205
//	  wave_form_data[5] =(short)moto_chassis[4].angle;

//���canͨ�ŵ�6������Ƿ�������
//		wave_form_data[0] =(short)imu.yaw;//refSysData.PowerHeatData_t.chassisPower;//chassis_yaw_speed.output;      //YAW ID:206
//	  wave_form_data[1] =(short)imu.gz;
//	  wave_form_data[2] =(short)chassis_yaw_angle.output;
//	  wave_form_data[3] =(short)chassis_yaw_angle.target;//motor_pid[3].target;//pan_tilt_yaw_speed.output;
//		wave_form_data[4] =(short)uart6_rx_buff[4];//moto_chassis[4].angle;//moto_chassis[6].round_cnt;//motor_pid[5].err;      //PITCH ID:205
//		wave_form_data[5] =(short)Armour_attack.pan_tilt_angel_err.Yaw_Err;//refSysData.PowerHeatData_t.chassisPower;

//����������
//		wave_form_data[0] =(short)remote_control.mouse.x;
//	  wave_form_data[1] =(short)remote_control.mouse.y;//MPU_Set_Accel_Fsr(2);
//	  wave_form_data[2] =(short)remote_control.mouse.press_left;//chassis_yaw.output;
//	  wave_form_data[3] =(short)remote_control.mouse.press_right;//mag_field_intensity;//�ų�ǿ��;
//	  wave_form_data[4] =(short)remote_control.keyBoard.key_code;//imu.gz;
//	  wave_form_data[5] =(short)chassis_yaw_speed.output;

//�Ӿ���Ϣ
		wave_form_data[0] =(short)pan_tilt_angle;
	  wave_form_data[1] =(short)chassis_yaw_angle.initial;
	  wave_form_data[2] =(short)Armour_attack.pan_tilt_angel_err.Yaw_Err;
	  wave_form_data[3] =(short)Armour_attack.pan_tilt_angel_err.Pitch_Err;
		wave_form_data[4] =(short)pan_tilt_yaw_motor.angle;//vision_yaw.output;;//moto_chassis[4].angle;//moto_chassis[6].round_cnt;//motor_pid[5].err;      //PITCH ID:205
		wave_form_data[5] =(short)UART6_Date[5];//refSysData.PowerHeatData_t.chassisPower;


		shanwai_send_wave_form();   //�����ݴ��䵽������λ�������Կ���ʵʱ����
		}
	
	

	

  /* USER CODE END 3 */

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

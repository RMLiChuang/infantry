/**
  ******************************************************************************
  * @file	super_cap.c
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
#include "super_cap.h"

//ADC_HandleTypeDef hadc1;

static void MX_ADC1_Init(void);

void Super_Cap_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStruct;
	//时钟已在别处打开
	GPIO_InitStruct.Pin =Super_Cap_Pin_OUT|Super_Cap_Pin_IN;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(Super_Cap_Port, &GPIO_InitStruct);
	
	MX_ADC1_Init();
}

static void MX_ADC1_Init(void)
{
	ADC_ChannelConfTypeDef sConfig;
	GPIO_InitTypeDef GPIO_InitStruct;

	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_ADC1_CLK_ENABLE();
	
	GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
	
	hadc1.Instance = ADC1;
	hadc1.Init.ClockPrescaler = ADC_CLOCK_SYNC_PCLK_DIV4;
	hadc1.Init.Resolution = ADC_RESOLUTION_12B;
	hadc1.Init.ScanConvMode = DISABLE;
	hadc1.Init.ContinuousConvMode = DISABLE;
	hadc1.Init.DiscontinuousConvMode = DISABLE;
	hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc1.Init.NbrOfConversion = 1;
	hadc1.Init.DMAContinuousRequests = DISABLE;
	hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	if (HAL_ADC_Init(&hadc1) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}

	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = 1;
	sConfig.SamplingTime = ADC_SAMPLETIME_3CYCLES;
	if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
	{
		_Error_Handler(__FILE__, __LINE__);
	}
}

//返回电压值，范围0~3.3V
float MY_ADC_GetValue(void)
{
	u8 i=1;
	u16 temp=0;
	
	HAL_ADC_Start(&hadc1);                               //开启ADC
	
	HAL_ADC_PollForConversion(&hadc1,10);                //轮询转换
	while(i)
		i--;
	i=1;
	temp=(u16)HAL_ADC_GetValue(&hadc1);			         //返回最近一次ADC1规则组的转换结果
	while(i)
		i--;			//此处若不进行此操作会导致主任务中无法调用此函数（不知为何）
	return (float)temp*(3.3/4096);
}

/*
思路：缓冲能量=60J，就保持给电容充电状态，如果缓冲能量小于60J，而且电容电压>0，就打开电容
	  放电，在电容放电状态下，如果缓冲能量依然小于60，则用超过值对输出进行线性限幅
	
	注意、赛场上机器人停下的时间很短，而一旦用到电容，消耗很快，因此在没超功率情况下尽量让电容
		  处于充电状态
	  KEY_SHIFT:加速档
	  //把电容按照0-100段显示，25左右就已经放完电了
	
	充电功率加上地盘功率，
	如果关闭充电，功率任然超限，就开始进行限速
	1、超功率，电容有电
	2、超功率，电容没电
	3、没超功率，电容有电
	4、没超功率，电容没电

	电容电量与充电功率有线性关系
	接近最低时，功率为0，电容检测电压为16（检测电压乘了100/2.4的系数）
	充满电时功率为60，电容检测电压为116(这些值可根据电容调节，调节完后的值可根据下宏定义改变
*/

#define CAP_MAX 120.0//电容最大容量
#define CAP_MIN 20.0
#define CHARGE_POWER_MAX 60.0//最大充电功率
#define CHARGE_POWER_MIN 0.0


u8 Cap_In_flag=0;//充电标志位
u8 Cap_Out_flag=0;//充电标志位
/*
//估算电容充电过程中消耗功率
参数：voltage：检测到的0-3.3范围的电压值
		 flag：是否处于充电状态
*/
float Calc_Cap_Power(float voltage,u8 flag)
{
	if(flag)
		return ((voltage*(100.0/2.4))*((CHARGE_POWER_MAX-CHARGE_POWER_MIN)/(CAP_MAX-CAP_MIN)))-(voltage*(100.0/2.4))*0.095;//补偿
	else
		return 0;
}
float more_Buffer=0;//本次超出量
float last_more_Buffer=0;//上次超出量
float last_voltage=0;
float current_voltage=0;
float current_voltage_change=0;//每次检测到的电压变化值
float last_voltage_change=0;//上次检测到的电压变化值


//void Super_Cap_control(refDataStruct refData,uint16_t *output)
//{
//	last_voltage_change=current_voltage_change;
//	last_voltage=current_voltage;
//	
//	current_voltage=MY_ADC_GetValue();//获取当前电压
//	current_voltage_change=current_voltage-last_voltage;	
//	if((last_voltage_change-current_voltage_change<0.5)||(last_voltage_change-current_voltage_change<0.5)>-0.5)//用微分思想大概判断是否在充电
//		Cap_In_flag=1;//表示在充电状态
//	
//	if(refData.PowerHeatData_t.chassisPowerBuffer<60)//功率超限
//	{
//		if(Cap_In_flag)//如果处于充电状态(若不处于充电状态时计算的充电功率是无效的（此时功率恒为0）)
//		{
//			if(refData.PowerHeatData_t.chassisPower-Calc_Cap_Power(MY_ADC_GetValue(),Cap_In_flag)>80)//表明底盘功率已经超限
//			{
//				Super_Cap_IN_OFF();
//				Cap_In_flag=0;
//				Super_Cap_OUT_ON();//开启电容供电
//				Cap_Out_flag=1;
//				return;
//			}
//		}
//	}
////	refSysData.PowerHeatData_t.chassisPower;//功率
//	if(KEY_SHIFT)
//	{
//		output[0]=output[0]*1.5;//增大1.5倍输出
//		output[1]=output[1]*1.5;
//		output[2]=output[2]*1.5;
//		output[3]=output[3]*1.5;
//	}
//	if()
//	
//	refSysData.PowerHeatData_t.chassisPowerBuffer;//60J能量缓冲
//}
//思路2，状态机控制
//调试时可打印以下值查看效果
//wave_form_data[0]=refSysData.PowerHeatData_t.chassisPower;//功率
//wave_form_data[1]=refSysData.PowerHeatData_t.chassisPowerBuffer;//60J能量缓冲
//wave_form_data[2]=(MY_ADC_GetValue()*(100.0/2.4));//电容电量
//wave_form_data[3]=(Calc_Cap_Power(MY_ADC_GetValue(),Cap_In_flag));//电容实时充电功率
//wave_form_data[4]=control_state*30;//控制状态标志
//shanwai_send_wave_form();   //将数据传输到三外上位机，可以看到实时波形
u8 chang_flag=0;//控制放电速率
u8 control_state=0;//控制状态标志
void Super_Cap_control(int16_t *output)
{
//	if(Cap_Out_flag)//控制放电速率
//	{
//		chang_flag=!chang_flag;
//		if(chang_flag)
//			Super_Cap_OUT_ON();
//		else
//			Super_Cap_OUT_OFF();
//	}
	//控制电容所处状态及应做出的反应
	switch (control_state)
    {
    	case 0://电容充电状态
		{
//			if(MY_ADC_GetValue()>=2.75)//已经充满电
//			{
//				Super_Cap_IN_OFF();
//				Cap_In_flag=0;
//			}
//			else
//			{
				Super_Cap_IN_ON();//开启充电
				Super_Cap_OUT_OFF();//充电时不能同时放电
				Cap_In_flag=1;
				Cap_Out_flag=0;
//			}
			control_state=1;
		}
    		break;
    	case 1://判断是否超功率
		{
			if(refSysData.PowerHeatData_t.chassisPowerBuffer<60)
			{
				if(refSysData.PowerHeatData_t.chassisPower-Calc_Cap_Power(MY_ADC_GetValue(),Cap_In_flag)>80)//表明底盘功率已经超限
				{
					control_state=2;
				}
				else
					control_state=3;
			}
			else
				control_state=0;//回到充电状态
		}
    		break;
		case 2://底盘超限
		{
			Super_Cap_OUT_ON();//开启电容供电
			Cap_Out_flag=1;
			Super_Cap_IN_OFF();//放电时不能同时充电
			Cap_In_flag=0;
			control_state=4;//电容供电后检测是否任然超限
		}
    		break;
    	case 3://电容导致超限
		{
			Super_Cap_IN_OFF();
			Cap_In_flag=0;
			control_state=1;//再次判断是否超限以及超限原因
		}
    		break;
		case 4://电容供电后检测是否任然超限
		{
			if(refSysData.PowerHeatData_t.chassisPowerBuffer<60)//如果开启电容的情况下任然超限就需要输出限幅了
			{
				output[0]=output[0]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
				output[1]=output[1]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
				output[2]=output[2]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
				output[3]=output[3]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
			}
			else
				control_state=0;//回到充电状态
		}
			break;
    }
}




/**
  ******************************************************************************
  * @file	super_cap.c
  * @author  ��С��
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
	//ʱ�����ڱ𴦴�
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

//���ص�ѹֵ����Χ0~3.3V
float MY_ADC_GetValue(void)
{
	u8 i=1;
	u16 temp=0;
	
	HAL_ADC_Start(&hadc1);                               //����ADC
	
	HAL_ADC_PollForConversion(&hadc1,10);                //��ѯת��
	while(i)
		i--;
	i=1;
	temp=(u16)HAL_ADC_GetValue(&hadc1);			         //�������һ��ADC1�������ת�����
	while(i)
		i--;			//�˴��������д˲����ᵼ�����������޷����ô˺�������֪Ϊ�Σ�
	return (float)temp*(3.3/4096);
}

/*
˼·����������=60J���ͱ��ָ����ݳ��״̬�������������С��60J�����ҵ��ݵ�ѹ>0���ʹ򿪵���
	  �ŵ磬�ڵ��ݷŵ�״̬�£��������������ȻС��60�����ó���ֵ��������������޷�
	
	ע�⡢�����ϻ�����ͣ�µ�ʱ��̣ܶ���һ���õ����ݣ����ĺܿ죬�����û����������¾����õ���
		  ���ڳ��״̬
	  KEY_SHIFT:���ٵ�
	  //�ѵ��ݰ���0-100����ʾ��25���Ҿ��Ѿ��������
	
	��繦�ʼ��ϵ��̹��ʣ�
	����رճ�磬������Ȼ���ޣ��Ϳ�ʼ��������
	1�������ʣ������е�
	2�������ʣ�����û��
	3��û�����ʣ������е�
	4��û�����ʣ�����û��

	���ݵ������繦�������Թ�ϵ
	�ӽ����ʱ������Ϊ0�����ݼ���ѹΪ16������ѹ����100/2.4��ϵ����
	������ʱ����Ϊ60�����ݼ���ѹΪ116(��Щֵ�ɸ��ݵ��ݵ��ڣ���������ֵ�ɸ����º궨��ı�
*/

#define CAP_MAX 120.0//�����������
#define CAP_MIN 20.0
#define CHARGE_POWER_MAX 60.0//����繦��
#define CHARGE_POWER_MIN 0.0


u8 Cap_In_flag=0;//����־λ
u8 Cap_Out_flag=0;//����־λ
/*
//������ݳ����������Ĺ���
������voltage����⵽��0-3.3��Χ�ĵ�ѹֵ
		 flag���Ƿ��ڳ��״̬
*/
float Calc_Cap_Power(float voltage,u8 flag)
{
	if(flag)
		return ((voltage*(100.0/2.4))*((CHARGE_POWER_MAX-CHARGE_POWER_MIN)/(CAP_MAX-CAP_MIN)))-(voltage*(100.0/2.4))*0.095;//����
	else
		return 0;
}
float more_Buffer=0;//���γ�����
float last_more_Buffer=0;//�ϴγ�����
float last_voltage=0;
float current_voltage=0;
float current_voltage_change=0;//ÿ�μ�⵽�ĵ�ѹ�仯ֵ
float last_voltage_change=0;//�ϴμ�⵽�ĵ�ѹ�仯ֵ


//void Super_Cap_control(refDataStruct refData,uint16_t *output)
//{
//	last_voltage_change=current_voltage_change;
//	last_voltage=current_voltage;
//	
//	current_voltage=MY_ADC_GetValue();//��ȡ��ǰ��ѹ
//	current_voltage_change=current_voltage-last_voltage;	
//	if((last_voltage_change-current_voltage_change<0.5)||(last_voltage_change-current_voltage_change<0.5)>-0.5)//��΢��˼�����ж��Ƿ��ڳ��
//		Cap_In_flag=1;//��ʾ�ڳ��״̬
//	
//	if(refData.PowerHeatData_t.chassisPowerBuffer<60)//���ʳ���
//	{
//		if(Cap_In_flag)//������ڳ��״̬(�������ڳ��״̬ʱ����ĳ�繦������Ч�ģ���ʱ���ʺ�Ϊ0��)
//		{
//			if(refData.PowerHeatData_t.chassisPower-Calc_Cap_Power(MY_ADC_GetValue(),Cap_In_flag)>80)//�������̹����Ѿ�����
//			{
//				Super_Cap_IN_OFF();
//				Cap_In_flag=0;
//				Super_Cap_OUT_ON();//�������ݹ���
//				Cap_Out_flag=1;
//				return;
//			}
//		}
//	}
////	refSysData.PowerHeatData_t.chassisPower;//����
//	if(KEY_SHIFT)
//	{
//		output[0]=output[0]*1.5;//����1.5�����
//		output[1]=output[1]*1.5;
//		output[2]=output[2]*1.5;
//		output[3]=output[3]*1.5;
//	}
//	if()
//	
//	refSysData.PowerHeatData_t.chassisPowerBuffer;//60J��������
//}
//˼·2��״̬������
//����ʱ�ɴ�ӡ����ֵ�鿴Ч��
//wave_form_data[0]=refSysData.PowerHeatData_t.chassisPower;//����
//wave_form_data[1]=refSysData.PowerHeatData_t.chassisPowerBuffer;//60J��������
//wave_form_data[2]=(MY_ADC_GetValue()*(100.0/2.4));//���ݵ���
//wave_form_data[3]=(Calc_Cap_Power(MY_ADC_GetValue(),Cap_In_flag));//����ʵʱ��繦��
//wave_form_data[4]=control_state*30;//����״̬��־
//shanwai_send_wave_form();   //�����ݴ��䵽������λ�������Կ���ʵʱ����
u8 chang_flag=0;//���Ʒŵ�����
u8 control_state=0;//����״̬��־
void Super_Cap_control(int16_t *output)
{
//	if(Cap_Out_flag)//���Ʒŵ�����
//	{
//		chang_flag=!chang_flag;
//		if(chang_flag)
//			Super_Cap_OUT_ON();
//		else
//			Super_Cap_OUT_OFF();
//	}
	//���Ƶ�������״̬��Ӧ�����ķ�Ӧ
	switch (control_state)
    {
    	case 0://���ݳ��״̬
		{
//			if(MY_ADC_GetValue()>=2.75)//�Ѿ�������
//			{
//				Super_Cap_IN_OFF();
//				Cap_In_flag=0;
//			}
//			else
//			{
				Super_Cap_IN_ON();//�������
				Super_Cap_OUT_OFF();//���ʱ����ͬʱ�ŵ�
				Cap_In_flag=1;
				Cap_Out_flag=0;
//			}
			control_state=1;
		}
    		break;
    	case 1://�ж��Ƿ񳬹���
		{
			if(refSysData.PowerHeatData_t.chassisPowerBuffer<60)
			{
				if(refSysData.PowerHeatData_t.chassisPower-Calc_Cap_Power(MY_ADC_GetValue(),Cap_In_flag)>80)//�������̹����Ѿ�����
				{
					control_state=2;
				}
				else
					control_state=3;
			}
			else
				control_state=0;//�ص����״̬
		}
    		break;
		case 2://���̳���
		{
			Super_Cap_OUT_ON();//�������ݹ���
			Cap_Out_flag=1;
			Super_Cap_IN_OFF();//�ŵ�ʱ����ͬʱ���
			Cap_In_flag=0;
			control_state=4;//���ݹ�������Ƿ���Ȼ����
		}
    		break;
    	case 3://���ݵ��³���
		{
			Super_Cap_IN_OFF();
			Cap_In_flag=0;
			control_state=1;//�ٴ��ж��Ƿ����Լ�����ԭ��
		}
    		break;
		case 4://���ݹ�������Ƿ���Ȼ����
		{
			if(refSysData.PowerHeatData_t.chassisPowerBuffer<60)//����������ݵ��������Ȼ���޾���Ҫ����޷���
			{
				output[0]=output[0]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
				output[1]=output[1]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
				output[2]=output[2]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
				output[3]=output[3]*(1-(60.0f-refSysData.PowerHeatData_t.chassisPowerBuffer)/60.0f);
			}
			else
				control_state=0;//�ص����״̬
		}
			break;
    }
}




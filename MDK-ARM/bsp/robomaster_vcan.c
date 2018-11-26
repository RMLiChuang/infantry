#include "robomaster_vcan.h"
#include "usart.h"

//��.c�ļ���Ҫ�����ɽ��������ַ��Ͳ���
short  wave_form_data[6] = {0};
void send_data(uint8_t date)
{
	HAL_UART_Transmit(&huart6,&date,1,10);
	//while( USART_GetFlagStatus(USART1,USART_FLAG_TC)!= SET);  
	
}
void shanwai_send_wave_form(void)
{
	uint8_t i;
	
	send_data(0x03);
	send_data(0xfc);
	for(i = 0;i<6;i++)
	{
	  send_data((wave_form_data[i]&0xff)); //�ַ��͵�λ�ڷ��͸�λ
	  send_data((wave_form_data[i]>>8));
		
	  
	}
	send_data(0xfc);
	send_data(0x03);
}	

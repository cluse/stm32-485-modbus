#include "bsp.h"
#include "pbdata.h"

void bsp_init(void)
{
	SystemInit();//72m
	LED_Config();
	USART_Configuration();//���ڳ�ʼ��
	KEY_Init();            //������ʼ��
	//USART3_Init(9600);    //����3��ʼ��-����485
}

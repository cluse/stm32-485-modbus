#include "bsp.h"
#include "pbdata.h"

void bsp_init(void)
{
	SystemInit();//72m
	LED_Config();
	USART_Configuration();//串口初始化
	KEY_Init();            //按键初始化
	//USART3_Init(9600);    //串口3初始化-驱动485
}

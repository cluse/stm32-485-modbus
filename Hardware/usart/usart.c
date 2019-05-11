#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usart.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

#if 1
#pragma import(__use_no_semihosting)             
//标准库需要的支持函数                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//定义_sys_exit()以避免使用半主机模式    
_sys_exit(int x) 
{ 
	x = x; 
} 
//重定义fputc函数 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//循环发送,直到发送完毕   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*
  USART配置GPIO配置
*/
void USART_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* 打开GPIOA时钟、AFIO时钟，USART1时钟 */
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOA | RCC_APB2Periph_AFIO, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_USART1, ENABLE);
	/* USART1 TX PA9 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_9;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
	/* USART1 RX PA10 */
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOA, &GPIO_InitStructure);
											
	/* USART 配置 */
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	/* 使能USART1接收中断 */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 

	/* 使能USART1 */
	USART_Cmd(USART1, ENABLE);
	/* 清除发送完成标志 */
	USART_ClearFlag(USART1, USART_FLAG_TC);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		   

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			 //串口中断设置
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
  串口发送函数
*/
void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat,uint16_t len)
{
  uint16_t i;
  for(i=0;i<len;i++)
  {
  	USART_SendData(USARTx, Dat[i]);
	  while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET);     
  }
}

/*
  串口发送字符串
*/
void USART_STR(USART_TypeDef* USARTx,char *str)
{
  uint8_t len,i;
	len=strlen(str);
	for(i=0;i<len;i++)
	{
	  	USART_SendData(USARTx, str[i]);
		while(USART_GetFlagStatus(USARTx, USART_FLAG_TC)==RESET); 	
	}
}







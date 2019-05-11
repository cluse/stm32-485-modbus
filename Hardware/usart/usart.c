#include "stm32f10x.h"
#include "stm32f10x_usart.h"
#include "usart.h"
#include "misc.h"
#include <stdio.h>
#include <string.h>

#if 1
#pragma import(__use_no_semihosting)             
//��׼����Ҫ��֧�ֺ���                 
struct __FILE 
{ 
	int handle; 

}; 

FILE __stdout;       
//����_sys_exit()�Ա���ʹ�ð�����ģʽ    
_sys_exit(int x) 
{ 
	x = x; 
} 
//�ض���fputc���� 
int fputc(int ch, FILE *f)
{      
	while((USART1->SR&0X40)==0);//ѭ������,ֱ���������   
    USART1->DR = (u8) ch;      
	return ch;
}
#endif 

/*
  USART����GPIO����
*/
void USART_Configuration(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	USART_InitTypeDef USART_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;

	/* ��GPIOAʱ�ӡ�AFIOʱ�ӣ�USART1ʱ�� */
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
											
	/* USART ���� */
	USART_DeInit(USART1);
	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(USART1, &USART_InitStructure);
	
	/* ʹ��USART1�����ж� */
	USART_ITConfig(USART1, USART_IT_RXNE, ENABLE); 

	/* ʹ��USART1 */
	USART_Cmd(USART1, ENABLE);
	/* ���������ɱ�־ */
	USART_ClearFlag(USART1, USART_FLAG_TC);

	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);		   

	/* Enable the USART1 Interrupt */
	NVIC_InitStructure.NVIC_IRQChannel = USART1_IRQn;			 //�����ж�����
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 1;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
}

/*
  ���ڷ��ͺ���
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
  ���ڷ����ַ���
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







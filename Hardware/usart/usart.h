#ifndef __USART_H__
#define __USART_H__

void USART_Configuration(void);
void USART_Send(USART_TypeDef* USARTx, uint8_t *Dat,uint16_t len);
void USART_STR(USART_TypeDef* USARTx,char *str);

#endif


#ifndef _pbdata_H
#define _pbdata_H

#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include <stdio.h>
#include "string.h"


//�������
extern u8 dt;    //ע��Ҫ�зֺ�

//���庯��
void delay(u32 nCount);  //ע��Ҫ�зֺ�
void delay_us(u32 nus);
void delay_ms(u16 nms);
void RCC_HSE_Configuration(void);

#endif
//��Ź��������͹�������

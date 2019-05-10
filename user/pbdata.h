#ifndef _pbdata_H
#define _pbdata_H

#include "stm32f10x.h"
#include "misc.h"
#include "stm32f10x_it.h"
#include <stdio.h>
#include "string.h"


//定义变量
extern u8 dt;    //注意要有分号

//定义函数
void delay(u32 nCount);  //注意要有分号
void delay_us(u32 nus);
void delay_ms(u16 nms);
void RCC_HSE_Configuration(void);

#endif
//存放公共变量和公共函数

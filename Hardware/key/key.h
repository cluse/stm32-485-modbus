#ifndef __KEY_H
#define __KEY_H	 

#include "pbdata.h"

#define KEY0  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_14)//��ȡ����0
#define KEY1  GPIO_ReadInputDataBit(GPIOB,GPIO_Pin_15)//��ȡ����1
 

#define KEY0_PRES	1		//KEY0  
#define KEY1_PRES	2		//KEY1 

void KEY_Init(void);//IO��ʼ��
u8 KEY_Scan(u8 mode);  	//����ɨ�躯��		

#endif

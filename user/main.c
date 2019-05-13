/******************** (C) COPYRIGHT 2017 Designed by Captain *********************
 * 文件名  ：main.c
 * 描述    : RS485双机收发测试
 *         ：将两块STM32F105开发板485总线接口相连，分别按下各自开发板上的独立按键
             能够向另一块开发板发送对应的数据，开发板接受到数据后，通过串口打印输出数据
 * 实验平台：STM32F105RBT6/RCT6开发板
 * 库版本  ：ST3.5.0
 * 时间    ：2018-10-18
 * 作者    ：老船长电子研发
  https://item.taobao.com/item.htm?id=556036697936
**********************************************************************************/
#include "pbdata.h"
#include "bsp.h"


void print_modbus(u8 *head,u8 salve,u8 key,u8 len,u8 *data)
{
	u8 i;
	printf("%s slave=%x key=%x len=%d data=",head,salve,key,len);
	for (i=0;i<len;i++) {
		printf("%x,",*data++);
	}
	printf("\n");
}

int main(void)
{
	u8 tx485buf1[] = {1,2,3,4,5};
	u8 tx485buf2[] = {6,7,8,9,0};
	u8 key;
	
	bsp_init(); //系统硬件初始化程序  

	Modbus_Master_Init();
	Modbus_Slave_Init();

	/*串口1 发送字符串*/
	USART_STR(USART1,"++++++++++++++++++++++++\r\n");
	USART_STR(USART1,"  STM32F105 Study Board \r\n");
	USART_STR(USART1,"    RS485双机数据通信   \r\n");
	USART_STR(USART1,"++++++++++++++++++++++++\r\n");		

	while (1)
	{  
		//GetKey(); //检测是否有按键按下			
		key=KEY_Scan(0);
		if(key==KEY0_PRES)//KEY0按下,发送一次数据
		{
			//RS485_Send_Data(tx485buf1,sizeof(tx485buf1));//发送5个字节 			
			//printf("tx485byf:%s\n",tx485buf1);
			if (ModBus_MasterSend(MODBUS_SLAVE_ADD,0,sizeof(tx485buf1),tx485buf1)) {
				//printf("ModBus_MasterSend tx485buf1\n");
				print_modbus("master ->",MODBUS_SLAVE_ADD,0,sizeof(tx485buf1),tx485buf1);
			}
			else {
				printf("err -> ModBus_MasterSend tx485buf1\n");
			}
		}
		else if(key==KEY1_PRES)
		{
			//RS485_Send_Data(tx485buf2,sizeof(tx485buf2));//发送5个字节 			
			//printf("tx485byf:%s\n",tx485buf2);
			if (ModBus_MasterSend(MODBUS_SLAVE_ADD,0,sizeof(tx485buf2),tx485buf2)) {
				//printf("ModBus_MasterSend tx485buf2\n");
				print_modbus("master ->",MODBUS_SLAVE_ADD,0,sizeof(tx485buf2),tx485buf2);
			}
			else {
				printf("err -> ModBus_MasterSend tx485buf2\n");
			}		
		}	

		if (is_ModBus_MasterReceive()) {
			//printf("ModBus_MasterGet Success\n");
			print_modbus("master <-",
				get_ModBus_MasterReceiveSlave(),
				get_ModBus_MasterReceiveKey(),
				get_ModBus_MasterReceiveLen(),
				get_ModBus_MasterReceiveData());
			empty_ModBus_MasterReceive();
		}

		delay_us(1000);
		ModBus_TimerEvent(1);
	}
}


#ifdef  USE_FULL_ASSERT

/**
  * @brief  Reports the name of the source file and the source line number
  *   where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* Infinite loop */
  while (1)
  {
  }
}
#endif

/******************* (C) COPYRIGHT 2009 STMicroelectronics *****END OF FILE****/

/******************** (C) COPYRIGHT 2017 Designed by Captain *********************
 * �ļ���  ��main.c
 * ����    : RS485˫���շ�����
 *         ��������STM32F105������485���߽ӿ��������ֱ��¸��Կ������ϵĶ�������
             �ܹ�����һ�鿪���巢�Ͷ�Ӧ�����ݣ���������ܵ����ݺ�ͨ�����ڴ�ӡ�������
 * ʵ��ƽ̨��STM32F105RBT6/RCT6������
 * ��汾  ��ST3.5.0
 * ʱ��    ��2018-10-18
 * ����    ���ϴ��������з�
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
	
	bsp_init(); //ϵͳӲ����ʼ������  

	Modbus_Master_Init();
	Modbus_Slave_Init();

	/*����1 �����ַ���*/
	USART_STR(USART1,"++++++++++++++++++++++++\r\n");
	USART_STR(USART1,"  STM32F105 Study Board \r\n");
	USART_STR(USART1,"    RS485˫������ͨ��   \r\n");
	USART_STR(USART1,"++++++++++++++++++++++++\r\n");		

	while (1)
	{  
		//GetKey(); //����Ƿ��а�������			
		key=KEY_Scan(0);
		if(key==KEY0_PRES)//KEY0����,����һ������
		{
			//RS485_Send_Data(tx485buf1,sizeof(tx485buf1));//����5���ֽ� 			
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
			//RS485_Send_Data(tx485buf2,sizeof(tx485buf2));//����5���ֽ� 			
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

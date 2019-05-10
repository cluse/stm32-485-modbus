#ifndef _485_H
#define _485_H

#include "stm32f10x.h"

#define bool	u8
#define false 0
#define true	1

//-------------------------------------------------
#define MODBUS_SPEED 9600
#define MODBUS_SLAVE_ADD 2

#define MASTER_IDLE_TIME 10

#define MODBUS_APPEND_LEN 5
#define BUF_MAX_LEN 64
struct MODBUS_BUF {
	bool active;
	u8 len;
	u8 buf[BUF_MAX_LEN];	//slave,key,len,d0,d1,...,crc0,crc1
};


//-------------------------------------------------
void Modbus_Master_Init(void);
void Modbus_Slave_Init(void);

bool ModBus_MasterSend(u8 slave,u8 key,u8 len,u8 *data);
bool ModBus_SlaveSend(u8 slave,u8 key,u8 len,u8 *data);

bool ModBus_MasterGet(struct MODBUS_BUF *lp);

void ModBus_TimerEvent(u8 tick_ms);



#endif




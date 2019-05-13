#ifndef _485_H
#define _485_H

#include "stm32f10x.h"


#define DEBUG_MODBUS


//-------------------------------------------------
#define bool	u8
#define false 0
#define true	1


//-------------------------------------------------
#define MODBUS_SPEED 9600
#define MODBUS_SLAVE_ADD 2

#define MASTER_IDLE_TIME 10


//-------------------------------------------------
void Modbus_Master_Init(void);
void Modbus_Slave_Init(void);

bool ModBus_MasterSend(u8 slave,u8 key,u8 len,u8 *data);
bool ModBus_SlaveSend(u8 slave,u8 key,u8 len,u8 *data);

bool is_ModBus_MasterReceive();
u8 get_ModBus_MasterReceiveSlave();
u8 get_ModBus_MasterReceiveKey();
u8 get_ModBus_MasterReceiveLen();
u8* get_ModBus_MasterReceiveData();
void empty_ModBus_MasterReceive();

void ModBus_TimerEvent(u8 tick_ms);



#endif




#include "485.h"
#include "bsp.h"
#include "pbdata.h"
#include "misc.h"

#include "modbus_crc.h"


//-------------------------------------------------
//static USART_TypeDef *master_uart;
//static USART_TypeDef *slave_uart;
#define master_uart USART3
#define slave_uart UART4

static struct MODBUS_BUF master_tx;
static struct MODBUS_BUF master_rx;
static struct MODBUS_BUF slave_tx;
static struct MODBUS_BUF slave_rx;

static u16 all_tick_ms = 0;
static u16 master_idle_ms;
static u16 slave_idle_ms;
#define MASTER_IDLE_RESET (master_idle_ms = all_tick_ms)
#define SLAVE_IDLE_RESET (slave_idle_ms = all_tick_ms)

static void Master_Receive(struct MODBUS_BUF *lp);
static void Slave_Receive(struct MODBUS_BUF *lp);


//-------------------------------------------------
static void USART_base(USART_TypeDef *reg_base)
{
	USART_InitTypeDef USART_InitStructure;
	USART_InitStructure.USART_BaudRate = MODBUS_SPEED;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_2;
	USART_InitStructure.USART_Parity = USART_Parity_No ;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Rx | USART_Mode_Tx;
	USART_Init(reg_base, &USART_InitStructure);
	USART_ITConfig(reg_base, USART_IT_RXNE, ENABLE);
	USART_Cmd(reg_base, ENABLE);
	USART_ClearFlag(reg_base, USART_FLAG_TC);
}

static void Master_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//interrupt
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_InitStructure.NVIC_IRQChannel = USART3_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 2;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//osc
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_USART3, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOB, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);//使能外设时钟  
	//485 clt pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_0 | GPIO_Pin_1 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOB, &GPIO_InitStructure);
	//tx pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOB, &GPIO_InitStructure);	
	//rx pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOB, &GPIO_InitStructure);   //初始化GPIOA
	//uart
	//master_uart = USART3;
	USART_base(master_uart);
}

static void Master_Tx_State(void)
{
	GPIO_SetBits(GPIOB,GPIO_Pin_1);
	GPIO_SetBits(GPIOB,GPIO_Pin_0);
}

static void Master_Rx_State(void)
{
	GPIO_ResetBits(GPIOB,GPIO_Pin_1);
	GPIO_ResetBits(GPIOB,GPIO_Pin_0);
}

static void Slave_Init(void)
{
	GPIO_InitTypeDef GPIO_InitStructure;
	NVIC_InitTypeDef NVIC_InitStructure;
	//interrupt
	NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1); 
	NVIC_InitStructure.NVIC_IRQChannel = UART4_IRQn;
	NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
	NVIC_InitStructure.NVIC_IRQChannelSubPriority = 3;
	NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&NVIC_InitStructure);
	//osc
	RCC_APB1PeriphClockCmd(RCC_APB1Periph_UART4,ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIOC, ENABLE);
	RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO, ENABLE);
	//485 clt pins
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_8 | GPIO_Pin_9 ;
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
	GPIO_Init(GPIOC, &GPIO_InitStructure);
	//tx pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_10;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP; //复用推挽输出
	GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
	GPIO_Init(GPIOC, &GPIO_InitStructure);	
	//rx pin
	GPIO_InitStructure.GPIO_Pin = GPIO_Pin_11;
	GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;   //浮空输入
	GPIO_Init(GPIOC, &GPIO_InitStructure);   //初始化GPIOA
	//uart
	//slave_uart = UART4;
	USART_base(slave_uart);
}

static void Slave_Tx_State(void)
{
	GPIO_SetBits(GPIOC,GPIO_Pin_9);
	GPIO_SetBits(GPIOC,GPIO_Pin_8);
}

static void Slave_Rx_State(void)
{
	GPIO_ResetBits(GPIOC,GPIO_Pin_9);
	GPIO_ResetBits(GPIOC,GPIO_Pin_8);
}


//-------------------------------------------------
static void buf_init(struct MODBUS_BUF *lp)
{
	lp->active = false;
	lp->len = 0;
}

static void buf_copy(struct MODBUS_BUF *src,struct MODBUS_BUF *dst)
{
	u8 i;
	dst->len = src->len;
	for (i=0;i<src->len;i++) {

		dst->buf[i] = src->buf[i];
	}
}

static void buf_add(struct MODBUS_BUF *lp,u8 val)
{
	if (lp->len < BUF_MAX_LEN) {
		lp->buf[lp->len] = val;
		lp->len++; 
	}
}

static u8 get_add_from_buf(struct MODBUS_BUF *lp)
{
	return lp->buf[0];
}

static u8 get_key_from_buf(struct MODBUS_BUF *lp)
{
	return lp->buf[1];
}

static u8 get_len_from_buf(struct MODBUS_BUF *lp)
{
	return lp->buf[2];
}

static void buf_calc_crc(u8 *buf,u8 len)
{
	u16 crc;
	crc = crc16tablefast(buf,len&0xff);
	buf[len] = ((crc>>8)&0xff);
	buf[len+1] = (crc&0xff);
}

static bool is_buf_crc_right(struct MODBUS_BUF *lp)
{
	u16 crc;
	u8 *buf;
	u8 len,crc_h,crc_l;
	buf = lp->buf;
	len = lp->len - 2;
	crc = crc16tablefast(buf,len&0xff);
	crc_h = ((crc>>8)&0xff);
	crc_l = (crc&0xff);
	if (crc_h != buf[len]) {
		return false;
	}
	if (crc_l != buf[len+1]) {
		return false;
	}
	return true;
}

static bool is_frame_vaild(struct MODBUS_BUF *lp)
{
	u8 len;
	if (lp->len < MODBUS_APPEND_LEN) {
		return false;
	}
	len = get_len_from_buf(lp);
	if (len > BUF_MAX_LEN - MODBUS_APPEND_LEN ||
		lp->len < len + MODBUS_APPEND_LEN) {
		return false;
	}
	return is_buf_crc_right(lp);
}

static void Master_Idle_Detect(void)
{
	if (all_tick_ms < master_idle_ms) {
		master_idle_ms = 0;
	}
	if (all_tick_ms > master_idle_ms + MASTER_IDLE_TIME) {
		master_tx.active = false;
	}
}

static void Slave_Idle_Detect(void)
{
	if (all_tick_ms < slave_idle_ms) {
		slave_idle_ms = 0;
	}
	if (all_tick_ms > slave_idle_ms + MASTER_IDLE_TIME) {
		slave_rx.active = false;
		slave_rx.len = 0;
	}
}


//-------------------------------------------------
static u8 master_tx_index;
static void Master_TxStart(void)
{
	if (master_tx.len > 0) {
		Master_Tx_State();
		MASTER_IDLE_RESET;
		master_tx.active = true;
		master_tx_index = 1;
		USART_ClearFlag(master_uart,USART_FLAG_TC);
		USART_ITConfig(master_uart, USART_IT_TC, ENABLE);
		USART_SendData(master_uart, master_tx.buf[0]);
	}
}

static void Master_IntEvent()
{

	MASTER_IDLE_RESET;
	if(USART_GetITStatus(master_uart, USART_IT_TC) != RESET) {
		if (master_tx_index < master_tx.len) {
			USART_SendData(master_uart, master_tx.buf[master_tx_index++]);
		}
		else {
			Master_Rx_State();
			USART_ITConfig(master_uart, USART_IT_TC, DISABLE);
			USART_ClearFlag(master_uart,USART_FLAG_TC);
			//master_tx.active = false;
		}
	}
	else if(USART_GetITStatus(master_uart, USART_IT_RXNE) != RESET) {
		u8 tmp =USART_ReceiveData(master_uart);
		if (!master_rx.active) {
			buf_add(&master_rx,tmp);
			if (is_frame_vaild(&master_rx)) {
				Master_Receive(&master_rx);
			}
		}
	} 
}

static void Master_Receive(struct MODBUS_BUF *lp)
{
	lp->active = true;
}


//-------------------------------------------------
static u8 slave_tx_index;
static void Slave_TxStart()
{
	if (slave_tx.len > 0) {
		Slave_Tx_State();
		delay(100);
		slave_tx.active = true;
		slave_tx_index = 1;
		USART_ClearFlag(slave_uart,USART_FLAG_TC);
		USART_ITConfig(slave_uart, USART_IT_TC, ENABLE);
		USART_SendData(slave_uart, slave_tx.buf[0]);
	}
}

static void Slave_IntEvent()
{
	SLAVE_IDLE_RESET;
	if(USART_GetITStatus(slave_uart, USART_IT_TC) != RESET) {
		if (slave_tx_index < slave_tx.len) {
			USART_SendData(slave_uart, slave_tx.buf[slave_tx_index++]);
		}
		else {
			Slave_Rx_State();
			USART_ITConfig(slave_uart, USART_IT_TC, DISABLE);
			USART_ClearFlag(slave_uart,USART_FLAG_TC);
			slave_tx.active = false;
		}
	}
	else if(USART_GetITStatus(slave_uart, USART_IT_RXNE) != RESET) {
		u8 tmp = USART_ReceiveData(slave_uart);
		buf_add(&slave_rx,tmp);
		if (is_frame_vaild(&slave_rx)) {
			Slave_Receive(&slave_rx);
			slave_rx.active = false;
			slave_rx.len = 0;
		}
	} 
}

static void Slave_Receive(struct MODBUS_BUF *lp)
{

	u8 address,key,len;
	address = get_add_from_buf(lp);
	key = get_key_from_buf(lp);
	len = get_len_from_buf(lp);
	if (address != MODBUS_SLAVE_ADD) {
		return;
	}
	buf_copy(lp,&slave_tx);
	Slave_TxStart();
}


//-------------------------------------------------
void Modbus_Master_Init()
{
	buf_init(&master_tx);
	buf_init(&master_rx);
	Master_Init();
	Master_Rx_State();
}

void Modbus_Slave_Init()
{
	buf_init(&slave_tx);
	buf_init(&slave_rx);
	Slave_Init();
	Slave_Rx_State();
}

static bool ModBus_Send(struct MODBUS_BUF *lp,u8 slave,u8 key,u8 len,u8 *data)
{
	u8 index;
	if (!lp->active) {
		if (len <= BUF_MAX_LEN - MODBUS_APPEND_LEN) {
			lp->buf[0] = slave;
			lp->buf[1] = key;
			lp->buf[2] = len;
			for (index=3;index<len+3;index++) {
				lp->buf[index] = *data++;
			}
			buf_calc_crc(lp->buf,len+3);
			lp->len = len + MODBUS_APPEND_LEN;
			return true;
		}
	}
	return false;
}

bool ModBus_MasterSend(u8 slave,u8 key,u8 len,u8 *data)
{
	if (ModBus_Send(&master_tx,slave,key,len,data)) {
		Master_TxStart();
		return true;
	}
	return false;
}

bool ModBus_SlaveSend(u8 slave,u8 key,u8 len,u8 *data)
{
	return ModBus_Send(&slave_tx,slave,key,len,data);
}

bool ModBus_MasterGet(struct MODBUS_BUF *lp)
{
	if (master_rx.active) {
		buf_copy(&master_rx,lp);
		master_rx.len = 0;
		master_rx.active = false;
		return true;
	}
	return false;
}

void ModBus_TimerEvent(u8 tick_ms)
{
	all_tick_ms += tick_ms;
	if (all_tick_ms > 10000) {
		all_tick_ms = 0;
	}
	Master_Idle_Detect();
	Slave_Idle_Detect();
}




//-------------------------------------------------
void USART3_IRQHandler(void)
{
	Master_IntEvent();
}

void UART4_IRQHandler(void)
{
	Slave_IntEvent();
}








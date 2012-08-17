#include "zoSmsMaster.h"
#include "zoCommands.h"

ZO_ERROR *zoSmsMasterError = &zoCommandsError;

void zoSmsMasterInit(ZO_PROTOCOL_HW_TYPE hw,ZO_PROTOCOL_HAL *hal, u08 localNodeID)
{
	zoProtocolInit(hw,hal,localNodeID);
	zoCommandsInit(hal);	
}

void zoSmsMasterInitI2c(void)
{
	static ZO_PROTOCOL_HAL hal;

	zoProtocolInit(ZO_PROTOCOL_HW_I2C,&hal,ZO_SMS_MASTER_DEFAULT_NODE_ID);
	zoCommandsInit(&hal);
}

void zoSmsMasterInitUart(void)
{
	static ZO_PROTOCOL_HAL hal;

	zoProtocolInit(ZO_PROTOCOL_HW_UART_5V,&hal,ZO_SMS_MASTER_DEFAULT_NODE_ID);
	zoCommandsInit(&hal);
}

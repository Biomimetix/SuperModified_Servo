#include <avr/io.h>
#include "zoProtocolUart.h"
#include "zoMcu.h"
#include "zoSystemTimer.h"

typedef enum {
	WAIT_ON_HEADER_0,
	WAIT_ON_HEADER_1,
	WAIT_ON_ADDRESSED_NODE_ID,
	WAIT_ON_OWN_NODE_ID,
	WAIT_ON_COMMAND_ID,
	WAIT_ON_BYTECOUNT,
	WAIT_ON_DATA,
	WAIT_ON_LRC
}ZO_PROTOCOL_UART_DECODER_STATE;

static ZO_PROTOCOL_UART_DECODER_STATE DecoderState = WAIT_ON_HEADER_0;
static volatile u08 OwnNodeID = 0;
static u08 zoProtocolUartLAM = 0xFF;

static u16 WaitOnNextCharacterTimer = 0;
static bool PacketRxStarted = FALSE;
static const u16 WaitOnNextCharTimeOutMiliSec = 50;


ZO_ERROR *zoProtocolUartError = &zoUartError;

void setReceivalOfPacketStarted(bool trueFalse)
{
	enterCritical();
	PacketRxStarted = trueFalse;
	exitCritical();
}

bool getReceivalOfPacketStarted(void)
{
	bool returnVal;

	enterCritical();
	returnVal = PacketRxStarted;
	exitCritical();

	return returnVal;
}

bool zoProtocolUartInit(ZO_PROTOCOL_HW_TYPE hw, u08 ownNodeID, u32 baudRate)
{
	bool success = TRUE;
	
	if( hw == ZO_PROTOCOL_HW_HALF_DUPLEX_RS485 )
		zoUartInitRs485(&PORTD,PD2);
	
	//no error checking for valid node ID, this is left to be done in a higher level
	OwnNodeID = ownNodeID;	//store the node ID

	if(!zoUartInit())
		success = FALSE;
	
	if(!zoUartSetBaud(baudRate))
		success = FALSE;

	zoSystemTimerTimeOutInit(&WaitOnNextCharacterTimer);

	return success;
}

bool zoProtocolUartPutPacket(const ZO_PROTOCOL_PACKET* packet)
{
	if(!zoUartPutChar(ZO_PROTOCOL_HEADER_0))
		return FALSE;

	if(!zoUartPutChar(ZO_PROTOCOL_HEADER_1))
		return FALSE;

	if(!zoUartPutChar(packet->AddressedNodeID))
		return FALSE;

	if(!zoUartPutChar(packet->OwnNodeID))
		return FALSE;

	if(!zoUartPutChar(packet->commandID))
		return FALSE;

	if(!zoUartPutChar(packet->byteCount))
		return FALSE;

	if(!zoUartPutData(packet->data,packet->byteCount))
		return FALSE;

	if(!zoUartPutChar(packet->lrc))
		return FALSE;
	
	return TRUE;
}

bool zoProtocolUartGetPacket(ZO_PROTOCOL_PACKET* packet)
{
	static u08 byteCount;
	bool IsWholePacket = FALSE;
	u08 c;

	if( getReceivalOfPacketStarted() && zoSystemTimerTimeOutExpired(&WaitOnNextCharacterTimer,WaitOnNextCharTimeOutMiliSec) )
	{
		DecoderState = WAIT_ON_HEADER_0;										//reset the decoder
		zoErrorPut(zoProtocolUartError,ZO_PROTOCOL_UART_ERROR_HALF_PACKET);		//indicate error

		//initialize timeout timer and flag for next pass
		zoSystemTimerTimeOutInit(&WaitOnNextCharacterTimer);
		setReceivalOfPacketStarted(FALSE);
	}

	if( !zoUartGetChar(&c) )
		return FALSE;

	zoSystemTimerTimeOutInit(&WaitOnNextCharacterTimer); //start counting timeout until reception of next character

	switch(DecoderState) 
	{
		case WAIT_ON_HEADER_0:
			if (c==ZO_PROTOCOL_HEADER_0)
			{
				DecoderState = WAIT_ON_HEADER_1;
				setReceivalOfPacketStarted(TRUE);
			}
			else
				DecoderState = WAIT_ON_HEADER_0;
			break;

		case WAIT_ON_HEADER_1:
			DecoderState = (c==ZO_PROTOCOL_HEADER_1)?WAIT_ON_ADDRESSED_NODE_ID:WAIT_ON_HEADER_0;
			break;

		case WAIT_ON_ADDRESSED_NODE_ID:
			if( ( (c & zoProtocolUartLAM) == (OwnNodeID & zoProtocolUartLAM ) ) ||
				( c == ZO_PROTOCOL_BROADCAST_ID ) )
			{
				DecoderState = WAIT_ON_OWN_NODE_ID;
				packet->AddressedNodeID = c;
			}
			else
			{
				DecoderState = WAIT_ON_HEADER_0;
				setReceivalOfPacketStarted(FALSE);
			}
			break;

		case WAIT_ON_OWN_NODE_ID:
			packet->OwnNodeID = c;
			DecoderState = WAIT_ON_COMMAND_ID;
			break;

		case WAIT_ON_COMMAND_ID:
			packet->commandID = c;
			DecoderState = WAIT_ON_BYTECOUNT;
			break;

		case WAIT_ON_BYTECOUNT:
			packet->byteCount = c;
			byteCount = packet->byteCount;	//store for internal use
			if(byteCount > 0)
				DecoderState = WAIT_ON_DATA;
			else
				DecoderState = WAIT_ON_LRC;
			break;

		case WAIT_ON_DATA:
			packet->data[packet->byteCount - byteCount--] = c;
			if(byteCount == 0)
				DecoderState =	WAIT_ON_LRC;
			break;

		case WAIT_ON_LRC:
			packet->lrc = c;
			DecoderState = WAIT_ON_HEADER_0; 
			IsWholePacket = TRUE;
			setReceivalOfPacketStarted(FALSE);
			break;
	}

	return IsWholePacket;
}

inline void zoProtocolUartSetLAM(u08 localAcceptanceMask)
{
	zoProtocolUartLAM = localAcceptanceMask;
}

inline bool zoProtocolUartSetBitrate(u32 bitsPerSecond)
{
	return zoUartSetBaud(bitsPerSecond);
}


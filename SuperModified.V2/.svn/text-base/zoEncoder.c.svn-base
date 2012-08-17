#include <avr/io.h>
#include "zoEncoder.h"
#include "zoMcu.h"
#include <stdlib.h>
#include "zoSms.h"
#include <avr/sfr_defs.h>
#include "zoSpi.h"
#include "zoString.h"
#include <util/delay.h>
#include "zoMotor.h"

#define VELOCITY_READ_FREQUENCY_APPROX		10
#define VELOCITY_READ_CONTROL_LOOP_ENTRIES	(ZO_SMS_CONTROL_LOOP_HZ_APPROX/ \
											VELOCITY_READ_FREQUENCY_APPROX)
#define MICROSECONDS_PER_SECOND				1000000UL
#define CLOCK_TICKS_PER_MICROSEC			(F_CPU/MICROSECONDS_PER_SECOND)

static volatile s16 DeltaPos=0;
static volatile s32 EncoderPos = 0;
static volatile s32 InitialAbsPos = 0;
static volatile s32 LastEncoderPos = 0;
static volatile u08 NoOfEntriesControlLoopIsr = 1;
static volatile u16 PositionAbsolute = 0;
static volatile u16 LastPositionAbsolute = 0;
static volatile s32 FullTurns = 0;

void zoEncoderInit(void)
{
	u16 pos;
	
	zoSpiInit();
	zoEncoderGetPosSpi();
	pos = zoEncoderGetPosSpi();

	enterCritical();
	InitialAbsPos = pos;
	PositionAbsolute = pos;
	LastPositionAbsolute = pos;
	DeltaPos = 0;
	EncoderPos = 0;
	LastEncoderPos = 0;
	NoOfEntriesControlLoopIsr = 1;
	FullTurns = 0;
	exitCritical();
}

s32 zoEncoderGetPos(void)
{
	s32 pos;

	enterCritical();
	LastPositionAbsolute = PositionAbsolute;
	PositionAbsolute = zoEncoderGetPosSpi();
	
	if(( (s32)PositionAbsolute - (s32)LastPositionAbsolute ) < -2000)
		FullTurns++;
	
	if(( (s32)PositionAbsolute - (s32)LastPositionAbsolute ) > 2000)
		FullTurns--;
	
	pos = (FullTurns << 12) + PositionAbsolute - InitialAbsPos;
	EncoderPos = pos;
	exitCritical();

	return pos;
}

s32 zoEncoderGetStoredPos(void)
{
	s32 pos;

	enterCritical();
	pos = EncoderPos;
	exitCritical();

	return pos;
}

u16 zoEncoderGetPosSpi(void)
{
	u08 dummy = 0;
	u08 data[2] = {0,0};
	u16 posSPI;

	PORTB &= ~_BV(PB2);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	__asm__ __volatile__ ("nop" ::);
	data[1] = zoSpiTransferByte(dummy);
	data[0] = zoSpiTransferByte(dummy);
	PORTB |= _BV(PB2);
	posSPI = strToU16(data)>>4;
		
	return posSPI;
}

u16 zoEncoderGetPosAbsolute(void)
{
	return zoEncoderGetPosSpi();
}

s16 zoEncoderGetVel(void)
{
	s16 vel;

	enterCritical();
	vel = DeltaPos;
	exitCritical();

	vel = (s16)( (double)(vel) *
				 Sms.Control.LoopHz / 
				 (double)VELOCITY_READ_CONTROL_LOOP_ENTRIES );

	return vel;
}

void zoEncoderReset(void)
{
	s32 pos;
	
	pos = zoEncoderGetPosSpi();

	enterCritical();
	InitialAbsPos = pos;
	PositionAbsolute = pos;
	LastPositionAbsolute = pos;
	DeltaPos = 0;
	EncoderPos = 0;
	LastEncoderPos = 0;
	NoOfEntriesControlLoopIsr = 1;
	FullTurns = 0;
	exitCritical();	
}

//this function needs to run inside the control loop, because accurate timing is needed
//in order to calculate velocity.

void zoEncoderSetVel(void)
{
	if(NoOfEntriesControlLoopIsr++ >= VELOCITY_READ_CONTROL_LOOP_ENTRIES)
	{
		NoOfEntriesControlLoopIsr = 1;
		DeltaPos = (s16)( (EncoderPos - LastEncoderPos) );
		LastEncoderPos = EncoderPos;
	}	
}

inline bool zoEncoderPollOverFlowError(void)
{
	bool ovfl = FALSE;
	
	enterCritical();
	if(	EncoderPos >= ((s32)0x7FFFFC00) )
		ovfl = TRUE;
	exitCritical();

	return ovfl;
}

inline bool zoEncoderPollUnderFlowError(void)
{
	bool unfl = FALSE;
	
	enterCritical();
	if(	EncoderPos <= ((s32)0x80000400) )
		unfl = TRUE;
	exitCritical();

	return unfl;
}

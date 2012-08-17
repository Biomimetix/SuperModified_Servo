//****************************************************************************************
//	File Name	: zoIcp.c 
//	Description	: Pulse width measurement utilizing the AVR Timer1 ICP module
//	Created		: 06/06/2010 
//	Target MCU	: ATMega328p
//	Author		: Sissakis Giannis
//  email		: info@01mech.com
//
//  Copyright (C) 2010 Zero One Mechatronics LP
//
//  This program is free software: you can redistribute it and/or modify
//  it under the terms of the GNU General Public License as published by
//  the Free Software Foundation, either version 3 of the License, or
//  (at your option) any later version.
//
//  This program is distributed in the hope that it will be useful,
//  but WITHOUT ANY WARRANTY; without even the implied warranty of
//  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
//  GNU General Public License for more details.
//
//  You should have received a copy of the GNU General Public License
//  along with this program.  If not, see <http://www.gnu.org/licenses/>.
//
//****************************************************************************************

#include "zoIcp.h"
#include "zoMcu.h"
#include <avr/io.h>
#include <avr/interrupt.h>

static volatile u08 IcpMode;
static volatile u16 IcpTimerTicks = 0;
static volatile u16 PulseOnTicks = 0;
static volatile u16 PulseOffTicks = 0;
static volatile u08 HighLow = 0 ;
static void(*IcpUserFunction)(u16 timerTicks) = 0;
static void (*OvflUserFunction)(void) = 0;
static void (*IcpFailedUserFunction)(void) = 0;

void zoIcpInit(void)
{
	//configure defaults
	zoIcpSetMode(ZO_ICP_MODE_DEFAULT);				//configure default mode
	zoIcpSetPrescaler(ZO_ICP_PRESCALE_DEFAULT);		//configure default prescaler
	
	//initialize timer1 used for input capture
	TCCR1A = 0x00;									//all waveform generation disabled
	TCCR1B |= _BV(ICNC1);							//enable input noise canceller
	
	zoIcpIsrEnable();	
	sei();											//enable global interrupts
}

void zoIcpSetPrescaler(ZO_ICP_PRESCALE prescale)
{
	enterCritical();
	TCCR1B = (TCCR1B & 0xF8) | prescale;
	exitCritical();
}

void zoIcpSetMode(ZO_ICP_MODE mode)
{
	enterCritical();

	IcpMode = mode;

	DDRB &= ~_BV(PB0);								//configure PB0 as input
	PORTB |= _BV(PB0);								//enable pull up

	if( mode == ZO_ICP_MODE_LOW_HIGH_LOW )
		TCCR1B |= _BV(6) ;		//configure edge select as per mode
	else
		TCCR1B &= ~_BV(6);

	exitCritical();
}

void zoIcpAttachUserFunctionOnCapture( void(*userFunc)(u16 timerTicks) )
{
	enterCritical();
	IcpUserFunction = userFunc;
	exitCritical();
}

void zoIcpAttachUserFunctionOnOverFlow(void(*userFunc)(void))
{
	enterCritical();
	OvflUserFunction = userFunc;
	exitCritical();
}

void zoIcpAttachUserFunctionOnCaptureFailure(void(*userFunc)(void))
{
	enterCritical();
	IcpFailedUserFunction = userFunc;
	exitCritical();
}

void zoIcpIsrEnable(void)
{
	enterCritical();
	TCNT1 = 0x0000;
	ICR1 = 0x0000;
	exitCritical();

	TIFR1 |= _BV(ICF1);								//clear ICF1 interrupt flag manually
	TIMSK1 |= _BV(ICIE1);							//enable input capture interrupt
}

void zoIcpIsrDisable(void)
{
	TIMSK1 &= ~_BV(5);								//disable input capture interrupt
	TIFR1 |= _BV(5);								//clear ICF1 interrupt flag manually

	enterCritical();
	TCNT1 = 0x0000;
	ICR1 = 0x0000;
	exitCritical();
}

inline void zoIcpOverFlowIsrEnable(void)
{
	TIMSK1 |= _BV(TOIE1); 
}

inline void zoIcpOverFlowIsrDisable(void)
{
	TIMSK1 &= ~_BV(TOIE1);
}

bool zoIcpGetPulseWidh(u16 *icpTimerTicks)
{
	enterCritical();
	*icpTimerTicks = IcpTimerTicks;
	exitCritical();

	if( *icpTimerTicks != 0 )
		return TRUE;
	else
		return FALSE;
}

ISR(TIMER1_CAPT_vect)
{
	u16 icr;

	icr = ICR1;

	if( (PINB & 0x01) == IcpMode  )
	{
		PulseOnTicks = icr;

		if ( IcpMode == ZO_ICP_MODE_LOW_HIGH_LOW )
			TCCR1B &= ~_BV(6);
		else
			TCCR1B |= _BV(6);

		HighLow++;
	}
	else
	{
		PulseOffTicks = icr;
		IcpTimerTicks = PulseOffTicks - PulseOnTicks;

		HighLow--;

		if ( IcpMode == ZO_ICP_MODE_LOW_HIGH_LOW )
			TCCR1B |= _BV(6);
		else
			TCCR1B &= ~_BV(6);
		
		TCNT1 = 0x0000;

		if( ( HighLow != 0 ) || ( PulseOnTicks > PulseOffTicks ) )							
		{
			HighLow = 0;							//set up for next time
			IcpTimerTicks = 0;						//indicate it by zero pulse width
			if(IcpFailedUserFunction)
				IcpFailedUserFunction();
		}
		else
		{
			if(IcpUserFunction)
				IcpUserFunction(IcpTimerTicks);
		}
	}
}


ISR(TIMER1_OVF_vect)
{
	if(OvflUserFunction)
		OvflUserFunction();
}
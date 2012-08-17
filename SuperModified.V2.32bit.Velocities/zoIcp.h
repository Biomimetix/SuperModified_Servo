#ifndef ZO_ICP_H
#define ZO_ICP_H

//****************************************************************************************
//	File Name	: zoIcp.h 
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

#include "zoTypes.h"

//configuration enumerations
typedef enum {
	ZO_ICP_MODE_HIGH_LOW_HIGH = 0,
	ZO_ICP_MODE_LOW_HIGH_LOW = 1
}ZO_ICP_MODE;

typedef enum {
	ZO_ICP_PRESCALE_1			= 0x01,		//CPU Clock - not prescaled
	ZO_ICP_PRESCALE_8			= 0x02,		//CPU Clock prescaled by 8
	ZO_ICP_PRESCALE_64			= 0x03,		//CPU Clock prescaled by 64
	ZO_ICP_PRESCALE_256			= 0x04,		//CPU Clock prescaled by 256
	ZO_ICP_PRESCALE_1024		= 0x05,		//CPU Clock prescaled by 1024
	ZO_ICP_PRESCALE_T1_FALLING	= 0x06,		//Ext. clock on T1, clock on falling edge
	ZO_ICP_PRESCALE_T1_RISING	= 0x07		//Ext. clock on T1, clock on rising edge
}ZO_ICP_PRESCALE;

//defaults
#define ZO_ICP_PRESCALE_DEFAULT		ZO_ICP_PRESCALE_1
#define ZO_ICP_MODE_DEFAULT			ZO_ICP_MODE_LOW_HIGH_LOW

void zoIcpInit(void);
bool zoIcpGetPulseWidh(u16 *icpTimerTicks);
void zoIcpSetPrescaler(ZO_ICP_PRESCALE prescale);
void zoIcpSetMode(ZO_ICP_MODE mode);
void zoIcpAttachUserFunctionOnCapture( void(*userFunc)(u16 timerTicks) );
void zoIcpAttachUserFunctionOnCaptureFailure(void(*userFunc)(void));
void zoIcpAttachUserFunctionOnOverFlow(void(*userFunc)(void));

void zoIcpIsrEnable(void);
void zoIcpIsrDisable(void);
void zoIcpOverFlowIsrEnable(void);
void zoIcpOverFlowIsrDisable(void);

#endif //ZO_ICP_H
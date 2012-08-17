//****************************************************************************************
//	File Name	: zoProtocolRcServo.c 
//	Description	: SuperModified rc servo compatibilty mode protocol. Implented utilizing 
//				  the AVR Input Capture Mode 	
//	Created		: 08/06/2010 
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

#include <avr/io.h>
#include <avr/sfr_defs.h>
#include "zoProtocolRcServo.h"
#include "zoIcp.h"
#include "zoMcu.h"
#include "zoCommands.h"
#include "zoMotor.h"
#include "zoEncoder.h"
#include "zoString.h"
#include "zoUart.h"
#include <stdlib.h>
#include "zoSms.h"

#define ENCODER_RESOLUTION					4096.0
#define CPU_CLOCK_TICKS_PER_SERVO_RANGE		( 40000.0 / 8.0 )
#define CLOCK_TICKS_TO_ENCODER_TICKS_FACTOR ( ENCODER_RESOLUTION / \
											 CPU_CLOCK_TICKS_PER_SERVO_RANGE )
#define CPU_CLOCK_TICKS_AT_ZERO_DEGREES		( 30000.0 / 8.0 )

static volatile u16 InitialPos = 0;

static volatile s16 PositionCommand = 0;
static volatile s16 PrevPositionCommand = 0;
static volatile s16 PrevPrevPositionCommand = 0;

static volatile u08 ValidCommandCount = 0;
static volatile bool IsRcServoStarted = FALSE;
static volatile bool IsNewRcServoSetPoint = FALSE;

static volatile bool goingup = TRUE;

void icpFailedInterruptHandler(void)
{
	IsRcServoStarted = FALSE;
	ValidCommandCount = 0;
	//Sms.Control.State = ZO_SMS_CONTROL_NO_CONTROL;
	//zoMotorSetState(FALSE);
}

void icpTimerOverFlowInterruptHandler(void)
{
	IsRcServoStarted = FALSE;
	ValidCommandCount = 0;
	//Sms.Control.State = ZO_SMS_CONTROL_NO_CONTROL;
	//zoMotorSetState(FALSE);
}

void icpInterruptHandler(u16 pulseWidth)
{
	if( IsRcServoStarted )
		IsNewRcServoSetPoint = TRUE;
	else
	{
		if( ValidCommandCount++ >= 10 )
		{
			Sms.BufferdSetPoint.type = ZO_SMS_SETPOINT_NONE;
			zoEncoderReset();
			Sms.Pid.PrevError = 0;
			Sms.Pid.Integral = 0;
			Sms.Pid.SetPoint = zoEncoderGetPos();
			zoMotorSetState(TRUE);
			InitialPos = zoEncoderGetPosAbsolute();
			Sms.Control.State = ZO_SMS_CONTROL_POSITION_CONTROL;
			IsRcServoStarted = TRUE;
			ValidCommandCount = 0;
		}
	}
}

void zoProtocolRcServoInit(void)
{
	zoIcpInit();
	zoIcpSetPrescaler(ZO_ICP_PRESCALE_8);

	enterCritical();
	InitialPos = 0;
	PositionCommand = 0;
	ValidCommandCount = 0;
	IsRcServoStarted = FALSE;
	IsNewRcServoSetPoint = FALSE;
	exitCritical();

	zoIcpAttachUserFunctionOnCapture(icpInterruptHandler);
	zoIcpAttachUserFunctionOnCaptureFailure(icpFailedInterruptHandler);
	zoIcpAttachUserFunctionOnOverFlow(icpTimerOverFlowInterruptHandler);
}

void zoProtocolRcServoParse(void)
{
	s16 commandPos;
	u16 pulseWidth;
	
	if( IsRcServoStarted && IsNewRcServoSetPoint )
	{
		IsNewRcServoSetPoint = FALSE;
		if(!zoIcpGetPulseWidh(&pulseWidth))
			return;
	
		if( ( pulseWidth > 1200 ) && ( pulseWidth < 6300 ) )
		{
			//convert pulse to ticks
			commandPos = (s16)( CLOCK_TICKS_TO_ENCODER_TICKS_FACTOR * 
				( (double)pulseWidth ) );

			//absolute positioning around zero
			if(InitialPos <= 2048)
				commandPos = commandPos - InitialPos - 3072;
			else
				commandPos = commandPos - InitialPos + 1024;

			//proceed values in time
			PrevPrevPositionCommand = PrevPositionCommand;
			PrevPositionCommand = PositionCommand;
			PositionCommand = (PositionCommand + commandPos)>>1;	//weighted running average

			//anti-spike filter
			if( ( abs(PositionCommand - PrevPositionCommand) > abs(PositionCommand - PrevPrevPositionCommand) ) 
				|| ( abs(PrevPrevPositionCommand - PrevPositionCommand) > abs(PositionCommand - PrevPrevPositionCommand) )  )
				PrevPositionCommand = (PositionCommand + PrevPrevPositionCommand)/2;
			
			//give the setpoint
			Sms.Pid.SetPoint = (s32)PrevPositionCommand;
		}
	}
}



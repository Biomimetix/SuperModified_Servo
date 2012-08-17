#include "zoSpi.h"

#include <avr/io.h>
#include "zoMcu.h"
#include <avr/interrupt.h>

void zoSpiInit(void)
{
	//Configure SPI pins
	DDRB |= _BV(2);		// SS must be output for Master mode to work
	DDRB |= _BV(3);		// set MOSI as output	
	DDRB &= ~_BV(4);	// set MISO as input
	DDRB |= _BV(5);		// set SCK as output
	PORTB |= _BV(5);	// set SCK high
	PORTB |= _BV(2);	//set SS high

	//Configure SPI functionality
	SPCR = 0x00;
	SPCR |= _BV(MSTR);	//configure as master
	SPCR |= _BV(SPR1);	//SPI clock = F_CPU/32
	SPCR |= _BV(CPHA);
	SPSR |= _BV(SPI2X);
	SPCR |= _BV(SPE);	//enable SPI
}

u08 zoSpiTransferByte(u08 byte)
{
	SPDR = byte;
	while(!(SPSR & _BV(SPIF)));
	return SPDR;
}
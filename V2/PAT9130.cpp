/*
 * PAT9130.cpp
 *
 * Created: 10/01/2019 11:21:48
 *  Author: David
 */ 

#include "PAT9130.h"
#include "Pins.h"
#include <avr/pgmspace.h>

const uint8_t LaserPowerSetting = 6;	// as recommended on PAT9130 datasheet
const uint8_t YResolution = 51;			// 51 * 50 = 2550 count/inch ~= 100 counts/mm

struct InitTableEntry
{
	uint8_t regnum;
	uint8_t value;
};

// PAT9130EW sensor recommended settings:
const PROGMEM InitTableEntry initTable[] =
{
	{ 0x7F, 0x00 },					// switch to bank0
	{ 0x05, 0xA8 },					// sleep mode disabled
	{ 0x09, 0x5A },					// disable write protect
	{ 0x51, LaserPowerSetting },	// set LD current source to 6
	{ 0x0D, 1 },					// CPI resolution setting for X-direction (we don't use X)
	{ 0x0E, YResolution },			// CPI resolution setting for Y-direction. 51 * 50 = 2550 count/inch ~= 100 counts/mm
	{ 0x07, 0x00 },
	{ 0x1B, 0x42 },
	{ 0x2E, 0x40 },
	{ 0x32, 0x40 },
	{ 0x33, 0x02 },
	{ 0x34, 0x00 },
	{ 0x36, 0xE0 },
	{ 0x38, 0xA0 },
	{ 0x39, 0x01 },
	{ 0x3E, 0x14 },
	{ 0x44, 0x02 },
	{ 0x4A, 0xE0 },
	{ 0x4F, 0x02 },
	{ 0x52, 0x0D },					// turn off internal VDDA
	{ 0x57, 0x03 },
	{ 0x59, 0x03 },
	{ 0x5B, 0x03 },
	{ 0x5C, 0xFF },

	{ 0x7F, 0x01 },					// switch to bank1
	{ 0x00, 0x25 },
	{ 0x07, 0x78 },
	{ 0x20, 0x00 },
	{ 0x21, 0x40 },
	{ 0x23, 0x00 },
	{ 0x2F, 0x64 },
	{ 0x37, 0x30 },
	{ 0x3B, 0x64 },
	{ 0x43, 0x0A },
	{ 0x59, 0x01 },
	{ 0x5A, 0x01 },
	{ 0x5C, 0x04 },
	{ 0x5E, 0x04 },

	{ 0x7F, 0x06 },					// switch to bank6
	{ 0x34, 0x03 },	

	{ 0x7F, 0x07 },					// switch to bank7
	{ 0x00, 0x01 },
	{ 0x02, 0xC4 },
	{ 0x03, 0x13 },
	{ 0x06, 0x0C },
	{ 0x0F, 0x0A },
	{ 0x14, 0x02 },
	{ 0x35, 0x39 },
	{ 0x36, 0x3F },
	{ 0x46, 0x03 },
	{ 0x47, 0x0F },
	{ 0x4B, 0x97 },
	
	{ 0x7F, 0x00 },					// switch to bank0
	{ 0x09, 0x00 },					// enable write protect
};

#define ARRAY_SIZE(_x)	(sizeof(_x)/sizeof(_x[0]))

// Initialize the device returning true if successful
bool PAT9130Init()
{
	// Set up the USI
	USICR = BITVAL(USIWM0);
	PORTA |= BITVAL(PortANCSBitNum) | BITVAL(PortASCLBitNum);
	DDRA |= BITVAL(PortANCSBitNum) | BITVAL(PortASCLBitNum);

	// Set up the PAT9130
	if (ReadRegister(PAT9130Register::productID1) == 0x31 && ReadRegister(PAT9130Register::productID2) == 0x61)		// check the product ID registers
	{
		// Initialise the PAT9130 registers from the table
		for (size_t i = 0; i < ARRAY_SIZE(initTable); ++i)
		{
			const uint8_t regnum = pgm_read_byte_near(&initTable[i].regnum);
			const uint8_t value = pgm_read_byte_near(&initTable[i].value);
			if (regnum == 0x7F)
			{
				WriteRegister((PAT9130Register)regnum, value);		// register 0x7F is write only
			}
			else if (!WriteAndCheckRegister((PAT9130Register)regnum, value))
			{
				return false;
			}
		}

		// Do a final check of the laser power setting before we turn the laser on
		if (ReadRegister(PAT9130Register::ldSrc) == LaserPowerSetting)
		{
			PORTA |= BITVAL(PortALDPBitNum);						// successful init, so turn the laser on
			return true;
		}
	}

	return false;
}

// Turn the laser off
void PAT9130LaserOff()
{
	PORTA &= ~BITVAL(PortALDPBitNum);
}

// If any motion has occurred since the last read, update dx and dy and return true
bool PAT9130ReadMotion(int16_t& dx, int16_t& dy)
{
	if (ReadRegister(PAT9130Register::motionStatus) & 0x80)
	{
		const uint8_t xlow = ReadRegister(PAT9130Register::deltaXlow);
		const uint8_t ylow = ReadRegister(PAT9130Register::deltaYlow);
		const uint8_t xhigh = ReadRegister(PAT9130Register::deltaXhigh);
		const uint8_t yhigh = ReadRegister(PAT9130Register::deltaYhigh);
		dx = (int16_t)(((uint16_t)xhigh << 8) | xlow);
		dy = (int16_t)(((uint16_t)yhigh << 8) | ylow);
		return true;
	}

	return false;
}

// Return the shutter register (8 bits)
uint8_t PAT9130ReadShutter()
{
	return ReadRegister(PAT9130Register::shutter);
}

// Return the brightness register (8 bits)
uint8_t PAT9130ReadImageQuality()
{
	return ReadRegister(PAT9130Register::iq);
}

// Return the brightness register (8 bits)
uint8_t PAT9130ReadBrightness()
{
	return ReadRegister(PAT9130Register::frameAvg);
}

// Delay for 250ns. This may delay for a little less if the processor clock is a little faster than 8MHz.
inline void Delay250ns()
{
	asm volatile ("nop; nop");
}

// Delay for at least 1us
void Delay1us()
{
	asm volatile ("nop; nop; nop; nop; nop");
	asm volatile ("nop; nop; nop; nop;");
}

// Write a register to the PAT9130
void WriteRegister(PAT9130Register regNum, uint8_t data)
{
	DDRA |= BITVAL(PortADOBitNum);				// make DO an output

	// Write the register address to the data out register. Top bit must be set to indicate a write access.
	USIDR = (uint8_t)regNum | 0x80;

	// Ensure SCLK is high
	if ((PINA & BITVAL(PortASCLBitNum)) == 0)
	{
		USICR |= BITVAL(USITC);
		Delay250ns();							// min. 250ns low time
	}

	PORTA &= ~BITVAL(PortANCSBitNum);			// bring NCS low
	Delay1us();

	// Send the address
	USICR |= BITVAL(USITC);
	Delay250ns();								// min. 250ns low time
	USICR |= BITVAL(USITC);
	Delay250ns();								// min. 250ns high time
	for (uint8_t i = 0; i < 7; ++i)
	{
		USICR |= BITVAL(USITC) | BITVAL(USICLK);
		Delay250ns();							// min. 250ns low time
		USICR |= BITVAL(USITC);
		Delay250ns();							// min. 250ns high time
	}

	// Send the data
	USIDR = data;
	USICR |= BITVAL(USITC);
	Delay250ns();								// min. 250ns low time
	USICR |= BITVAL(USITC);
	Delay250ns();								// min. 250ns high time
	for (uint8_t i = 0; i < 7; ++i)
	{
		USICR |= BITVAL(USITC) | BITVAL(USICLK);
		Delay250ns();							// min. 250ns low time
		USICR |= BITVAL(USITC);
		Delay250ns();							// min. 250ns high time
	}

	Delay1us();
	PORTA |= BITVAL(PortANCSBitNum);			// bring NCS high
	Delay1us();
	Delay1us();
}

// Read a register from the PAT9130
uint8_t ReadRegister(PAT9130Register regNum)
{
	DDRA |= BITVAL(PortADOBitNum);				// make DO an output

	// Write the register address to the data out register. Top bit must be clear to indicate a read access.
	USIDR = (uint8_t)regNum;

	// Ensure SCLK is high
	if ((PINA & BITVAL(PortASCLBitNum)) == 0)
	{
		USICR |= BITVAL(USITC);
		Delay250ns();							// min. 250ns low time
	}

	PORTA &= ~BITVAL(PortANCSBitNum);			// bring NCS low
	Delay1us();

	// Send the address
	USICR |= BITVAL(USITC);
	Delay250ns();								// min. 250ns low time
	USICR |= BITVAL(USITC);
	Delay250ns();								// min. 250ns high time
	for (uint8_t i = 0; i < 7; ++i)
	{
		USICR |= BITVAL(USITC) | BITVAL(USICLK);
		Delay250ns();							// min. 250ns low time
		USICR |= BITVAL(USITC);
		Delay250ns();							// min. 250ns high time
	}

	Delay250ns();								// make DO an input to release the bus
	DDRA &= ~BITVAL(PortADOBitNum);

	// Read the data
	for (uint8_t i = 0; i < 8; ++i)
	{
		USICR |= BITVAL(USITC);
		Delay250ns();							// min. 250ns low time
		USICR |= BITVAL(USITC) | BITVAL(USICLK);
		Delay250ns();							// min. 250ns high time
	}

	Delay1us();
	PORTA |= BITVAL(PortANCSBitNum);			// bring NCS high
	Delay1us();
	Delay1us();

	return USIDR;
}

// Write a register and read it back to check that it was written correctly. Return true if success.
bool WriteAndCheckRegister(PAT9130Register regNum, uint8_t data)
{
	for (uint8_t i = 0; i < 3; ++i)
	{
		WriteRegister(regNum, data);
		if (ReadRegister(regNum) == data)
		{
			return true;
		}
	}
	return false;
}

// End

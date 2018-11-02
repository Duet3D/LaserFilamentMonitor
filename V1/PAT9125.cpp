/*
 * CPPFile1.cpp
 *
 * Created: 03/01/2018 19:34:06
 *  Author: David
 */ 

#include "ecv.h"
#include <stdint.h>
#include "USI_TWI_Master.h"

// PAT9125 address is 0x73 with IDSEL connected to Vcc, 0x75 with IDSEL connected to ground, 0x79 with IDSEL not connected
const uint8_t PATaddress = 0x75;

static uint8_t messageBuf[4];

static bool OTS_RegWrite(uint8_t reg, uint8_t data)
{
	messageBuf[0] = (PATaddress << TWI_ADR_BITS) | (0u << TWI_READ_BIT);
	messageBuf[1] = reg;
	messageBuf[2] = data;
	const bool ok = USI_TWI_Start_Read_Write(messageBuf, 3);
	USI_TWI_Master_Stop();
	return ok;
}

static uint8_t OTS_RegRead(uint8_t reg)
{
	messageBuf[0] = (PATaddress << TWI_ADR_BITS) | (0u << TWI_READ_BIT);
	messageBuf[1] = reg;
	bool ok = USI_TWI_Start_Read_Write(messageBuf, 2);							// set register address
	if (ok)
	{
		messageBuf[0] = (PATaddress << TWI_ADR_BITS) | (1u << TWI_READ_BIT);	// construct the message
		ok = USI_TWI_Start_Read_Write(messageBuf, 2);							// read the register
		USI_TWI_Master_Stop();
		if (ok)
		{
			return messageBuf[1];
		}
	}
	return 0;
}

//=================================================================================================
// Sensor register write then read Command.
// A write command followed by a read command is to ensure the recommended settings are written correctly.
//=================================================================================================
static void OTS_RegWriteRead(uint8_t address, uint8_t wdata)
{
	uint8_t rdata;
	do
	{
		OTS_RegWrite(address, wdata);	// Write data to specified address
		rdata = OTS_RegRead(address);	// Read back previous written data
	} while(rdata != wdata);			// Check if the data is correctly written
}

bool OTS_Sensor_Init()
{
	USI_TWI_Master_Initialise();

	// Read sensor_pid in address 0x00 to check if the serial link is valid, read value should be 0x31.
	const bool ok = OTS_RegRead(0x00) == 0x31;
	if (ok)
	{
		// PAT9125 sensor recommended settings
		OTS_RegWrite(0x7F, 0x00);		// switch to bank0, not allowed to perform OTS_RegWriteRead
		OTS_RegWrite(0x06, 0x97);		// software reset (i.e. set bit7 to 1). OTS_RegWriteRead is not allowed because this bit will clear to 0 automatically.
		_delay_us(2000);				// delay 2ms
		OTS_RegWrite(0x06, 0x17);		// ensure the sensor has left the reset state.
		OTS_RegWriteRead(0x09, 0x5A);	// disable write protect
		OTS_RegWriteRead(0x0D, 1);		// set X-axis resolution (we are not interested in it)
		OTS_RegWriteRead(0x0E, 254);	// set Y-axis resolution (254 * 5 cpi = 50 counts/mm)
		OTS_RegWriteRead(0x19, 0x04);	// set 12-bit X/Y data format
		if (OTS_RegRead(0x5E) == 0x04)
		{
			OTS_RegWriteRead(0x5E, 0x08);
			if (OTS_RegRead(0x5D) == 0x10)
			{
				OTS_RegWriteRead(0x5D, 0x19);
			}
		}
		OTS_RegWriteRead(0x09, 0x00);	// enable write protect
	}
	return ok;
}

//=================================================================================================
// 1. Read the Motion bit (bit7 in address 0x02) to check if the motion data of X/Y are available to read.
// 2. If Motion bit=1, read X/Y motion data in address 0x03, 0x04 and 0x12.
// 3. The 12-bit X/Y motion data are in 2’s compliment format and range from -2048 to +2047
//=================================================================================================
bool OTS_Sensor_ReadMotion(int16_t& dx, int16_t& dy)
{
	if (OTS_RegRead(0x02) & 0x80)		// check motion bit in bit7
	{
		uint16_t deltaX = OTS_RegRead(0x03);
		uint16_t deltaY = OTS_RegRead(0x04);
		uint16_t deltaXY_h = OTS_RegRead(0x12);

		deltaX |= ((deltaXY_h << 4) & 0x0F00);
		if (deltaX & 0x0800)
		{
			deltaX |= 0xf000;
		}

		deltaY |= ((deltaXY_h << 8) & 0x0F00);
		if (deltaY & 0x800)
		{
			deltaY |= 0xf000;
		}
		dx = (int16_t)deltaX;
		dy = (int16_t)deltaY;
		return true;
	}

	return false;						// no new motion to report
}

// Return the shutter register (5 bits)
uint8_t OTS_Sensor_Read_shutter()
{
	return OTS_RegRead(0x14) & 0x3F;
}

// Return the brightness register (8 bits)
uint8_t OTS_Sensor_Read_brightness()
{
	return OTS_RegRead(0x17);
}

// End

/*
 * PAT9130.h
 *
 * Created: 10/01/2019 11:12:38
 *  Author: David
 */ 


#ifndef PAT9130_H_
#define PAT9130_H_

#include <avr/io.h>

// PAT9130 registers numbers

enum class PAT9130Register : uint8_t
{
	productID1 = 0x00,
	productID2 = 0x01,
	motionStatus = 0x02,
	deltaXlow = 0x03,
	deltaYlow = 0x04,
	configuration = 0x06,
	writeProtect = 0x09,
	resX = 0x0D,
	resY = 0x0E,
	deltaXhigh = 0x11,
	deltaYhigh = 0x12,
	iq = 0x13,
	shutter = 0x15,
	frameAvg = 0x17,
	mfioConfig = 0x26,
	ldSrc = 0x51
};

// Functions to control the PAT9130
bool PAT9130Init();									// Initialize the device returning true if successful
bool PAT9130ReadMotion(int16_t& dx, int16_t& dy);	// If any motion has occurred since the last read, update dx and dy and return true
uint8_t PAT9130ReadShutter();						// Return the shutter register (8 bits)
uint8_t PAT9130ReadBrightness();					// Return the brightness register (8 bits)
uint8_t PAT9130ReadImageQuality();					// Return the image quality register (8 bits)
void PAT9130LaserOff();								// Turn the laser off

// Functions to read or write a PAT9130 register on an ATTINY
void WriteRegister(PAT9130Register regNum, uint8_t value);
uint8_t ReadRegister(PAT9130Register regNum);
bool WriteAndCheckRegister(PAT9130Register regNum, uint8_t data);

#endif /* PAT9130_H_ */
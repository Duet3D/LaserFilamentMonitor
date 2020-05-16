/*
 * Pins.h
 *
 * Created: 10/01/2019 12:27:46
 *  Author: David
 */ 


#ifndef PINS_H_
#define PINS_H_

const unsigned int PortANCSBitNum = 0;						// NCS signal to PAT9130
const unsigned int PortALDPBitNum = 2;						// pin that drives LDP on the PAT9130
const unsigned int PortASwitchBitNum = 3;					// optional filament present switch, has an external pullup resistor
const unsigned int PortASCLBitNum = 4;
const unsigned int PortADOBitNum = 5;
const unsigned int PortADIBitNum = 6;
const unsigned int PortAOutBitNum = 7;

const unsigned int PortBMFIOInputBitNum = 0;				// MFIO pin
const unsigned int PortBLedGreenBitNum = 1;					// green LED
const unsigned int PortBLedRedBitNum = 2;					// red LED

const unsigned int AdcVddaInputChannel = 1;					// PA1/ADC1 monitors VDDA to the PAT9130

#define BITVAL(_x) static_cast<uint8_t>(1u << (_x))

const uint8_t LedRed = BITVAL(PortBLedRedBitNum);
const uint8_t LedGreen = BITVAL(PortBLedGreenBitNum);
#define PORT_LED	PORTB

#endif /* PINS_H_ */

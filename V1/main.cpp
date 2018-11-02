/*
 * FilamentSensor.cpp
 *
 * Created: 20/04/2017 21:41:39
 * Authors: tony@think3dprint3d.com and dcrocker@eschertech.com
 
 Library and example code from jkl http://www.cs.cmu.edu/~dst/ARTSI/Create/PC%20Comm/
 and from the arduino version more focused on the ATTiny85 http://playground.arduino.cc/Code/USIi2c
 
 2017-08 12 Changed bit rate to 2000bps

 */ 

#include "ecv.h"

#ifdef __ECV__
#define __attribute__(_x)
#define __volatile__
#pragma ECV noverifyincludefiles
#endif

#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#include <avr/fuse.h>

#ifdef __ECV__
#pragma ECV verifyincludefiles
#undef cli
#undef sei
extern void cli();
extern void sei();
#endif

#include "PAT9125.h"
#include "USI_TWI_Master.h"									// for _delay_us

#define DOUBLE_SPEED	(0)									// set nonzero for 2000bps, zero for 1000bps

#define BITVAL(_x) static_cast<uint8_t>(1u << (_x))

const uint32_t F_CPU = 8000000UL;

const unsigned int PortOutBitNum = 7;
const unsigned int PortSwitchBitNum = 2;					// optional filament present switch, has an external pullup resistor

const unsigned int PortLedRedBitNum = 0;					// red LED
const unsigned int PortLedGreenBitNum = 1;					// green LED
const unsigned int PortMotionInputBitNum = 2;

const unsigned int Pad0BitNum = 0;							// pad 0 on underside
const unsigned int Pad1BitNum = 1;							// pad 1 on underside

const uint8_t LedRed = BITVAL(PortLedRedBitNum);
const uint8_t LedGreen = BITVAL(PortLedGreenBitNum);

const uint8_t PortADdrBits = BITVAL(PortOutBitNum);
const uint8_t PortBDdrBits = LedRed | LedGreen;

const uint8_t PortAPullupBits = BITVAL(Pad0BitNum) | BITVAL(Pad1BitNum) | BITVAL(PortSwitchBitNum) | BITVAL(3);		// unused pins, enable pullups on them
const uint8_t PortBPullupBits = BITVAL(PortMotionInputBitNum);		// input pins that need pullups enabled

#define PORT_LED	PORTB
#define PORT_OUT	PORTA
#define PORT_PADS	PORTA
#define PIN_SWITCH	PINA
#define PIN_PADS	PINA
#define DDR_PADS	DDRA

#if DOUBLE_SPEED
const uint16_t TicksPerSecond = 2000;						// this must be the same as the desired bit rate
#else
const uint16_t TicksPerSecond = 1000;						// this must be the same as the desired bit rate
#endif

// Error codes (expressed as number of blinks
const uint8_t FLASHES_OK = 3;
const uint8_t FLASHES_ERR_INIT = 5;

#if DOUBLE_SPEED
const uint16_t MinOutputIntervalTicks = TicksPerSecond/50;	// send the angle 50 times per second while it is changing
const uint16_t MinPulseIntervalTicks = TicksPerSecond/400;	// max 200 output transitions/sec in pulse mode (100mm/sec)
#else
const uint16_t MinOutputIntervalTicks = TicksPerSecond/10;	// send the angle 10 times per second while it is changing
const uint16_t MinPulseIntervalTicks = TicksPerSecond/200;	// max 200 output transitions/sec in pulse mode (100mm/sec)
#endif

const uint16_t MaxOutputIntervalTicks = TicksPerSecond/2;	// send the angle at least every half second

const uint16_t kickFrequency = 10;							// how often we kick the watchdog
const uint16_t kickIntervalTicks = TicksPerSecond/kickFrequency;

const uint16_t ParityBit = 0x8000u;							// adjusted so that there is an even number of bits
const uint16_t QualityBit = 0x4000u;						// set in words containing status information, clear in words containing position information
const uint16_t ErrorBit = 0x2000u;							// set if the sensor failed to initialise or self-test
const uint16_t SwitchOpenBit = 0x1000u;						// set if the optional switch is open, or the switch not connected

const uint16_t ErrorBlinkTicks = TicksPerSecond/4;			// fast error blinks
const uint16_t OkBlinkTicks = TicksPerSecond/4;				// fast OK blinks

#ifndef __ECV__
FUSES = {0xE2u, 0xDFu, 0xFFu};		// 8MHz RC clock
#endif

uint16_t lastOutval = 0;
volatile uint16_t tickCounter = 0;
uint16_t lastKickTicks = 0;
uint16_t lastPollTicks = 0;
uint16_t lastOutTicks = 0;
bool usePulseMode = false;

// Forward declarations
void blink(uint8_t count, uint8_t leds, uint16_t delayOn, uint16_t delayOff);
void SendWord(uint16_t data, uint8_t ledMask);

// Get a 16-bit volatile value from outside the ISR. As it's more than 8 bits long, we need to disable interrupts while fetching it.
inline uint16_t GetVolatileWord(volatile uint16_t& val)
writes(volatile)
{
	cli();
	const uint16_t locVal = val;
	sei();
	return locVal;
}

// Check whether we need to kick the watchdog
void CheckWatchdog()
writes(lastKickTicks; volatile)
{
	if (GetVolatileWord(tickCounter) - lastKickTicks >= kickIntervalTicks)
	{
#ifndef __ECV__
		wdt_reset();											// kick the watchdog
#endif
		lastKickTicks += kickIntervalTicks;
	}
}

// Delay for a specified number of ticks, kicking the watchdog as needed
void DelayTicks(uint16_t ticks)
writes(lastKickTicks; volatile)
{
	const uint16_t startTicks = GetVolatileWord(tickCounter);
	for (;;)
	{
		CheckWatchdog();
		if (GetVolatileWord(tickCounter) - startTicks >= ticks)
		{
			break;
		}
	}
}

// Report an error. The output must have been in the low state for at least 10 ticks before calling this.
void ReportError(uint8_t errorNum)
{
	SendWord(ErrorBit, 0);										// send error to the Duet
	blink(errorNum, LedRed, ErrorBlinkTicks, ErrorBlinkTicks);	// short error blinks, also leaves the output in the low state for ErrorBlinkTicks
}

int main(void)
{  
	// Set up the I/O ports
	PORTA = PortAPullupBits;
	PORTB = PortBPullupBits;
	DDRA = PortADdrBits;
	DDRB = PortBDdrBits;

	// Setup the timer to generate the tick interrupt
#if DOUBLE_SPEED
	// For a tick rate of 2000Hz we need a total divisor of 4000, for example 250 * 16 or 125 * 32.
	// Timer/counter 0 only offers prescalers of 1, 8, 64, 256 and 1024. But we can get 16 by using dual slope mode and prescaler 8.
	TCCR0A = BITVAL(WGM00);										// phase correct PWM mode, count up to OCR0A then count down
	TCCR0B = BITVAL(WGM02) | BITVAL(CS01);						// prescaler 8
	OCR0A = F_CPU/(16 * TicksPerSecond) - 1;					// set the period to the bit time
#else
	// For a tick rate of 1000 we need a total divisor of 8000, for example 125 * 64
	TCCR0A = BITVAL(WGM01);										// CTC mode, count up to OCR0A then start again from zero
	TCCR0B = BITVAL(CS01) | BITVAL(CS00);						// prescaler 64
	OCR0A = F_CPU/(64 * TicksPerSecond) - 1;					// set the period to the bit time
#endif
	TCNT0 = 0;
	TIMSK0 |= BITVAL(OCIE0A);									// enable timer compare match interrupt

#ifndef __ECV__													// eCv++ doesn't understand gcc assembler syntax
	wdt_enable(WDTO_500MS);										// enable the watchdog
#endif

	sei();

	for (;;)
	{
		PORT_OUT &= ~BITVAL(PortOutBitNum);						// ensure output is in default low state
		DelayTicks(2 * TicksPerSecond);							// allow the power voltage to stabilise, or give a break from flashing the previous error (also kicks the watchdog)
		if (OTS_Sensor_Init())
		{
			break;
		}
		ReportError(FLASHES_ERR_INIT);
	}

	blink(FLASHES_OK, LedGreen, OkBlinkTicks, OkBlinkTicks);	// blink 3 times after successful initialisation

	PORT_OUT &= ~BITVAL(PortOutBitNum);							// ensure output is in default low state

	// Test for Pad0 and Pad1 shorted together, which means we need to use simple pulse mode output
	PORT_PADS &= ~BITVAL(Pad0BitNum);							// disable Pad0 pullup, set it low when we make it an output
	DDR_PADS |= BITVAL(Pad0BitNum);								// make Pad0 an output
	_delay_us(10);												// give the input time to settle
	usePulseMode = (PIN_PADS & BITVAL(Pad1BitNum)) == 0;		// see if setting Pad0 to output low has made Pad1 low too
	DDR_PADS &= ~BITVAL(Pad0BitNum);							// make Pad0 an input again
	PORT_PADS |= BITVAL(Pad0BitNum);							// enable pullup on Pad0

	if (usePulseMode)
	{
		PORT_LED |= LedRed;										// alternate red and green LEDs, starting with red
	}

	uint16_t currentSteps = 0;
	bool sentPosition = false;
	for (;;)
	{
		CheckWatchdog();

		const uint16_t now = GetVolatileWord(tickCounter);
		const uint16_t diff = now - lastPollTicks;				// how long since we polled the sensor

		if (usePulseMode)
		{
			if (diff >= MinPulseIntervalTicks)
			{
				// Update the current filament position
				int16_t dx, dy;
				if (OTS_Sensor_ReadMotion(dx, dy))
				{
					currentSteps += (uint16_t)dy;				// we use the Y motion and we are assuming 2's complement representation here
				}
				lastPollTicks = now;

				// Generate a transition on the output every 0.5mm of motion, which is every 25 counts
				if (dy >= 25)
				{
					PORT_OUT ^= BITVAL(PortOutBitNum);
					dy -= 25;
					PORT_LED ^= LedRed | LedGreen;
				}
				else if (dy <= -25)
				{
					PORT_OUT ^= BITVAL(PortOutBitNum);
					dy += 25;
					PORT_LED ^= LedRed | LedGreen;
				}
			}
		}
		else
		{
			// Produce the output signal related to the accumulated position if it is due
			if (diff >= MinOutputIntervalTicks)
			{
				// Update the current filament position
				int16_t dx, dy;
				if (OTS_Sensor_ReadMotion(dx, dy))
				{
					currentSteps += (uint16_t)dy;				// we use the Y motion and we are assuming 2's complement representation here
				}
				lastPollTicks = now;

				// Generate the output value
				uint16_t outVal = currentSteps & 1023;			// restrict it to 10 bits just like the mechanical filament sensor
				if ((PIN_SWITCH & BITVAL(PortSwitchBitNum)) != 0)
				{
					outVal |= SwitchOpenBit;					// send the switch bit too
				}

				if (outVal != lastOutval || (!sentPosition && now - lastOutTicks >= MaxOutputIntervalTicks))
				{
					lastOutTicks = now;
					lastOutval = outVal;
					SendWord(outVal, LedGreen);
					sentPosition = true;
				}
				else if (now - lastOutTicks >= MaxOutputIntervalTicks)
				{
					// Send the other status
					lastOutTicks = now;
					const uint16_t outWord = ((uint16_t)OTS_Sensor_Read_shutter() << 8) | OTS_Sensor_Read_brightness() | QualityBit;
					SendWord(outWord, LedRed);
					sentPosition = false;
				}
			}
		}
	}

#ifdef __ECV__
	return 0;
#endif
}

// Timer ISR for setting output flag
#ifdef __ECV__
void TIM0_COMPA_vect()
#else
ISR(TIM0_COMPA_vect)
#endif
{
	tickCounter++;
}

// Wait for the next tick. This does not call the watchdog, so don't call this too many times without making a call to checkWatchdog.
inline void WaitForNextTick()
{
	const volatile uint8_t * const tickLsb = reinterpret_cast<const volatile uint8_t *>(&tickCounter);
	const uint8_t initialCount = *tickLsb;
	while (*tickLsb == initialCount) { }
}

// Send a 16-bit word
void SendWord(uint16_t data, uint8_t ledMask)
{
	PORT_LED |= ledMask;

	// Calculate the parity bit
	uint8_t data8 = (uint8_t)((data >> 8) ^ data);
	data8 ^= (data8 >> 4);
	data8 ^= (data8 >> 2);
	data8 ^= (data8 >> 1);
	if (data8 & 1)
	{
		data ^= ParityBit;
	}

	WaitForNextTick();							// this one will be a full bit length
	PORT_OUT |= BITVAL(PortOutBitNum);			// set output high for the start bit
	WaitForNextTick();
	PORT_OUT &= ~BITVAL(PortOutBitNum);			// return output to low for the end of the start bit

	// Send 4 nibbles + stuffing bits
	for (uint8_t nibble = 0; nibble < 4; ++nibble)
	{
		bool b;
		for (uint8_t i = 0; i < 4; ++i)
		{
			b = ((data & 0x8000u) != 0);
			WaitForNextTick();
			if (b)
			{
				PORT_OUT |= BITVAL(PortOutBitNum);
			}
			else
			{
				PORT_OUT &= ~BITVAL(PortOutBitNum);
			}
			data <<= 1;
		}

		// Send the stuffing bit, which is the opposite of the last bit
		WaitForNextTick();
		if (b)
		{
			PORT_OUT &= ~BITVAL(PortOutBitNum);
		}
		else
		{
			PORT_OUT |= BITVAL(PortOutBitNum);
		}

		CheckWatchdog();
	}

	// Stop bit
	WaitForNextTick();
	PORT_OUT &= ~BITVAL(PortOutBitNum);			// return output to default low state
	WaitForNextTick();
	WaitForNextTick();

	PORT_LED &= ~ledMask;
}

/*------------------------------------------------------------------------
**  blinkCustom - function to blink LED for count passed in
**		Assumes that leds are all on the same port. 
**     Custom on and off times can be set in ~0.1s increments
** ---------------------------------------------------------------------*/
void blink(uint8_t count, uint8_t ledMask, uint16_t delayOn, uint16_t delayOff)
{
	while (count != 0)
	{
		PORT_LED |= ledMask;
		DelayTicks(delayOn);
		PORT_LED &= ~ledMask;
		DelayTicks(delayOff);
		count--;
	}
}

// End

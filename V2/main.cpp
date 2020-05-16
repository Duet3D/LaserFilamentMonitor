/*
 * LaserFilamentMonitor-v2.cpp
 *
 * Created: 10/01/2019 11:08:47
 * Author : David
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

#include "Pins.h"
#include "PAT9130.h"

constexpr uint8_t FirmwareVersion = 2;

#define DOUBLE_SPEED	(0)									// set nonzero for 2000bps, zero for 1000bps

constexpr uint32_t F_CPU = 8000000UL;
constexpr unsigned int Min3V3 = 3000;						// minimum supply voltage in mV
constexpr unsigned int Max3V3 = 3600;						// maximum supply voltage in mV
constexpr unsigned int Vdda = 1900;							// VDDA voltage in mV
constexpr uint8_t MinAdcVal = (uint8_t)(((uint32_t)Vdda * 256)/Max3V3);
constexpr uint8_t MaxAdcVal = (uint8_t)(((uint32_t)Vdda * 256)/Min3V3);

#if DOUBLE_SPEED
const uint16_t TicksPerSecond = 2000;						// this must be the same as the desired bit rate
#else
const uint16_t TicksPerSecond = 1000;						// this must be the same as the desired bit rate
#endif

// Error codes (expressed as number of blinks
const uint8_t FlashesOk = 3;
const uint8_t FlashesVccError = 4;
const uint8_t FlashesInitError = 5;

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

// Data format sent to the Duet
// Bit 15 is always the parity bit (P)
//
// Version 1 sensor:
//  Data word:			P00S00pppppppppp		S = switch open, pppppppppp = 10-bit filament position (50 counts/mm)
//  Error word:			P010000000000000
//  Quality word:		P10sssssbbbbbbbb		sssss = shutter, bbbbbbbb = brightness
//
// Version 2 sensor (this firmware):
//  Data word:			P00S 1ppp pppppppp		S = switch open, ppppppppppp = 11-bit filament position (100 counts/mm)
//  Error word:			P010 0000 0000eeee		eeee = error code
//	Version word:		P110 0000 vvvvvvvv		vvvvvvvv = sensor/firmware version, at least 2
//  Image quality word: P110 0001 qqqqqqqq		qqqqqqqq = image quality
//  Brightness word:	P110 0010 bbbbbbbb		bbbbbbbb = brightness
//  Shutter word:		P110 0011 ssssssss		ssssssss = shutter

const uint16_t ParityBit = 0x8000u;							// adjusted so that there is an even number of bits
const uint16_t SwitchOpenBit = 0x1000u;						// set in position words if the optional switch is open, or the switch not connected

const uint16_t PositionBits = 0x0800u;						// set in words carrying position data
const uint16_t ErrorBits = 0x2000u;							// set if the sensor failed to initialize or self-test
const uint16_t VersionBits = 0x6000u;						// set in words containing version information
const uint16_t ImageQualityBits = 0x6100u;					// set in words containing image quality information
const uint16_t ShutterBits = 0x6200u;						// set in words containing shutter information
const uint16_t BrightnessBits = 0x6300u;					// set in words containing brightness information

// LED flash timing
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

uint8_t nextStatusToSend = 0;

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
	SendWord(ErrorBits | errorNum, 0);							// send error to the Duet without lighting the LED
	blink(errorNum, LedRed, ErrorBlinkTicks, ErrorBlinkTicks);	// red LED error blinks, also leaves the output in the low state for ErrorBlinkTicks
}

// Check the supply voltage returning true if error
bool CheckVcc()
{
	const uint8_t adc = ADCH;									// result is left-adjusted and we only need 8 bits, so no need to read ADCL
	if (adc < MinAdcVal || adc > MaxAdcVal)
	{
		PAT9130LaserOff();
		return true;
	}
	return false;
}

int main(void)
{  
	// Set up the I/O ports
	PORTA = BITVAL(PortASwitchBitNum) | BITVAL(PortANCSBitNum) | BITVAL(PortASCLBitNum);	// ensure NCS doesn't go low when we set it to be an output; laser is off
	PORTB = BITVAL(PortBMFIOInputBitNum);						// set pullup on MFIO
	DDRA = BITVAL(PortAOutBitNum) | BITVAL(PortANCSBitNum) | BITVAL(PortASCLBitNum) | BITVAL(PortALDPBitNum);
	DDRB = LedRed | LedGreen;

	// Set up the ADC in free-running mode
	DIDR0 = BITVAL(AdcVddaInputChannel);						// disable the digital input buffer
	ADMUX = (uint8_t)AdcVddaInputChannel;						// select the input from the PAT9130 Vdd supply, use Vcc as the reference
	ADCSRB = BITVAL(ADLAR);										// free running, unipolar input mode, left-adjust result
	ADCSRA = BITVAL(ADEN) | BITVAL(ADATE) | BITVAL(ADIF) | BITVAL(ADPS2) | BITVAL(ADPS0);

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
	ADCSRA |= BITVAL(ADSC);										// start first ADC conversion

	for (;;)
	{
		PORTA &= ~BITVAL(PortAOutBitNum);						// ensure output is in default low state
		DelayTicks(2 * TicksPerSecond);							// allow the power voltage to stabilise, or give a break from flashing the previous error (also kicks the watchdog)
		if (CheckVcc())
		{
			ReportError(FlashesVccError);
		}
		else if (!PAT9130Init())
		{
			ReportError(FlashesInitError);
		}
		else
		{
			blink(FlashesOk, LedGreen, OkBlinkTicks, OkBlinkTicks);		// blink 3 times after successful initialisation

			PORTA &= ~BITVAL(PortAOutBitNum);							// ensure output is in default low state

			uint16_t currentSteps = 0;
			bool sentPosition = false;
			for (;;)
			{
				CheckWatchdog();
				if (CheckVcc())
				{
					ReportError(FlashesVccError);
					break;
				}

				const uint16_t now = GetVolatileWord(tickCounter);
				const uint16_t diff = now - lastPollTicks;				// how long since we polled the sensor

				// Produce the output signal related to the accumulated position if it is due
				if (diff >= MinOutputIntervalTicks)
				{
					// Update the current filament position
					int16_t dx, dy;
					if (PAT9130ReadMotion(dx, dy))
					{
						currentSteps += (uint16_t)dy;					// we use the Y motion and we are assuming 2's complement representation here
					}
					lastPollTicks = now;

					// Generate the output value
					uint16_t outVal = (currentSteps & 2047) | PositionBits;	// restrict it to 11 bits
					if ((PINA & BITVAL(PortASwitchBitNum)) != 0)
					{
						outVal |= SwitchOpenBit;						// send the switch bit too
					}

					if (outVal != lastOutval || (!sentPosition && now - lastOutTicks >= MaxOutputIntervalTicks))
					{
						lastOutTicks = now;
						lastOutval = outVal;
						SendWord(outVal, LedGreen);
						sentPosition = true;
					}
					else if (nextStatusToSend < 4)
					{
						lastOutTicks = now;
						uint16_t outWord;
						switch (nextStatusToSend)
						{
						case 0:
							outWord = VersionBits | FirmwareVersion;
							break;
						case 1:
							outWord = ImageQualityBits | PAT9130ReadImageQuality();
							break;
						case 2:
							outWord = ShutterBits | PAT9130ReadShutter();
							break;
						case 3:
							outWord = BrightnessBits | PAT9130ReadBrightness();
							break;
						}
						SendWord(outWord, LedRed);
						sentPosition = false;
						++nextStatusToSend;
					}
					else if (sentPosition && (now - lastOutTicks >= MaxOutputIntervalTicks - MinOutputIntervalTicks))
					{
						nextStatusToSend = 0;
					}
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
	PORTA |= BITVAL(PortAOutBitNum);			// set output high for the start bit
	WaitForNextTick();
	PORTA &= ~BITVAL(PortAOutBitNum);			// return output to low for the end of the start bit

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
				PORTA |= BITVAL(PortAOutBitNum);
			}
			else
			{
				PORTA &= ~BITVAL(PortAOutBitNum);
			}
			data <<= 1;
		}

		// Send the stuffing bit, which is the opposite of the last bit
		WaitForNextTick();
		if (b)
		{
			PORTA &= ~BITVAL(PortAOutBitNum);
		}
		else
		{
			PORTA |= BITVAL(PortAOutBitNum);
		}

		CheckWatchdog();
	}

	// Stop bit
	WaitForNextTick();
	PORTA &= ~BITVAL(PortAOutBitNum);			// return output to default low state
	WaitForNextTick();
	WaitForNextTick();

	PORT_LED &= ~ledMask;
}

/*------------------------------------------------------------------------
**  blink - function to blink LED for count passed in
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

// Temp for debug
void FlashGreen(uint8_t count)
{
	blink(count, LedGreen, TicksPerSecond/4, TicksPerSecond/4);
}

// End

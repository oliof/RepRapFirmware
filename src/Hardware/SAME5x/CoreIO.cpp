/*
 * Core_C.cpp
 *
 *  Created on: 16 Jun 2020
 *      Author: David
 *
 *  Glue to allow some of our C++ functions to be called from C
 */

#include <CoreIO.h>
#include <Hardware/IoPorts.h>
#include "DmacManager_SAME5x.h"
#include "Interrupts.h"
#include "AnalogIn.h"
#include "AnalogOut.h"
#include <peripheral_clk_config.h>
#include <hal_usb_device.h>

// Serial device support
Uart serialUart0(Serial0SercomNumber, Sercom0RxPad, 512, 512);
SerialCDC serialUSB(UsbVBusPin, 512, 512);

# if !defined(SERIAL0_ISR0) || !defined(SERIAL0_ISR2) || !defined(SERIAL0_ISR3)
#  error SERIAL0_ISRn not defined
# endif

void SERIAL0_ISR0() noexcept
{
	serialUart0.Interrupt0();
}

void SERIAL0_ISR2() noexcept
{
	serialUart0.Interrupt2();
}

void SERIAL0_ISR3() noexcept
{
	serialUart0.Interrupt3();
}

// IoPort::SetPinMode calls this
extern "C" void pinMode(Pin pin, enum PinMode mode) noexcept
{
	if (pin < NumTotalPins)
	{
		switch (mode)
		{
		case INPUT:
			ClearPinFunction(pin);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
			break;

		case INPUT_PULLUP:
			ClearPinFunction(pin);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_UP);
			break;

		case INPUT_PULLDOWN:
			ClearPinFunction(pin);
			// The direction must be set before the pullup, otherwise setting the pullup doesn't work
			gpio_set_pin_direction(pin, GPIO_DIRECTION_IN);
			gpio_set_pin_pull_mode(pin, GPIO_PULL_DOWN);
			break;

		case OUTPUT_LOW:
			ClearPinFunction(pin);
			gpio_set_pin_level(pin, false);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
			break;

		case OUTPUT_HIGH:
			ClearPinFunction(pin);
			gpio_set_pin_level(pin, true);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OUT);
			break;

		case AIN:
			// The SAME70 errata says we must disable the pullup resistor before enabling the AFEC channel
			gpio_set_pin_pull_mode(pin, GPIO_PULL_OFF);
			gpio_set_pin_direction(pin, GPIO_DIRECTION_OFF);		// disable the data input buffer
			SetPinFunction(pin, GpioPinFunction::B);				// ADC is always on peripheral B
			break;

		default:
			break;
		}
	}
}

// IoPort::ReadPin calls this
extern "C" bool digitalRead(Pin pin) noexcept
{
	if (pin < NumTotalPins)
	{
		const uint8_t port = GPIO_PORT(pin);
		const uint32_t pinMask = 1U << GPIO_PIN(pin);
		return (hri_port_read_IN_reg(PORT, port) & pinMask) != 0;
	}

	return false;
}

// IoPort::WriteDigital calls this
extern "C" void digitalWrite(Pin pin, bool high) noexcept
{
	if (pin < NumTotalPins)
	{
		const uint8_t port = GPIO_PORT(pin);
		const uint32_t pinMask = 1U << GPIO_PIN(pin);
		if (high)
		{
			hri_port_set_OUT_reg(PORT, port, pinMask);
		}
		else
		{
			hri_port_clear_OUT_reg(PORT, port, pinMask);
		}
	}
}

// Tick handler
static volatile uint64_t g_ms_ticks = 0;		// Count of 1ms time ticks

uint32_t millis() noexcept
{
    return (uint32_t)g_ms_ticks;
}

uint64_t millis64() noexcept
{
	hal_atomic_t flags;
	atomic_enter_critical(&flags);
	const uint64_t ret = g_ms_ticks;			// take a copy with interrupts disabled to guard against rollover while we read it
	atomic_leave_critical(&flags);
	return ret;
}

void CoreSysTick() noexcept
{
	g_ms_ticks++;
}

static void UsbInit() noexcept
{
	// Set up USB clock
	hri_gclk_write_PCHCTRL_reg(GCLK, USB_GCLK_ID, CONF_GCLK_USB_SRC | GCLK_PCHCTRL_CHEN);
	hri_mclk_set_AHBMASK_USB_bit(MCLK);
	hri_mclk_set_APBBMASK_USB_bit(MCLK);

//	usb_d_init();

	// Set up USB pins
	// This is the code generated by Atmel Start. I don't know whether it is all necessary.
	gpio_set_pin_direction(PortAPin(24), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(24), false);
	gpio_set_pin_pull_mode(PortAPin(24), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(24), PINMUX_PA24H_USB_DM);

	gpio_set_pin_direction(PortAPin(25), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(25), false);
	gpio_set_pin_pull_mode(PortAPin(25), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(25), PINMUX_PA25H_USB_DP);
}

static void SdhcInit() noexcept
{
	// Set up SDHC clock
	hri_mclk_set_AHBMASK_SDHC1_bit(MCLK);
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID, CONF_GCLK_SDHC1_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));
	hri_gclk_write_PCHCTRL_reg(GCLK, SDHC1_GCLK_ID_SLOW, CONF_GCLK_SDHC1_SLOW_SRC | (1 << GCLK_PCHCTRL_CHEN_Pos));

	// Set up SDHC pins
#if 1
	// This is the code generated by Atmel Start. I don't know whether it is all necessary.
	gpio_set_pin_direction(PortAPin(21), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(21), false);
	gpio_set_pin_pull_mode(PortAPin(21), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(21), PINMUX_PA21I_SDHC1_SDCK);

	gpio_set_pin_direction(PortAPin(20), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortAPin(20), false);
	gpio_set_pin_pull_mode(PortAPin(20), GPIO_PULL_OFF);
	gpio_set_pin_function(PortAPin(20), PINMUX_PA20I_SDHC1_SDCMD);

	gpio_set_pin_direction(PortBPin(18), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(18), false);
	gpio_set_pin_pull_mode(PortBPin(18), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(18), PINMUX_PB18I_SDHC1_SDDAT0);

	gpio_set_pin_direction(PortBPin(19), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(19), false);
	gpio_set_pin_pull_mode(PortBPin(19), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(19), PINMUX_PB19I_SDHC1_SDDAT1);

	gpio_set_pin_direction(PortBPin(20), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(20), false);
	gpio_set_pin_pull_mode(PortBPin(20), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(20), PINMUX_PB20I_SDHC1_SDDAT2);

	gpio_set_pin_direction(PortBPin(21), GPIO_DIRECTION_OUT);
	gpio_set_pin_level(PortBPin(21), false);
	gpio_set_pin_pull_mode(PortBPin(21), GPIO_PULL_OFF);
	gpio_set_pin_function(PortBPin(21), PINMUX_PB21I_SDHC1_SDDAT3);
#else
	// Setup SD card interface pins
	for (Pin p : SdMciPins)
	{
		SetPinFunction(p, SdMciPinsFunction);
	}
#endif
}

// Random number generator
static void RandomInit()
{
	hri_mclk_set_APBCMASK_TRNG_bit(MCLK);
	hri_trng_set_CTRLA_ENABLE_bit(TRNG);
}

// Serial interface
static void SerialInit()
{
	SetPinFunction(Serial0TxPin, Serial0PinFunction);
	SetPinFunction(Serial0RxPin, Serial0PinFunction);
	// We don't make the init call here, that's done by the GCodes module
}

void CoreInit() noexcept
{
	// Ensure the Ethernet PHY or WiFi module is held reset
	pinMode(EthernetPhyResetPin, OUTPUT_LOW);

	DmacManager::Init();
	UsbInit();
	SdhcInit();
	RandomInit();

	// Initialise the I/O subsystem
	InitialisePinChangeInterrupts();

	// Initialise analog in and PWM out
	AnalogIn::Init();
	AnalogOut::Init();

	SerialInit();

	// Ethernet pins are now set up in the Ethernet driver
}

void WatchdogInit() noexcept
{
	hri_mclk_set_APBAMASK_WDT_bit(MCLK);
	delayMicroseconds(5);
	hri_wdt_write_CTRLA_reg(WDT, 0);
	hri_wdt_write_CONFIG_reg(WDT, WDT_CONFIG_PER_CYC1024);		// about 1 second
	hri_wdt_write_EWCTRL_reg(WDT, WDT_EWCTRL_EWOFFSET_CYC512);	// early warning control, about 0.5 second
	hri_wdt_set_INTEN_EW_bit(WDT);								// enable early earning interrupt
	hri_wdt_write_CTRLA_reg(WDT, WDT_CTRLA_ENABLE);
}

void watchdogReset() noexcept
{
	// If we kick the watchdog too often, sometimes it resets us. It uses a 1024Hz nominal clock, so presumably it has to be reset less often than that.
	if ((((uint32_t)g_ms_ticks) & 0x07) == 0)
	{
		WDT->CLEAR.reg = 0xA5;
	}
}

void Reset() noexcept
{
	SCB->AIRCR = (0x5FA << 16) | (1u << 2);						// reset the processor
	for (;;) { }
}

void EnableTcClock(unsigned int tcNumber, uint32_t gclkVal) noexcept
{
	static constexpr uint8_t TcClockIDs[] =
	{
		TC0_GCLK_ID, TC1_GCLK_ID, TC2_GCLK_ID, TC3_GCLK_ID, TC4_GCLK_ID,
		TC5_GCLK_ID, TC6_GCLK_ID, TC7_GCLK_ID
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TcClockIDs[tcNumber], gclkVal | (1 << GCLK_PCHCTRL_CHEN_Pos));

	switch (tcNumber)
	{
	case 0:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC0; break;
	case 1:	MCLK->APBAMASK.reg |= MCLK_APBAMASK_TC1; break;
	case 2:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC2; break;
	case 3:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TC3; break;
	case 4:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC4; break;
	case 5:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TC5; break;
	case 6: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC6; break;
	case 7: MCLK->APBDMASK.reg |= MCLK_APBDMASK_TC7; break;
	}
}

void EnableTccClock(unsigned int tccNumber, uint32_t gclkVal) noexcept
{
	static constexpr uint8_t TccClockIDs[] =
	{
		TCC0_GCLK_ID, TCC1_GCLK_ID, TCC2_GCLK_ID,
		TCC3_GCLK_ID, TCC4_GCLK_ID
	};

	hri_gclk_write_PCHCTRL_reg(GCLK, TccClockIDs[tccNumber], gclkVal | (1 << GCLK_PCHCTRL_CHEN_Pos));

	switch (tccNumber)
	{
	case 0:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC0; break;
	case 1:	MCLK->APBBMASK.reg |= MCLK_APBBMASK_TCC1; break;
	case 2:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC2; break;
	case 3:	MCLK->APBCMASK.reg |= MCLK_APBCMASK_TCC3; break;
	case 4:	MCLK->APBDMASK.reg |= MCLK_APBDMASK_TCC4; break;
	}
}

AnalogChannelNumber PinToAdcChannel(Pin p) noexcept
{
	return (p < ARRAY_SIZE(PinTable)) ? PinTable[p].adc : AdcInput::none;
}

// Random number generator
uint32_t trueRandom() noexcept
{
	while (!hri_trng_get_INTFLAG_reg(TRNG, TRNG_INTFLAG_DATARDY)) { }		// Wait until data ready
	return hri_trng_read_DATA_reg(TRNG);
}

// End

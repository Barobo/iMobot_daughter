#include "lpc17xx.h"
#include "adc.h"
#include "global.h"
#include "hardware_def.h"

void AdcInit(void)
{
	uint32_t pclkdiv, pclk;

	// Power on ADC peripheral
	LPC_SC->PCONP |= _BIT(PCADC);

	// config pins as adc input, see 108 for config info
	set_gpio_select(VSENSE_BAT, 1);
	set_gpio_select(ISENSE_VCC, 1);
	set_gpio_select(ISENSE_BUS, 1);
	set_gpio_select(VSENSE_IR_BACK, 3);
	set_gpio_select(VSENSE_IR_FRONT, 3);

	// By default, the PCLKSELx value is zero, thus, the PCLK for
	// all the peripherals is 1/4 of the SystemFrequency.
	pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03; // Bit 6~7 is for UART0
	pclk = SystemCoreClock / 4;

	LPC_ADC->ADCR = (0x01 << 0); // SEL=1,select channel 0 on ADC0
	LPC_ADC->ADCR |= ((pclk / ADC_CLK - 1) << 8); // CLKDIV = Fpclk / ADC_CLK - 1
	LPC_ADC->ADCR |= (0 << 16); // BURST = 0, no BURST, software controlled
	LPC_ADC->ADCR |= (0 << 17); // CLKS = 0, 11 clocks/10 bits
	LPC_ADC->ADCR |= (1 << 21); // PDN = 1, normal operation
	LPC_ADC->ADCR |= (0 << 24); // START = 0 A/D conversion stops
	LPC_ADC->ADCR |= (0 << 27); // EDGE = 0 (CAP/MAT singal falling,trigger A/D conversion)
}

int32_t AdcRead(uint8_t channelNum)
{
	uint32_t regVal, ADC_Data;


	LPC_ADC->ADCR &= 0xFFFFFF00;
	LPC_ADC->ADCR |= _BIT(24) | _BIT(channelNum);
	// switch channel,start A/D convert
	while (1)
	{
		regVal = *(volatile unsigned long *) (LPC_ADC_BASE + ADC_OFFSET	+ ADC_INDEX * channelNum);
		if (regVal & ADC_DONE) // is the conversion done?
		{
			break;
		}
	}

	LPC_ADC->ADCR &= 0xF8FFFFFF; // stop ADC
	if (regVal & ADC_OVERRUN) // check for overrun
	{
		return (-1);
	}

	// return value is 12bits
	ADC_Data = (regVal >> 4) & 0xFFF;
	return (ADC_Data);
}

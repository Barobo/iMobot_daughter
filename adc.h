#ifndef __ADC_H 
#define __ADC_H

#define ADC_OFFSET		0x10
#define ADC_INDEX		4

#define ADC_DONE		0x80000000
#define ADC_OVERRUN		0x40000000
#define ADC_ADINT		0x00010000
#define PCADC			12
#define ADC_CLK			1000000

extern void AdcInit(void);
extern int32_t AdcRead(uint8_t channelNum);
#endif // end __ADC_H

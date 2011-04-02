#ifndef __ADC_H 
#define __ADC_H

/* If Burst mode is enabled, make sure interrupt flag is set. */
#define ADC_INTERRUPT_FLAG	0/* 1 is interrupt driven, 0 is polling */
#define BURST_MODE			0   /* Burst mode works in interrupt driven mode only. */

#define ADC_OFFSET		0x10
#define ADC_INDEX		4

#define ADC_DONE		0x80000000
#define ADC_OVERRUN		0x40000000
#define ADC_ADINT		0x00010000

#define ADC_NUM			8		/* for LPCxxxx */
#define ADC_CLK			1000000		/* set to 1Mhz */


extern uint32_t ADCInit( uint32_t ADC_Clk );
extern uint32_t ADC0Read( uint8_t channelNum );
extern void ADCBurstRead( void );
#endif /* end __ADC_H */
/*****************************************************************************
**                            End Of File
******************************************************************************/

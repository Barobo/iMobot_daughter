//#include "lpc17xx.h"
//#include "adc.h"
//#include "global.h"
//#include "hardware_def.h"
//
//uint32_t AdcInit( uint32_t ADC_Clk )
//{
//  uint32_t pclkdiv, pclk;
//
//  // Configure ADC clock
//  LPC_SC->PCONP |= _BIT(12);
//
//  /* all the related pins are set to ADC inputs, AD0.0~7 */
//  LPC_PINCON->PINSEL0 |= 0x0F000000;	/* P0.12~13, A0.6~7, function 11 */
//  LPC_PINCON->PINSEL1 &= ~0x003FC000;	/* P0.23~26, A0.0~3, function 01 */
//  LPC_PINCON->PINSEL1 |= 0x00154000;
//  LPC_PINCON->PINSEL3 |= 0xF0000000;	/* P1.30~31, A0.4~5, function 11 */
//
//  /* By default, the PCLKSELx value is zero, thus, the PCLK for
//  all the peripherals is 1/4 of the SystemFrequency. */
//  /* Bit 6~7 is for UART0 */
//  pclkdiv = (LPC_SC->PCLKSEL0 >> 6) & 0x03;
//  switch ( pclkdiv )
//  {
//	case 0x00:
//	default:
//	  pclk = SystemFrequency/4;
//	break;
//	case 0x01:
//	  pclk = SystemFrequency;
//	break;
//	case 0x02:
//	  pclk = SystemFrequency/2;
//	break;
//	case 0x03:
//	  pclk = SystemFrequency/8;
//	break;
//  }
//
//  LPC_ADC->ADCR = ( 0x01 << 0 ) | 	/* SEL=1,select channel 0 on ADC0 */
//		( ( pclk  / ADC_Clk - 1 ) << 8 ) |  /* CLKDIV = Fpclk / ADC_Clk - 1 */
//		( 0 << 16 ) | 		/* BURST = 0, no BURST, software controlled */
//		( 0 << 17 ) |  		/* CLKS = 0, 11 clocks/10 bits */
//		( 1 << 21 ) |  		/* PDN = 1, normal operation */
//		( 0 << 24 ) |  		/* START = 0 A/D conversion stops */
//		( 0 << 27 );		/* EDGE = 0 (CAP/MAT singal falling,trigger A/D conversion) */
//  return (TRUE);
//}
//
//uint32_t ADC0Read( uint8_t channelNum )
//{
//#if !ADC_INTERRUPT_FLAG
//  uint32_t regVal, ADC_Data;
//#endif
//
//  /* channel number is 0 through 7 */
//  if ( channelNum >= ADC_NUM )
//  {
//	channelNum = 0;		/* reset channel number to 0 */
//  }
//  LPC_ADC->ADCR &= 0xFFFFFF00;
//  LPC_ADC->ADCR |= (1 << 24) | (1 << channelNum);
//				/* switch channel,start A/D convert */
//  while (1)			/* wait until end of A/D convert */
//  {
//	regVal = *(volatile unsigned long *)(LPC_ADC_BASE
//			+ ADC_OFFSET + ADC_INDEX * channelNum);
//	/* read result of A/D conversion */
//	if ( regVal & ADC_DONE )
//	{
//	  break;
//	}
//  }
//
//  LPC_ADC->ADCR &= 0xF8FFFFFF;	/* stop ADC now */
//  if ( regVal & ADC_OVERRUN )	/* save data when it's not overrun, otherwise, return zero */
//  {
//	return ( 0 );
//  }
//  ADC_Data = ( regVal >> 4 ) & 0xFFF;
//  return ( ADC_Data );	/* return A/D conversion value */
//}

#include "lpc17xx.h"
#include "encoder.h"
#include "hardware_def.h"
#include "timer.h"

#ifdef INT_ENCODER_READ
volatile uint32_t last_rising_edge[4] = {0,0,0,0};
volatile uint32_t timer_counts[4]     = {0,0,0,0};
volatile uint32_t timer_period[4]     = {0,0,0,0};
const    uint8_t  encoder_loc[4]      = {ENC_BACK_SIDE, ENC_BACK_FRONT, ENC_FRONT_SIDE, ENC_FRONT_FRONT};
#endif

int32_t get_encoder_duty_percent(uint32_t channel)
{
#ifdef INT_ENCODER_READ
    uint32_t ret = 100 * timer_counts[channel] / timer_period[channel];
    if (ret > 100)
        return (-1);
    else
        return (ret);
#else
    uint32_t cnt_high = 0;
    uint32_t cnt_low = 0;

    // wait for the pin to go low
    while(get_gpio_pin(ENC_FRONT_SIDE));
    // now wait for it to go high again
    while(!get_gpio_pin(ENC_FRONT_SIDE));
    // once it does
    while(get_gpio_pin(ENC_FRONT_SIDE))
    {
    	cnt_high++; // count
    }
    // when it goes low again
    while(!get_gpio_pin(ENC_FRONT_SIDE))
    {
    	cnt_low++; // count
    }
    return (100 * cnt_high / (cnt_high + cnt_low));
#endif
}

#ifdef INT_ENCODER_READ
void EINT3_IRQHandler(void)
{
    uint8_t i =  0;
	LPC_SC->EXTINT |= EINT3;

	for (i = 0;i < 4;i++)
	{
	    if (LPC_GPIOINT->IO2IntStatR & _BIT(encoder_loc[i] - 200))
	    {
	        timer_period[i] = enc_now - last_rising_edge[i];
	        last_rising_edge[i] = enc_now;
	        LPC_GPIOINT->IO2IntClr |= _BIT(encoder_loc[i] - 200);
	    }
	    else if (LPC_GPIOINT->IO2IntStatF & _BIT(encoder_loc[i] - 200))
	    {
	        timer_counts[i] = enc_now - last_rising_edge[i];
	        LPC_GPIOINT->IO2IntClr |= _BIT(encoder_loc[i] - 200);
	    }
	}
}
#endif
void EncoderInit(void)
{
#ifdef INT_ENCODER_READ
    uint8_t i = 0;

    LPC_GPIOINT->IO2IntEnR = 0;
    LPC_GPIOINT->IO2IntEnF = 0;

    for (i = 0;i < 4;i++)
    {
        set_gpio_select(encoder_loc[i],  0);
        LPC_GPIOINT->IO2IntEnR |= _BIT(encoder_loc[i] - 200);
        LPC_GPIOINT->IO2IntEnF |= _BIT(encoder_loc[i] - 200);
    }

	LPC_SC->EXTINT |= EINT3;
	LPC_SC->EXTMODE = EINT3_EDGE;
	LPC_SC->EXTPOLAR = 0;

	NVIC_EnableIRQ(EINT3_IRQn);
#else
	set_gpio_select(ENC_FRONT_SIDE,0);	// gpio
	set_gpio_dir(ENC_FRONT_SIDE,0);		// input
#endif
}

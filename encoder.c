#include "global.h"
#include "encoder.h"
#include "timer.h"

int32_t EncoderRead(uint32_t channel)
{
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
}

void EncoderInit(void)
{
	set_gpio_select(ENC_FRONT_SIDE,0);	// gpio
	set_gpio_dir(ENC_FRONT_SIDE,0);		// input
}

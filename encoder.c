#include "global.h"
#include "encoder.h"
#include "timer.h"

int32_t EncoderRead(uint32_t channel)
{
    uint32_t cnt_high = 0;
    uint32_t cnt_low = 0;

    // wait for the pin to go low
    while(get_gpio_pin(channel));
    // now wait for it to go high again
    while(!get_gpio_pin(channel));
    // once it does
    while(get_gpio_pin(channel))
    {
        cnt_high++; // count
    }
    // when it goes low again
    while(!get_gpio_pin(channel))
    {
        cnt_low++; // count
    }
    return (100 * cnt_high / (cnt_high + cnt_low));
}

void EncoderInit(uint32_t channel)
{
    set_gpio_select(channel, 0); // gpio
    set_gpio_dir(channel, 0); // input
}

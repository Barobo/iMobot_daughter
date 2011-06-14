#include "global.h"
#include "hardware_def.h"
#include "encoder.h"
#include "timer.h"
#include "consoleprint.h"
char print_buffer[30];

int32_t EncoderRead(uint32_t channel)
{
    uint32_t cnt_high = 0;
    uint32_t cnt_low = 0;
    uint32_t timeout = 0;
    int32_t enc;
    // wait for the pin to go low
    while(get_gpio_pin(channel))
    {
        if (++timeout == 3000)
            goto err;
    }
    timeout = 0;
    // now wait for it to go high again
    while(!get_gpio_pin(channel))
    {
        if (++timeout == 3000)
            goto err;
    }
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
    enc = (ENCODER_RANGE * cnt_high / (cnt_high + cnt_low));
    switch(channel) {
    case ENC_BACK_SIDE:
    	enc -= BS_CENTER*ENCODER_MULTIPLIER;
    	break;
    case ENC_BACK_FRONT:
    	enc -= BF_CENTER*ENCODER_MULTIPLIER;
    	enc = enc * -1;
    	break;
    case ENC_FRONT_SIDE:
    	enc -= FS_CENTER*ENCODER_MULTIPLIER;
    	break;
    case ENC_FRONT_FRONT:
    	enc -= FF_CENTER*ENCODER_MULTIPLIER;
    	enc = enc * -1;
    	break;
    default:
    	goto err;
    }
    /* Make sure enc is a positive value in the range 0-360 */
    while(enc < 0) {
    	enc += ENCODER_RANGE;
    }
    enc = enc % ENCODER_RANGE;
    return enc;
    err:
    return (-1);
}

void PrintAllEncoders(void)
{
    snprintf(print_buffer, 30, "%d %d %d %d\n",
            EncoderRead(ENC_BACK_SIDE),
            EncoderRead(ENC_BACK_FRONT),
            EncoderRead(ENC_FRONT_SIDE),
            EncoderRead(ENC_FRONT_FRONT));
    consoleprint(print_buffer);
}

void EncoderInit(void)
{
    set_gpio_select(ENC_BACK_SIDE, 0);
    set_gpio_dir(ENC_BACK_SIDE, 0);

    set_gpio_select(ENC_BACK_FRONT, 0);
    set_gpio_dir(ENC_BACK_FRONT, 0);

    set_gpio_select(ENC_FRONT_SIDE, 0);
    set_gpio_dir(ENC_FRONT_SIDE, 0);

    set_gpio_select(ENC_FRONT_FRONT, 0);
    set_gpio_dir(ENC_FRONT_FRONT, 0);
}

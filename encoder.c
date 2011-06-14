#include "global.h"
#include "hardware_def.h"
#include "encoder.h"
#include "timer.h"
#include "consoleprint.h"
char print_buffer[30];

char encoder_old[4];

int32_t EncoderRead(uint32_t channel)
{
    uint32_t cnt_high        = 0;
    uint32_t cnt_low         = 0;
    uint32_t timeout         = 0;
    uint32_t ret             = 0;
    uint32_t encoder_current = 0;
    uint8_t  index           = 0;

    // map enums to gpio
    switch(channel)
    {
        case ENC_FRONT_SIDE:
            index = ENC_FS;
            break;
        case ENC_FRONT_FRONT:
            index = ENC_FF;
            break;
        case ENC_BACK_FRONT:
            index = ENC_BF;
            break;
        case ENC_BACK_SIDE:
            index = ENC_BS;
            break;
        default:
            goto err;
    }
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

    //LPF: New = alpha * input + (1 - alpha) * old
    encoder_current = (100 * cnt_high / (cnt_high + cnt_low));
    ret = (encoder_current / ENC_A_DENOM) + (encoder_old[index] - encoder_old[index] / ENC_A_DENOM);
    encoder_old[index] = encoder_current;

    return ret;
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

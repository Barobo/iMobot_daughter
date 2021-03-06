#include "hardware_def.h"
#include "global.h"

// See user manual page 108 for function descriptions
void set_gpio_select(int id, int function)
{
    if(id < 16)
    {
        LPC_PINCON->PINSEL0 &= ~(3 << (id * 2));
        LPC_PINCON->PINSEL0 |= (function << (id * 2));
    }
    else if(id < 100)
    {
        LPC_PINCON->PINSEL1 &= ~(3 << ((id - 16) * 2));
        LPC_PINCON->PINSEL1 |= (function << ((id - 16) * 2));
    }
    else if(id < 116)
    {
        LPC_PINCON->PINSEL2 &= ~(3 << ((id - 100) * 2));
        LPC_PINCON->PINSEL2 |= (function << ((id - 100) * 2));
    }
    else if(id < 200)
    {
        LPC_PINCON->PINSEL3 &= ~(3 << ((id - 116) * 2));
        LPC_PINCON->PINSEL3 |= (function << ((id - 116) * 2));
    }
    else if(id < 216)
    {
        LPC_PINCON->PINSEL4 &= ~(3 << ((id - 200) * 2));
        LPC_PINCON->PINSEL4 |= (function << ((id - 200) * 2));
    }
    else if(id < 300)
    {
        LPC_PINCON->PINSEL5 &= ~(3 << ((id - 216) * 2));
        LPC_PINCON->PINSEL5 |= (function << ((id - 216) * 2));
    }
}

void set_gpio_pull(int id, int up)
{
    if(id < 16)
    {
        LPC_PINCON->PINMODE0 &= ~(3 << (id * 2));
        LPC_PINCON->PINMODE0 |= (up << (id * 2));
    }
    else if(id < 100)
    {
        LPC_PINCON->PINMODE1 &= ~(3 << ((id - 16) * 2));
        LPC_PINCON->PINMODE1 |= (up << ((id - 16) * 2));
    }
    else if(id < 116)
    {
        LPC_PINCON->PINMODE2 &= ~(3 << ((id - 100) * 2));
        LPC_PINCON->PINMODE2 |= (up << ((id - 100) * 2));
    }
    else if(id < 200)
    {
        LPC_PINCON->PINMODE3 &= ~(3 << ((id - 116) * 2));
        LPC_PINCON->PINMODE3 |= (up << ((id - 116) * 2));
    }
    else if(id < 216)
    {
        LPC_PINCON->PINMODE4 &= ~(3 << ((id - 200) * 2));
        LPC_PINCON->PINMODE4 |= (up << ((id - 200) * 2));
    }
    else if(id < 300)
    {
        LPC_PINCON->PINMODE5 &= ~(3 << ((id - 216) * 2));
        LPC_PINCON->PINMODE5 |= (up << ((id - 216) * 2));
    }
}

void set_gpio_od(int id, int od)
{
    if(id < 16)
    {
        LPC_PINCON->PINMODE_OD0 &= ~(3 << (id * 2));
        LPC_PINCON->PINMODE_OD0 |= (od << (id * 2));
    }
    else if(id < 100)
    {
        LPC_PINCON->PINMODE_OD1 &= ~(3 << ((id - 16) * 2));
        LPC_PINCON->PINMODE_OD1 |= (od << ((id - 16) * 2));
    }
    else if(id < 116)
    {
        LPC_PINCON->PINMODE_OD2 &= ~(3 << ((id - 100) * 2));
        LPC_PINCON->PINMODE_OD2 |= (od << ((id - 100) * 2));
    }
    else if(id < 200)
    {
        LPC_PINCON->PINMODE_OD3 &= ~(3 << ((id - 116) * 2));
        LPC_PINCON->PINMODE_OD3 |= (od << ((id - 116) * 2));
    }
    else if(id < 216)
    {
        LPC_PINCON->PINMODE_OD4 &= ~(3 << ((id - 200) * 2));
        LPC_PINCON->PINMODE_OD4 |= (od << ((id - 200) * 2));
    }
}

void set_gpio_dir(int id, int state)
{
    if(state == GPIO_OUTPUT)
    {
        if(id < 100)
        {
            LPC_GPIO0->FIODIR |= _BIT(id);
        }
        else if(id < 200)
        {
            LPC_GPIO1->FIODIR |= _BIT(id - 100);
        }
        else if(id < 300)
        {
            LPC_GPIO2->FIODIR |= _BIT(id - 200);
        }
    }
    else // GPIO_INPUT
    {
        if(id < 100)
        {
            LPC_GPIO0->FIODIR &= ~_BIT(id);
        }
        else if(id < 200)
        {
            LPC_GPIO1->FIODIR &= ~_BIT(id - 100);
        }
        else if(id < 300)
        {
            LPC_GPIO2->FIODIR &= ~_BIT(id - 200);
        }
    }
}

void set_gpio_pin(int id, int state)
{
    switch(state)
    {
        case GPIO_ON:
            if(id < 100)
            {
                LPC_GPIO0->FIOSET = _BIT(id);
            }
            else if(id < 200)
            {
                LPC_GPIO1->FIOSET = _BIT(id - 100);
            }
            else if(id < 300)
            {
                LPC_GPIO2->FIOSET = _BIT(id - 200);
            }
            break;
        case GPIO_OFF:
            if(id < 100)
            {
                LPC_GPIO0->FIOCLR = _BIT(id);
            }
            else if(id < 200)
            {
                LPC_GPIO1->FIOCLR = _BIT(id - 100);
            }
            else if(id < 300)
            {
                LPC_GPIO2->FIOCLR = _BIT(id - 200);
            }
            break;
        case GPIO_TOGGLE:
            set_gpio_pin(id, (get_gpio_pin(id) ? GPIO_OFF : GPIO_ON));
            break;
        default:
            break;
    }
}

int get_gpio_pin(int id)
{
    if(id < 100 && LPC_GPIO0->FIOPIN & _BIT(id))
    {
        return (1);
    }
    else if(id < 200 && LPC_GPIO1->FIOPIN & _BIT(id - 100))
    {
        return (1);
    }
    else if(id < 300 && LPC_GPIO2->FIOPIN & _BIT(id - 200))
    {
        return (1);
    }
    return 0;
}

void GpioInit(void)
{
#ifdef LPC_X_DEBUG
    set_gpio_select(LED2, 0);
    set_gpio_dir(LED2, GPIO_OUTPUT);
#endif
    set_gpio_select(RED_LED, 0);
    set_gpio_dir(RED_LED, GPIO_OUTPUT);
    set_gpio_select(GREEN_LED, 0);
    set_gpio_dir(GREEN_LED, GPIO_OUTPUT);

    set_gpio_dir(SEN_1_EN,GPIO_OUTPUT);
    set_gpio_dir(SEN_2_EN,GPIO_OUTPUT);
    set_gpio_dir(SEN_3_EN,GPIO_OUTPUT);
    set_gpio_dir(SEN_4_EN,GPIO_OUTPUT);
    set_gpio_dir(SEN_5_EN,GPIO_OUTPUT);
    set_gpio_dir(SEN_6_EN,GPIO_OUTPUT);
    set_gpio_pin(SEN_1_EN,GPIO_ON);
    set_gpio_pin(SEN_2_EN,GPIO_ON);
    set_gpio_pin(SEN_3_EN,GPIO_ON);
    set_gpio_pin(SEN_4_EN,GPIO_ON);
    set_gpio_pin(SEN_5_EN,GPIO_ON);
    set_gpio_pin(SEN_6_EN,GPIO_ON);
    // Init Timer

}

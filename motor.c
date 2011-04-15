#include "global.h"
#include "motor.h"

uint32_t MotorInit()
{
    LPC_PINCON->PINSEL4 = 0x00001555; /* set GPIOs for all PWM pins on PWM0 */

    LPC_PWM1->TCR = TCR_RESET; /* Counter Reset */
    LPC_PWM1->PR = 0x00; /* count frequency:Fpclk */
    LPC_PWM1->MCR = PWMMR0I; /* interrupt on PWMMR0, reset on PWMMR0, reset TC if PWM0 matches */

    LPC_PWM1->MR0 = 10000;
    LPC_PWM1->MR3 = 0;
    LPC_PWM1->MR4 = 0;
    LPC_PWM1->MR5 = 0;
    LPC_PWM1->MR6 = 0;

    /* all PWM latch enabled */
    LPC_PWM1->LER = LER0_EN | LER3_EN | LER4_EN | LER5_EN | LER6_EN;

    //LPC_GPIO0->FIOMASK |= _BIT(16);

    set_gpio_select(M1_PWM, 1);
    set_gpio_select(M1_DIR1, 0);
    set_gpio_select(M1_DIR2, 0);

    set_gpio_select(M2_PWM, 1);
    set_gpio_select(M2_DIR1, 0);
    set_gpio_select(M2_DIR2, 0);

    set_gpio_select(M3_PWM, 1);
    set_gpio_select(M3_DIR1, 0);
    set_gpio_select(M3_DIR2, 0);

    set_gpio_select(M4_PWM, 1);
    set_gpio_select(M4_DIR1, 0);
    set_gpio_select(M4_DIR2, 0);

    set_gpio_dir(M1_DIR1, 1);
    set_gpio_dir(M1_DIR2, 1);
    set_gpio_dir(M2_DIR1, 1);
    set_gpio_dir(M2_DIR2, 1);
    set_gpio_dir(M3_DIR1, 1);
    set_gpio_dir(M3_DIR2, 1);
    set_gpio_dir(M4_DIR1, 1);
    set_gpio_dir(M4_DIR2, 1);

    set_gpio_pin(M1_DIR1, 0);
    set_gpio_pin(M1_DIR2, 0);

    //NVIC_EnableIRQ(PWM1_IRQn);
    return 1;
}

#define MOTOR_SCALE 100
void set_motor(uint32_t ChannelNum, int cycle)
{
    switch(ChannelNum)
    {
        case 0:
            if(cycle > 0)
            {
                set_gpio_pin(M1_DIR1, GPIO_OFF);
                set_gpio_pin(M1_DIR2, GPIO_ON);
                LPC_PWM1->MR4 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M1_DIR1, GPIO_ON);
                set_gpio_pin(M1_DIR2, GPIO_OFF);
                LPC_PWM1->MR4 = -cycle * MOTOR_SCALE;
            }
            break;
        case 1:
            if(cycle > 0)
            {
                set_gpio_pin(M2_DIR1, GPIO_OFF);
                set_gpio_pin(M2_DIR2, GPIO_ON);
                LPC_PWM1->MR3 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M2_DIR1, GPIO_ON);
                set_gpio_pin(M2_DIR2, GPIO_OFF);
                LPC_PWM1->MR3 = -cycle * MOTOR_SCALE;
            }
            break;
        case 2:
            if(cycle > 0)
            {
                set_gpio_pin(M3_DIR1, GPIO_OFF);
                set_gpio_pin(M3_DIR2, GPIO_ON);
                LPC_PWM1->MR6 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M3_DIR1, GPIO_ON);
                set_gpio_pin(M3_DIR2, GPIO_OFF);
                LPC_PWM1->MR6 = -cycle * MOTOR_SCALE;
            }
            break;
        case 3:
            if(cycle > 0)
            {
                set_gpio_pin(M4_DIR1, GPIO_OFF);
                set_gpio_pin(M4_DIR2, GPIO_ON);
                LPC_PWM1->MR5 = cycle * MOTOR_SCALE;
            }
            else
            {
                set_gpio_pin(M4_DIR1, GPIO_ON);
                set_gpio_pin(M4_DIR2, GPIO_OFF);
                LPC_PWM1->MR5 = -cycle * MOTOR_SCALE;
            }
            break;
    }

    LPC_PWM1->LER = LER3_EN | LER4_EN | LER5_EN | LER6_EN;
}

void MotorStart()
{
    /* All single edge, all enable */
    LPC_PWM1->PCR = PWMENA3 | PWMENA4 | PWMENA5 | PWMENA6;
    LPC_PWM1->TCR = TCR_CNT_EN | TCR_PWM_EN; /* counter enable, PWM enable */
}

void MotorStop()
{
    LPC_PWM1->PCR = 0;
    LPC_PWM1->TCR = 0x00; /* Stop all PWMs */
}

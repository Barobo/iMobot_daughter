#include "lpc17xx.h"
#include "timer.h"
#include "callback.h"
#include "global.h"

volatile uint32_t now = 0;

// timer 0 is for general purpose timing (callbacks)
void TIMER0_IRQHandler(void)
{
    LPC_TIM0->IR = 1; /* clear interrupt flag */
    ServiceCallbacks(++now);
    return;
}

void TIMER1_IRQHandler(void)
{
    LPC_TIM1->IR = 1; /* clear interrupt flag */
    return;
}

void EnableTimer(uint8_t timer_num)
{
    if(timer_num == 0)
    {
        LPC_TIM0->TCR = 1;
    }
    else
    {
        LPC_TIM1->TCR = 1;
    }
    return;
}

void DisableTimer(uint8_t timer_num)
{
    if(timer_num == 0)
    {
        LPC_TIM0->TCR = 0;
    }
    else
    {
        LPC_TIM1->TCR = 0;
    }
    return;
}

void ResetTimer(uint8_t timer_num)
{
    uint32_t regVal;

    if(timer_num == 0)
    {
        regVal = LPC_TIM0->TCR;
        regVal |= 0x02;
        LPC_TIM0->TCR = regVal;
    }
    else
    {
        regVal = LPC_TIM1->TCR;
        regVal |= 0x02;
        LPC_TIM1->TCR = regVal;
    }
    return;
}

uint32_t InitTimer(uint8_t timer_num, uint32_t interval)
{
    if(timer_num == 0)
    {
    	// Interval is in 1us.
    	// The main clock is 100Mhz, so each clock cycle is 10ns.
        LPC_TIM0->MR0 = (MAIN_CLOCK / BASE_TIMER_DIVISOR) * interval;
        LPC_TIM0->MCR = 3; // Interrupt and Reset on MR0

        set_callback_divisor(interval);

        NVIC_EnableIRQ(TIMER0_IRQn);
        return (TRUE);
    }
    else if(timer_num == 1)
    {
        LPC_TIM0->MR0 = (MAIN_CLOCK / BASE_TIMER_DIVISOR) * interval;
        LPC_TIM1->MCR = 3; // Interrupt and Reset on MR1

        NVIC_EnableIRQ(TIMER1_IRQn);
        return (TRUE);
    }
    return (FALSE);
}

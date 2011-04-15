#ifndef TIMER_H
#define TIMER_H

extern volatile uint32_t now;

void TIMER0_IRQHandler(void);
void TIMER1_IRQHandler(void);
void TimerEnable(uint8_t timer_num);
void TimerDisable(uint8_t timer_num);
void TimerReset(uint8_t timer_num);
uint32_t TimerInit(uint8_t timer_num, uint32_t interval);

#define BASE_TIMER_DIVISOR 1000000 // 1Mhz = 1us per timer tick
#endif // TIMER_H

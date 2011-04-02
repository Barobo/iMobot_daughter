#ifndef TIMER_H
#define TIMER_H

extern volatile uint32_t now;

void     TIMER0_IRQHandler(void);
void     TIMER1_IRQHandler(void);
void     EnableTimer(uint8_t timer_num);
void     DisableTimer(uint8_t timer_num);
void     ResetTimer(uint8_t timer_num);
uint32_t InitTimer(uint8_t timer_num, uint32_t interval);

#define BASE_TIMER_DIVISOR 1000000 // 1Mhz = 1us per timer tick

#endif // TIMER_H

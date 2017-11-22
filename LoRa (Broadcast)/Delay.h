#ifndef DELAY_H
#define DELAY_H

volatile uint64_t seconds_since_start;

/* Prototipes *****************************************************************/
void delay_us(uint32_t us);
void delay_ms(uint32_t ms);
void delay_s(uint32_t s);

uint64_t millis(void);

#endif

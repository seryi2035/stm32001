
#include "stm32f10x.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_tim.h"
#include "misc.h"

void TIM2_init(void);
void delay_us(uint32_t n_usec);
void delay_ms(uint32_t n_msec);
void TIM4_init(void);
//void TIM4_IRQHandler(void);
void TIM3_init(void);
//void TIM3_IRQHandler(void);

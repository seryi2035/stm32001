#ifndef	_DS18B20
#define	_DS18B20


#include "stm32f10x.h"
#include "stm32f10x_gpio.h"
#include "stm32f10x_rcc.h"
#include "stm32f10x_pwr.h"
#include "stm32f10x_usart.h"
#include "stm32f10x_bkp.h"
#include "stdio.h"
#include "misc.h"
#include "001.h"
#include "tim2_delay.h"

#define uint8 u8
#define int16 s16
#define uint16 u16

#define ONE_WIRE_IN()  (GPIOA->CR1 |= GPIO_Pin_3;GPIOA->DDR &= ~GPIO_Pin_3)
#define ONE_WIRE_READ() (GPIOA->IDR & GPIO_Pin_3)
#define ONE_WIRE_OUT() (GPIOA->CR1 &= ~GPIO_Pin_3;GPIOA->DDR |= GPIO_Pin_3)

extern int16 ds1820_readtemp(void);
extern void ds1820_startconversion(void);
extern void ds1820_set9bit(void);
//extern float ds1820_read(void);
//int8 onewire_read(void);
//void onewire_write(int8 data);
//void onewire_reset(void);
//void delay_us(uint16 period);
#endif


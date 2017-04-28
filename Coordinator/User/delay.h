#ifndef  _DELAY_H
#define  _DELAY_H

#include "stm32f10x.h"

void delay_ms(u16 nms);
void delay_us(u32 nus);
void delay_init(u8 SYSCLK);
//void Delay(unsigned long delay_time);
void get_ms(unsigned long *time);
#endif

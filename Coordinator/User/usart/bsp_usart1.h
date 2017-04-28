#ifndef __USART1_H
#define	__USART1_H

#include "stm32f10x.h"
#include <stdio.h>

void USART1_Config(void);
void Uart1_Put_Buf(unsigned char *DataToSend , u8 data_num);
void Uart1_Put_String(unsigned char *Str);
void Uart1_IRQ(void);
#endif /* __USART1_H */


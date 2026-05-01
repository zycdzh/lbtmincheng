#ifndef PRINTF_H
#define PRINTF_H
#include "main.h"



void uart7_tx_dma_init(void);
void uart7_tx_dma_enable(uint8_t *data, uint16_t len);
void usart_printf(const char *fmt,...);



#endif

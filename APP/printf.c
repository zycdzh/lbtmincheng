#include "main.h"
#include "printf.h"

extern UART_HandleTypeDef huart7;
extern DMA_HandleTypeDef hdma_uart7_tx;

/*
格式
usart_printf("%d,%.2f/n"x,y);

*/


// DMA 初始化，启用 UART7 的 DMA 发送功能
void uart7_tx_dma_init(void)
{
    // 使能 DMA 发送请求
    SET_BIT(huart7.Instance->CR3, USART_CR3_DMAT);
	 __HAL_LINKDMA(&huart7, hdmatx, hdma_uart7_tx);
}

void uart7_tx_dma_enable(uint8_t *data, uint16_t len)
{



	HAL_UART_Transmit_DMA(&huart7,data, len);
}

// 格式化字符串并通过 DMA 发送
void usart_printf(const char *fmt,...)
{
    static uint8_t tx_buf[256] = {0};
    static va_list ap;
    static uint16_t len;
    va_start(ap, fmt);

    //return length of string 
    //返回字符串长度
    len = vsprintf((char *)tx_buf, fmt, ap);

    va_end(ap);

   
		    HAL_UART_Transmit_DMA(&huart7,tx_buf, len);

}

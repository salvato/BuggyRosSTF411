#pragma once

#ifdef __cplusplus
extern "C" {
#endif

#include "stm32f4xx_hal.h"
#include "stm32f4xx_ll_tim.h"


#define USART2_DMA_TX_IRQHandler DMA1_Stream6_IRQHandler
#define USART2_DMA_RX_IRQHandler DMA1_Stream5_IRQHandler


void TIM2_IRQHandler(void);
void TIM5_IRQHandler(void);
void EXTI15_10_IRQHandler(void);
void USART2_DMA_RX_IRQHandler(void);
void USART2_DMA_TX_IRQHandler(void);
void USART2_IRQHandler(void);
void HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle);

#ifdef __cplusplus
}
#endif

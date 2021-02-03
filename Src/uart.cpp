#include "uart.h"
#include "utility.h"
#include "string.h"

extern UART_HandleTypeDef huart2;
extern DMA_HandleTypeDef  hdma_usart2_tx;
extern DMA_HandleTypeDef  hdma_usart2_rx;

extern unsigned int baudRate;


void
SerialPortInit(void) {
    // Init DMA First
    __HAL_RCC_DMA1_CLK_ENABLE();
    // DMA interrupt init
    // DMA1_Stream5_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream5_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream5_IRQn);

    // DMA1_Stream6_IRQn interrupt configuration
    HAL_NVIC_SetPriority(DMA1_Stream6_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(DMA1_Stream6_IRQn);

    huart2.Instance = USART2;
    huart2.Init.BaudRate     = baudRate;
    huart2.Init.WordLength   = UART_WORDLENGTH_8B;
    huart2.Init.StopBits     = UART_STOPBITS_1;
    huart2.Init.Parity       = UART_PARITY_NONE;
    huart2.Init.HwFlowCtl    = UART_HWCONTROL_NONE;
    huart2.Init.Mode         = UART_MODE_TX_RX;
    huart2.Init.OverSampling = UART_OVERSAMPLING_16;
    if(HAL_UART_Init(&huart2) != HAL_OK)
        Error_Handler();
}


// @brief UART MSP Initialization
// This function configures the hardware resources used
// @param huart: UART handle pointer
// @retval None
void
HAL_UART_MspInit(UART_HandleTypeDef* huart) {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    if(huart->Instance==USART2) {
        __HAL_RCC_USART2_CLK_ENABLE();
        __HAL_RCC_GPIOA_CLK_ENABLE();

        // USART2 GPIO Configuration
        // PA2     ------> USART2_TX
        // PA3     ------> USART2_RX
        GPIO_InitStruct.Pin       = GPIO_PIN_2|GPIO_PIN_3;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_PP;
        GPIO_InitStruct.Pull      = GPIO_NOPULL;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF7_USART2;
        HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

        // USART2 DMA Init
        // USART2_RX Init
        hdma_usart2_rx.Instance = DMA1_Stream5;
        hdma_usart2_rx.Init.Channel             = DMA_CHANNEL_4;
        hdma_usart2_rx.Init.Direction           = DMA_PERIPH_TO_MEMORY;
        hdma_usart2_rx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart2_rx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart2_rx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_rx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart2_rx.Init.Mode                = DMA_CIRCULAR;
        hdma_usart2_rx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_usart2_rx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_usart2_rx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmarx, hdma_usart2_rx);

        // USART2_TX Init
        hdma_usart2_tx.Instance = DMA1_Stream6;
        hdma_usart2_tx.Init.Channel             = DMA_CHANNEL_4;
        hdma_usart2_tx.Init.Direction           = DMA_MEMORY_TO_PERIPH;
        hdma_usart2_tx.Init.PeriphInc           = DMA_PINC_DISABLE;
        hdma_usart2_tx.Init.MemInc              = DMA_MINC_ENABLE;
        hdma_usart2_tx.Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        hdma_usart2_tx.Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        hdma_usart2_tx.Init.Mode                = DMA_NORMAL;
        hdma_usart2_tx.Init.Priority            = DMA_PRIORITY_HIGH;
        hdma_usart2_tx.Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        if(HAL_DMA_Init(&hdma_usart2_tx) != HAL_OK) {
            Error_Handler();
        }
        __HAL_LINKDMA(huart, hdmatx, hdma_usart2_tx);

        // USART2 interrupt Init
        HAL_NVIC_SetPriority(USART2_IRQn, 0, 0);
        HAL_NVIC_EnableIRQ(USART2_IRQn);
    }
}


void
HAL_UART_MspDeInit(UART_HandleTypeDef* huart) {
    if(huart->Instance==USART2) {
        __HAL_RCC_USART2_CLK_DISABLE();

        // USART2 GPIO Configuration
        // PA2     ------> USART2_TX
        // PA3     ------> USART2_RX
        HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);
        // USART2 DMA DeInit
        HAL_DMA_DeInit(huart->hdmarx);
        HAL_DMA_DeInit(huart->hdmatx);
        // USART2 interrupt DeInit
        HAL_NVIC_DisableIRQ(USART2_IRQn);
    }
}

#include "i2c.h"
#include "utility.h"
#include "string.h" // for memset()


#define I2C_SPEEDCLOCK 400000 // Hz

#define USART2_FORCE_RESET()   __HAL_RCC_USART2_FORCE_RESET()
#define USARt2_RELEASE_RESET() __HAL_RCC_USART2_RELEASE_RESET()


extern I2C_HandleTypeDef  hi2c2;

void
I2C2_Init(void) {
    __HAL_RCC_I2C2_CLK_ENABLE();
    hi2c2.Instance = I2C2;
    hi2c2.Init.ClockSpeed      = I2C_SPEEDCLOCK;
    hi2c2.Init.DutyCycle       = I2C_DUTYCYCLE_2;
    hi2c2.Init.OwnAddress1     = 0;
    hi2c2.Init.AddressingMode  = I2C_ADDRESSINGMODE_7BIT;
    hi2c2.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c2.Init.OwnAddress2     = 0;
    hi2c2.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c2.Init.NoStretchMode   = I2C_NOSTRETCH_DISABLE;
    if(HAL_I2C_Init(&hi2c2) != HAL_OK) {
        Error_Handler();
    }
}


// @brief I2C MSP Initialization
// This function configures the hardware resources used
// @param hi2c: I2C handle pointer
// @retval None
void
HAL_I2C_MspInit(I2C_HandleTypeDef* hi2c) {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));
    if(hi2c->Instance==I2C2) {
        __HAL_RCC_GPIOB_CLK_ENABLE();
        // I2C2 GPIO Configuration
        // PB10  ------> I2C2_SCL (CN10 - 25)
        // PB3   ------> I2C2_SDA (CN10 - 31)
        GPIO_InitStruct.Pin       = GPIO_PIN_10;
        GPIO_InitStruct.Mode      = GPIO_MODE_AF_OD;
        GPIO_InitStruct.Pull      = GPIO_PULLUP;
        GPIO_InitStruct.Speed     = GPIO_SPEED_FREQ_VERY_HIGH;
        GPIO_InitStruct.Alternate = GPIO_AF4_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        GPIO_InitStruct.Pin       = GPIO_PIN_3;
        GPIO_InitStruct.Alternate = GPIO_AF9_I2C2;
        HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

        __HAL_RCC_I2C2_CLK_ENABLE();
    }
}


// @brief I2C MSP De-Initialization
// This function freeze the hardware resources used
// @param hi2c: I2C handle pointer
// @retval None

void
HAL_I2C_MspDeInit(I2C_HandleTypeDef* hi2c) {
    if(hi2c->Instance == I2C2) {
        __HAL_RCC_I2C2_CLK_DISABLE();
        // I2C1 GPIO Configuration
        // PB10  ------> I2C1_SCL
        // PB3   ------> I2C1_SDA
        HAL_GPIO_DeInit(GPIOB, GPIO_PIN_3 | GPIO_PIN_10);
    }
}


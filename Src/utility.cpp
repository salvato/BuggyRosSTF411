#include "utility.h"
#include "stm32f4xx_hal.h"
#include "string.h" // for memset()

// System Clock Configuration
//    The system Clock is configured as follow :
//        System Clock source            = PLL (HSI)
//        SYSCLK(Hz)                     = 100000000
//        HCLK(Hz)                       = 100000000
//        AHB Prescaler                  = 1
//        APB1 Prescaler                 = 2
//        APB2 Prescaler                 = 1
//        HSI Frequency(Hz)              = 16000000
//        PLL_M                          = 16
//        PLL_N                          = 400
//        PLL_P                          = 4
//        PLL_Q                          = 7
//        VDD(V)                         = 3.3
//        Main regulator output voltage  = Scale2 mode
//        Flash Latency(WS)              = 3

void
Error_Handler(int error) {
    while(true) {
        HAL_GPIO_TogglePin(Alarm_GPIO_Port, Alarm_GPIO_Pin);
        HAL_Delay(500+error);
    }
}


void
GPIO_Init(void) {
    GPIO_InitTypeDef GPIO_InitStruct;
    memset(&GPIO_InitStruct, 0, sizeof(GPIO_InitStruct));

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();

    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);

    // Push Button
    GPIO_InitStruct.Pin  = B1_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_IT_FALLING;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);
    HAL_NVIC_SetPriority(EXTI15_10_IRQn, 0, 0);

    // Led On Board
    GPIO_InitStruct.Pin   = LD2_Pin;
    GPIO_InitStruct.Mode  = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull  = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(LD2_GPIO_Port, &GPIO_InitStruct);

    // Left Motor IN1
    GPIO_InitStruct.Pin   = LMOTOR_IN1;
    HAL_GPIO_Init(LM_IN1_PORT, &GPIO_InitStruct);

    // Left Motor IN2
    GPIO_InitStruct.Pin   = LMOTOR_IN2;
    HAL_GPIO_Init(LM_IN2_PORT, &GPIO_InitStruct);

    // Right Motor IN3
    GPIO_InitStruct.Pin   = LMOTOR_IN3;
    HAL_GPIO_Init(RM_IN3_PORT, &GPIO_InitStruct);

    // Right Motor IN4
    GPIO_InitStruct.Pin   = LMOTOR_IN4;
    HAL_GPIO_Init(RM_IN4_PORT, &GPIO_InitStruct);

    // Alarm Pin
    GPIO_InitStruct.Pin   = Alarm_GPIO_Pin;
    HAL_GPIO_Init(Alarm_GPIO_Port, &GPIO_InitStruct);
}


void
SystemClock_Config(void) {
    RCC_OscInitTypeDef RCC_OscInitStruct;
    RCC_ClkInitTypeDef RCC_ClkInitStruct;
    memset(&RCC_OscInitStruct, 0, sizeof(RCC_OscInitStruct));
    memset(&RCC_ClkInitStruct, 0, sizeof(RCC_ClkInitStruct));

    __HAL_RCC_PWR_CLK_ENABLE();
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
    RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
    RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
    RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
    RCC_OscInitStruct.PLL.PLLM            = 16;
    RCC_OscInitStruct.PLL.PLLN            = 400;
    RCC_OscInitStruct.PLL.PLLP            = RCC_PLLP_DIV4;
    RCC_OscInitStruct.PLL.PLLQ            = 7;
    if(HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType      = RCC_CLOCKTYPE_HCLK   |
                                       RCC_CLOCKTYPE_SYSCLK |
                                       RCC_CLOCKTYPE_PCLK1  |
                                       RCC_CLOCKTYPE_PCLK2;
    RCC_ClkInitStruct.SYSCLKSource   = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.AHBCLKDivider  = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

    if(HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }
}


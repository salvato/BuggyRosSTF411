#pragma once

#define SLOW_MOTORS

#ifdef __cplusplus
extern "C" {
#endif

#define B1_Pin          GPIO_PIN_13
#define B1_GPIO_Port    GPIOC
#define LD2_Pin         GPIO_PIN_5
#define LD2_GPIO_Port   GPIOA
#define Alarm_GPIO_Pin  GPIO_PIN_10
#define Alarm_GPIO_Port GPIOA

#define LMOTOR_IN1      GPIO_PIN_8
#define LMOTOR_IN2      GPIO_PIN_9
#define LMOTOR_IN3      GPIO_PIN_10
#define LMOTOR_IN4      GPIO_PIN_11

#define LM_IN1_PORT      GPIOC
#define LM_IN2_PORT      GPIOC
#define RM_IN3_PORT      GPIOC
#define RM_IN4_PORT      GPIOC

void Error_Handler(int error=0);
void SystemClock_Config(void);
void GPIO_Init(void);

#ifdef __cplusplus
}
#endif

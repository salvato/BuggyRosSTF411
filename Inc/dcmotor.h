#pragma once

#include <stm32f4xx_hal.h>


class DcMotor
{
public:
    explicit DcMotor(GPIO_TypeDef* _forwardPort, uint16_t _forwardPin,
                     GPIO_TypeDef* _reversePort,  uint16_t _reversePin,
                     GPIO_TypeDef* _pwmPort,  uint16_t _pwmPin,
                     TIM_HandleTypeDef *_hPwmTimer, uint32_t _channel);
    void stop();
    void setSpeed(int speed);

private:
    GPIO_TypeDef* forwardPort;
    GPIO_TypeDef* reversePort;
    GPIO_TypeDef* pwmPort;

    uint16_t pwmPin;
    uint16_t forwardPin;
    uint16_t reversePin;

    TIM_HandleTypeDef* htimPWM;
    uint32_t           channel;
};

#pragma once

#include <stm32f4xx_hal.h>

//#define N_AVG 7

class Encoder
{
public:
    Encoder(TIM_HandleTypeDef* hEncodertimer);
    void    start();
    void    reset();
    double  readAndResetCounts();
    int64_t readTotal();
    void    resetTotal();
    int64_t readAndResetTotal();

private:
    TIM_HandleTypeDef* htimer;
    uint32_t           encoderChannel;
    uint64_t           total;
};

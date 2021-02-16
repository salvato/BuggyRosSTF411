#include "encoder.h"
#include "utility.h"
#include "string.h"


// 12 CPR Quadrature Encoder ??
// Motor Gear Ratio 1:9
// Quadrature encoder mode 3 (x4 mode)


Encoder::Encoder(TIM_HandleTypeDef *hEncodertimer)
    : htimer(hEncodertimer)
    , total(0)
{
    start();
}


void
Encoder::start() {
    if(HAL_TIM_Encoder_Start(htimer, TIM_CHANNEL_ALL))
        Error_Handler();
}


double
Encoder::readAndResetCounts() { // in Counts

#if defined(SLOW_MOTORS)
    int16_t counts = -int16_t(htimer->Instance->CNT);
#else
    int16_t counts = int16_t(htimer->Instance->CNT);
#endif
    htimer->Instance->CNT = 0;
    total += counts;
    return double(counts);
}


void
Encoder::reset() {
    htimer->Instance->CNT = 0;
}


int64_t Encoder::readTotal() {
    return total;
}


void
Encoder::resetTotal() {
    total = 0;
}


int64_t
Encoder::readAndResetTotal() {
    int32_t saveTotal = total;
    total = 0;
    return saveTotal;
}

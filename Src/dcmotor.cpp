#include "dcmotor.h"
#include "string.h" // for memset()
#include "utility.h"


// Caratteristiche Motore:

// Tensione applicabile: 3 - 12 Vdc
// Velocità di rotazione 130RPM @ 12V - ???mA
// Coppia max ???gf @ ?V
// Riduzione 1:9
// Diametro Ruota 69mm

// Resistenza Avvolgimento: ??
// Induttanza Avvolgimento: ??

// http://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling

// (J)     moment of inertia of the rotor     ??.? kg.m^2
// (b)     motor viscous friction constant    ?.?  N.m.s
// (Ke)    electromotive force constant       ?.?? V/rad/sec
// (Kt)    motor torque constant              ?.?? N.m/Amp
// (R)     electric resistance                ?.?  Ohm
// (L)     electric inductance                ?.?  H

DcMotor::DcMotor(GPIO_TypeDef* _forwardPort, uint16_t _forwardPin,
                 GPIO_TypeDef* _reversePort,  uint16_t _reversePin,
                 GPIO_TypeDef* _pwmPort,  uint16_t _pwmPin,
                 TIM_HandleTypeDef* _hPwmTimer, uint32_t _channel)
    : forwardPort(_forwardPort)
    , reversePort(_reversePort)
    , pwmPort(_pwmPort)
    , pwmPin(_pwmPin)
    , forwardPin(_forwardPin)
    , reversePin(_reversePin)
    , htimPWM(_hPwmTimer)
    , channel(_channel)
{
}


void
DcMotor::stop() {
    HAL_GPIO_WritePin(forwardPort, forwardPin, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(reversePort, reversePin, GPIO_PIN_RESET);
    if(channel == TIM_CHANNEL_1)
        htimPWM->Instance->CCR1 = 0;
    else // (channel == TIM_CHANNEL_2)
        htimPWM->Instance->CCR2 = 0;

     HAL_TIM_PWM_Stop(htimPWM, channel);
}


void
DcMotor::setSpeed(int speed) {
    if(speed >= 0) {
        HAL_GPIO_WritePin(forwardPort, forwardPin, GPIO_PIN_SET);
        HAL_GPIO_WritePin(reversePort, reversePin, GPIO_PIN_RESET);
    }
    else { // (speed < 0)
        HAL_GPIO_WritePin(forwardPort, forwardPin, GPIO_PIN_RESET);
        HAL_GPIO_WritePin(reversePort, reversePin, GPIO_PIN_SET);
        speed = -speed;
    }
    if(channel == TIM_CHANNEL_1)
        htimPWM->Instance->CCR1 = speed;
    else // (channel == TIM_CHANNEL_2)
        htimPWM->Instance->CCR2 = speed;

    HAL_TIM_PWM_Start(htimPWM, channel);
}

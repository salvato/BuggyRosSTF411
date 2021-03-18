#pragma once

#include "dcmotor.h"
#include "encoder.h"
#include "PID_v1.h"


class ControlledMotor
{
public:
    ControlledMotor(DcMotor* _pMotor, Encoder* _pEncoder, uint32_t samplingFrequency);
    void Update();
    void setTargetSpeed(double newSpeed); // in m/s
    void setPID(double p, double i, double d);
    void setP(double value);
    void setI(double value);
    void setD(double value);
    void Stop();
    double getCurrentSpeed(); // in m/s
    double spaceTraveled(); // in m
    void resetPosition();

public:

private:
    DcMotor* pMotor;
    Encoder* pEncoder;
    PID*     pPID;

private:
    double currentSpeed; // in Encoder_Counts/sec
    double output;
    double setpoint;
    double sampleTime;
    double Kp;
    double Ki;
    double Kd;
    int    POn;
};

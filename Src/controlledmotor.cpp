#include "controlledmotor.h"


extern double encoderCountsPerMeter; // defined in main.cpp


ControlledMotor::ControlledMotor(DcMotor *_pMotor, Encoder *_pEncoder, uint32_t samplingFrequency)
    : pMotor(_pMotor)
    , pEncoder(_pEncoder)
    , sampleTime(1.0/double(samplingFrequency))// in sec.
    , Kp(0.43)
    , Ki(0.2)
    , Kd(0.1)
    , POn(PID::P_ON_E)
{
    output       = 0.0; // in PWM units
    currentSpeed = 0.0; // in Counts/sec
    setpoint     = 0.0; // in Counts/sec

//        PID::PID(double* Input, double* Output, double* Setpoint,
//                 double Kp, double Ki, double Kd,
//                 int POn, int ControllerDirection)
    pPID = new PID(&currentSpeed, &output, &setpoint,
                   Kp, Ki, Kd,
                   PID::P_ON_E, PID::DIRECT);

    pPID->SetOutputLimits(-255.0, 255.0);
    pPID->SetMode(PID::AUTOMATIC);
}


// Must be called regularly to maintain the programmed speed
void
ControlledMotor::Update() {
    currentSpeed = pEncoder->readAndResetCounts()/sampleTime; // in Counts/sec
    if(pPID->Compute())
        pMotor->setSpeed(output);// Update new speed
}


// Speed in m/s
void
ControlledMotor::setTargetSpeed(double newSpeed) {
    // Calculate setpoint as counts/sec
    setpoint = newSpeed*encoderCountsPerMeter;
    pPID->SetMode(PID::AUTOMATIC);
}


void
ControlledMotor::setPID(double p, double i, double d) {
    Kp = p;
    Ki = i;
    Kd = d;
    pPID->SetTunings(Kp, Ki, Kd, POn);
}


void
ControlledMotor::setP(double value) {
    Kp = value;
    pPID->SetTunings(Kp, Ki, Kd, POn);
}


void
ControlledMotor::setI(double value){
    Ki = value;
    pPID->SetTunings(Kp, Ki, Kd, POn);
}


void
ControlledMotor::setD(double value){
    Kd = value;
    pPID->SetTunings(Kp, Ki, Kd, POn);
}


void
ControlledMotor::Stop() {
    pMotor->stop();
    setpoint = 0.0;
    pPID->SetMode(PID::MANUAL);
}


double
ControlledMotor::getCurrentSpeed() { // in m/s
    return currentSpeed/encoderCountsPerMeter;
}


double
ControlledMotor::spaceTraveled () { // in [m]
    return double(pEncoder->readAndResetTotal())/encoderCountsPerMeter;
}

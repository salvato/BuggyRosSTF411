/*
// To configure QtCreator in order to Debug and Run programs
// on STM32F boards see:
//
// https://github.com/nlhans/qt-baremetal
// https://electronics.stackexchange.com/questions/212018/debugging-an-arm-stm32-microcontroller-using-qt-creator
//
//
// Motor Gear 9:1
//
// Tire's Diameter 0.069 [m]
// Encoder Counts per Tire Turn = 12.0*9.0*4.0 = 432 ==>
// Encoder Counts per [m] = 1993
//
// Tire's Distance 0.2 [m]

//=========================================
// On Board Peripherals
//=========================================
// PA5  (CN10 11)   ------> Led On Board
// PC13 (CN7  23)   ------> Button On Board


//===================================
// USART2 GPIO Configuration
//===================================
// GND (CN10 20)    ------> GND
// PA2 (CN10 35)    ------> USART2_TX
// PA3 (CN10 37)    ------> USART2_RX


//===================================
// I2C2 GPIO Configuration
//===================================
// VIN  (CN7  16)    ------> +3.3V
// GND  (CN7  20)    ------> GND
// PB10 (CN10 25)    ------> I2C2_SCL
// PB3  (CN10 31)    ------> I2C2_SDA


//================================================
// TIM1 (32 Bit) GPIO Configuration (Left Encoder)
//================================================
// PA8 (CN10 21)    ------> TIM1_CH1
// PA9 (CN10 23)    ------> TIM1_CH2


//=================================================
// TIM4 (32 Bit) GPIO Configuration (Right Encoder)
//=================================================
// PB6 (CN10 17)    ------> TIM4_CH1
// PB7 (CN7  21)    ------> TIM4_CH2


//=============================================
// TIM2 GPIO Configuration (Periodic Interrupt)
//=============================================
// Channel2         ------> Motors Sampling
// Channel3         ------> Sonar Sampling
// Channel4         ------> IMU Sampling


//==================================
// TIM3 GPIO Configuration (PWM)
//==================================
// PA6 (CN10 13)    ------> TIM3_CH1
// PA7 (CN10 15)    ------> TIM3_CH2


//==========================================
// TIM5 GPIO Configuration (Sonar)
//==========================================
// PA0 (CN7 - 28)  ------> TIM3_CH1 (Pulse)
// PA1 (CN7 - 30)  ------> TIM3_CH2 (Echo)


//====================================
// Left Motor Direction Pins
// PC8  (CN10  2)    ------> LM298 IN1
// PC9  (CN10  1)    ------> LM298 IN2
//====================================


//====================================
// Right Motor Direction Pins
// PC10 (CN7  1)    ------> LM298 IN3
// PC11 (CN7  2)    ------> LM298 IN4
//====================================

//====================================
// Used Peripherals:
// Timers:
//     TIM1 ---> Encoder (Left)
//     TIM2 ---> Periodic Interrupt
//     TIM3 ---> PWM (Motors)
//     TIM4 ---> Encoder (Right)
//     TIM9 ---> Ultrasound Sensor
//====================================

//=============================================================================
// ROS information:
// http://wiki.ros.org/navigation/Tutorials/RobotSetup
//=============================================================================
*/

#include "main.h"
#include "tim.h"
#include "i2c.h"
#include "uart.h"
#include "utility.h"
#include "encoder.h"
#include "dcmotor.h"
#include "PID_v1.h"
#include "controlledmotor.h"
#include "ADXL345.h"
#include "ITG3200.h"
#include "mpu6050.h"
#include "HMC5883L.h"
#include "MadgwickAHRS.h"
#include "string.h" // for memset()
#include "stdio.h"
// ROS includes
#include <ros.h>
#include <tf/tf.h>
#include <geometry_msgs/Twist.h>   // Received Speed Data
#include <geometry_msgs/Vector3.h> // Received PID Values
#include <nav_msgs/Odometry.h>     // Published Robot Odometry
#include <sensor_msgs/Imu.h>       // Published IMU Data
#include <sensor_msgs/Range.h>     // Published Sonar Data
#include <sensor_msgs/MagneticField.h> // Published Compass Data
#include <std_msgs/UInt8.h>


#ifndef M_PI
    #define M_PI 3.14159265358979323846
#endif
#define DEG2RAD(x) (x)*M_PI/180.0

//#define USE_SONAR
//#define USE_IMU_MPU6050

#define MOTOR_UPDATE_CHANNEL    HAL_TIM_ACTIVE_CHANNEL_2
#define MOTOR_CHANNEL           TIM_CHANNEL_2

#define ODOMETRY_UPDATE_CHANNEL HAL_TIM_ACTIVE_CHANNEL_3
#define ODOMETRY_CHANNEL        TIM_CHANNEL_3

#define SONAR_UPDATE_CHANNEL    ODOMETRY_UPDATE_CHANNEL
#define SONAR_CHANNEL           ODOMETRY_CHANNEL

#define IMU_UPDATE_CHANNEL      HAL_TIM_ACTIVE_CHANNEL_4
#define IMU_CHANNEL             TIM_CHANNEL_4

#define SAMPLING_IRQ            TIM2_IRQn


///==============================
/// Private function prototypes
///==============================
static void Setup();
static void Loop();
static void Init_Hardware();
static bool IMU_Init();
static void Init_ROS();
static void resetOdometry();
static bool updateOdometry();
static void calibrateIMU();
static void calculateVariance(float* data, double* var, int nData);


///================
/// ROS callbacks
///================
static void targetSpeed_cb(const geometry_msgs::Twist& speed);
static void left_PID_cb(const geometry_msgs::Vector3& msg);
static void right_PID_cb(const geometry_msgs::Vector3& msg);
static void calibrateIMU_cb(const std_msgs::UInt8& msg);
static void resetOdometry_cb(const std_msgs::UInt8& msg);


///===================
/// Hardware Handles
///===================
TIM_HandleTypeDef  hSamplingTimer;     // Periodic Sampling Timer
TIM_HandleTypeDef  hLeftEncoderTimer;  // Left Motor Encoder Timer
TIM_HandleTypeDef  hRightEncoderTimer; // Right Motor Encoder Timer
TIM_HandleTypeDef  hPwmTimer;          // Dc Motors PWM (Speed control)
TIM_HandleTypeDef  hSonarEchoTimer;    // To Measure the Radar Echo Pulse Duration
TIM_HandleTypeDef  hSonarPulseTimer;   // To Generate the Radar Trigger Pulse

I2C_HandleTypeDef  hi2c2;

UART_HandleTypeDef huart2;
DMA_HandleTypeDef  hdma_usart2_tx;
DMA_HandleTypeDef  hdma_usart2_rx;


///==================
/// Buggy Mechanics
///==================
static const double WHEEL_DIAMETER               = 0.067;  // [m]
static const double TRACK_LENGTH                 = 0.209;    // Tire's Distance [m]
#if defined(SLOW_MOTORS)
    static const int    ENCODER_COUNTS_PER_TIRE_TURN = 12*9*10*4; // Slow Motors !!!
#else
    static const int    ENCODER_COUNTS_PER_TIRE_TURN = 12*9*4; // = 432 ==>
#endif


///===================
/// Shared variables
///===================

unsigned int baudRate = 921600; /// We can try greater speeds...Require changing the value in:
///                                 /opt/ros/noetic/lib/rosserial_python/serial_node.py

double periodicClockFrequency = 10.0e6;// 10MHz
double pwmClockFrequency      = 3.0e4; // 30KHz (corresponding to ~120Hz PWM Period)

double sonarClockFrequency    = 10.0e6;  // 10MHz (100ns period)
double sonarPulseDelay        = 10.0e-6; // in seconds
double sonarPulseWidth        = 10.0e-6; // in seconds
double soundSpeed             = 340.0;   // in m/s

double encoderCountsPerMeter  = ENCODER_COUNTS_PER_TIRE_TURN/(M_PI*WHEEL_DIAMETER);


///====================
/// Private variables
///====================
Encoder*         pLeftEncoder          = nullptr;
Encoder*         pRightEncoder         = nullptr;
DcMotor*         pLeftMotor            = nullptr;
DcMotor*         pRightMotor           = nullptr;
ControlledMotor* pLeftControlledMotor  = nullptr;
ControlledMotor* pRightControlledMotor = nullptr;

/// Inital values for PIDs (externally estimated)
const double LeftP  = 0.101;
const double LeftI  = 0.008;
const double LeftD  = 0.003;
const double RightP = 0.101;
const double RightI = 0.008;
const double RightD = 0.003;

MPU6050 mpu6050;
MPU6050::MPU6050_t mpuData;

ADXL345  Acc;      /// 400KHz I2C Capable. Maximum Output Data Rate is 800 Hz
ITG3200  Gyro;     /// 400KHz I2C Capable
HMC5883L Magn;     /// 400KHz I2C Capable, left at the default 15Hz data Rate
Madgwick Madgwick; /// ~13us per Madgwick.update() with NUCLEO-F411RE
float qw,  qx,  qy,  qz;
float MagValues[3]   = {0.0};
float AccelValues[3] = {0.0};
float GyroValues[3]  = {0.0};
double odom_pose[3]  = {0.0};


///===================
/// Update Intervals
///===================
uint32_t IMUSamplingFrequency    = 400; // [Hz]
uint32_t IMUSamplingPulses       = uint32_t(periodicClockFrequency/IMUSamplingFrequency +0.5); // [Hz]
uint32_t motorSamplingFrequency  = 100;  // [Hz]
uint32_t motorSamplingPulses     = uint32_t(periodicClockFrequency/motorSamplingFrequency+0.5); // [Hz]
uint32_t odometryUpdateFrequency = 4*30;  // [Hz]
uint32_t odometrySamplingPulses  = uint32_t(periodicClockFrequency/odometryUpdateFrequency+0.5); // [Hz]
uint32_t sonarSamplingFrequency  = odometryUpdateFrequency;  // [Hz] (Max 40Hz)
uint32_t sonarSamplingPulses     = uint32_t(periodicClockFrequency/sonarSamplingFrequency+0.5);


bool isIMUpresent     = false;
bool isMPU6050present = false;

bool isTimeToUpdateSonar    = false;
bool isTimeToUpdateOdometry = false;

/// Captured Values for Sonar Echo Width Calculation
volatile uint32_t uwIC2Value1    = 0;
volatile uint32_t uwIC2Value2    = 0;
volatile uint32_t uwDiffCapture  = 0;
volatile uint16_t uhCaptureIndex = 0;

double leftTargetSpeed  = 0.0;
double rightTargetSpeed = 0.0;


///=============
/// ROS stuff
///=============
ros::NodeHandle nh;

ros::Time last_cmd_vel_time;

nav_msgs::Odometry odom;
ros::Publisher odom_pub("odom", &odom);

sensor_msgs::Imu imuData;
sensor_msgs::MagneticField compassData;
ros::Publisher imu_pub("imu_data", &imuData);
ros::Publisher mag_pub("mag_data", &compassData);

# if defined(USE_IMU_MPU6050)
    TODO:
#endif

#if defined(USE_SONAR)
    sensor_msgs::Range obstacleDistance;
    ros::Publisher obstacleDistance_pub("buggyDistance", &obstacleDistance);
#endif

ros::Subscriber<geometry_msgs::Twist>   targetSpeed_sub("cmd_vel", &targetSpeed_cb);
ros::Subscriber<geometry_msgs::Vector3> left_PID_sub("leftPID", &left_PID_cb);
ros::Subscriber<geometry_msgs::Vector3> right_PID_sub("rightPID", &right_PID_cb);
ros::Subscriber<std_msgs::UInt8>        calibrate_IMU_sub("recalibrate_imu", &calibrateIMU_cb);
ros::Subscriber<std_msgs::UInt8>        reset_odometry_sub("reset_odometry", &resetOdometry_cb);

///=======================================================================
///                                Main
///=======================================================================
int
main(void) {
    Setup();
    while(true) {
        Loop();
    }
}


///=======================================================================
///                                Init
///=======================================================================
static void
Setup() {
    Init_Hardware();
    Init_ROS();
    HAL_NVIC_EnableIRQ(EXTI15_10_IRQn);
}


///=======================================================================
///    Main Loop
///=======================================================================

int nn = 0;

static void
Loop() {
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    /// Wait until Serial Node is Up and Ready
    while(!nh.connected()) {
        nh.spinOnce();
        HAL_Delay(10);
    }

    /// Now Serial Node is Up and Ready
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_SET);
    resetOdometry();
    calibrateIMU();
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    HAL_Delay(1000);

    // Start the Periodic Sampling:
    nh.loginfo("Buggy Ready...");

    last_cmd_vel_time = nh.now();
    nn = 0;
    HAL_NVIC_EnableIRQ(SAMPLING_IRQ);
    // Start the Periodic Sampling Counters:
    HAL_TIM_OC_Start_IT(&hSamplingTimer, IMU_CHANNEL);      // IMU
    HAL_TIM_OC_Start_IT(&hSamplingTimer, MOTOR_CHANNEL);    // Motors
    HAL_TIM_OC_Start_IT(&hSamplingTimer, ODOMETRY_CHANNEL); // Odometry & Sonar
    // Enable and set Button EXTI Interrupt
    while(nh.connected()) {
        if(isTimeToUpdateOdometry) {
            isTimeToUpdateOdometry = false;
            nn += 1;
            if(nn == 1) {
                HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
                updateOdometry();
                odom_pub.publish(&odom);
            }
            else if(nn == 2) {
                if(isIMUpresent) {
                    imu_pub.publish(&imuData);
                }
            }
            else if(nn == 3) {
                if(isIMUpresent) {
                    mag_pub.publish(&compassData);
                }
            }
            else {
                nn = 0;
                if(isMPU6050present) {
                    imuData.linear_acceleration.x = mpuData.Ax;
                    imuData.linear_acceleration.y = mpuData.Ay;
                    imuData.linear_acceleration.z = mpuData.Az;
                    imuData.angular_velocity.x    = mpuData.Gx;
                    imuData.angular_velocity.y    = mpuData.Gy;
                    imuData.angular_velocity.z    = mpuData.Gz;
                    //TODO Prepare MPU6050 Data to send
                }
            }
        }
#if defined(USE_SONAR)
        if(isTimeToUpdateSonar) {
            isTimeToUpdateSonar = false;
            HAL_NVIC_DisableIRQ(SAMPLING_IRQ);
            obstacleDistance.range = 0.5 * soundSpeed*(double(uwDiffCapture)/sonarClockFrequency); // [m]
            HAL_NVIC_EnableIRQ(SAMPLING_IRQ);
            obstacleDistance.header.stamp = nh.now();
            obstacleDistance.radiation_type = obstacleDistance.ULTRASOUND;
            obstacleDistance_pub.publish(&obstacleDistance);
            //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        }
#endif
        /// If No New Speed Data have been Received in the Right Time
        /// Halt the Robot to avoid possible damages
        if((nh.now()-last_cmd_vel_time).toSec() > 0.5) {
            leftTargetSpeed  = 0.0; // in m/s
            rightTargetSpeed = 0.0; // in m/s
        }
        pLeftControlledMotor->setTargetSpeed(leftTargetSpeed);
        pRightControlledMotor->setTargetSpeed(rightTargetSpeed);
        nh.spinOnce();
    } // while(nh.connected()

    /// Serial Node Disconnected
    HAL_GPIO_WritePin(LD2_GPIO_Port, LD2_Pin, GPIO_PIN_RESET);
    leftTargetSpeed  = 0.0; // in m/s
    rightTargetSpeed = 0.0; // in m/s
    pLeftControlledMotor->setTargetSpeed(leftTargetSpeed);
    pRightControlledMotor->setTargetSpeed(rightTargetSpeed);

    // Stop the Periodic Sampling:
    // Start the Periodic Sampling Counters:
    HAL_TIM_OC_Stop_IT(&hSamplingTimer, IMU_CHANNEL);      // IMU
    HAL_TIM_OC_Stop_IT(&hSamplingTimer, MOTOR_CHANNEL);    // Motors
    HAL_TIM_OC_Stop_IT(&hSamplingTimer, ODOMETRY_CHANNEL); // Odometry & Sonar
    // Enable and set Button EXTI Interrupt
    HAL_NVIC_DisableIRQ(SAMPLING_IRQ);
}
///=======================================================================
///    End Loop
///=======================================================================


//=========================
// Calculate the odometry
//=========================
bool
updateOdometry() {
    /// Wheel's Path Length in the Sampling Interval
    double wheel_l = pLeftControlledMotor->spaceTraveled();
    double wheel_r = pRightControlledMotor->spaceTraveled();
    odom.header.stamp = nh.now();
    /// Robot's Axis Rotation in the Sampling Interval
    double delta_theta = (wheel_r-wheel_l)/TRACK_LENGTH;
    /// Compute Updated Odometric Pose (approximated via Runge Kutta 2nd Order Integration)
    double delta_s = 0.5*(wheel_r+wheel_l);
    odom_pose[0] += delta_s * cos(odom_pose[2]+(0.5*delta_theta));
    odom_pose[1] += delta_s * sin(odom_pose[2]+(0.5*delta_theta));
    odom_pose[2] += delta_theta;
    odom.pose.pose.position.x = odom_pose[0];
    odom.pose.pose.position.y = odom_pose[1];
    /// Compute Updated Odometric Orientation
    odom.pose.pose.orientation = tf::createQuaternionFromYaw(odom_pose[2]);
    /// Update the Odometry Twist (Instantaneouse Velocities)
    odom.twist.twist.linear.x  = delta_s * odometryUpdateFrequency;     // v
    // Only for debugging
    ///    odom.twist.twist.linear.y  = odom_pose[2];
    ///    odom.twist.twist.linear.y  = wheel_l;
    ///    odom.twist.twist.linear.z  = wheel_r;
    odom.twist.twist.angular.z = delta_theta * odometryUpdateFrequency; // w

    return true;
}


static void
resetOdometry() {
    pLeftControlledMotor->resetPosition();
    pRightControlledMotor->resetPosition();
}


void
Init_Hardware() {
    HAL_Init();           // Initialize the HAL Library
    SystemClock_Config(); // Initialize System Clock
    GPIO_Init();          // Initialize On Board Peripherals
    SerialPortInit();     // Initialize the Serial Communication Port (/dev/ttyACM0)

    // Initialize Encoders
    LeftEncoderTimerInit();
    RightEncoderTimerInit();
    pLeftEncoder  = new Encoder(&hLeftEncoderTimer);
    pRightEncoder = new Encoder(&hRightEncoderTimer);

    // Initialize Dc Motors
    PwmTimerInit();

    // =============> DcMotor(forwardPort, forwardPin, reversePort,  reversePin,
    //                        pwmPort,  pwmPin, pwmTimer, timerChannel)
#if defined(SLOW_MOTORS)
    pLeftMotor  = new DcMotor(GPIOC, GPIO_PIN_9, GPIOC, GPIO_PIN_8,
    GPIOA, GPIO_PIN_6, &hPwmTimer, TIM_CHANNEL_1);
    pRightMotor = new DcMotor(GPIOC, GPIO_PIN_11, GPIOC, GPIO_PIN_10,
    GPIOA, GPIO_PIN_7, &hPwmTimer, TIM_CHANNEL_2);
#else
    pLeftMotor  = new DcMotor(GPIOC, GPIO_PIN_8, GPIOC, GPIO_PIN_9,
    GPIOA, GPIO_PIN_6, &hPwmTimer, TIM_CHANNEL_1);
    pRightMotor = new DcMotor(GPIOC, GPIO_PIN_10, GPIOC, GPIO_PIN_11,
    GPIOA, GPIO_PIN_7, &hPwmTimer, TIM_CHANNEL_2);
#endif

    /// Initialize Motor Controllers
    pLeftControlledMotor  = new ControlledMotor(pLeftMotor,  pLeftEncoder,  motorSamplingFrequency);
    pRightControlledMotor = new ControlledMotor(pRightMotor, pRightEncoder, motorSamplingFrequency);
    pLeftControlledMotor->setPID(LeftP, LeftI, LeftD);
    pRightControlledMotor->setPID(RightP, RightI, RightD);

    /// Initialize IMU's (if presents)
    I2C2_Init();
    isIMUpresent     = IMU_Init();// Initialize 10DOF Sensor
    isMPU6050present = mpu6050.Init(&hi2c2);

#if defined (USE_SONAR)
    // Initialize Sonar
    SonarEchoTimerInit();
    SonarPulseTimerInit();
#endif

    // Initialize Periodic Samplig Timer
    SamplingTimerInit(IMUSamplingPulses, motorSamplingPulses, odometrySamplingPulses);
}


static void
calculateVariance(float* data, double* var, int nData) {
    double avg[3] = {0.0};
    int j;
    for(int i=0; i<nData; i++) {
        j = 3*i;
        avg[0] += data[j];
        avg[1] += data[j+1];
        avg[2] += data[j+2];
    }
    avg[0] /= double(nData);
    avg[1] /= double(nData);
    avg[2] /= double(nData);
    var[0] = var[1] = var[2] = 0.0;
    for(int i=0; i<nData; i++) {
        j = 3*i;
        var[0] += (data[j]  -avg[0])*(data[j]  -avg[0]);
        var[1] += (data[j+1]-avg[1])*(data[j+1]-avg[1]);
        var[2] += (data[j+2]-avg[2])*(data[j+2]-avg[2]);
    }
    var[0] /= double(nData);
    var[1] /= double(nData);
    var[2] /= double(nData);
}


bool
IMU_Init() {
    if(!Acc.init(ADXL345_ADDR_ALT_LOW, &hi2c2))
        return false;

    if(!Gyro.init(ITG3200_ADDR_AD0_LOW, &hi2c2))
        return false;
    HAL_Delay(100);
     // Calibrate the ITG3200 (Assuming a STATIC SENSOR !)
    Gyro.zeroCalibrate(1200);

    if(!Magn.init(HMC5883L_Address, &hi2c2))
        return false;
    HAL_Delay(100);
    if(Magn.SetScale(1300) != 0)
        return false;
    HAL_Delay(100);
    if(Magn.SetMeasurementMode(Measurement_Continuous) != 0)
        return false;

    Madgwick.begin(float(IMUSamplingFrequency));
    /// Since we Assume that the Buggy is stationary...
    GyroValues[0] = GyroValues[1] = GyroValues[2] = 0.0;
    //while(!Gyro.isRawDataReadyOn()) {}
    //Gyro.readGyro(GyroValues);
    while(!Acc.getInterruptSource(7)) {}
    Acc.get_Gxyz(AccelValues);
    while(!Magn.isDataReady()) {}
    Magn.ReadScaledAxis(MagValues);

    /// Since we start in an arbitrary orientation we have to converge to
    /// the initial estimate of the attitude (Assuming a Static Sensor !)
    for(int i=0; i<20000; i++) { // ~13us per Madgwick.update() with NUCLEO-F411RE
        Madgwick.update(GyroValues, AccelValues, MagValues);
    }

    return true;
}


void
Init_ROS() {
    // TODO: Assign more realistic values for each quantity
    double pcov[36] = { 0.1, 0.0, 0.0,   0.0,   0.0,   0.0,
                        0.0, 0.1, 0.0,   0.0,   0.0,   0.0,
                        0.0, 0.0, 1.0e6, 0.0,   0.0,   0.0, // Z axis not valid
                        0.0, 0.0, 0.0,   1.0e6, 0.0,   0.0, // Pitch and ...
                        0.0, 0.0, 0.0,   0.0,   1.0e6, 0.0, // Roll not valid
                        0.0, 0.0, 0.0,   0.0,   0.0,   0.02};

    // Values that Never Change
    memcpy(&(odom.pose.covariance),  pcov, sizeof(double)*36);
    memcpy(&(odom.twist.covariance), pcov, sizeof(double)*36);
    odom.pose.pose.position.z = 0.0;
    odom.twist.twist.linear.y = 0.0;
    odom.twist.twist.linear.z = 0.0;
    odom.header.frame_id = {"odom"};
    odom.child_frame_id = {"base_link"};

    imuData.header.frame_id = {"imu_link"};

    nh.initNode();

    if(!nh.subscribe(targetSpeed_sub))
        Error_Handler();
    if(!nh.subscribe(left_PID_sub))
        Error_Handler();
    if(!nh.subscribe(right_PID_sub))
        Error_Handler();

    if(!nh.advertise(odom_pub))
        Error_Handler();
    if(!nh.advertise(imu_pub))
        Error_Handler();
    if(!nh.advertise(mag_pub))
        Error_Handler();
#if defined (USE_SONAR)
    if(!nh.advertise(obstacleDistance_pub))
        Error_Handler();
#endif
}


// Initializes the Global MCU specific package (MSP).
void
HAL_MspInit(void) {
    __HAL_RCC_SYSCFG_CLK_ENABLE();
    __HAL_RCC_PWR_CLK_ENABLE();
    HAL_NVIC_SetPriorityGrouping(NVIC_PRIORITYGROUP_0);
}


static void
resetOdometry_cb(const std_msgs::UInt8& msg) {
    (void)msg;
    resetOdometry();
}


static void
calibrateIMU_cb(const std_msgs::UInt8& msg) {
    (void)msg;
    leftTargetSpeed  = 0.0; // in m/s
    rightTargetSpeed = 0.0; // in m/s
    last_cmd_vel_time = nh.now();
    pLeftControlledMotor->setTargetSpeed(leftTargetSpeed);
    pRightControlledMotor->setTargetSpeed(rightTargetSpeed);
    HAL_Delay(1000); // Ensure a Stationary Buggy
    bool bSamplingActive = NVIC_GetEnableIRQ(SAMPLING_IRQ) != 0;
    if(bSamplingActive) {
        HAL_NVIC_DisableIRQ(SAMPLING_IRQ);
    }
    calibrateIMU();
    if(bSamplingActive) {
        HAL_NVIC_EnableIRQ(SAMPLING_IRQ);
    }
}


// The received LinearTarget speed is in m/s
// The received AngularTarget speed is in rad/s
static void
targetSpeed_cb(const geometry_msgs::Twist& speed) {
    double angSpeed  = speed.angular.z*TRACK_LENGTH*0.5; // Vt = Omega * R
    leftTargetSpeed  = speed.linear.x - angSpeed;        // in m/s
    rightTargetSpeed = speed.linear.x + angSpeed;        // in m/s
    last_cmd_vel_time = nh.now();
}


static void
left_PID_cb(const geometry_msgs::Vector3& msg) {
    pLeftControlledMotor->setPID(msg.x, msg.y, msg.z);
}


static void
right_PID_cb(const geometry_msgs::Vector3& msg) {
    pRightControlledMotor->setPID(msg.x, msg.y, msg.z);
}


static void
calibrateIMU() {
    int nData = 100;
    float data[3*100] = {0.0};
    double variance[3];
    int j;
    for(int i=0; i<nData; i++) {
        j = 3 * i;
        while(!Acc.getInterruptSource(7)) {}
        Acc.get_Gxyz(&data[j]);
        nh.spinOnce();
    }
    calculateVariance(data, variance, nData);
    memset(imuData.linear_acceleration_covariance,
           0,
           sizeof(imuData.linear_acceleration_covariance));
    imuData.linear_acceleration_covariance[0] = variance[0];
    imuData.linear_acceleration_covariance[4] = variance[1];
    imuData.linear_acceleration_covariance[8] = variance[2];
    nh.loginfo("imuData.linear_acceleration_covariance Calculated...");

    for(int i=0; i<nData; i++) {
        j = 3 * i;
        while(!Gyro.isRawDataReadyOn()) {}
        Gyro.readGyro(&data[j]);
        nh.spinOnce();
    }
    calculateVariance(data, variance, nData);
    memset(imuData.angular_velocity_covariance, 0, sizeof(imuData.angular_velocity_covariance));
    imuData.angular_velocity_covariance[0] = variance[0];
    imuData.angular_velocity_covariance[4] = variance[1];
    imuData.angular_velocity_covariance[8] = variance[2];
    nh.loginfo("imuData.angular_velocity_covariance Calculated...");

    for(int i=0; i<nData; i++) {
        while(!Gyro.isRawDataReadyOn()) {}
        Gyro.readGyro(GyroValues);
        while(!Acc.getInterruptSource(7)) {}
        Acc.get_Gxyz(AccelValues);
        while(!Magn.isDataReady()) {}
        Magn.ReadScaledAxis(MagValues);
        Madgwick.update(GyroValues, AccelValues, MagValues);
        j = 3 * i;
        data[j]   = Madgwick.getRollRadians();
        data[j+1] = Madgwick.getPitchRadians();
        data[j+2] = Madgwick.getYawRadians();
        nh.spinOnce();
    }
    calculateVariance(data, variance, nData);
    memset(imuData.orientation_covariance,
           0,
           sizeof(imuData.orientation_covariance));
    imuData.orientation_covariance[0] = variance[0];
    imuData.orientation_covariance[4] = variance[1];
    imuData.orientation_covariance[8] = variance[2];
    nh.loginfo("imuData.orientation_covariance Calculated...");

    Madgwick.getRotation(&qw, &qx, &qy, &qz);
    imuData.orientation.x = qx;
    imuData.orientation.y = qy;
    imuData.orientation.z = qz;
    imuData.orientation.w = qw;

    for(int i=0; i<nData; i++) {
        Magn.ReadScaledAxis(MagValues);
        j = 3 * i;
        data[j]   = MagValues[0];
        data[j+1] = MagValues[1];
        data[j+2] = MagValues[2];
        nh.spinOnce();
    }
    calculateVariance(data, variance, nData);
    memset(compassData.magnetic_field_covariance,
           0,
           sizeof(compassData.magnetic_field_covariance));
    compassData.magnetic_field_covariance[0] = variance[0];
    compassData.magnetic_field_covariance[4] = variance[1];
    compassData.magnetic_field_covariance[8] = variance[2];
    nh.loginfo("compassData.magnetic_field_covariance Calculated...");
}


// To handle the TIM2 (Periodic Sampler) interrupt.
void
TIM2_IRQHandler(void) {
    // Will call HAL_TIM_OC_DelayElapsedCallback(&hSamplingTimer)
    HAL_TIM_IRQHandler(&hSamplingTimer);
}


// To Restart Periodic Timers
void
HAL_TIM_OC_DelayElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim->Instance == hSamplingTimer.Instance) {
        if(htim->Channel == MOTOR_UPDATE_CHANNEL) { // Time to Update Motors Data ? (50Hz)
            htim->Instance->CCR2 += motorSamplingPulses;
            if(pLeftControlledMotor) {
                pLeftControlledMotor->Update();
            }
            if(pRightControlledMotor) {
                pRightControlledMotor->Update();
            }
        }
        else if(htim->Channel == IMU_UPDATE_CHANNEL) { // Time to Update IMU Data ? (400Hz)
            htim->Instance->CCR4 += IMUSamplingPulses;
            if(isMPU6050present)
                mpu6050.Read_All(&hi2c2, &mpuData);
            if(isIMUpresent) {
                Acc.get_Gxyz(AccelValues);
                Gyro.readGyro(GyroValues);
                Magn.ReadScaledAxis(MagValues);
                Madgwick.update(GyroValues, AccelValues, MagValues); // ~13us
                Madgwick.getRotation(&qw, &qx, &qy, &qz);
                imuData.orientation.w = qw;
                imuData.orientation.x = qy;
                imuData.orientation.y = qx;
                imuData.orientation.z = qz;
                compassData.header.stamp = nh.now();
                compassData.magnetic_field.x = MagValues[0];
                compassData.magnetic_field.y = MagValues[1];
                compassData.magnetic_field.z = MagValues[2];
                // Convert accel from g to m/sec^2
                imuData.linear_acceleration.x = AccelValues[0]* 9.80665 ;
                imuData.linear_acceleration.y = AccelValues[1]* 9.80665 ;
                imuData.linear_acceleration.z = AccelValues[2]* 9.80665 ;
                // Convert gyroscope from degrees/sec to radians/sec
                imuData.angular_velocity.x = DEG2RAD(GyroValues[0]);
                imuData.angular_velocity.y = DEG2RAD(GyroValues[1]);
                imuData.angular_velocity.z = DEG2RAD(GyroValues[2]);
                imuData.header.stamp = nh.now();
                //HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            }
        }
        else if(htim->Channel == ODOMETRY_UPDATE_CHANNEL) { // Time to Update Odometry
            htim->Instance->CCR3 += odometrySamplingPulses;
#if defined(USE_SONAR)
            uhCaptureIndex = 0;
            LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
            LL_TIM_EnableCounter(hSonarPulseTimer.Instance);
#endif
            isTimeToUpdateOdometry = true; // We use this Timer to send the updated Odometry
        }
    } // if(htim->Instance == hSamplingTimer.Instance)
}


#if defined(USE_SONAR)
// To handle the TIM5 (Sonar Echo) interrupt.
// Will call HAL_TIM_IC_CaptureCallback(&hSonarEchoTimer)
void
TIM5_IRQHandler(void) {
    HAL_TIM_IRQHandler(&hSonarEchoTimer);
}


// Called when the Sonar Echo Signal Change Level
void
HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) {
    if(uhCaptureIndex == 0) { // Get the 1st Input Capture value
        // Select the next edge of the active transition on the TI2 channel: falling edge
        LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_FALLING);
        uwIC2Value1 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        uhCaptureIndex = 1;
    }
    else if(uhCaptureIndex == 1) { // Get the 2nd Input Capture value
        // Select the next edge of the active transition on the TI2 channel: rising edge
        LL_TIM_IC_SetPolarity(TIM5, LL_TIM_CHANNEL_CH2, LL_TIM_IC_POLARITY_RISING);
        uwIC2Value2 = HAL_TIM_ReadCapturedValue(htim, TIM_CHANNEL_2);
        /// Echo duration computation
        if(uwIC2Value2 >= uwIC2Value1) {
            uwDiffCapture = (uwIC2Value2 - uwIC2Value1);
        }
        else if(uwIC2Value2 < uwIC2Value1) { // 0xFFFFFFFF is max TIM5_CCRx value
            uwDiffCapture = ((0xFFFFFFFF - uwIC2Value1) + uwIC2Value2) + 1;
        }
        uhCaptureIndex = 0;
        isTimeToUpdateSonar = true;
    }
}
#endif


// To handle the EXTI line[15:10] interrupts.
void
EXTI15_10_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    HAL_GPIO_EXTI_IRQHandler(B1_Pin);
}


// EXTI line[15:10] interrupts handler
void
HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    if(GPIO_Pin == B1_Pin) {

    }
}


// Tx Transfer completed callback
void
HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart;
    nh.getHardware()->flush();
}


// Rx Transfer completed callback
void
HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    (void)huart;
    nh.getHardware()->reset_rbuf();
}


// To handle DMA TX interrupt request.
void
USART2_DMA_TX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmatx);
}


// To handle DMA RX interrupt request.
void
USART2_DMA_RX_IRQHandler(void) {
    HAL_DMA_IRQHandler(huart2.hdmarx);
}


// To handle USART2 interrupt request.
void
USART2_IRQHandler(void) { // defined in file "startup_stm32f411xe.s"
    HAL_UART_IRQHandler(&huart2);
}


void
HAL_UART_ErrorCallback(UART_HandleTypeDef *UartHandle) {
    (void)UartHandle;
    Error_Handler();
}


#ifdef  USE_FULL_ASSERT
    char txBuffer[256];

    void
    assert_failed(uint8_t *file, uint32_t line) {
        sprintf((char *)txBuffer, "%s - line: %d", file, line);
        nh.loginfo(txBuffer);
        while(true) {
            HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
            HAL_Delay(200);
        }
    }
#endif // USE_FULL_ASSERT
